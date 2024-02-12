#if defined(ESP32) && !defined(ESP32C3)

#include <Arduino.h>
#include "EscDriverEsp32.h"
#include "Debug_Espfc.h"

static const size_t DURATION_CLOCK = 25; // [ns] doubled value to increase precision
#define TO_INTERVAL_US(v) (1 * 1000 * 1000 / (v)) // [us]

// faster esc response, but unsafe (no task synchronisation)
// set to 0 in case of issues
#if defined(ESP32S3) || defined(ESP32S2)
#define ESPFC_RMT_BYPASS_WRITE_SYNC 0
#else
#define ESPFC_RMT_BYPASS_WRITE_SYNC 1
#endif

#if ESPFC_RMT_BYPASS_WRITE_SYNC
IRAM_ATTR static esp_err_t _rmt_fill_tx_items(rmt_channel_t channel, const rmt_item32_t* item, uint16_t item_num, uint16_t mem_offset)
{
  //RMT_CHECK(channel < RMT_CHANNEL_MAX, RMT_CHANNEL_ERROR_STR, (0));
  //RMT_CHECK((item != NULL), RMT_ADDR_ERROR_STR, ESP_ERR_INVALID_ARG);
  //RMT_CHECK((item_num > 0), RMT_DRIVER_LENGTH_ERROR_STR, ESP_ERR_INVALID_ARG);

  /*Each block has 64 x 32 bits of data*/
  //uint8_t mem_cnt = RMT.conf_ch[channel].conf0.mem_size;
  //RMT_CHECK((mem_cnt * RMT_MEM_ITEM_NUM >= item_num), RMT_WR_MEM_OVF_ERROR_STR, ESP_ERR_INVALID_ARG);

  //rmt_fill_memory(channel, item, item_num, mem_offset);
  //portENTER_CRITICAL(&rmt_spinlock);
  RMT.apb_conf.fifo_mask = RMT_DATA_MODE_MEM;
  //portEXIT_CRITICAL(&rmt_spinlock);
  //for(size_t i = 0; i < item_num; i++) {
  //  RMTMEM.chan[channel].data32[i + mem_offset].val = item[i].val;
  //}
  uint32_t *from = (uint32_t *)item;
  volatile uint32_t *to = (volatile uint32_t *)&RMTMEM.chan[channel].data32[0].val;
  to += mem_offset;
  while (item_num--) {
    *to++ = *from++;
  }
  return ESP_OK;
}
IRAM_ATTR static esp_err_t _rmt_tx_start(rmt_channel_t channel, bool tx_idx_rst)
{
  //RMT_CHECK(channel < RMT_CHANNEL_MAX, RMT_CHANNEL_ERROR_STR, ESP_ERR_INVALID_ARG);
  //portENTER_CRITICAL(&rmt_spinlock);
  if(tx_idx_rst) {
    RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
  }
  RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
  RMT.conf_ch[channel].conf1.tx_start = 1;
  //portEXIT_CRITICAL(&rmt_spinlock);
  return ESP_OK;
}
#else
#define _rmt_tx_start rmt_tx_start
#define _rmt_fill_tx_items rmt_fill_tx_items
#endif

bool EscDriverEsp32::_tx_end_installed = false;

EscDriverEsp32 *EscDriverEsp32::instances[] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

EscDriverEsp32::EscDriverEsp32() : _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50), _digital(false)
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    _channel[i].dev.gpio_num = gpio_num_t(-1);
  }
}

void EscDriverEsp32::end()
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    if (!_channel[i].attached()) continue;
    rmt_driver_uninstall(_channel[i].dev.channel);
  }
  _protocol = ESC_PROTOCOL_DISABLED;
}

int EscDriverEsp32::begin(EscProtocol protocol, bool async, int32_t rate, int timer)
{
  (void)timer; // unused

  _protocol = protocol;
  _digital = isDigital(protocol);
  _async = async;
  _rate = rate;
  _interval = TO_INTERVAL_US(_rate);

  return 1;
}

int EscDriverEsp32::attach(size_t channel, int pin, int pulse)
{
  if (channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  initChannel(channel, (gpio_num_t)pin, pulse);
  return 1;
}

int EscDriverEsp32::write(size_t channel, int pulse)
{
  if (channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _channel[channel].pulse = pulse;
  return 1;
}

void EscDriverEsp32::apply()
{
  if (_protocol == ESC_PROTOCOL_DISABLED) return;
  if (_async) return;
  transmitAll();
}

int EscDriverEsp32::pin(size_t channel) const
{
  if (channel < 0 || channel >= ESC_CHANNEL_COUNT) return -1;
  return _channel[channel].dev.gpio_num;
}

void EscDriverEsp32::initChannel(int i, gpio_num_t pin, int pulse)
{
  if (pin == -1) return;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  _channel[i].protocol = _protocol;
  _channel[i].pulse = pulse;
  _channel[i].divider = getClockDivider();
  _channel[i].pulse_min = getPulseMin();
  _channel[i].pulse_max = getPulseMax();
  _channel[i].pulse_space = getPulseInterval();

  // specification 0:37%, 1:75% of 1670ns for dshot600
  //_channel[i].dshot_t0h = getDshotPulse(625);
  //_channel[i].dshot_t0l = getDshotPulse(1045);
  //_channel[i].dshot_t1h = getDshotPulse(1250);
  //_channel[i].dshot_t1l = getDshotPulse(420);

  // betaflight 0:35%, 1:70% of 1670ns for dshot600
  _channel[i].dshot_t0h = getDshotPulse(584 - 16);
  _channel[i].dshot_t0l = getDshotPulse(1086 - 32);
  _channel[i].dshot_t1h = getDshotPulse(1170 - 32);
  _channel[i].dshot_t1l = getDshotPulse(500 - 16);

  _channel[i].dev.gpio_num = pin;
  _channel[i].dev.rmt_mode = RMT_MODE_TX;
  _channel[i].dev.channel = (rmt_channel_t)i;
  _channel[i].dev.clk_div = _channel[i].divider;
  _channel[i].dev.mem_block_num = 1;
  _channel[i].dev.tx_config.loop_en = 0;
  _channel[i].dev.tx_config.idle_output_en = 1;
  _channel[i].dev.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  // unused
  _channel[i].dev.tx_config.carrier_duty_percent = 50;
  _channel[i].dev.tx_config.carrier_freq_hz = 1000;
  _channel[i].dev.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  _channel[i].dev.tx_config.carrier_en = 0;

  instances[i] = this;

  rmt_config(&_channel[i].dev);
  rmt_driver_install(_channel[i].dev.channel, 0, 0);
  if (/*_async && */!_tx_end_installed)
  {
    _tx_end_installed = true;
    rmt_register_tx_end_callback(txDoneCallback, NULL);
  }
  if (_async)
  {
    rmt_set_tx_intr_en(_channel[i].dev.channel, true);
    txDoneCallback((rmt_channel_t)i, NULL); // start generating pulses
  }
}

void EscDriverEsp32::txDoneCallback(rmt_channel_t channel, void *arg)
{
  PIN_DEBUG(HIGH);
  if (instances[channel] && instances[channel]->_async)
  {
    instances[channel]->transmitOne(channel);
  }
  PIN_DEBUG(LOW);
}

void EscDriverEsp32::transmitOne(uint8_t i)
{
  if (!_channel[i].attached()) return;
  if (_digital)
  {
    writeDshotCommand(i, _channel[i].pulse);
  }
  else
  {
    writeAnalogCommand(i, _channel[i].pulse);
  }
  transmitCommand(i);
}

void EscDriverEsp32::transmitAll()
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    if (!_channel[i].attached()) continue;
    if (_digital)
    {
      writeDshotCommand(i, _channel[i].pulse);
    }
    else
    {
      writeAnalogCommand(i, _channel[i].pulse);
    }
  }
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    if (!_channel[i].attached()) continue;
    transmitCommand(i);
  }
}

void EscDriverEsp32::writeAnalogCommand(uint8_t channel, int32_t pulse)
{
  Slot &slot = _channel[channel];
  int minPulse = 800;
  int maxPulse = 2200;
  if (_protocol == ESC_PROTOCOL_BRUSHED)
  {
    minPulse = 1000;
    maxPulse = 2000;
  }
  pulse = constrain(pulse, minPulse, maxPulse);
  int duration = map(pulse, 1000, 2000, slot.pulse_min, slot.pulse_max);

  int count = 2;
  if (_async)
  {
    int space = slot.pulse_space - duration;
    slot.setDuration(0, duration, 1);
    slot.setDuration(1, space, 0);
    slot.setTerminate(2, 0);
    count = 3;
  }
  else
  {
    slot.setDuration(0, duration, 1);
    slot.setTerminate(1, 0);
  }

  _rmt_fill_tx_items(_channel[channel].dev.channel, _channel[channel].items, count, 0);
}

void EscDriverEsp32::writeDshotCommand(uint8_t channel, int32_t pulse)
{
  pulse = constrain(pulse, 0, 2000);
  // scale to dshot commands (0 or 48-2047)
  int value = pulse > 1000 ? PWM_TO_DSHOT(pulse) : 0;
  uint16_t frame = dshotEncode(value);

  Slot &slot = _channel[channel];
  for (size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
    slot.setDshotBit(i, val);
  }
  slot.setTerminate(DSHOT_BIT_COUNT, 0);

  _rmt_fill_tx_items(slot.dev.channel, slot.items, Slot::ITEM_COUNT, 0);
}

void EscDriverEsp32::transmitCommand(uint8_t channel)
{
  _rmt_tx_start(_channel[channel].dev.channel, true);
}

uint32_t EscDriverEsp32::getClockDivider() const
{
  switch (_protocol)
  {
    case ESC_PROTOCOL_PWM:        return 24;
    case ESC_PROTOCOL_ONESHOT125: return 4;
    case ESC_PROTOCOL_ONESHOT42:  return 2;
    case ESC_PROTOCOL_MULTISHOT:  return 2;
    case ESC_PROTOCOL_BRUSHED:    return 8;
    default: return 1;
  }
}

uint32_t EscDriverEsp32::getPulseMin() const
{
  switch (_protocol)
  {
    case ESC_PROTOCOL_PWM:        return getAnalogPulse(1000 * 1000);
    case ESC_PROTOCOL_ONESHOT125: return getAnalogPulse(125 * 1000);
    case ESC_PROTOCOL_ONESHOT42:  return getAnalogPulse(42 * 1000);
    case ESC_PROTOCOL_MULTISHOT:  return getAnalogPulse(5 * 1000);
    case ESC_PROTOCOL_BRUSHED:    return 0;
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
    default:
      return 0;
  }
}

uint32_t EscDriverEsp32::getPulseMax() const
{
  switch (_protocol)
  {
  case ESC_PROTOCOL_PWM:        return getAnalogPulse(2000 * 1000);
  case ESC_PROTOCOL_ONESHOT125: return getAnalogPulse(250 * 1000);
  case ESC_PROTOCOL_ONESHOT42:  return getAnalogPulse(84 * 1000);
  case ESC_PROTOCOL_MULTISHOT:  return getAnalogPulse(25 * 1000);
  case ESC_PROTOCOL_BRUSHED:    return getAnalogPulse(_interval * 1000);
  case ESC_PROTOCOL_DSHOT150:
  case ESC_PROTOCOL_DSHOT300:
  case ESC_PROTOCOL_DSHOT600:
  default:
    return 2048;
  }
}

uint32_t EscDriverEsp32::getPulseInterval() const
{
  return getAnalogPulse(_interval * 1000);
}

uint32_t EscDriverEsp32::getAnalogPulse(int32_t ns) const
{
  int div = getClockDivider();
  return ns / ((div * DURATION_CLOCK) / 2);
}

uint16_t EscDriverEsp32::getDshotPulse(uint32_t width) const
{
  int div = getClockDivider();
  switch (_protocol)
  {
    case ESC_PROTOCOL_DSHOT150: return ((width * 4) / (div * DURATION_CLOCK / 2));
    case ESC_PROTOCOL_DSHOT300: return ((width * 2) / (div * DURATION_CLOCK / 2));
    case ESC_PROTOCOL_DSHOT600: return ((width * 1) / (div * DURATION_CLOCK / 2));
    case ESC_PROTOCOL_BRUSHED:
    case ESC_PROTOCOL_PWM:
    case ESC_PROTOCOL_ONESHOT125:
    case ESC_PROTOCOL_ONESHOT42:
    case ESC_PROTOCOL_MULTISHOT:
    default:
      return 0;
  }
}

bool EscDriverEsp32::isDigital(EscProtocol protocol) const
{
  switch (protocol)
  {
    case ESC_PROTOCOL_DSHOT150:
    case ESC_PROTOCOL_DSHOT300:
    case ESC_PROTOCOL_DSHOT600:
      return true;
    default:
      return false;
  }
}

#endif