#if defined(ESP32) && !defined(ESP32C3)

#include <Arduino.h>
#include "EscDriverEsp32.h"
#include "Debug_Espfc.h"
#include "soc/rmt_periph.h"
#include "hal/rmt_ll.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"

static const size_t DURATION_CLOCK = 25; // [ns] doubled value to increase precision
#define TO_INTERVAL_US(v) (1 * 1000 * 1000 / (v)) // [us]

#define RMT_RX_CHANNEL_ENCODING_START (SOC_RMT_CHANNELS_PER_GROUP - SOC_RMT_TX_CANDIDATES_PER_GROUP)
#define RMT_TX_CHANNEL_ENCODING_END   (SOC_RMT_TX_CANDIDATES_PER_GROUP - 1)

#define RMT_IS_RX_CHANNEL(channel) ((channel) >= RMT_RX_CHANNEL_ENCODING_START)
#define RMT_IS_TX_CHANNEL(channel) ((channel) <= RMT_TX_CHANNEL_ENCODING_END)
#define RMT_ENCODE_RX_CHANNEL(encode_chan) ((encode_chan + RMT_RX_CHANNEL_ENCODING_START))

// faster esc response, but unsafe (no task synchronisation)
// set to 0 in case of issues
#if defined(ESP32S3) || defined(ESP32S2)
#define ESPFC_RMT_BYPASS_WRITE_SYNC 1
#else
#define ESPFC_RMT_BYPASS_WRITE_SYNC 1
#endif

#if ESPFC_RMT_BYPASS_WRITE_SYNC
IRAM_ATTR static esp_err_t _rmt_fill_tx_items(rmt_channel_t channel, const rmt_item32_t* item, uint16_t item_num, uint16_t mem_offset)
{
  //rmt_ll_enable_mem_access(&RMT, true);
  rmt_ll_write_memory(&RMTMEM, channel, item, item_num, mem_offset);
  rmt_ll_tx_reset_pointer(&RMT, channel);
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_MEM_OWNER_TX);
  return ESP_OK;
}
IRAM_ATTR static esp_err_t _rmt_tx_start(rmt_channel_t channel, bool tx_idx_rst)
{
  if(tx_idx_rst) {
    rmt_ll_tx_reset_pointer(&RMT, channel); // BC
  }
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_MEM_OWNER_TX); // moved to _rmt_fill_tx_items
  rmt_ll_clear_tx_end_interrupt(&RMT, channel);
  rmt_ll_tx_start(&RMT, channel);
  return ESP_OK;
}
#else
#define _rmt_tx_start rmt_tx_start
#define _rmt_fill_tx_items rmt_fill_tx_items
#endif

static int IRAM_ATTR _rmt_rx_get_mem_len_in_isr(rmt_channel_t channel)
{
    volatile rmt_item32_t *data = (rmt_item32_t *)RMTMEM.chan[RMT_ENCODE_RX_CHANNEL(channel)].data32;
    for (size_t idx = 0; idx < RMT_MEM_ITEM_NUM; idx++) {
        if (data[idx].duration0 == 0) {
            return idx;
        } else if (data[idx].duration1 == 0) {
            return idx + 1;
        }
    }
    return RMT_MEM_ITEM_NUM;
}

static void IRAM_ATTR _rmt_zero_mem(rmt_channel_t channel, size_t len)
{
  volatile rmt_item32_t *data = (rmt_item32_t *)RMTMEM.chan[channel].data32;
  for (size_t idx = 0; idx < len; idx++) {
    data[idx].val = 0;
  }
}

bool EscDriverEsp32::_tx_end_installed = false;

EscDriverEsp32 *EscDriverEsp32::instances[] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

EscDriverEsp32::EscDriverEsp32() : _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50), _digital(false), _invert(false), _channel_mask(0)
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    _channel[i].pin = gpio_num_t(-1);
  }
}

void EscDriverEsp32::end()
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    if (!_channel[i].attached()) continue;
    rmt_driver_uninstall((rmt_channel_t)i);
  }
  _protocol = ESC_PROTOCOL_DISABLED;
}

int EscDriverEsp32::begin(EscProtocol protocol, bool async, int32_t rate, int timer)
{
  (void)timer; // unused

  _protocol = protocol;
  _digital = isDigital(protocol);
  if(_digital) _invert = true;
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
  return _channel[channel].pin;
}

void EscDriverEsp32::initChannel(int i, gpio_num_t pin, int pulse)
{
  if (pin == -1) return;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, _invert ? HIGH : LOW);
  if(_invert) gpio_pullup_en((gpio_num_t)pin); // ?

  _channel[i].pin = pin;
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

  instances[i] = this;

  // init as TX
  rmt_config_t conf = RMT_DEFAULT_CONFIG_TX(pin, (rmt_channel_t)i);
  conf.clk_div = _channel[i].divider;
  conf.tx_config.idle_level = _invert ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;
  rmt_config(&conf);

  // add RX specifics
  int rx_ch = RMT_ENCODE_RX_CHANNEL(i);
  rmt_ll_rx_set_idle_thres(&RMT, rx_ch, 1000);
  rmt_ll_rx_set_filter_thres(&RMT, rx_ch, 10);
  rmt_ll_rx_enable_filter(&RMT, rx_ch, 1);
  _rmt_zero_mem((rmt_channel_t)rx_ch, RMT_MEM_ITEM_NUM);

#if SOC_RMT_SUPPORT_TX_SYNCHRO
  // sync all channels
  if(!_async)
  {
    rmt_ll_tx_enable_sync(&RMT, true);
    _channel_mask |= (1 << i);
    rmt_ll_tx_add_to_sync_group(&RMT, i);
    rmt_ll_tx_reset_channels_clock_div(&RMT, _channel_mask);
  }
#endif

  // install driver
  rmt_driver_install((rmt_channel_t)i, 0, ESP_INTR_FLAG_IRAM);

  // install tx_end callback if async pwm or bidir dshot
  if(_async || (_digital && _invert))
  {
    // install only once
    if(!_tx_end_installed)
    {
      _tx_end_installed = true;
      rmt_register_tx_end_callback(txDoneCallback, NULL);
    }
    rmt_ll_enable_tx_end_interrupt(&RMT, i, true);
    if (_async)
    {
      txDoneCallback((rmt_channel_t)i, NULL); // start generating pulses
    }
  }
}

void EscDriverEsp32::modeTx(rmt_channel_t channel)
{
  //PIN_DEBUG(HIGH);
  disableRx(channel);
  enableTx(channel);
  //PIN_DEBUG(LOW);
}

void EscDriverEsp32::modeRx(rmt_channel_t channel)
{
  //PIN_DEBUG(HIGH);
  disableTx(channel);
  enableRx(channel);
  //PIN_DEBUG(LOW);
}

void EscDriverEsp32::enableTx(rmt_channel_t channel)
{
  gpio_num_t gpio_num = (gpio_num_t)_channel[(size_t)channel].pin;

  //gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
  gpio_ll_input_disable(&GPIO, gpio_num);
  gpio_ll_output_enable(&GPIO, gpio_num);
  esp_rom_gpio_connect_out_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel].tx_sig, false, 0);
  rmt_ll_enable_tx_end_interrupt(&RMT, channel, true);
}

void EscDriverEsp32::disableTx(rmt_channel_t channel)
{
  rmt_ll_tx_stop(&RMT, channel);
}

void EscDriverEsp32::enableRx(rmt_channel_t channel)
{
  // NOTE: time critical function, execution must not exceed 5-6us
  rmt_channel_t rx_ch = (rmt_channel_t)RMT_ENCODE_RX_CHANNEL(channel);
  gpio_num_t gpio_num = (gpio_num_t)_channel[(size_t)channel].pin;

  //gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
  gpio_ll_input_enable(&GPIO, gpio_num);
  gpio_ll_output_disable(&GPIO, gpio_num);
  esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[rx_ch].rx_sig, false);

  //rmt_rx_start((rmt_channel_t)i, true);
  rmt_ll_rx_set_mem_owner(&RMT, rx_ch, RMT_MEM_OWNER_RX);
  rmt_ll_rx_reset_pointer(&RMT, rx_ch);
  rmt_ll_clear_rx_end_interrupt(&RMT, rx_ch);
  rmt_ll_enable_rx_end_interrupt(&RMT, rx_ch, true);
  _rmt_zero_mem(rx_ch, 1); // clear first item of rx buffer to avoid reading tx items
  rmt_ll_rx_enable(&RMT, rx_ch, true);
}

void EscDriverEsp32::disableRx(rmt_channel_t channel)
{
  rmt_channel_t rx_ch = (rmt_channel_t)RMT_ENCODE_RX_CHANNEL(channel);

  //rmt_rx_stop((rmt_channel_t)channel);
  rmt_ll_enable_rx_end_interrupt(&RMT, rx_ch, false);
  rmt_ll_rx_enable(&RMT, rx_ch, false);
}

void EscDriverEsp32::txDoneCallback(rmt_channel_t channel, void *arg)
{
  //PIN_DEBUG(HIGH);
  auto instance = instances[channel];
  if(!instance) return;
  if(instance->_async)
  {
    instance->transmitOne(channel);
  }
  if(instance->_digital && instance->_invert)
  {
    instance->modeRx(channel);
  }
  //PIN_DEBUG(LOW);
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
  readTelemetry();
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

void EscDriverEsp32::readTelemetry()
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    if (!_channel[i].attached()) continue;
    if (!_digital) continue;
    if (!_invert) continue;
    _channel[i].telemetryValue = _rmt_rx_get_mem_len_in_isr((rmt_channel_t)i);
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

  _rmt_fill_tx_items((rmt_channel_t)channel, _channel[channel].items, count, 0);
}

void EscDriverEsp32::writeDshotCommand(uint8_t channel, int32_t pulse)
{
  if(_digital && _invert)
  {
    modeTx((rmt_channel_t)channel);
  }

  pulse = constrain(pulse, 0, 2000);
  // scale to dshot commands (0 or 48-2047)
  int value = pulse > 1000 ? PWM_TO_DSHOT(pulse) : 0;
  uint16_t frame = dshotEncode(value, _invert);

  Slot& slot = _channel[channel];
  for (size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
    slot.setDshotBit(i, val, _invert);
  }
  slot.setTerminate(DSHOT_BIT_COUNT, _invert);

  _rmt_fill_tx_items((rmt_channel_t)channel, slot.items, Slot::ITEM_COUNT, 0);
}

void EscDriverEsp32::transmitCommand(uint8_t channel)
{
  _rmt_tx_start((rmt_channel_t)channel, true);
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