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
#define RMT_ENCODE_RX_CHANNEL(encode_chan) ((encode_chan + RMT_RX_CHANNEL_ENCODING_START))

#define ESPFC_RMT_RX_BUFFER_SIZE 128

// faster esc response, but unsafe (no task synchronisation)
// set to 0 in case of issues
#define ESPFC_RMT_BYPASS_WRITE_SYNC 1

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
  if(tx_idx_rst)
  {
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

static void IRAM_ATTR _rmt_rx_zero_mem(rmt_channel_t channel, size_t len)
{
  volatile rmt_item32_t *data = (rmt_item32_t *)RMTMEM.chan[RMT_ENCODE_RX_CHANNEL(channel)].data32;
  for (size_t idx = 0; idx < len; idx++)
  {
    data[idx].val = 0;
  }
}

// using map() in isr cause crash
static long IRAM_ATTR _map(long x, long in_min, long in_max, long out_min, long out_max)
{
  const long in_range = in_max - in_min;
  if(in_range == 0) return out_min;
  const long out_range = out_max - out_min;
  const long delta = x - in_min;
  return (delta * out_range) / in_range + out_min;
}

bool EscDriverEsp32::_tx_end_installed = false;

EscDriverEsp32 *EscDriverEsp32::instances[] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

EscDriverEsp32::EscDriverEsp32() : _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50), _digital(false), _dshot_tlm(false), _channel_mask(0)
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

int EscDriverEsp32::begin(const EscConfig& conf)
{
  _protocol = ESC_PROTOCOL_SANITIZE(conf.protocol);
  _async = _protocol == ESC_PROTOCOL_BRUSHED ? true : conf.async; // force async for brushed
  _rate = constrain(conf.rate, 50, 8000);
  _interval = TO_INTERVAL_US(_rate);
  _digital = isDigital(_protocol);
  _dshot_tlm = conf.dshotTelemetry && (_protocol == ESC_PROTOCOL_DSHOT300 || _protocol == ESC_PROTOCOL_DSHOT600);
  _timer.setInterval(1000000);

  return 1;
}

int EscDriverEsp32::attach(size_t channel, int pin, int pulse)
{
  if (channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  initChannel(channel, (gpio_num_t)pin, pulse);
  return 1;
}

int IRAM_ATTR EscDriverEsp32::write(size_t channel, int pulse)
{
  if (channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  _channel[channel].pulse = pulse;
  return 1;
}

void IRAM_ATTR EscDriverEsp32::apply()
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

uint32_t IRAM_ATTR EscDriverEsp32::telemetry(size_t channel) const
{
  if (channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
  return _channel[channel].telemetryValue;
}

void EscDriverEsp32::initChannel(int i, gpio_num_t pin, int pulse)
{
  if (pin == -1) return;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, _dshot_tlm ? HIGH : LOW);

  _channel[i].pin = pin;
  _channel[i].protocol = _protocol;
  _channel[i].pulse = pulse;
  _channel[i].divider = getClockDivider();
  _channel[i].pulse_min = getPulseMin();
  _channel[i].pulse_max = getPulseMax();
  _channel[i].pulse_space = nsToTicks(_interval * 1000);

  // specification 0:37%, 1:75% of 1670ns for dshot600
  //_channel[i].dshot_t0h = getDshotPulse(625);
  //_channel[i].dshot_t0l = getDshotPulse(1045);
  //_channel[i].dshot_t1h = getDshotPulse(1250);
  //_channel[i].dshot_t1l = getDshotPulse(420);

  // betaflight 0:35%, 1:70% of 1670ns for dshot600
  _channel[i].dshot_t0h = getDshotPulse(584);
  _channel[i].dshot_t0l = getDshotPulse(1086);
  _channel[i].dshot_t1h = getDshotPulse(1170);
  _channel[i].dshot_t1l = getDshotPulse(500);
  _channel[i].dshot_tlm_bit_len = (_channel[i].dshot_t0h + _channel[i].dshot_t0l) * 4 / 5;

  instances[i] = this;

  rmt_channel_t rx_ch = (rmt_channel_t)RMT_ENCODE_RX_CHANNEL(i);
  bool splitted = i != rx_ch; // rx and tx are separated

  if(_digital && _dshot_tlm)
  {
    // setup RX
    rmt_config_t rxconf = RMT_DEFAULT_CONFIG_RX(pin, rx_ch);
    rxconf.clk_div = _channel[i].divider;
    rxconf.rx_config.filter_en = true;
    rxconf.rx_config.idle_threshold = nsToTicks(30000); // max bit len, (30us)
    rxconf.rx_config.filter_ticks_thresh = 40; // min bit len, 80 ticks = 1us for div = 1, max value = 255
    rmt_config(&rxconf);
    gpio_pullup_en((gpio_num_t)pin); // needed ?

    if(splitted)
    {
      rmt_driver_install((rmt_channel_t)rx_ch, ESPFC_RMT_RX_BUFFER_SIZE, ESP_INTR_FLAG_IRAM);
    }
  }

  // setup TX
  rmt_config_t txconf = RMT_DEFAULT_CONFIG_TX(pin, (rmt_channel_t)i);
  txconf.clk_div = _channel[i].divider;
  txconf.tx_config.idle_level = _dshot_tlm ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;
  rmt_config(&txconf);
  rmt_driver_install((rmt_channel_t)i, splitted ? 0 : ESPFC_RMT_RX_BUFFER_SIZE, ESP_INTR_FLAG_IRAM);

#if 0 && SOC_RMT_SUPPORT_TX_SYNCHRO
  // sync all channels
  if(!_async)
  {
    rmt_ll_tx_enable_sync(&RMT, true);
    _channel_mask |= (1 << i);
    rmt_ll_tx_add_to_sync_group(&RMT, i);
    rmt_ll_tx_reset_channels_clock_div(&RMT, _channel_mask);
  }
#endif

  // install tx_end callback if async pwm or bidir dshot
  if(_async || (_digital && _dshot_tlm))
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

void IRAM_ATTR EscDriverEsp32::modeTx(rmt_channel_t channel)
{
  disableRx(channel);
  enableTx(channel);
}

void IRAM_ATTR EscDriverEsp32::modeRx(rmt_channel_t channel)
{
  disableTx(channel);
  enableRx(channel);
}

void IRAM_ATTR EscDriverEsp32::enableTx(rmt_channel_t channel)
{
  gpio_num_t gpio_num = (gpio_num_t)_channel[(size_t)channel].pin;

  //gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
  gpio_ll_input_disable(&GPIO, gpio_num);
  gpio_ll_output_enable(&GPIO, gpio_num);
  esp_rom_gpio_connect_out_signal(gpio_num, rmt_periph_signals.groups[0].channels[channel].tx_sig, false, false);
  rmt_ll_enable_tx_end_interrupt(&RMT, channel, true);
}

void IRAM_ATTR EscDriverEsp32::disableTx(rmt_channel_t channel)
{
  rmt_ll_tx_stop(&RMT, channel);
}

void IRAM_ATTR EscDriverEsp32::enableRx(rmt_channel_t channel)
{
  // NOTE: time critical function, execution must not exceed 5-6us
  rmt_channel_t rx_ch = (rmt_channel_t)RMT_ENCODE_RX_CHANNEL(channel);
  gpio_num_t gpio_num = (gpio_num_t)_channel[(size_t)channel].pin;

  //gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
  gpio_ll_input_enable(&GPIO, gpio_num);
  gpio_ll_output_disable(&GPIO, gpio_num);
  esp_rom_gpio_connect_in_signal(gpio_num, rmt_periph_signals.groups[0].channels[rx_ch].rx_sig, false);

  //rmt_rx_start((rmt_channel_t)i, true);
  rmt_ll_rx_set_mem_owner(&RMT, channel, RMT_MEM_OWNER_RX);
  rmt_ll_rx_reset_pointer(&RMT, channel);
  rmt_ll_clear_rx_end_interrupt(&RMT, channel);
  rmt_ll_enable_rx_end_interrupt(&RMT, channel, true);
  _rmt_rx_zero_mem(channel, 2); // clear first item of rx buffer to avoid reading tx items
  rmt_ll_rx_enable(&RMT, channel, true);
}

void IRAM_ATTR EscDriverEsp32::disableRx(rmt_channel_t channel)
{
  //rmt_channel_t rx_ch = (rmt_channel_t)RMT_ENCODE_RX_CHANNEL(channel);

  //rmt_rx_stop((rmt_channel_t)channel);
  rmt_ll_enable_rx_end_interrupt(&RMT, channel, false);
  rmt_ll_rx_enable(&RMT, channel, false);
}

void IRAM_ATTR EscDriverEsp32::txDoneCallback(rmt_channel_t channel, void *arg)
{
  auto instance = instances[channel];
  if(!instance) return;
  if(instance->_async)
  {
    instance->transmitOne(channel);
  }
  if(instance->_digital && instance->_dshot_tlm)
  {
    instance->modeRx(channel);
  }
}

void IRAM_ATTR EscDriverEsp32::transmitOne(uint32_t i)
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

void IRAM_ATTR EscDriverEsp32::transmitAll()
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

void IRAM_ATTR EscDriverEsp32::readTelemetry()
{
  for (size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
  {
    if(!_digital || !_dshot_tlm) continue;
    if(!_channel[i].attached()) continue;

    rmt_channel_t rx_ch = (rmt_channel_t)RMT_ENCODE_RX_CHANNEL(i);

    RingbufHandle_t rb = NULL;
    if(ESP_OK != rmt_get_ringbuf_handle((rmt_channel_t)rx_ch, &rb)) continue;

    size_t rmt_len = 0;
    rmt_item32_t* data = (rmt_item32_t*)xRingbufferReceive(rb, &rmt_len, 0);
    if (data)
    {
      uint32_t value = extractTelemetryGcr((uint32_t*)data, rmt_len >> 2, _channel[i].dshot_tlm_bit_len);

      vRingbufferReturnItem(rb, (void *)data);
      
      _channel[i].telemetryValue = value;
    }
  }
}

void IRAM_ATTR EscDriverEsp32::writeAnalogCommand(uint32_t channel, int32_t pulse)
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
  int duration = _map(pulse, 1000, 2000, slot.pulse_min, slot.pulse_max);

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

void IRAM_ATTR EscDriverEsp32::writeDshotCommand(uint32_t channel, int32_t pulse)
{
  if(_digital && _dshot_tlm)
  {
    modeTx((rmt_channel_t)channel);
  }

  pulse = constrain(pulse, 0, 2000);
  // scale to dshot commands (0 or 48-2047)
  int value = dshotConvert(pulse);
  uint16_t frame = dshotEncode(value, _dshot_tlm);

  Slot& slot = _channel[channel];
  for (size_t i = 0; i < DSHOT_BIT_COUNT; i++)
  {
    int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
    slot.setDshotBit(i, val, _dshot_tlm);
  }
  slot.setTerminate(DSHOT_BIT_COUNT, _dshot_tlm);

  _rmt_fill_tx_items((rmt_channel_t)channel, slot.items, Slot::ITEM_COUNT, 0);
}

void IRAM_ATTR EscDriverEsp32::transmitCommand(uint32_t channel)
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
    case ESC_PROTOCOL_MULTISHOT:  return 1;
    case ESC_PROTOCOL_BRUSHED:    return 8;
    default: return 1;
  }
}

uint32_t EscDriverEsp32::getPulseMin() const
{
  switch (_protocol)
  {
    case ESC_PROTOCOL_PWM:        return nsToTicks(1000 * 1000);
    case ESC_PROTOCOL_ONESHOT125: return nsToTicks(125 * 1000);
    case ESC_PROTOCOL_ONESHOT42:  return nsToTicks(42 * 1000);
    case ESC_PROTOCOL_MULTISHOT:  return nsToTicks(5 * 1000);
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
  case ESC_PROTOCOL_PWM:        return nsToTicks(2000 * 1000);
  case ESC_PROTOCOL_ONESHOT125: return nsToTicks(250 * 1000);
  case ESC_PROTOCOL_ONESHOT42:  return nsToTicks(84 * 1000);
  case ESC_PROTOCOL_MULTISHOT:  return nsToTicks(25 * 1000);
  case ESC_PROTOCOL_BRUSHED:    return nsToTicks(_interval * 1000);
  case ESC_PROTOCOL_DSHOT150:
  case ESC_PROTOCOL_DSHOT300:
  case ESC_PROTOCOL_DSHOT600:
  default:
    return 2048;
  }
}

uint32_t EscDriverEsp32::nsToTicks(uint32_t ns) const
{
  int div = getClockDivider();
  // multiply by 2 as DURATION_CLOCK is aleredy doubled, and round to nearest integer
  return (ns * 2 + ((div * DURATION_CLOCK) >> 1)) / div / DURATION_CLOCK;
}

uint16_t EscDriverEsp32::getDshotPulse(uint32_t width) const
{
  switch (_protocol)
  {
    case ESC_PROTOCOL_DSHOT150: return nsToTicks(width * 4);
    case ESC_PROTOCOL_DSHOT300: return nsToTicks(width * 2);
    case ESC_PROTOCOL_DSHOT600: return nsToTicks(width * 1);
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