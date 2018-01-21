#ifndef _ESP32_ESC_DRIVER_H_
#define _ESP32_ESC_DRIVER_H_

#if defined(ESP32)

#include "EscDriver.h"
#include "Arduino.h"
#include <driver/rmt.h>

//#define DURATION  12.5 /* flash 80MHz => minimum time unit in ns */
static const size_t DURATION = 25; // duoble value to increase precision
static const size_t DSHOT_BITS = 16;

class Esp32EscDriver
{
  public:
    class Slot {
      public:
        rmt_config_t dev;
        uint16_t pulse;
        uint16_t divider;
        uint32_t pulse_min;
        uint32_t pulse_max;
        uint16_t dshot_t0h;
        uint16_t dshot_t0l;
        uint16_t dshot_t1h;
        uint16_t dshot_t1l;
        rmt_item32_t items[DSHOT_BITS];
    };

    Esp32EscDriver(): _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50)
    {

    }

    int begin(EscProtocol protocol, bool async, int16_t rate)
    {
      _protocol = protocol;
      _async = async;
      _rate = rate;

      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        _channel[i].divider = getClockDivider();
        _channel[i].pulse_min = getPulseMin();
        _channel[i].pulse_max = getPulseMax();

        _channel[i].dshot_t0h = getDshotPulse(625);
        _channel[i].dshot_t0l = getDshotPulse(1045);
        _channel[i].dshot_t1h = getDshotPulse(1250);
        _channel[i].dshot_t1l = getDshotPulse(420);

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

        if(_channel[i].dev.gpio_num != -1)
        {
          rmt_config(&_channel[i].dev);
          rmt_driver_install(_channel[i].dev.channel, 0, 0);
        }
      }
      return 1;
    }

    int attach(size_t channel, int pin, int pulse)
    {
      if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
      _channel[channel].pulse = pulse;
      _channel[channel].dev.gpio_num = (gpio_num_t)pin;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
      return 1;
    }

    int write(size_t channel, int pulse)
    {
      if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
      _channel[channel].pulse = pulse;
      return 1;
    }

    void apply()
    {
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        if(_channel[i].dev.gpio_num == -1) continue;
        if(isDigital())
        {
          writeDshotCommand(i, _channel[i].pulse);
        }
        else
        {
          writeAnalogCommand(i, _channel[i].pulse);
        }
      }
    }

    void writeAnalogCommand(uint8_t channel, uint16_t pulse)
    {
      uint16_t val = map(pulse, 1000, 2000, _channel[channel].pulse_min, _channel[channel].pulse_max);
      _channel[channel].items[0].duration0 = val;
      _channel[channel].items[0].level0 = 1;
      _channel[channel].items[0].duration1 = 16;
      _channel[channel].items[0].level1 = 0;
      rmt_write_items(_channel[channel].dev.channel, _channel[channel].items, 1, 0);
    }

    void writeDshotCommand(uint8_t channel, uint16_t pulse)
    {
      uint16_t val = pulse; // map(pulse, 1000, 2000, _channel[channel].pulse_min, _channel[channel].pulse_max);
      for(int b = 0; b < DSHOT_BITS; b++)
      {
        if((val >> b) & 1)
        {
          _channel[channel].items[b].duration0 = _channel[channel].dshot_t1h;
          _channel[channel].items[b].level0 = 1;
          _channel[channel].items[b].duration1 = _channel[channel].dshot_t1l;
          _channel[channel].items[b].level1 = 0;
        }
        else
        {
          _channel[channel].items[b].duration0 = _channel[channel].dshot_t0h;
          _channel[channel].items[b].level0 = 1;
          _channel[channel].items[b].duration1 = _channel[channel].dshot_t1l;
          _channel[channel].items[b].level1 = 0;
        }
      }
      rmt_write_items(_channel[channel].dev.channel, _channel[channel].items, DSHOT_BITS, 0);
    }

    uint32_t getClockDivider() const
    {
      switch(_protocol)
      {
        case ESC_PROTOCOL_PWM: return 6;
        default: return 2;
      }
    }

    uint32_t getPulseMin() const
    {
      int div = getClockDivider();
      switch(_protocol)
      {
        case ESC_PROTOCOL_PWM:        return (1000 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_ONESHOT125: return ( 125 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_ONESHOT42:  return (  42 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_MULTISHOT:  return (   5 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_BRUSHED: // TODO
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_DSHOT1200:
        default:
          return 0;
      }
    }

    uint32_t getPulseMax() const
    {
      int div = getClockDivider();
      switch(_protocol)
      {
        case ESC_PROTOCOL_PWM:        return (2000 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_ONESHOT125: return ( 250 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_ONESHOT42:  return (  84 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_MULTISHOT:  return (  20 * 1000) / ((div * DURATION) / 2);
        case ESC_PROTOCOL_BRUSHED: // TODO
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_DSHOT1200:
        default:
          return 2048;
      }
    }

    uint16_t getDshotPulse(uint16_t width) const
    {
      int div = getClockDivider();
      switch(_protocol)
      {
        case ESC_PROTOCOL_DSHOT150:  return width / ((div * DURATION) / 2) * 4;
        case ESC_PROTOCOL_DSHOT300:  return width / ((div * DURATION) / 2) * 2;
        case ESC_PROTOCOL_DSHOT600:  return width / ((div * DURATION) / 2);
        case ESC_PROTOCOL_DSHOT1200: return width / ((div * DURATION) / 2) / 2;
        case ESC_PROTOCOL_BRUSHED: // TODO
        case ESC_PROTOCOL_PWM:
        case ESC_PROTOCOL_ONESHOT125:
        case ESC_PROTOCOL_ONESHOT42:
        case ESC_PROTOCOL_MULTISHOT:
        default:
          return 0;
      }
    }

    bool isDigital() const
    {
      switch(_protocol)
      {
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_DSHOT1200:
          return true;
        default:
          return false;
      }
    }

  private:
    EscProtocol _protocol;
    bool _async;
    int16_t _rate;
    Slot _channel[ESC_CHANNEL_COUNT];
};

#endif

#endif
