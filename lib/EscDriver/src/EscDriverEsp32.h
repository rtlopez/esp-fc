#ifndef _ESC_DRIVER_ESP32_H_
#define _ESC_DRIVER_ESP32_H_

#if defined(ESP32)

#include "EscDriver.h"
#include <Arduino.h>
#include <driver/rmt.h>

//#define DURATION  12.5 /* flash 80MHz => minimum time unit in ns */
static const size_t DURATION = 25; // [ns] duoble value to increase precision
static const size_t DSHOT_BITS = 16;

#define TO_INTERVAL(v) (1*1000*1000 / (v)) // [us]

class EscDriverEsp32;
static EscDriverEsp32* instances[] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

class EscDriverEsp32: public EscDriverBase
{
  public:
    class Slot {
      public:
        rmt_config_t dev;
        rmt_item32_t items[DSHOT_BITS + 1];
        EscProtocol protocol;
        int32_t pulse_min;
        int32_t pulse_max;
        int32_t pulse_space;
        uint16_t pulse;
        uint16_t divider;
        uint16_t dshot_t0h;
        uint16_t dshot_t0l;
        uint16_t dshot_t1h;
        uint16_t dshot_t1l;
    };

    EscDriverEsp32(): _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50)
    {
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        _channel[i].dev.gpio_num = gpio_num_t(-1);
      }
      begin(_protocol, _async, _rate);
    }

    void end()
    {
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        if((int)_channel[i].dev.gpio_num == -1) continue;
        rmt_driver_uninstall(_channel[i].dev.channel);
      }
    }

    int begin(EscProtocol protocol, bool async, int32_t rate)
    {
      if(_async) rmt_register_tx_end_callback(NULL, NULL); // unregister old callback

      _protocol = protocol;
      _async = async & false;
      _rate = rate;
      _interval = TO_INTERVAL(_rate);
      
      if(_async) rmt_register_tx_end_callback(&txDoneCallback, NULL);

      return 1;
    }   

    int attach(size_t channel, int pin, int pulse)
    {
      if(channel < 0 || channel >= ESC_CHANNEL_COUNT) return 0;
      initChannel(channel, (gpio_num_t)pin, pulse);
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
      if(_async) return;
      transmitAll();
    }

  private:
    void initChannel(int i, gpio_num_t pin, int pulse)
    {
      if(pin == -1) return;

      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);

      _channel[i].protocol = _protocol;
      _channel[i].pulse = pulse;
      _channel[i].dev.gpio_num = pin;
      _channel[i].divider   = getClockDivider();
      _channel[i].pulse_min = getPulseMin();
      _channel[i].pulse_max = getPulseMax();
      _channel[i].pulse_space = getPulseInterval();

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
   
      instances[i] = this;

      rmt_config(&_channel[i].dev);
      rmt_driver_install(_channel[i].dev.channel, 0, 0);

      if(_async) txDoneCallback((rmt_channel_t)i, NULL);
    }

    static void txDoneCallback(rmt_channel_t channel, void *arg)
    {
      if(!instances[channel]) return;
      EscDriverEsp32* instance = instances[channel];
      instance->transmitOne(channel);
    }

    void transmitOne(uint8_t i)
    {
      if((int)_channel[i].dev.gpio_num == -1) return;
      if(isDigital())
      {
        writeDshotCommand(i, _channel[i].pulse);
      }
      else
      {
        writeAnalogCommand(i, _channel[i].pulse);
      }
      transmitCommand(i);
    }

    void transmitAll()
    {
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        if((int)_channel[i].dev.gpio_num == -1) continue;
        if(isDigital())
        {
          writeDshotCommand(i, _channel[i].pulse);
        }
        else
        {
          writeAnalogCommand(i, _channel[i].pulse);
        }
      }
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        if((int)_channel[i].dev.gpio_num == -1) continue;
        transmitCommand(i);
      }
    }

    void writeAnalogCommand(uint8_t channel, int32_t pulse)
    {
      pulse = constrain(pulse, 1000, 2000) - 1000;
      uint16_t val = fixPulse(map(pulse, 0, 1000, _channel[channel].pulse_min, _channel[channel].pulse_max));
      int space = _async ? _channel[channel].pulse_space - val : 0;

      int i = 0;

      _channel[channel].items[i].duration0 = val;
      _channel[channel].items[i].level0 = 1;
      _channel[channel].items[i].duration1 = 0;
      _channel[channel].items[i].level1 = 0;

      int filled = 0;
      while(true)
      {
        i++;
  
        int fill = fixPulse(space - filled);
        _channel[channel].items[i].duration0 = (uint16_t)fill;
        _channel[channel].items[i].level0 = 1;
        filled += fill;

        fill = fixPulse(space - filled);
        _channel[channel].items[i].duration1 = (uint16_t)fill;
        _channel[channel].items[i].level1 = 0;
        filled += fill;

        if(filled >= space || i >= DSHOT_BITS - 1) break;
      }

      i++;
      //_channel[channel].items[1].val = 0; // terminator
      _channel[channel].items[i].duration0 = 0;
      _channel[channel].items[i].level0 = 0;
      _channel[channel].items[i].duration1 = 0;
      _channel[channel].items[i].level1 = 0;

      rmt_fill_tx_items(_channel[channel].dev.channel, _channel[channel].items, i + 1, 0);
      //rmt_write_items(_channel[channel].dev.channel, _channel[channel].items, 1, 0);
    }

    uint16_t fixPulse(int32_t pulse)
    {
      if(pulse < 0) return 0;
      if(pulse > 0x7fff) return 0x7fff;
      return pulse;
    }

    void writeDshotCommand(uint8_t channel, int32_t pulse)
    {
      pulse = constrain(pulse, 0, 2000);
      int value = 0; // disarmed
      // scale to dshot commands (0 or 48-2047)
      if(pulse > 1000)
      {
        value = PWM_TO_DSHOT(pulse);
      }
      uint16_t val = dshotEncode(value);
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
      //_channel[channel].items[DSHOT_BITS].val = 0; // terminator
      _channel[channel].items[DSHOT_BITS].duration0 = 0;
      _channel[channel].items[DSHOT_BITS].level0 = 0;
      _channel[channel].items[DSHOT_BITS].duration1 = 0;
      _channel[channel].items[DSHOT_BITS].level1 = 0;

      rmt_fill_tx_items(_channel[channel].dev.channel, _channel[channel].items, DSHOT_BITS + 1, 0);
      //rmt_write_items(_channel[channel].dev.channel, _channel[channel].items, DSHOT_BITS, 0);
    }

    void transmitCommand(uint8_t channel)
    {
      rmt_tx_start(_channel[channel].dev.channel, true);
    }

    uint32_t getClockDivider() const
    {
      switch(_protocol)
      {
        case ESC_PROTOCOL_BRUSHED: return 6;
        case ESC_PROTOCOL_PWM: return 6;
        default: return 2;
      }
    }

    uint32_t getPulseMin() const
    {
      switch(_protocol)
      {
        case ESC_PROTOCOL_PWM:        return getAnalogPulse(1000 * 1000);
        case ESC_PROTOCOL_ONESHOT125: return getAnalogPulse( 125 * 1000);
        case ESC_PROTOCOL_ONESHOT42:  return getAnalogPulse(  42 * 1000);
        case ESC_PROTOCOL_MULTISHOT:  return getAnalogPulse(   5 * 1000);
        case ESC_PROTOCOL_BRUSHED:    return 0;
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
      switch(_protocol)
      {
        case ESC_PROTOCOL_PWM:        return getAnalogPulse(2000 * 1000);
        case ESC_PROTOCOL_ONESHOT125: return getAnalogPulse( 250 * 1000);
        case ESC_PROTOCOL_ONESHOT42:  return getAnalogPulse(  84 * 1000);
        case ESC_PROTOCOL_MULTISHOT:  return getAnalogPulse(  25 * 1000);
        case ESC_PROTOCOL_BRUSHED:    return getAnalogPulse(_interval);
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        case ESC_PROTOCOL_DSHOT1200:
        default:
          return 2048;
      }
    }

    uint32_t getPulseInterval() const
    {
      return getAnalogPulse(_interval);
    }

    uint32_t getAnalogPulse(int32_t width) const
    {
      int div = getClockDivider();
      return width / ((div * DURATION) / 2);
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

    Slot _channel[ESC_CHANNEL_COUNT];
    EscProtocol _protocol;
    int32_t _async;
    int32_t _rate;
    int32_t _interval;
};

#endif

#endif
