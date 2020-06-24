#ifndef _ESC_DRIVER_ESP32_H_
#define _ESC_DRIVER_ESP32_H_

#if defined(ESP32)

#include "EscDriver.h"
#include <Arduino.h>
#include <driver/rmt.h>

//#define DURATION  12.5 /* flash 80MHz => minimum time unit in ns */
static const size_t DURATION_CLOCK = 25; // [ns] duobled value to increase precision
static const size_t ITEM_COUNT = EscDriverBase::DSHOT_BIT_COUNT + 1;
static const int32_t DURATION_MAX = 0x7fff; // max in 15 bits

#define TO_INTERVAL(v) (1 * 1000 * 1000 / (v)) // [us]

// faster esc response, but unsafe (no task synchronisation)
#define ESPFC_RMT_BYPASS_WRITE_SYNC 1

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
  for(size_t i = 0; i < item_num; i++) {
    RMTMEM.chan[channel].data32[i + mem_offset].val = item[i].val;
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


class EscDriverEsp32;
static EscDriverEsp32* instances[] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

class EscDriverEsp32: public EscDriverBase
{
  public:
    class Slot {
      public:
        rmt_config_t dev;
        rmt_item32_t items[ITEM_COUNT];
        EscProtocol protocol;
        int32_t pulse_min;
        int32_t pulse_max;
        int32_t pulse_space;
        int32_t pulse;
        uint16_t divider;
        uint16_t dshot_t0h;
        uint16_t dshot_t0l;
        uint16_t dshot_t1h;
        uint16_t dshot_t1l;

        void setTerminate(int item)
        {
          items[item].val = 0ul;
        }

        void setDshotBit(int item, bool val)
        {
          const uint32_t th = (val ? dshot_t1h : dshot_t0h) & 0x7fff;
          const uint32_t tl = (val ? dshot_t1l : dshot_t0l) & 0x7fff;
          items[item].val = th | 1 << 15 | tl << 16;
          //items[item].duration0 = th;
          //items[item].level0 = 1;
          //items[item].duration1 = tl;
          //items[item].level1 = 0;
        }

        void setDuration(int item, int duration, bool val)
        {
          if(duration >= 2)
          {
            int half = duration / 2;
            items[item].level0 = val;
            items[item].duration0 = half & DURATION_MAX;
            items[item].level1 = val;
            items[item].duration1 = (duration - half) & DURATION_MAX;
          }
          else
          {
            items[item].level0 = 0;
            items[item].duration0 = 1;
            items[item].level1 = 0;
            items[item].duration1 = 1;
          }
        }

        bool attached()
        {
          return (int)dev.gpio_num != -1;
        }
    };

    EscDriverEsp32(): _protocol(ESC_PROTOCOL_PWM), _async(true), _rate(50)
    {
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        _channel[i].dev.gpio_num = gpio_num_t(-1);
      }
      //begin(_protocol, _async, _rate);
    }

    void end()
    {
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        if(!_channel[i].attached()) continue;
        rmt_driver_uninstall(_channel[i].dev.channel);
      }
    }

    int begin(EscProtocol protocol, bool async, int32_t rate, int timer = 0)
    {
      (void)timer; // unused
      if(_async) rmt_register_tx_end_callback(NULL, NULL); // unregister old callback

      _protocol = protocol;
      _async = async;// && false;
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
      _channel[i].divider   = getClockDivider();
      _channel[i].pulse_min = getPulseMin();
      _channel[i].pulse_max = getPulseMax();
      _channel[i].pulse_space = getPulseInterval();

      // specification 0:37%, 1:75% of 1670ns for dshot600
      //_channel[i].dshot_t0h = getDshotPulse(625);
      //_channel[i].dshot_t0l = getDshotPulse(1045);
      //_channel[i].dshot_t1h = getDshotPulse(1250);
      //_channel[i].dshot_t1l = getDshotPulse(420);

      // betaflight 0:35%, 1:70% of 1670ns for dshot600
      _channel[i].dshot_t0h = getDshotPulse(584 - 2);
      _channel[i].dshot_t0l = getDshotPulse(1086 - 2);
      _channel[i].dshot_t1h = getDshotPulse(1170 - 2);
      _channel[i].dshot_t1l = getDshotPulse(500 - 2);

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

      if(_async) txDoneCallback((rmt_channel_t)i, NULL);
    }

    static void txDoneCallback(rmt_channel_t channel, void *arg)
    {
      if(!instances[channel] || !instances[channel]->_async) return;
      instances[channel]->transmitOne(channel);
    }

    void transmitOne(uint8_t i)
    {
      if(!_channel[i].attached()) return;
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
      bool digital = isDigital();
      for(size_t i = 0; i < ESC_CHANNEL_COUNT; i++)
      {
        if(!_channel[i].attached()) continue;
        if(digital)
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
        if(!_channel[i].attached()) continue;
        transmitCommand(i);
      }
    }

    void writeAnalogCommand(uint8_t channel, int32_t pulse)
    {
      Slot& slot = _channel[channel];
      int minPulse = 800;
      int maxPulse = 2200;
      if(_protocol == ESC_PROTOCOL_BRUSHED)
      {
        minPulse = 1000;
        maxPulse = 2000;
      }
      pulse = constrain(pulse, minPulse, maxPulse);
      int duration = map(pulse, 1000, 2000, slot.pulse_min, slot.pulse_max);
      
      int count = 2;
      if(_async)
      {
        int space = slot.pulse_space - duration;
        slot.setDuration(0, duration, 1);
        slot.setDuration(1, space, 0);
        slot.setTerminate(2);
        count = 3;
      }
      else
      {
        slot.setDuration(0, duration, 1);
        slot.setTerminate(1);
      }

      _rmt_fill_tx_items(_channel[channel].dev.channel, _channel[channel].items, count, 0);
    }

    void writeDshotCommand(uint8_t channel, int32_t pulse)
    {
      pulse = constrain(pulse, 0, 2000);
      // scale to dshot commands (0 or 48-2047)
      int value = pulse > 1000 ? PWM_TO_DSHOT(pulse) : 0;
      uint16_t frame = dshotEncode(value);

      Slot& slot = _channel[channel];
      for(size_t i = 0; i < DSHOT_BIT_COUNT; i++)
      {
        int val = (frame >> (DSHOT_BIT_COUNT - 1 - i)) & 0x01;
        slot.setDshotBit(i, val);
      }
      slot.setTerminate(DSHOT_BIT_COUNT);

      _rmt_fill_tx_items(slot.dev.channel, slot.items, ITEM_COUNT, 0);
    }

    void transmitCommand(uint8_t channel)
    {
      _rmt_tx_start(_channel[channel].dev.channel, true);
    }

    uint32_t getClockDivider() const
    {
      switch(_protocol)
      {
        case ESC_PROTOCOL_PWM: return 24;
        case ESC_PROTOCOL_ONESHOT125: return 4;
        case ESC_PROTOCOL_ONESHOT42: return 2;
        case ESC_PROTOCOL_MULTISHOT: return 2;
        case ESC_PROTOCOL_BRUSHED: return 8;
        default: return 1;
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
        //case ESC_PROTOCOL_DSHOT1200:
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
        case ESC_PROTOCOL_BRUSHED:    return getAnalogPulse(_interval * 1000);
        case ESC_PROTOCOL_DSHOT150:
        case ESC_PROTOCOL_DSHOT300:
        case ESC_PROTOCOL_DSHOT600:
        //case ESC_PROTOCOL_DSHOT1200:
        default:
          return 2048;
      }
    }

    uint32_t getPulseInterval() const
    {
      return getAnalogPulse(_interval * 1000);
    }

    uint32_t getAnalogPulse(int32_t ns) const
    {
      int div = getClockDivider();
      return ns / ((div * DURATION_CLOCK) / 2);
    }

    uint16_t getDshotPulse(uint16_t width) const
    {
      int div = getClockDivider();
      switch(_protocol)
      {
        case ESC_PROTOCOL_DSHOT150:  return (width / (div * DURATION_CLOCK / 2)) * 4;
        case ESC_PROTOCOL_DSHOT300:  return (width / (div * DURATION_CLOCK / 2)) * 2;
        case ESC_PROTOCOL_DSHOT600:  return (width / (div * DURATION_CLOCK / 2)) * 1;
        //case ESC_PROTOCOL_DSHOT1200: return (width / (div * DURATION_CLOCK / 2)) / 2;
        case ESC_PROTOCOL_BRUSHED:
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
        //case ESC_PROTOCOL_DSHOT1200:
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
