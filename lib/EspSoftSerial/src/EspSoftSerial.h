#ifndef _ESP_SOFT_SERIAL_H_
#define _ESP_SOFT_SERIAL_H_

#if defined(ESP8266)

#include <Arduino.h>
#include "Buffer.h"

class EspSoftSerialConfig
{
  public:
    EspSoftSerialConfig(): EspSoftSerialConfig(0, -1) {}
    EspSoftSerialConfig(uint32_t baud_, int8_t rx_pin_, int8_t parity_type_ = 0, int8_t stop_bits_ = 1, bool inverted_ = false, int8_t data_bits_ = 8):
      baud(baud_), rx_pin(rx_pin_), parity_type(parity_type_), stop_bits(stop_bits_), inverted(inverted_), data_bits(data_bits_) {
        data_bits = constrain(data_bits, 5, 8);
      }
    int32_t baud;
    int8_t rx_pin;
    int8_t parity_type;
    int8_t stop_bits;
    bool inverted;
    int8_t data_bits;
};

class EspSoftSerial: public Stream
{
  public:
    enum State
    {
      STATE_IDLE,
      STATE_DATA,
      STATE_PARITY,
      STATE_STOP,
    };

    EspSoftSerial(): _rx_state(STATE_IDLE), _rx_data(0) {}

    int begin(int baud);

    int begin(const EspSoftSerialConfig& conf);

    int available() override
    {
      return _rx_buff.count();
    }

    int read() override
    {
      return _rx_buff.get();
    }

    int peek() override
    {
      return _rx_buff.peek();
    }

    virtual size_t write(uint8_t c) override
    {
      return 0;
    }

    void flush() override
    {

    }

    size_t availableForWrite()
    {
      return 0;
    }

    static void rx_read_bit_isr() ICACHE_RAM_ATTR;
    static void rx_start_isr() ICACHE_RAM_ATTR;

  private:
    void rxReadBit() ICACHE_RAM_ATTR;
    void rxStart() ICACHE_RAM_ATTR;

    uint32_t nsToTicks(uint32_t ns);
    void timerInit();
    void timerStop();

    EspSoftSerialConfig _conf;
    uint32_t _delay;
    uint32_t _delay_start;

    State _rx_state;
    int32_t _rx_data_count;
    int32_t _rx_data;

    Buffer<int16_t, 64> _rx_buff;

    static EspSoftSerial * _instance;
};

#endif // ESP8266

#endif
