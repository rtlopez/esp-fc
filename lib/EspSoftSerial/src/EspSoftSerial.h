#ifndef _ESP_SOFT_SERIAL_H_
#define _ESP_SOFT_SERIAL_H_

#include "Arduino.h"
#include "EspGpio.h"
#include "Queue.h"

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

class EspSoftSerial
{
  public:
    enum State
    {
      STATE_IDLE,
      STATE_START,
      STATE_DATA,
      STATE_PARITY,
      STATE_STOP
    };

    EspSoftSerial(): _rx_state(STATE_IDLE), _rx_buff(128) {}

    int begin(int baud);

    int begin(const EspSoftSerialConfig& conf);

    size_t available()
    {
      return _rx_buff.count();
    }

    char read()
    {
      return _rx_buff.pop();
    }

    char peek()
    {
      return _rx_buff.peek();
    }

    virtual size_t write(uint8_t c)
    {
      return 0;
    }

    void flush()
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
    void timerWrite(uint32_t ticks) ICACHE_RAM_ATTR;

    EspSoftSerialConfig _conf;
    uint32_t _delay;
    uint32_t _delay_start;

    State _rx_state;
    uint8_t _rx_data;
    uint8_t _rx_data_count;

    Queue<char> _rx_buff;
    static EspSoftSerial * _instance;
};

#endif
