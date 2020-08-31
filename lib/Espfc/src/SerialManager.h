#ifndef _ESPFC_SERIAL_MANAGER_H_
#define _ESPFC_SERIAL_MANAGER_H_

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Device/SerialDeviceAdapter.h"
#include "EspSoftSerial.h"
#include "Msp/MspProcessor.h"
#include "Cli.h"
#include "Wireless.h"
#include "Telemetry.h"

namespace {
#if defined(ESP32)

  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SERIAL)
    static HardwareSerial Serial(0);
    static HardwareSerial Serial1(1);
    static HardwareSerial Serial2(2);
  #endif
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart0(Serial);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart2(Serial2);

#elif defined(ESP8266)

  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SERIAL)
    static HardwareSerial Serial(0);
  #endif
  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SERIAL1)
    static HardwareSerial Serial1(1);
  #endif
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart0(Serial);
  static Espfc::Device::SerialDeviceAdapter<HardwareSerial> uart1(Serial1);
  #if defined(USE_SOFT_SERIAL)
    static EspSoftSerial softSerial;
    static Espfc::Device::SerialDeviceAdapter<EspSoftSerial> soft0(softSerial);
  #endif

#else
  #error "unsupported platform"
#endif
}

namespace Espfc {

class SerialManager
{
  public:
    SerialManager(Model& model): _model(model), _msp(model), _cli(model), _wireless(model), _telemetry(model), _current(SERIAL_UART_0) {}

    int begin()
    {
      _wireless.begin();

      for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
      {
        Device::SerialDevice * port = getSerialPortById((SerialPort)i);
        if(!port) continue;

        const SerialPortConfig& spc = _model.config.serial[i];
        if(!spc.functionMask) continue;

        SerialDeviceConfig sdc;

#if defined(ESP32)
        sdc.tx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_TX];
        sdc.rx_pin = _model.config.pin[i * 2 + PIN_SERIAL_0_RX];
        if(sdc.tx_pin == -1 && sdc.rx_pin == -1) continue;
#endif
        sdc.baud = spc.baud;

        if(spc.functionMask & SERIAL_FUNCTION_RX_SERIAL)
        {
          switch(_model.config.input.serialRxProvider)
          {
            case SERIALRX_SBUS:
              sdc.baud = 100000;
              sdc.parity = SERIAL_PARITY_EVEN;
              sdc.stop_bits = SERIAL_STOP_BITS_2;
              sdc.inverted = true;
              break;
            default:
              break;
          }
        }
        else if(spc.functionMask & SERIAL_FUNCTION_BLACKBOX)
        {
          sdc.baud = spc.blackboxBaud;
          if(sdc.baud == 230400 || sdc.baud == 460800) {
            sdc.stop_bits = SERIAL_STOP_BITS_2;
          }
        }

        if(!sdc.baud) continue;

        port->flush();
        delay(10);
        port->begin(sdc);
        _model.state.serial[i].stream = port;

        if(i == SERIAL_UART_0)
        {
          LOG_SERIAL_INIT(port)
          //port->setDebugOutput(true);
        }
        _model.logger.info().log(F("UART")).log(i).log(spc.id).log(spc.functionMask).log(sdc.baud).log(sdc.tx_pin).logln(sdc.rx_pin);
      }
      return 1;
    }

    int update()
    {
      if(!_model.state.serialTimer.check()) return 0;

      SerialPortState& ss = _model.state.serial[_current];
      const SerialPortConfig& sc = _model.config.serial[_current];
      Stream * stream = ss.stream;

      {
        Stats::Measure measure(_model.state.stats, COUNTER_SERIAL);
        if(!stream || sc.functionMask & SERIAL_FUNCTION_RX_SERIAL) {
          next();
          return 0;
        }

        uint32_t now = millis();
        if(!ss.availableFrom && stream->available()) ss.availableFrom = now;
        bool timeout = ss.availableFrom && now - ss.availableFrom > 10;

        size_t count = 0;
        if(stream->available() > 3 || timeout)
        {
          ss.availableFrom = 0;
          while(stream->available())
          {
            char c = stream->read();
            if(sc.functionMask & SERIAL_FUNCTION_MSP)
            {
              bool consumed = _msp.process(c, ss.mspRequest, ss.mspResponse, *stream);
              if(!consumed)
              {
                _cli.process(c, ss.cliCmd, *stream);
              }
            }
            if(++count > 127) break;
          }
        }
      }
      if(sc.functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY && _model.state.telemetryTimer.check())
      {
        _telemetry.process(*stream);
      }

      if(_current == SERIAL_SOFT_0)
      {
        _wireless.update();
      }

      next();

      return 1;
    }

    static Device::SerialDevice * getSerialPortById(SerialPort portId)
    {
      switch(portId)
      {
        case SERIAL_UART_0: return &uart0;
        case SERIAL_UART_1: return &uart1;
#if defined(ESP32)
        case SERIAL_UART_2: return &uart2;
#elif defined(USE_SOFT_SERIAL)
        case SERIAL_SOFT_0: return &soft0;
#endif
        default: return nullptr;
      }
    }

  private:
    void next()
    {
      _current++;
      if(_current >= SERIAL_UART_COUNT) _current = 0;
    }

    Model& _model;
    Msp::MspProcessor _msp;
    Cli _cli;
    Wireless _wireless;
    Telemetry _telemetry;
    size_t _current;
};

}

#endif