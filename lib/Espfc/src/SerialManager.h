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
#include "Debug_Espfc.h"

namespace {

#ifdef ESPFC_SERIAL_0
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_0_DEV_T> _uart0(ESPFC_SERIAL_0_DEV);
#endif

#ifdef ESPFC_SERIAL_1
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_1_DEV_T> _uart1(ESPFC_SERIAL_1_DEV);
#endif

#ifdef ESPFC_SERIAL_2
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_2_DEV_T> _uart2(ESPFC_SERIAL_2_DEV);
#endif

#ifdef ESPFC_SERIAL_USB
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_USB_DEV_T> _usb(ESPFC_SERIAL_USB_DEV);
#endif

#ifdef ESPFC_SERIAL_SOFT_0_RX
  static EspSoftSerial softSerial;
  static Espfc::Device::SerialDeviceAdapter<EspSoftSerial> _soft0(softSerial);
#endif

}

namespace Espfc {

class SerialManager
{
  public:
    SerialManager(Model& model): _model(model), _msp(model), _cli(model), _wireless(model), _telemetry(model), _current(SERIAL_UART_0) {}

    int begin()
    {
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
      _wireless.begin();
#endif

      for(int i = 0; i < SERIAL_UART_COUNT; i++)
      {
        Device::SerialDevice * port = getSerialPortById((SerialPort)i);
        if(!port)
        {
          //D("uart-no-port", i, (bool)port);
          continue;
        }

        const SerialPortConfig& spc = _model.config.serial[i];
        if(!spc.functionMask)
        {
          //D("uart-no-func", i, spc.id, spc.functionMask, spc.baud);
          continue;
        }

        SerialDeviceConfig sdc;
        sdc.baud = spc.baud;

#ifdef ESPFC_SERIAL_USB
        const bool hasUsbPort = true;
        const bool isUsbPort = i == SERIAL_USB;
#else
        const bool hasUsbPort = false;
        const bool isUsbPort = false;
#endif

#ifdef ESPFC_SERIAL_REMAP_PINS
        if(!isUsbPort)
        {
          const size_t pin_idx = 2 * (hasUsbPort ? i - 1 : i);
          sdc.tx_pin = _model.config.pin[pin_idx + PIN_SERIAL_0_TX];
          sdc.rx_pin = _model.config.pin[pin_idx + PIN_SERIAL_0_RX];
          if(sdc.tx_pin == -1 && sdc.rx_pin == -1)
          {
            //D("uart-no-pins", i, spc.id, spc.functionMask, spc.baud);
            continue;
          }
        }
#else
      (void)(isUsbPort && hasUsbPort);
#endif

        if(spc.functionMask & SERIAL_FUNCTION_RX_SERIAL)
        {
          switch(_model.config.input.serialRxProvider)
          {
            case SERIALRX_SBUS:
              sdc.baud = 100000ul;
              sdc.parity = SDC_SERIAL_PARITY_EVEN;
              sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
              sdc.inverted = true;
              break;
            case SERIALRX_CRSF:
              sdc.baud = 420000ul;
              //sdc.parity = SDC_SERIAL_PARITY_EVEN;
              //sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
              //sdc.inverted = true;
              break;
            default:
              break;
          }
        }
        else if(spc.functionMask & SERIAL_FUNCTION_BLACKBOX)
        {
          sdc.baud = spc.blackboxBaud;
          if(sdc.baud == 230400 || sdc.baud == 460800)
          {
            sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
          }
        }

        if(!sdc.baud)
        {
          //D("uart-no-baud", i, spc.id, spc.functionMask, spc.baud);
          continue;
        }

        if(!isUsbPort) {
          //D("uart-flush", i, spc.id, spc.functionMask, spc.baud);
          port->flush();
          delay(10);

          //D("uart-begin", i, spc.id, spc.functionMask, spc.baud, sdc.tx_pin, sdc.rx_pin);
          port->begin(sdc);
        }
        _model.state.serial[i].stream = port;

        if(i == ESPFC_SERIAL_DEBUG_PORT)
        {
          initDebugStream(port);
        }

        _model.logger.info().log(F("UART")).log(i).log(spc.id).log(spc.functionMask).log(sdc.baud).log(sdc.tx_pin).logln(sdc.rx_pin);
      }
      return 1;
    }

    int update()
    {
      //D("serial", _current);
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

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
      if(_current == SERIAL_SOFT_0)
      {
        _wireless.update();
      }
#endif

      next();

      return 1;
    }

    static Device::SerialDevice * getSerialPortById(SerialPort portId)
    {
      switch(portId)
      {
#ifdef ESPFC_SERIAL_0
        case SERIAL_UART_0: return &_uart0;
#endif
#ifdef ESPFC_SERIAL_1
        case SERIAL_UART_1: return &_uart1;
#endif
#ifdef ESPFC_SERIAL_2
        case SERIAL_UART_2: return &_uart2;
#endif
#ifdef ESPFC_SERIAL_USB
        case SERIAL_USB: return &_usb;
#endif
#ifdef ESPFC_SERIAL_SOFT_0_RX
        case SERIAL_SOFT_0: return &_soft0;
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