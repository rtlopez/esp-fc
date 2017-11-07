#ifndef _ESPFC_MSP_H_
#define _ESPFC_MSP_H_

#include <stdlib.h>
#include <stdint.h>
#include <cstring>
#include "Arduino.h"
#include "msp/msp_protocol.h"
#include "platform.h"

static const char * flightControllerIdentifier = BETAFLIGHT_IDENTIFIER;
static const char * boardIdentifier = "ESPF";

#include "Model.h"

namespace Espfc {

static const size_t MSP_BUF_SIZE = 192;

class Msp
{
  public:
    enum MspState
    {
      STATE_IDLE,
      STATE_HEADER_START,
      STATE_HEADER_M,
      STATE_HEADER_ARROW,
      STATE_HEADER_SIZE,
      STATE_HEADER_CMD,
      STATE_RECEIVED
    };

    enum MspType
    {
      TYPE_CMD,
      TYPE_REPLY
    };

    class MspMessage
    {
      public:
        MspMessage(): expected(0), received(0), read(0) {}
        MspState state;
        MspType dir;
        uint8_t cmd;
        uint8_t expected;
        uint8_t received;
        uint8_t read;
        uint8_t buffer[MSP_BUF_SIZE];
        uint8_t checksum;

        int remain()
        {
          return received - read;
        }

        void advance(size_t size)
        {
          read += size;
        }

        uint8_t readU8()
        {
          return buffer[read++];
        }

        uint16_t readU16()
        {
          uint16_t ret;
          ret = readU8();
          ret |= readU8() << 8;
          return ret;
        }

        uint32_t readU32()
        {
          uint32_t ret;
          ret = readU8();
          ret |= readU8() <<  8;
          ret |= readU8() << 16;
          ret |= readU8() << 24;
          return ret;
        }
    };

    class MspResponse {
      public:
        MspResponse(): len(0) {}
        uint8_t cmd;
        int8_t  result;
        uint8_t direction;
        uint8_t len;
        uint8_t data[MSP_BUF_SIZE];

        void writeData(const char * v, int size)
        {
          while(size-- > 0) writeU8(*v++);
        }

        void writeString(const char * v)
        {
          while(*v) writeU8(*v++);
        }

        void writeString(const __FlashStringHelper *ifsh)
        {
          PGM_P p = reinterpret_cast<PGM_P>(ifsh);
          while(true)
          {
            uint8_t c = pgm_read_byte(p++);
            if (c == 0) break;
            writeU8(c);
          }
        }

        void writeU8(uint8_t v)
        {
          data[len++] = v;
        }

        void writeU16(uint16_t v)
        {
          writeU8(v >> 0);
          writeU8(v >> 8);
        }

        void writeU32(uint32_t v)
        {
          writeU8(v >> 0);
          writeU8(v >> 8);
          writeU8(v >> 16);
          writeU8(v >> 24);
        }
    };


    Msp(Model& model): _model(model) {}

    bool process(char c, Stream& s)
    {
      switch(_msg.state)
      {
        case STATE_IDLE:               // sync char 1 '$'
          if(c == '$') _msg.state = STATE_HEADER_START;
          break;

        case STATE_HEADER_START:       // sync char 2 'M'
            if(c == 'M') _msg.state = STATE_HEADER_M;
            else _msg.state = STATE_IDLE;
            break;

        case STATE_HEADER_M:               // direction (should be >)
          switch(c)
          {
            case '>':
              _msg.dir = TYPE_REPLY;
              _msg.state = STATE_HEADER_ARROW;
              break;
            case '<':
              _msg.dir = TYPE_CMD;
              _msg.state = STATE_HEADER_ARROW;
              break;
            default:
              _msg.state = STATE_IDLE;
          }
          break;
        case STATE_HEADER_ARROW:
          if (c > MSP_BUF_SIZE) _msg.state = STATE_IDLE;
          else
          {
            _msg.expected = c;
            _msg.received = 0;
            _msg.read = 0;
            _msg.checksum = 0;
            _msg.checksum ^= c;
            _msg.state = STATE_HEADER_SIZE;
          }
          break;

        case STATE_HEADER_SIZE:
          _msg.cmd = c;
          _msg.checksum ^= c;
          _msg.state = STATE_HEADER_CMD;
          break;

        case STATE_HEADER_CMD:
          if(_msg.received < _msg.expected)
          {
            _msg.checksum ^= c;
            _msg.buffer[_msg.received++] = c;
          }
          else if(_msg.received >= _msg.expected)
          {
            _msg.state = _msg.checksum == c ? STATE_RECEIVED : STATE_IDLE;
          }

        default:
          break;
      }

      if(_msg.state == STATE_RECEIVED)
      {
        _msg.state = STATE_IDLE;
        debugMessage(_msg);
        switch(_msg.dir)
        {
          case TYPE_CMD:
            processCommand(_msg, s);
            break;
          case TYPE_REPLY:
            //processCommand(_msg, s);
            break;
        }
        return true;
      }

      return _msg.state != STATE_IDLE;
    }

    void debugMessage(const MspMessage& m)
    {
      if(debugSkip(m.cmd)) return;

      Serial1.print(m.dir == TYPE_REPLY ? '>' : '<'); Serial1.print(' ');
      Serial1.print(m.cmd); Serial1.print(' ');
      Serial1.print(m.expected); Serial1.print(' ');
      for(size_t i = 0; i < m.expected; i++)
      {
        Serial1.print(m.buffer[i]); Serial1.print(' ');
      }
      Serial1.println();
    }

    void debugResponse(const MspResponse& r)
    {
      if(debugSkip(r.cmd)) return;

      Serial1.print(r.result == 1 ? '>' : (r.result == -1 ? '!' : '@')); Serial1.print(' ');
      Serial1.print(r.cmd); Serial1.print(' ');
      Serial1.print(r.len); Serial1.print(' ');
      for(size_t i = 0; i < r.len; i++)
      {
        Serial1.print(r.data[i]); Serial1.print(' ');
      }
      Serial1.println();
    }

    void processCommand(MspMessage& m, Stream& s)
    {
      MspResponse r;
      r.cmd = m.cmd;
      r.result = 1;
      switch(m.cmd)
      {
        case MSP_API_VERSION:
          r.writeU8(MSP_PROTOCOL_VERSION);
          r.writeU8(API_VERSION_MAJOR);
          r.writeU8(API_VERSION_MINOR);
          break;

        case MSP_FC_VARIANT:
          r.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
          break;

        case MSP_FC_VERSION:
          r.writeU8(FC_VERSION_MAJOR);
          r.writeU8(FC_VERSION_MINOR);
          r.writeU8(FC_VERSION_PATCH_LEVEL);
          break;

        case MSP_BOARD_INFO:
          r.writeData(boardIdentifier, BOARD_IDENTIFIER_LENGTH);
          r.writeU16(0); // No other build targets currently have hardware revision detection.
          r.writeU8(0);  // 0 == FC
          break;

        case MSP_BUILD_INFO:
          r.writeData(buildDate, BUILD_DATE_LENGTH);
          r.writeData(buildTime, BUILD_TIME_LENGTH);
          r.writeData(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
          break;

        case MSP_UID:
          {
            r.writeU32(ESP.getChipId());
            r.writeU32(ESP.getFlashChipId());
            r.writeU32(ESP.getFlashChipSize());
          }
          break;

        case MSP_STATUS_EX:
        case MSP_STATUS:
          r.writeU16(_model.state.loopTimer.interval);
          r.writeU16(0); // i2c error count
          //         acc,     baro,    mag,     gps,     sonar,   gyro
          r.writeU16(_model.accelActive() << 0 | 0 << 1 | _model.magActive() << 2 | 0 << 3 | 0 << 4 | 1 << 5);
          r.writeU32(_model.state.modeMask); // flight mode flags
          r.writeU8(0); // pid profile
          r.writeU16(lrintf(_model.state.stats.getTotalLoad(_model.state.gyroTimer.interval)));
          if (m.cmd == MSP_STATUS_EX) {
            r.writeU8(1);
            r.writeU8(0);
          } else {  // MSP_STATUS
            r.writeU16(_model.state.gyroTimer.interval); // gyro cycle time
          }

          // flight mode flags (above 32 bits)
          r.writeU8(0); // count

          // Write arming disable flags
          r.writeU8(16);  // 1 byte, flag count
          r.writeU32(0);  // 4 bytes, flags
          break;

        case MSP_NAME:
          r.writeString(_model.config.modelName);
          break;

        case MSP_SET_NAME:
          std::memset(&_model.config.modelName, 0, MODEL_NAME_LEN + 1);
          for(size_t i = 0; i < std::min((size_t)m.received, MODEL_NAME_LEN); i++)
          {
            _model.config.modelName[i] = m.readU8();
          }
          break;

        case MSP_BOXNAMES:
          r.writeString(F("ARM;ANGLE;AIRMODE;BUZZER;FAILSAFE;"));
          break;

        case MSP_BOXIDS:
          r.writeU8(MODE_ARMED);
          r.writeU8(MODE_ANGLE);
          r.writeU8(MODE_AIRMODE);
          r.writeU8(MODE_BUZZER);
          r.writeU8(MODE_FAILSAFE);
          break;

        case MSP_MODE_RANGES:
          for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
          {
            r.writeU8(_model.config.conditions[i].id);
            r.writeU8(_model.config.conditions[i].ch);
            r.writeU8(_model.config.conditions[i].min);
            r.writeU8(_model.config.conditions[i].max);
          }
          break;

        case MSP_SET_MODE_RANGE:
          {
            size_t i = m.readU8();
            if(i < ACTUATOR_CONDITIONS)
            {
              _model.config.conditions[i].id = m.readU8();
              _model.config.conditions[i].ch = m.readU8();
              _model.config.conditions[i].min = m.readU8();
              _model.config.conditions[i].max = m.readU8();
            }
            else
            {
              r.result = -1;
            }
          }
          break;

        case MSP_ANALOG:
          r.writeU8(_model.state.battery.voltage);  // voltage
          r.writeU16(0); // mah drawn
          r.writeU16(0); // rssi
          r.writeU16(0); // amperage
          break;

        case MSP_FEATURE_CONFIG:
          r.writeU32(_model.config.featureMask);
          break;

        case MSP_SET_FEATURE_CONFIG:
          _model.config.featureMask = m.readU32();
          _model.update();
          break;

        case MSP_BATTERY_CONFIG:
          r.writeU8(34);  // vbatmincellvoltage
          r.writeU8(42);  // vbatmaxcellvoltage
          r.writeU8(_model.config.vbatCellWarning);  // vbatwarningcellvoltage
          r.writeU16(0); // batteryCapacity
          r.writeU8(1);  // voltageMeterSource
          r.writeU8(0);  // currentMeterSource
          break;

        case MSP_SET_BATTERY_CONFIG:
          m.readU8();  // vbatmincellvoltage
          m.readU8();  // vbatmaxcellvoltage
          _model.config.vbatCellWarning = m.readU8();  // vbatwarningcellvoltage
          m.readU16(); // batteryCapacity
          m.readU8();  // voltageMeterSource
          m.readU8();  // currentMeterSource
          break;

        case MSP_BATTERY_STATE:
          // battery characteristics
          r.writeU8(_model.state.battery.cells); // cell count, 0 indicates battery not detected.
          r.writeU16(0); // capacity in mAh

          // battery state
          r.writeU8(_model.state.battery.voltage); // in 0.1V steps
          r.writeU16(0);  // milliamp hours drawn from battery
          r.writeU16(0); // send current in 0.01 A steps, range is -320A to 320A

          // battery alerts
          r.writeU8(0);
          break;

        case MSP_VOLTAGE_METERS:
          for(int i = 0; i < 1; i++)
          {
            r.writeU8(i + 10);  // meter id (10-19 vbat adc)
            r.writeU8(_model.state.battery.voltage);  // meter value
          }
          break;

        case MSP_CURRENT_METERS:
          break;

        case MSP_VOLTAGE_METER_CONFIG:
          r.writeU8(1); // num voltage sensors
          for(int i = 0; i < 1; i++)
          {
            r.writeU8(5); // frame size
            r.writeU8(i + 10); // id (10-19 vbat adc)
            r.writeU8(0); // type resistor divider
            r.writeU8(_model.config.vbatScale); // scale
            r.writeU8(_model.config.vbatResDiv);  // resdivval
            r.writeU8(_model.config.vbatResMult);  // resdivmultiplier
          }
          break;

        case MSP_SET_VOLTAGE_METER_CONFIG:
          {
            int id = m.readU8();
            if(id == 10 + 0) // id (10-19 vbat adc, allow only 10)
            {
              _model.config.vbatScale = m.readU8();
              _model.config.vbatResDiv = m.readU8();
              _model.config.vbatResMult = m.readU8();
            }
          }
          break;

        case MSP_DATAFLASH_SUMMARY:
          r.writeU8(0); // FlashFS is neither ready nor supported
          r.writeU32(0);
          r.writeU32(0);
          r.writeU32(0);
          break;

        case MSP_ACC_TRIM:
          r.writeU16(0); // pitch
          r.writeU16(0); // roll
          break;

        case MSP_MIXER_CONFIG:
          r.writeU8(3); // mixerMode, QUAD_X
          r.writeU8(_model.config.yawReverse); // yaw_motors_reversed
          break;

        case MSP_SET_MIXER_CONFIG:
          m.readU8(); // mixerMode, QUAD_X
          _model.config.yawReverse = m.readU8(); // yaw_motors_reversed
          break;

        case MSP_SENSOR_CONFIG:
          r.writeU8(_model.config.accelDev); // 3 acc mpu6050
          r.writeU8(_model.config.baroDev);  // 2 baro bmp085
          r.writeU8(_model.config.magDev);   // 3 mag hmc5883l
          break;

        case MSP_SET_SENSOR_CONFIG:
          _model.config.accelDev = m.readU8(); // 3 acc mpu6050
          _model.config.baroDev = m.readU8();  // 2 baro bmp085
          _model.config.magDev = m.readU8();   // 3 mag hmc5883l
          _model.update();
          break;

        case MSP_SENSOR_ALIGNMENT:
          r.writeU8(_model.config.gyroAlign); // gyro align
          r.writeU8(_model.config.accelAlign); // acc align
          r.writeU8(_model.config.magAlign); // mag align
          break;

        case MSP_SET_SENSOR_ALIGNMENT:
          _model.config.gyroAlign = m.readU8(); // gyro align
          _model.config.accelAlign = m.readU8(); // acc align
          _model.config.magAlign = m.readU8(); // mag align
          break;

        case MSP_CF_SERIAL_CONFIG:
          for(int i = SERIAL_UART_0; i < SERIAL_UART_COUNT; i++)
          {
            r.writeU8(_model.config.serial[i].id); // identifier
            r.writeU16(_model.config.serial[i].functionMask); // functionMask
            r.writeU8(_model.config.serial[i].baudIndex); // msp_baudrateIndex
            r.writeU8(0); // gps_baudrateIndex
            r.writeU8(0); // telemetry_baudrateIndex
            r.writeU8(_model.config.serial[i].blackboxBaudIndex); // blackbox_baudrateIndex
          }
          break;

        case MSP_SET_CF_SERIAL_CONFIG:
          {
            const int packetSize = 1 + 2 + 4;
            while(m.remain() >= packetSize)
            {
              int id = m.readU8();
              if(id != SERIAL_UART_0 && id != SERIAL_UART_1)
              {
                m.advance(packetSize - 1);
                continue;
              }
              _model.config.serial[id].id = id;
              _model.config.serial[id].functionMask = m.readU16();
              _model.config.serial[id].baudIndex = m.readU8();
              m.readU8();
              m.readU8();
              _model.config.serial[id].blackboxBaudIndex = m.readU8();
            }
          }
          _model.update();
          break;

        case MSP_BLACKBOX_CONFIG:
          r.writeU8(1); // Blackbox supported
          r.writeU8(_model.config.blackboxDev); // device serial or none
          r.writeU8(1); // blackboxGetRateNum());
          r.writeU8(1); // blackboxGetRateDenom());
          r.writeU16(_model.config.blackboxPdenom); // p_denom
          break;

        case MSP_SET_BLACKBOX_CONFIG:
          // Don't allow config to be updated while Blackbox is logging
          if (true) {
            _model.config.blackboxDev = m.readU8();
            const int rateNum = m.readU8(); // was rate_num
            const int rateDenom = m.readU8(); // was rate_denom
            if (m.remain() >= 2) {
                _model.config.blackboxPdenom = m.readU16(); // p_denom specified, so use it directly
            } else {
                // p_denom not specified in MSP, so calculate it from old rateNum and rateDenom
                //p_denom = blackboxCalculatePDenom(rateNum, rateDenom);
                (void)(rateNum + rateDenom);
            }
          }
          break;

        case MSP_ATTITUDE:
          r.writeU16(lrintf(degrees(_model.state.angle.x) * 10)); // roll
          r.writeU16(lrintf(degrees(_model.state.angle.y) * 10)); // pitch
          r.writeU16(lrintf(degrees(-_model.state.angle.z))); // yaw
          break;

        case MSP_BEEPER_CONFIG:
          r.writeU32(~_model.config.buzzer.beeperMask); // beeper mask
          break;

        case MSP_SET_BEEPER_CONFIG:
          _model.config.buzzer.beeperMask = ~m.readU32(); // beeper mask
          break;

        case MSP_BOARD_ALIGNMENT_CONFIG:
          r.writeU16(0); // roll
          r.writeU16(0); // pitch
          r.writeU16(0); // yaw
          break;

        case MSP_RX_MAP:
          for(size_t i = 0; i < INPUT_CHANNELS; i++)
          {
            r.writeU8(_model.config.inputMap[i]);
          }
          break;

        case MSP_RSSI_CONFIG:
          r.writeU8(0);
          break;

        case MSP_MOTOR_CONFIG:
          r.writeU16(_model.config.outputMinThrottle); // minthrottle
          r.writeU16(_model.config.outputMaxThrottle); // maxthrottle
          r.writeU16(_model.config.outputMinCommand); // mincommand
          break;

        case MSP_SET_MOTOR_CONFIG:
          _model.config.outputMinThrottle = m.readU16(); // minthrottle
          _model.config.outputMaxThrottle = m.readU16(); // maxthrottle
          _model.config.outputMinCommand = m.readU16(); // mincommand
          _model.update();
          break;

        case MSP_MOTOR_3D_CONFIG:
          r.writeU16(1406); // deadband3d_low;
          r.writeU16(1514); // deadband3d_high;
          r.writeU16(1460); // neutral3d;
          break;

        case MSP_ARMING_CONFIG:
          r.writeU8(5); // auto_disarm delay
          r.writeU8(0);  // disarm kill switch
          r.writeU8(180); // small angle
          break;

        case MSP_RC_DEADBAND:
          r.writeU8(_model.config.inputDeadband);
          r.writeU8(0); // yaw deadband
          r.writeU8(0); // alt hod deadband
          r.writeU16(0); // deadband 3d throttle
          break;

        case MSP_SET_RC_DEADBAND:
          _model.config.inputDeadband = m.readU8();
          m.readU8(); // yaw deadband
          m.readU8(); // alt hod deadband
          m.readU16(); // deadband 3d throttle
          break;

        case MSP_RX_CONFIG:
          r.writeU8(0); // serialrx_provider
          r.writeU16(_model.config.inputMaxCheck); //maxcheck
          r.writeU16(_model.config.inputMidRc); //midrc
          r.writeU16(_model.config.inputMinCheck); //mincheck
          r.writeU8(0); // spectrum bind
          r.writeU16(_model.config.inputMinRc); //min_us
          r.writeU16(_model.config.inputMaxRc); //max_us
          r.writeU8(_model.config.inputInterpolation); // rc interpolation
          r.writeU8(_model.config.inputInterpolationInterval); // rc interpolation interval
          r.writeU16(1500); // airmode activate threshold
          r.writeU8(0); // rx spi prot
          r.writeU32(0); // rx spi id
          r.writeU8(0); // rx spi chan count
          r.writeU8(0); // fpv camera angle
          break;

        case MSP_SET_RX_CONFIG:
          m.readU8(); // serialrx_provider
          _model.config.inputMaxCheck = m.readU16(); //maxcheck
          _model.config.inputMidRc = m.readU16(); //midrc
          _model.config.inputMinCheck = m.readU16(); //mincheck
          m.readU8(); // spectrum bind
          _model.config.inputMinRc = m.readU16(); //min_us
          _model.config.inputMaxRc = m.readU16(); //max_us
          _model.config.inputInterpolation = m.readU8(); // rc interpolation
          _model.config.inputInterpolationInterval = m.readU8(); // rc interpolation interval
          m.readU16(); // airmode activate threshold
          m.readU8(); // rx spi prot
          m.readU32(); // rx spi id
          m.readU8(); // rx spi chan count
          m.readU8(); // fpv camera angle
          _model.update();
          break;

        case MSP_FAILSAFE_CONFIG:
          r.writeU8(0); // failsafe_delay
          r.writeU8(0); // failsafe_off_delay
          r.writeU16(1000); //failsafe_throttle
          r.writeU8(0); // failsafe_kill_switch
          r.writeU16(0); // failsafe_throttle_low_delay
          r.writeU8(1); //failsafe_procedure; default drop
          break;

        case MSP_SET_FAILSAFE_CONFIG:
          m.readU8(); //failsafe_delay
          m.readU8(); //failsafe_off_delay
          m.readU16(); //failsafe_throttle
          m.readU8(); //failsafe_kill_switch
          m.readU16(); //failsafe_throttle_low_delay
          m.readU8(); //failsafe_procedure
          break;

        case MSP_RXFAIL_CONFIG:
          for (size_t i = 0; i < INPUT_CHANNELS; i++)
          {
            r.writeU8(_model.config.failsafeMode[i]);
            r.writeU16(_model.config.failsafeValue[i]);
          }
          break;

        case MSP_SET_RXFAIL_CONFIG:
          {
            size_t i = m.readU8();
            if (i < INPUT_CHANNELS) {
              _model.config.failsafeMode[i] = m.readU8(); // mode
              _model.config.failsafeValue[i] = m.readU16(); // pulse
            } else {
              r.result = -1;
            }
          }
          break;

        case MSP_RC:
          for(size_t i = 0; i < INPUT_CHANNELS; i++)
          {
            r.writeU16(_model.state.inputUs[i]);
          }
          break;

        case MSP_RC_TUNING:
          r.writeU8(_model.config.inputRate[0]);
          r.writeU8(_model.config.inputExpo[0]);
          for(size_t i = 0; i < 3; i++)
          {
            r.writeU8(_model.config.inputSuperRate[i]);
          }
          r.writeU8(0); // dyn thr pid
          r.writeU8(50); // thrMid8
          r.writeU8(0);  // thr expo
          r.writeU16(1650); // tpa breakpoint
          r.writeU8(_model.config.inputExpo[2]); // yaw expo
          r.writeU8(_model.config.inputRate[2]); // yaw rate
          break;

        case MSP_SET_RC_TUNING:
          if(m.remain() >= 10)
          {
            _model.config.inputRate[0] = _model.config.inputRate[1] = m.readU8();
            _model.config.inputExpo[0] = _model.config.inputExpo[1] = m.readU8();
            for(size_t i = 0; i < 3; i++)
            {
              _model.config.inputSuperRate[i] = m.readU8();
            }
            m.readU8(); // dyn thr pid
            m.readU8(); // thrMid8
            m.readU8();  // thr expo
            m.readU16(); // tpa breakpoint
            if(m.remain() >= 1)
            {
              _model.config.inputExpo[2] = m.readU8(); // yaw expo
            }
            if(m.remain() >= 1)
            {
              _model.config.inputRate[2]  = m.readU8(); // yaw rate
            }
          }
          else
          {
            r.result = -1;
            // error
          }
          break;

        case MSP_ADVANCED_CONFIG:
          r.writeU8(_model.config.gyroSync);
          r.writeU8(_model.config.loopSync);
          r.writeU8(_model.config.outputAsync);
          r.writeU8(_model.config.outputProtocol);
          r.writeU16(_model.config.outputRate);
          r.writeU16(450); // dshot idle
          r.writeU8(0); // 32k gyro
          r.writeU8(0); // PWM inversion
          break;

        case MSP_SET_ADVANCED_CONFIG:
          _model.config.gyroSync = m.readU8();
          _model.config.loopSync = m.readU8();
          _model.config.outputAsync = m.readU8();
          _model.config.outputProtocol = m.readU8();
          _model.config.outputRate = m.readU16();
          m.readU16(); // dshot idle
          m.readU8();  // 32k gyro
          m.readU8();  // PWM inversion
          _model.update();
          break;

        case MSP_GPS_CONFIG:
          r.writeU8(0); // provider
          r.writeU8(0); // sbasMode
          r.writeU8(0); // autoConfig
          r.writeU8(0); // autoBaud
          break;

        case MSP_COMPASS_CONFIG:
          r.writeU16(0); // mag_declination * 10
          break;

        case MSP_FILTER_CONFIG:
          r.writeU8(_model.config.gyroFilter.freq); // gyro lpf
          r.writeU16(_model.config.dtermFilter.freq); // dterm lpf
          r.writeU16(_model.config.yawFilter.freq);  // yaw lpf
          r.writeU16(_model.config.gyroNotch1Filter.freq);  // gyro notch 1 hz
          r.writeU16(_model.config.gyroNotch1Filter.cutoff);  // gyro notch 1 cutoff
          r.writeU16(_model.config.dtermNotchFilter.freq);  // dterm notch hz
          r.writeU16(_model.config.dtermNotchFilter.cutoff);  // dterm notch cutoff
          r.writeU16(_model.config.gyroNotch2Filter.freq);  // gyro notch 2 hz
          r.writeU16(_model.config.gyroNotch2Filter.cutoff);  // gyro notch 2 cutoff
          r.writeU8(_model.config.dtermFilter.type);
          break;

        case MSP_SET_FILTER_CONFIG:
          _model.config.gyroFilter.freq = m.readU8();
          _model.config.dtermFilter.freq = m.readU16();
          _model.config.yawFilter.freq = m.readU16();
          if (m.remain() >= 8) {
              _model.config.gyroNotch1Filter.freq = m.readU16();
              _model.config.gyroNotch1Filter.cutoff = m.readU16();
              _model.config.dtermNotchFilter.freq = m.readU16();
              _model.config.dtermNotchFilter.cutoff = m.readU16();
          }
          if (m.remain() >= 4) {
              _model.config.gyroNotch2Filter.freq = m.readU16();
              _model.config.gyroNotch2Filter.cutoff = m.readU16();
          }
          if (m.remain() >= 1) {
              _model.config.dtermFilter.type = (FilterType)m.readU8();
          }
          _model.update();
          break;

        case MSP_PID_CONTROLLER:
          r.writeU8(1); // betaflight controller id
          break;

        case MSP_PIDNAMES:
          r.writeString(F("ROLL;PITCH;YAW;ALT;Pos;PosR;NavR;LEVEL;MAG;VEL;"));
          break;

        case MSP_PID:
          for(size_t i = 0; i < PID_ITEM_COUNT; i++)
          {
            r.writeU8(_model.config.pid[i].P);
            r.writeU8(_model.config.pid[i].I);
            r.writeU8(_model.config.pid[i].D);
          }
          break;

        case MSP_SET_PID:
          for (int i = 0; i < PID_ITEM_COUNT; i++)
          {
            _model.config.pid[i].P = m.readU8();
            _model.config.pid[i].I = m.readU8();
            _model.config.pid[i].D = m.readU8();
          }
          _model.update();
          break;

        case MSP_PID_ADVANCED:
          r.writeU16(0);
          r.writeU16(0);
          r.writeU16(0); // was pidProfile.yaw_p_limit
          r.writeU8(0); // reserved
          r.writeU8(0); //vbatPidCompensation;
          r.writeU8(0); //setpointRelaxRatio;
          r.writeU8(_model.config.dtermSetpointWeight);
          r.writeU8(0); // reserved
          r.writeU8(0); // reserved
          r.writeU8(0); // reserved
          r.writeU16(0); //rateAccelLimit;
          r.writeU16(0); //yawRateAccelLimit;
          r.writeU8(_model.config.angleLimit); // levelAngleLimit;
          r.writeU8(0); // was pidProfile.levelSensitivity
          r.writeU16(0); //itermThrottleThreshold;
          r.writeU16(0); //itermAcceleratorGain;
          break;

        case MSP_SET_PID_ADVANCED:
          m.readU16();
          m.readU16();
          m.readU16(); // was pidProfile.yaw_p_limit
          m.readU8(); // reserved
          m.readU8();
          m.readU8();
          _model.config.dtermSetpointWeight = m.readU8();
          m.readU8(); // reserved
          m.readU8(); // reserved
          m.readU8(); // reserved
          m.readU16();
          m.readU16();
          if (m.remain() >= 2) {
              _model.config.angleLimit = m.readU8();
              m.readU8(); // was pidProfile.levelSensitivity
          }
          if (m.remain() >= 4) {
              m.readU16();
              m.readU16();
          }
          _model.update();
          break;

        case MSP_RAW_IMU:
          for (int i = 0; i < 3; i++)
          {
            r.writeU16(lrintf(_model.state.accel[i] * 500));
          }
          for (int i = 0; i < 3; i++)
          {
            r.writeU16(lrintf(degrees(_model.state.gyro[i])));
          }
          for (int i = 0; i < 3; i++)
          {
            r.writeU16(lrintf(_model.state.magRaw[i]));
          }
          break;

        case MSP_MOTOR:
          for (size_t i = 0; i < OUTPUT_CHANNELS; i++)
          {
            if (i >= OUTPUT_CHANNELS || _model.config.outputPin[i] == -1)
            {
              r.writeU16(0);
              continue;
            }
            r.writeU16(_model.state.outputUs[i]);
          }
          break;

        case MSP_SET_MOTOR:
          for (size_t i = 0; i < OUTPUT_CHANNELS; i++)
          {
            _model.state.outputDisarmed[i] = m.readU16();
          }
          break;

        case MSP_SERVO:
          for(size_t i = 0; i < 8; i++)
          {
            r.writeU16(1500);
          }
          break;

        case MSP_ACC_CALIBRATION:
          if(!_model.isActive(MODE_ARMED)) _model.calibrate();
          break;

        case MSP_MAG_CALIBRATION:
          break;

        case MSP_DEBUG:
          for (int i = 0; i < 4; i++) {
            r.writeU16(_model.state.debug[i]);
          }
          break;

        case MSP_EEPROM_WRITE:
          _model.save();
          break;

        case MSP_RESET_CONF:
          if(!_model.isActive(MODE_ARMED))
          {
            _model.reset();
            _model.load();
            _model.update();
          }
          break;

        case MSP_REBOOT:
          _reboot = true;
          break;

        default:
          r.result = 0;
          break;
      }
      sendResponse(r, s);
    }

    bool debugSkip(uint8_t cmd)
    {
      return true;
      if(cmd == MSP_STATUS) return true;
      if(cmd == MSP_STATUS_EX) return true;
      if(cmd == MSP_BOXNAMES) return true;
      if(cmd == MSP_ANALOG) return true;
      if(cmd == MSP_ATTITUDE) return true;
      if(cmd == MSP_RC) return true;
      if(cmd == MSP_RAW_IMU) return true;
      if(cmd == MSP_MOTOR) return true;
      if(cmd == MSP_SERVO) return true;
      if(cmd == MSP_BATTERY_STATE) return true;
      if(cmd == MSP_VOLTAGE_METERS) return true;
      if(cmd == MSP_CURRENT_METERS) return true;
      return false;
    }

    void sendResponse(MspResponse& r, Stream& s)
    {
      debugResponse(r);

      uint8_t hdr[5] = { '$', 'M', '>' };
      if(r.result == -1)
      {
        hdr[2] = '!'; // error ??
      }
      hdr[3] = r.len;
      hdr[4] = r.cmd;
      uint8_t checksum = crc(0, &hdr[3], 2);
      s.write(hdr, 5);
      if(r.len > 0)
      {
        s.write(r.data, r.len);
        checksum = crc(checksum, r.data, r.len);
      }
      s.write(checksum);
      postCommand();
    }

    void postCommand()
    {
      if(_reboot) ESP.restart();
    }

    uint8_t crc(uint8_t checksum, const uint8_t *data, int len)
    {
      while (len-- > 0)
      {
        checksum ^= *data++;
      }
      return checksum;
    }

  private:
    Model& _model;
    MspMessage _msg;
    bool _reboot;
};

}

#endif
