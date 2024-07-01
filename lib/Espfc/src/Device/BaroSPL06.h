#ifndef _ESPFC_DEVICE_BARO_SPL06_H_
#define _ESPFC_DEVICE_BARO_SPL06_H_

#include "BusDevice.h"
#include "BaroDevice.h"
#include "Debug_Espfc.h"

#define SPL06_ADDRESS_FIRST          0x76
#define SPL06_ADDRESS_SECOND         0x77

#define SPL06_WHOAMI_ID              0x10
#define SPL06_WHOAMI_REG             0x0D

#define SPL06_VERSION_REG            0xD1
#define SPL06_RESET_REG              0x0C
#define SPL06_RESET_VAL              0x89

#define SPL06_CALIB_REG              0x10
#define SPL06_CALIB_REG_COEF_LEN     18

#define SPL06_PRESSURE_CFG_REG			(0x06)	/* Pressure configuration Reg */
#define SPL06_TEMPERATURE_CFG_REG		(0x07)	/* Temperature configuration Reg */
#define SPL06_MODE_CFG_REG				(0x08)  /* Mode and Status Configuration */
#define SPL06_INT_FIFO_CFG_REG			(0x09)	/* Interrupt and FIFO Configuration */
#define SPL06_INT_STATUS_REG			(0x0A)	/* Interrupt Status Reg */
#define SPL06_FIFO_STATUS_REG			(0x0B)	/* FIFO Status Reg */
#define SPL06_RST_REG					(0x0C)  /* Softreset Register */

#define SPL06_PRESSURE_REG           0x00
#define SPL06_TEMPERATURE_REG        0x03

#define SPL06_DATA_FRAME_SIZE			(6)

#define SPL06_CONTINUOUS_MODE			(0x07)

#define TEMPERATURE_INTERNAL_SENSOR		(0)
#define TEMPERATURE_EXTERNAL_SENSOR		(1)

//测量次数 times / S
#define SPL06_MWASURE_1					(0x00)
#define SPL06_MWASURE_2					(0x01)
#define SPL06_MWASURE_4					(0x02)
#define SPL06_MWASURE_8					(0x03)
#define SPL06_MWASURE_16				(0x04)
#define SPL06_MWASURE_32				(0x05)
#define SPL06_MWASURE_64				(0x06)
#define SPL06_MWASURE_128				(0x07)

//过采样率
#define SPL06_OVERSAMP_1				(0x00)
#define SPL06_OVERSAMP_2				(0x01)
#define SPL06_OVERSAMP_4				(0x02)
#define SPL06_OVERSAMP_8				(0x03)
#define SPL06_OVERSAMP_16				(0x04)
#define SPL06_OVERSAMP_32				(0x05)
#define SPL06_OVERSAMP_64				(0x06)
#define SPL06_OVERSAMP_128				(0x07)


#define P_MEASURE_RATE 			SPL06_MWASURE_16 	//每秒测量次数
#define P_OVERSAMP_RATE 		SPL06_OVERSAMP_64	//过采样率
#define SPL06_PRESSURE_CFG		(P_MEASURE_RATE<<4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE 			SPL06_MWASURE_16 	//每秒测量次数
#define T_OVERSAMP_RATE 		SPL06_OVERSAMP_8	//过采样率
#define SPL06_TEMPERATURE_CFG	(TEMPERATURE_EXTERNAL_SENSOR<<7 | T_MEASURE_RATE<<4 | T_OVERSAMP_RATE)

#define SPL06_MODE				(SPL06_CONTINUOUS_MODE)
#define  SPL06_TIME_MS	10
const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


namespace Espfc {

namespace Device {

class BaroSPL06: public BaroDevice
{
  public:
    struct CalibrationData {
      int16_t c0;
      int16_t c1;
      int32_t c00;
      int32_t c10;
      int16_t c01;
      int16_t c11;
      int16_t c20;
      int16_t c21;
      int16_t c30;
    } __attribute__ ((__packed__));

    int begin(BusDevice * bus) override
    {
      return begin(bus, SPL06_ADDRESS_FIRST) ? 1 : begin(bus, SPL06_ADDRESS_SECOND) ? 1 : 0;
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
    
      uint8_t buffer[SPL06_CALIB_REG_COEF_LEN] = {0};
      
      setBus(bus, addr);

      if(!testConnection()) return 0;

      writeReg(SPL06_RESET_REG, SPL06_RESET_VAL); // device reset
      delay(50);

      kt = 0;

      kp = 0;

      _bus->read(_addr, SPL06_CALIB_REG, SPL06_CALIB_REG_COEF_LEN, buffer); // read callibration
	
      _cal.c0 = (int16_t)buffer[0]<<4 | buffer[1]>>4;
      _cal.c0 = (_cal.c0 & 0x0800) ? (_cal.c0 | 0xF000) : _cal.c0;
    	
      _cal.c1 = (int16_t)(buffer[1] & 0x0F)<<8 | buffer[2];
      _cal.c1 = (_cal.c1 & 0x0800) ? (_cal.c1 | 0xF000) : _cal.c1;
    	
      _cal.c00 = (int32_t)buffer[3]<<12 | (int32_t)buffer[4]<<4 | (int32_t)buffer[5]>>4;
      _cal.c00 = (_cal.c00 & 0x080000) ? (_cal.c00 | 0xFFF00000) : _cal.c00;
    	
      _cal.c10 = (int32_t)(buffer[5] & 0x0F)<<16 | (int32_t)buffer[6]<<8 | (int32_t)buffer[7];
      _cal.c10 = (_cal.c10 & 0x080000) ? (_cal.c10 | 0xFFF00000) : _cal.c10;
    	
      _cal.c01 = (int16_t)buffer[8]<<8 | buffer[9];
      _cal.c11 = (int16_t)buffer[10]<<8 | buffer[11];
      _cal.c20 = (int16_t)buffer[12]<<8 | buffer[13];
      _cal.c21 = (int16_t)buffer[14]<<8 | buffer[15];
      _cal.c30 = (int16_t)buffer[16]<<8 | buffer[17];

      kp = scaleFactor[SPL06_OVERSAMP_64];
	  writeReg(SPL06_PRESSURE_CFG_REG, SPL06_MWASURE_16<<4 | SPL06_OVERSAMP_64);


      kt = scaleFactor[SPL06_OVERSAMP_64];
      writeReg(SPL06_TEMPERATURE_CFG_REG, SPL06_MWASURE_16<<4 | SPL06_OVERSAMP_64 | 0x80);//Using mems temperature
      writeReg(SPL06_INT_FIFO_CFG_REG, 0x04 | 0x08);

      writeReg(SPL06_MODE_CFG_REG, SPL06_MODE); // set sampling mode

      delay(20);

      return 1;
    }

    BaroDeviceType getType() const override
    {
      return BARO_SPL06;
    }

    virtual float readTemperature() override
    {
      float fTCompensate;
      float fTsc;

      fTsc = _raw_temp / (float)kt;
      fTCompensate =  _cal.c0 * 0.5 + _cal.c1 * fTsc;

      return fTCompensate;
    }

    virtual float readPressure() override
    {
      float fTsc, fPsc;
      float qua2, qua3;
      float fPCompensate;

      readMesurment();

      fTsc = _raw_temp / (float)kt;
      fPsc = _raw_pressure / (float)kp;
      qua2 = _cal.c10 + fPsc * (_cal.c20 + fPsc* _cal.c30);
      qua3 = fTsc * fPsc * (_cal.c11 + fPsc * _cal.c21);
	  //qua3 = 0.9f *fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);

      fPCompensate = _cal.c00 + fPsc * qua2 + fTsc * _cal.c01 + qua3;
	  //fPCompensate = spl06Calib.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06Calib.c01 + qua3;
      return fPCompensate;

      
    }

    void setMode(BaroDeviceMode mode)
    {
      (void)mode;
    }

    virtual int getDelay(BaroDeviceMode mode) const override
    {
      switch(mode)
      {
        case BARO_MODE_TEMP:
          return 0;
        default:
          //return 5500; // if sapling X1
          //return 7500; // if sampling X2
          return 11500;  // if sampling X4
      }
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->read(_addr, SPL06_WHOAMI_REG, 1, &whoami);
      return whoami == SPL06_WHOAMI_ID;
    }

  protected:
    void readMesurment()
    {
      uint8_t buffer[6] = {0, 0, 0, 0, 0, 0};
      _bus->readFast(_addr, SPL06_PRESSURE_REG, 6, buffer);

      _raw_pressure = buffer[2] | (buffer[1] << 8) | (buffer[0] << 16);
      _raw_pressure = (_raw_pressure & 0x800000) ? (0xFF000000 | _raw_pressure) : _raw_pressure;

      _raw_temp = buffer[5] | (buffer[4] << 8) | (buffer[3] << 16);
      _raw_temp = (_raw_temp & 0x800000) ? (0xFF000000 | _raw_temp) : _raw_temp;
    }

    int8_t writeReg(uint8_t reg, uint8_t val)
    {
      return _bus->write(_addr, reg, 1, &val);
    }

    int8_t _mode;
    int32_t _t_fine;
    int32_t _raw_temp;
    int32_t _raw_pressure;
    CalibrationData _cal;
    int32_t kp;
    int32_t kt;
};

}

}

#endif
