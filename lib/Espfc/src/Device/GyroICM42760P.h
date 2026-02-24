#ifndef _ESPFC_DEVICE_GYRO_ICM42670P_H_
#define _ESPFC_DEVICE_GYRO_ICM42670P_H_

#include "BusDevice.h"
#include "GyroDevice.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

// Register Map for ICM-42670-P
#define ICM42670P_REG_MCLK_RDY       0x00
#define ICM42670P_REG_DEVICE_CONFIG  0x11
#define ICM42670P_REG_PWR_MGMT0      0x1F
#define ICM42670P_REG_GYRO_CONFIG0   0x20
#define ICM42670P_REG_ACCEL_CONFIG0  0x21
#define ICM42670P_REG_WHO_AM_I       0x75
#define ICM42670P_REG_ACCEL_DATA_X1  0x0B
#define ICM42670P_REG_GYRO_DATA_X1   0x11

// Constants
#define ICM42670P_WHO_AM_I_CONST     0x67
#define ICM42670P_RESET_BIT          0x01

// Power Management (Bits 3:2 = Gyro Mode, Bits 1:0 = Accel Mode)
// 11 = Low Noise (LN) Mode
#define ICM42670P_PWR_LN_MODE        0x0F 

// Config Values (FS=2000dps/16g, ODR=1.6kHz)
// FS_SEL (Bits 6:5): 00 = 2000dps / 16g
// ODR (Bits 3:0): 0101 (0x05) = 1.6kHz (LN)
#define ICM42670P_CONFIG_DEFAULT     0x05 

namespace Espfc {

namespace Device {

class GyroICM42670P: public GyroDevice
{
  public:
    int begin(BusDevice * bus) override
    {
      // Try default address first (0x68), then alternate (0x69)
      return begin(bus, 0x68) ? 1 : begin(bus, 0x69);
    }

    int begin(BusDevice * bus, uint8_t addr) override
    {
      setBus(bus, addr);

      if(!testConnection()) return 0;

      // 1. Soft Reset
      _bus->writeByte(_addr, ICM42670P_REG_DEVICE_CONFIG, ICM42670P_RESET_BIT);
      delay(100);

      // 2. Power Management (CRITICAL: Enable Gyro & Accel in Low Noise Mode)
      // Unlike MPU6000, this sensor defaults to sleep.
      _bus->writeByte(_addr, ICM42670P_REG_PWR_MGMT0, ICM42670P_PWR_LN_MODE);
      delay(2); // Wait for PLL

      // 3. Configure Gyro (±2000dps, 1.6kHz)
      _bus->writeByte(_addr, ICM42670P_REG_GYRO_CONFIG0, ICM42670P_CONFIG_DEFAULT);

      // 4. Configure Accel (±16g, 1.6kHz)
      _bus->writeByte(_addr, ICM42670P_REG_ACCEL_CONFIG0, ICM42670P_CONFIG_DEFAULT);

      return 1;
    }

    GyroDeviceType getType() const override
    {
      return GYRO_ICM42670P;
    }

    int FAST_CODE_ATTR readGyro(VectorInt16& v) override
    {
      uint8_t buffer[6];
      // Read 6 bytes starting from GYRO_DATA_X1 (0x11)
      _bus->readFast(_addr, ICM42670P_REG_GYRO_DATA_X1, 6, buffer);

      v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
      v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
      v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

      return 1;
    }

    int readAccel(VectorInt16& v) override
    {
      uint8_t buffer[6];
      // Read 6 bytes starting from ACCEL_DATA_X1 (0x0B)
      _bus->readFast(_addr, ICM42670P_REG_ACCEL_DATA_X1, 6, buffer);

      v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
      v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
      v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

      return 1;
    }

    void setDLPFMode(uint8_t mode) override
    {
        // ICM-42670-P has fixed ODR/Filter presets in CONFIG0.
        // For this simple implementation, we stick to the default initialized in begin().
    }

    int getRate() const override
    {
      return 1600; // We configured 1.6kHz in begin()
    }

    void setRate(int rate) override
    {
       // Helper function to change ODR could go here, 
       // but 1.6kHz is standard for Betaflight-style loops.
    }

    bool testConnection() override
    {
      uint8_t whoami = 0;
      _bus->readByte(_addr, ICM42670P_REG_WHO_AM_I, &whoami);
      return (whoami == ICM42670P_WHO_AM_I_CONST);
    }
};

}

}

#endif