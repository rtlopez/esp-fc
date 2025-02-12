/*
 * GyroICM40608.h
 *
 *  Created on: 2020. 12. 10.
 *  Ported by: Harry Sun
 *
 *  Ported from: https://github.com/Mert-Kilic/ICM-42688-P/blob/main/icm.c
 *
 *  The source of port was intended to be used for ICM42688P,42653,42605.
 *  Code was modified to be used for ICM40608.
 */
#ifndef _ESPFC_DEVICE_GYRO_ICM40608_H_
#define _ESPFC_DEVICE_GYRO_ICM40608_H_

#include "BusDevice.h"
#include "GyroDevice.h"
#include "helper_3dmath.h"
#include "Debug_Espfc.h"

// b1101000 b1101001 (0x68, 0x69)
#define ICM40608_ADDRESS_FIRST 0x68
#define ICM40608_ADDRESS_SECOND 0x69
#define ICM40608_WHO_AM_I_CONST 0x39
#define ICM40608_RA_WHO_AM_I 0x75

// defines from https://github.com/betaflight/betaflight/blob/master/src/main/drivers/accgyro/accgyro_spi_icm426xx.c
#define ICM40608_CLKIN_FREQ 32000

#define ICM40608_RA_REG_BANK_SEL 0x76
#define ICM40608_BANK_SELECT0 0x00
#define ICM40608_BANK_SELECT1 0x01
#define ICM40608_BANK_SELECT2 0x02
#define ICM40608_BANK_SELECT3 0x03
#define ICM40608_BANK_SELECT4 0x04

#define ICM40608_INTF_CONFIG1 0x4D
#define ICM40608_INTF_CONFIG1_AFSR_MASK 0xC0
#define ICM40608_INTF_CONFIG1_AFSR_DISABLE 0x40

#define ICM40608_RA_PWR_MGMT0 0x4E // User Bank 0
#define ICM40608_PWR_MGMT0_ACCEL_MODE_LN (3 << 0)
#define ICM40608_PWR_MGMT0_GYRO_MODE_LN (3 << 2)
#define ICM40608_PWR_MGMT0_GYRO_ACCEL_MODE_OFF ((0 << 0) | (0 << 2))
#define ICM40608_PWR_MGMT0_TEMP_DISABLE_OFF (0 << 5)
#define ICM40608_PWR_MGMT0_TEMP_DISABLE_ON (1 << 5)

#define ICM40608_RA_GYRO_CONFIG0 0x4F
#define ICM40608_RA_ACCEL_CONFIG0 0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM40608_RA_GYRO_CONFIG_STATIC3 0x0C  // User Bank 1
#define ICM40608_RA_GYRO_CONFIG_STATIC4 0x0D  // User Bank 1
#define ICM40608_RA_GYRO_CONFIG_STATIC5 0x0E  // User Bank 1
#define ICM40608_RA_ACCEL_CONFIG_STATIC2 0x03 // User Bank 2
#define ICM40608_RA_ACCEL_CONFIG_STATIC3 0x04 // User Bank 2
#define ICM40608_RA_ACCEL_CONFIG_STATIC4 0x05 // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM40608_RA_GYRO_ACCEL_CONFIG0 0x52 // User Bank 0
#define ICM40608_ACCEL_UI_FILT_BW_LOW_LATENCY (15 << 4)
#define ICM40608_GYRO_UI_FILT_BW_LOW_LATENCY (15 << 0)
// ----------------------------------------------------------

#define ICM40608_RA_GYRO_DATA_X1 0x25  // User Bank 0
#define ICM40608_RA_ACCEL_DATA_X1 0x1F // User Bank 0

#define ICM40608_RA_INT_CONFIG 0x14 // User Bank 0
#define ICM40608_INT1_MODE_PULSED (0 << 2)
#define ICM40608_INT1_MODE_LATCHED (1 << 2)
#define ICM40608_INT1_DRIVE_CIRCUIT_OD (0 << 1)
#define ICM40608_INT1_DRIVE_CIRCUIT_PP (1 << 1)
#define ICM40608_INT1_POLARITY_ACTIVE_LOW (0 << 0)
#define ICM40608_INT1_POLARITY_ACTIVE_HIGH (1 << 0)

#define ICM40608_RA_INT_CONFIG0 0x63 // User Bank 0
#define ICM40608_UI_DRDY_INT_CLEAR_ON_SBR ((0 << 5) || (0 << 4))
#define ICM40608_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM40608_UI_DRDY_INT_CLEAR_ON_F1BR ((1 << 5) || (0 << 4))
#define ICM40608_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR ((1 << 5) || (1 << 4))

#define ICM40608_RA_INT_CONFIG1 0x64 // User Bank 0
#define ICM40608_INT_ASYNC_RESET_BIT 4
#define ICM40608_INT_TDEASSERT_DISABLE_BIT 5
#define ICM40608_INT_TDEASSERT_ENABLED (0 << ICM40608_INT_TDEASSERT_DISABLE_BIT)
#define ICM40608_INT_TDEASSERT_DISABLED (1 << ICM40608_INT_TDEASSERT_DISABLE_BIT)
#define ICM40608_INT_TPULSE_DURATION_BIT 6
#define ICM40608_INT_TPULSE_DURATION_100 (0 << ICM40608_INT_TPULSE_DURATION_BIT)
#define ICM40608_INT_TPULSE_DURATION_8 (1 << ICM40608_INT_TPULSE_DURATION_BIT)

#define ICM40608_RA_INT_SOURCE0 0x65 // User Bank 0
#define ICM40608_UI_DRDY_INT1_EN_DISABLED (0 << 3)
#define ICM40608_UI_DRDY_INT1_EN_ENABLED (1 << 3)

// specific to CLKIN configuration
#define ICM40608_INTF_CONFIG5 0x7B // User Bank 1
#define ICM40608_INTF_CONFIG1_CLKIN (1 << 2)
#define ICM40608_INTF_CONFIG5_PIN9_FUNCTION_MASK (3 << 1)  // PIN9 mode config
#define ICM40608_INTF_CONFIG5_PIN9_FUNCTION_CLKIN (2 << 1) // PIN9 as CLKIN

namespace Espfc
{
  namespace Device
  {
    class GyroICM40608 : public GyroDevice
    {
    public:
      int begin(BusDevice *bus) override
      {
        return begin(bus, ICM40608_ADDRESS_FIRST) ? 1 : begin(bus, ICM40608_ADDRESS_SECOND) ? 1
                                                                                            : 0;
      }

      int begin(BusDevice *bus, uint8_t addr) override
      {
        setBus(bus, addr);
        // https://github.com/Mert-Kilic/ICM-42688-P/blob/main/icm.c
        uint8_t configure_reset = 0x01;
        uint8_t fifo_conf_data = 0x03;
        uint8_t buffer = 0x1F; // temperature sensor enabled. RC oscillator is on, gyro and accelerometer low noise mode,
        uint8_t fifo_init = 0x40;
        _bus->writeByte(_addr, 0x11, configure_reset); // DEVICE_CONFIG 0x11
        delay(100);
        _bus->writeByte(_addr, ICM40608_RA_REG_BANK_SEL, ICM40608_BANK_SELECT0); // select bank 0
        _bus->writeByte(_addr, ICM40608_RA_PWR_MGMT0, buffer);
        delay(100);
        if (!testConnection())
          return 0;
        _bus->writeByte(_addr, 0x16, fifo_init);      // FIFO_CONFIG_INIT 0x16
        _bus->writeByte(_addr, 0x5F, fifo_conf_data); // FIFO_CONFIGURATION 0x5F
        delay(100);
        return 1;
      }

      GyroDeviceType getType() const override
      {
        return GYRO_ICM40608;
      }

      int readGyro(VectorInt16 &v) override
      {
        uint8_t fifo_data[16];
        _bus->readFast(_addr, 0x30, 16, fifo_data); // FIFO_DATA_REG 0x30
        v.x = (((int16_t)fifo_data[7]) << 8) | fifo_data[8];
        v.y = (((int16_t)fifo_data[9]) << 8) | fifo_data[10];
        v.z = (((int16_t)fifo_data[11]) << 8) | fifo_data[12];
        return 1;
      }

      int readAccel(VectorInt16 &v) override
      {
        uint8_t fifo_data[16];
        _bus->readFast(_addr, 0x30, 16, fifo_data); // FIFO_DATA_REG 0x30
        v.x = (((int16_t)fifo_data[1]) << 8) | fifo_data[2];
        v.y = (((int16_t)fifo_data[3]) << 8) | fifo_data[4];
        v.z = (((int16_t)fifo_data[5]) << 8) | fifo_data[6];
        return 1;
      }

      void setDLPFMode(uint8_t mode) override
      {
      }

      int getRate() const override
      {
        return 3200;
      }

      void setRate(int rate) override
      {
      }

      // Ported from Betaflight
      bool testConnection() override
      {
        uint8_t whoami = 0;
        _bus->readByte(_addr, ICM40608_RA_WHO_AM_I, &whoami);
        return whoami == ICM40608_WHO_AM_I_CONST;
      }
    };

  }

}

#endif
