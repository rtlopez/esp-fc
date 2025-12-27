#ifndef _ESPFC_DEVICE_MAG_QMC5338P_H_
#define _ESPFC_DEVICE_MAG_QMC5338P_H_

#include "MagDevice.h"
#include "BusDevice.h"

#define QMC5883P_ADDRESS            0x2C
#define QMC5883P_DEFAULT_ADDRESS    0x2C

#define QMC5883P_REG_CHIPID         0x00
#define QMC5883P_REG_XOUT_LSB       0x01
#define QMC5883P_REG_XOUT_MSB       0x02
#define QMC5883P_REG_YOUT_LSB       0x03
#define QMC5883P_REG_YOUT_MSB       0x04
#define QMC5883P_REG_ZOUT_LSB       0x05
#define QMC5883P_REG_ZOUT_MSB       0x06
#define QMC5883P_REG_STATUS         0x09
#define QMC5883P_REG_CONTROL1       0x0A
#define QMC5883P_REG_CONTROL2       0x0B

#define QMC5883P_RANGE_30G          0x00
#define QMC5883P_RANGE_12G          0x01
#define QMC5883P_RANGE_8G           0x02
#define QMC5883P_RANGE_2G           0x03

#define QMC5883P_MODE_CONTINUOUS    0x03

namespace Espfc {
namespace Device {

class MagQMC5338P : public MagDevice 
{
public:
    int begin(BusDevice* bus) override 
    {
        return begin(bus, QMC5883P_DEFAULT_ADDRESS);
    }

    int begin(BusDevice* bus, uint8_t addr) override {
        setBus(bus, addr);

        if (!testConnection()) return 0;

        // 🔑 Step 1: Enable Set/Reset (write 0x29 to Z_MSB)
        _bus->writeByte(_addr, QMC5883P_REG_ZOUT_MSB, 0x29);

        // 🔑 Step 2: Set Range in CONTROL2[3:2] (Adafruit style)
        _currentRange = QMC5883P_RANGE_8G;
        setMode(_currentRange); // This will write to CONTROL2

        // 🔑 Step 3: Configure CONTROL1
        uint8_t ctrl1 = 
            QMC5883P_MODE_CONTINUOUS |  // [1:0]
            (0x02 << 2) |               // ODR = 100Hz [3:2]
            (0x03 << 4) |               // OSR = 1 [5:4]
            (0x00 << 6);                // DSR = 1 [7:6]
        _bus->writeByte(_addr, QMC5883P_REG_CONTROL1, ctrl1);

        // Initial read
        uint8_t buffer[6];
        _bus->read(_addr, QMC5883P_REG_XOUT_LSB, 6, buffer);

        return 1;
    }

    int readMag(VectorInt16& v) override {
        uint8_t buffer[6];
        if (_bus->read(_addr, QMC5883P_REG_XOUT_LSB, 6, buffer) != 6) {
            return 0;
        }

        // 🔑 Read raw values EXACTLY like Adafruit (no sign flip yet)
        v.x = (int16_t)((buffer[1] << 8) | buffer[0]);
        v.y = (int16_t)((buffer[3] << 8) | buffer[2]);
        v.z = (int16_t)((buffer[5] << 8) | buffer[4]);

        return 1;
    }

    const VectorFloat convert(const VectorInt16& v) const override {
        // 🔑 Use Adafruit's conversion factors (based on actual range)
        float lsb_per_gauss;
        switch (_currentRange) {
            case QMC5883P_RANGE_30G: lsb_per_gauss = 1000.0f;  break;
            case QMC5883P_RANGE_12G: lsb_per_gauss = 2500.0f;  break;
            case QMC5883P_RANGE_8G:  lsb_per_gauss = 3750.0f;  break;
            case QMC5883P_RANGE_2G:  lsb_per_gauss = 15000.0f; break;
            default: lsb_per_gauss = 3750.0f;
        }
        float scale = 1.0f / lsb_per_gauss;
        return VectorFloat{ v.x * scale, v.y * scale, v.z * scale };
    }

    int getRate() const override {
        return 100;
    }

    virtual MagDeviceType getType() const override {
        return MAG_QMC5883P;
    }

    // 🔑 CORRECT setMode: writes range to CONTROL2[3:2]
    void setMode(uint8_t range) {
        _currentRange = range;
        // Range is in bits [3:2] of CONTROL2
        uint8_t ctrl2 = (range << 2);
        _bus->writeByte(_addr, QMC5883P_REG_CONTROL2, ctrl2);
    }

    bool testConnection() override {
        uint8_t chip_id;
        if (_bus->read(_addr, QMC5883P_REG_CHIPID, 1, &chip_id) != 1) {
            return false;
        }
        return chip_id == 0x80; 
    }

private:
    uint8_t _currentRange = QMC5883P_RANGE_8G;
};

} 
} 

#endif


