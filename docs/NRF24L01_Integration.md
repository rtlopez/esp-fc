# NRF24L01+PA+LNA Wireless Transceiver Integration Guide

## Overview
The esp-fc firmware now includes native support for the **NRF24L01+PA+LNA 2.4GHz Wireless Transceiver** using SPI connections. This enables wireless communication between your ESP32 drone controller and external devices.

## Hardware Connections

### ESP32 to NRF24L01+PA+LNA Wiring

| Signal | ESP32 Pin | NRF24L01 Pin | Notes |
|--------|-----------|--------------|-------|
| SCK (Clock) | GPIO 18 | SCK | SPI Clock |
| MOSI (Master Out) | GPIO 23 | MOSI | SPI Master Out Slave In |
| MISO (Master In) | GPIO 19 | MISO | SPI Master In Slave Out |
| CS (Chip Select) | GPIO 5 | CSN | NRF24L01 Chip Select |
| CE (Chip Enable) | GPIO 4 | CE | NRF24L01 Mode Control (Optional) |
| GND | GND | GND | Ground |
| 3.3V | 3.3V | VCC | Power Supply (add 10µF capacitor) |

**⚠️ IMPORTANT**: The NRF24L01+PA+LNA requires a **low-impedance power supply**. Add a **10µF ceramic capacitor** very close to the VCC and GND pins to prevent voltage drops during high current transmission peaks.

### Recommended Wiring Diagram
```
ESP32                          NRF24L01+PA+LNA
====                           ===============
3.3V  -------[10µF Caps]-----> VCC
                |
               GND <----------- GND
GPIO 23 (MOSI) ----------------> MOSI
GPIO 18 (SCK)  ----------------> SCK  
GPIO 19 (MISO) <---------------- MISO
GPIO 5 (CS)    ----------------> CSN
GPIO 4 (Optional) ----+------> CE (tie to VCC if not used)
```

## Configuration

### Default Pin Assignment
By default, the NRF24L01+PA+LNA uses **PIN_SPI_CS2** which is GPIO 5. You can verify this in `platformio.ini` or modify the pin configuration if needed.

### Enable the Module
The wireless transceiver is disabled by default. To enable it:

1. **Via CLI**: Open the esp-fc CLI interface and set:
   ```
   set wireless_transceiver_dev = 2      # Enable NRF24L01
   set wireless_transceiver_enabled = 1  # Turn on
   ```

2. **Via Configuration Code**: Edit your config and set:
   ```cpp
   model.config.wireless_transceiver.dev = WIRELESS_NRF24L01;
   model.config.wireless_transceiver.enabled = true;
   ```

### Configuration Options

```cpp
struct WirelessTransceiverConfig
{
  int8_t bus = BUS_AUTO;           // Auto-detect SPI bus
  int8_t dev = WIRELESS_NONE;      // Device type (WIRELESS_NRF24L01)
  uint8_t channel = 76;            // RF Channel (0-125, default 2.476 GHz)
  uint8_t power = 3;               // TX Power (0-3, where 3 = 0dBm max)
  uint8_t address[5] = {...};      // 5-byte address
  uint8_t payload = 32;            // Packet size (1-32 bytes)
  bool enabled = false;            // Enable/disable module
};
```

### Set Channel
The NRF24L01 operates in the 2.4 GHz ISM band (2.400-2.525 GHz). Each channel is 1 MHz wide.

**Frequency = 2400 + channel (MHz)**

Common channels:
- Channel 2 → 2402 MHz
- Channel 76 → 2476 MHz (default)
- Channel 125 → 2525 MHz

```cpp
model.config.wireless_transceiver.channel = 76;  // 2.476 GHz
```

### Set TX Power
Power level options (higher number = more power):
- 0: -18 dBm (minimum range, lowest power consumption)
- 1: -12 dBm
- 2: -6 dBm
- 3: 0 dBm (maximum range, for PA+LNA version)

```cpp
model.config.wireless_transceiver.power = 3;  // Max power for PA+LNA
```

### Set Wireless Address
Each NRF24L01 has a 5-byte address (40-bit). Both receiver and transmitter must use the same address.

```cpp
uint8_t address[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
model.config.wireless_transceiver.setAddress(address, 5);
```

## Runtime State

The wireless state is accessible via:
```cpp
struct WirelessState
{
  Device::WirelessDevice* dev;     // Device pointer
  bool present;                     // Device detected
  uint8_t channel;                  // Current channel
  uint8_t power;                    // Current TX power
  uint32_t packetsRx;               // Total packets received
  uint32_t packetsTx;               // Total packets sent
  uint32_t packetErrors;            // Total transmission errors
  int32_t lastRssi;                 // Last RSSI (signal strength)
  Utils::Timer timer;               // Timing for operations
};
```

Access via:
```cpp
if (model.state.wireless.present)
{
  Serial.print("Packets RX: ");
  Serial.println(model.state.wireless.packetsRx);
}
```

## API Usage

### Send Data
```cpp
uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
uint8_t length = 4;

if (model.state.wireless.dev)
{
  int result = model.state.wireless.dev->send(data, length);
  if (result == 1)
  {
    Serial.println("Data sent successfully");
  }
}
```

### Receive Data
```cpp
uint8_t rxBuffer[32];
uint8_t rxLength = 32;

if (model.state.wireless.dev && model.state.wireless.dev->isDataAvailable())
{
  if (model.state.wireless.dev->receive(rxBuffer, &rxLength) == 1)
  {
    Serial.print("Received ");
    Serial.print(rxLength);
    Serial.println(" bytes");
  }
}
```

### Check Signal Strength
```cpp
if (model.state.wireless.lastRssi != 0)
{
  Serial.print("RSSI: ");
  Serial.println(model.state.wireless.lastRssi);
}
```

### Change Channel at Runtime
```cpp
if (model.state.wireless.dev)
{
  model.state.wireless.dev->setChannel(80);  // 2.480 GHz
}
```

## Performance Characteristics

| Specification | Value |
|--------------|-------|
| Frequency Band | 2.400 - 2.525 GHz |
| Channels | 125 |
| Data Rate | 2 Mbps (default) |
| Transmit Power (PA+LNA) | 0 dBm (+20 dBm with antenna) |
| Range (open air) | ~100-300 meters |
| Packet Size | 1-32 bytes |
| CRC | Enabled (1 byte) |

## Troubleshooting

### Module Not Detected
1. **Check power supply**: Verify 3.3V is present and stable
2. **Check capacitor**: Ensure 10µF capacitor is connected close to VCC/GND
3. **Verify SPI pins**: Confirm correct GPIO pins for SCK, MOSI, MISO
4. **Test CS pin**: GPIO 5 should go low during communication

### No Communication
1. **Verify channel**: Both devices must use the same channel
2. **Verify address**: Both devices must use the same 5-byte address
3. **Check antenna**: For PA+LNA version, verify antenna is properly connected
4. **Monitor RSSI**: Use `model.state.wireless.lastRssi` to check signal strength

### High Error Rate
1. **Reduce TX power** temporarily to avoid saturation
2. **Move devices apart** to reduce RF interference
3. **Change channel** to avoid 2.4GHz WiFi interference:
   - WiFi typically uses channels 1, 6, 11 (2.412, 2.437, 2.462 GHz)
   - Use channel 76+ (2.476+ GHz) to minimize interference
4. **Check power supply** again - unstable voltage increases errors

## Serial Debugging

Enable debug output for wireless module (modify platformio.ini):
```ini
[env:esp32]
build_flags =
  -DESPFC_DEBUG_WIRELESS  # Enable wireless debug logging
```

## Advanced: Direct SPI Access

If you need low-level SPI control:
```cpp
// Access the wireless device directly
Device::Wireless::NRF24L01* nrf = 
  static_cast<Device::Wireless::NRF24L01*>(model.state.wireless.dev);

// Perform advanced operations
uint8_t status = nrf->getStatus();
nrf->flushRx();
nrf->flushTx();
```

## Example: Simple Telemetry Link

```cpp
// In your control loop:
void wirelessUpdate()
{
  if (!model.state.wireless.present || !model.state.wireless.dev) 
    return;

  // Send telemetry data
  uint8_t telemetry[8];
  telemetry[0] = model.state.gyro.raw.x >> 8;
  telemetry[1] = model.state.gyro.raw.x & 0xFF;
  telemetry[2] = model.state.gyro.raw.y >> 8;
  telemetry[3] = model.state.gyro.raw.y & 0xFF;
  telemetry[4] = model.state.gyro.raw.z >> 8;
  telemetry[5] = model.state.gyro.raw.z & 0xFF;
  telemetry[6] = model.state.input.us[AXIS_THRUST] >> 8;
  telemetry[7] = model.state.input.us[AXIS_THRUST] & 0xFF;

  model.state.wireless.dev->send(telemetry, 8);

  // Receive commands
  if (model.state.wireless.dev->isDataAvailable())
  {
    uint8_t rxBuffer[32];
    uint8_t rxLength = 32;
    if (model.state.wireless.dev->receive(rxBuffer, &rxLength) == 1)
    {
      // Process received command
      if (rxBuffer[0] == CMD_ARM)
      {
        model.state.mode.mask |= (1 << MODE_ARMED);
      }
    }
  }
}
```

## File Structure

New files added to support NRF24L01:
- `lib/Espfc/src/Device/Wireless/WirelessDevice.hpp` - Base wireless device class
- `lib/Espfc/src/Device/Wireless/WirelessDevice.cpp` - Implementation
- `lib/Espfc/src/Device/Wireless/NRF24L01.hpp` - NRF24L01 driver header
- `lib/Espfc/src/Device/Wireless/NRF24L01.cpp` - NRF24L01 driver implementation

Modified files:
- `lib/Espfc/src/ModelConfig.h` - Added WirelessTransceiverConfig
- `lib/Espfc/src/ModelState.h` - Added WirelessState
- `lib/Espfc/src/Hardware.h` - Added wireless device detection
- `lib/Espfc/src/Hardware.cpp` - Implemented detectWireless()
- `lib/Espfc/src/Model.h` - No changes (uses ModelState)

## Future Enhancements
- [ ] ACK/retransmission support
- [ ] Address-based packet filtering
- [ ] RSSI-based automatic power control
- [ ] Channel hopping for interference avoidance
- [ ] Enhanced error recovery

## References
- [NRF24L01 Datasheet](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf)
- [NRF24L01 Register Map](https://cdn.shopify.com/s/files/1/0549/8653/products/nrf24l01p_register_map_0.jpg)
- [RF Channel Selection Guide](https://en.wikipedia.org/wiki/IEEE_802.11#Naming_and_terminology)
