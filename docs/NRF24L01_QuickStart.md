# NRF24L01 Quick Start Guide

## 5-Minute Setup

### 1. Hardware Setup
Connect your NRF24L01+PA+LNA to ESP32:

```
ESP32          NRF24L01
====           ========
3.3V    -----> VCC  (+ 10µF cap to GND)
GND     -----> GND
GPIO 23 -----> MOSI
GPIO 18 -----> SCK
GPIO 19 <----- MISO  
GPIO 5  -----> CSN (CS)
GPIO 4  -----> CE (tie to VCC if not using)
```

**Power Supply**: Use a stable 3.3V source with a **10µF bypass capacitor** right next to the module.

### 2. Upload Firmware
The firmware already includes NRF24L01 support (just built successfully).

### 3. Enable in CLI
Connect to the ESP32 via serial (115200 baud) and enter CLI:

```bash
# Enable NRF24L01
set wireless_transceiver_dev = 2
set wireless_transceiver_enabled = 1
save
reboot
```

### 4. Verify It Works
```bash
# Check if module is detected
status wireless
```

You should see:
```
Wireless: Present = 1, Channel = 76, Power = 3
```

## Usage Examples

### Monitor Link Status
```bash
# Every time you connect to CLI, you'll see:
# Wireless TX packets:  123
# Wireless RX packets:  456  
# Wireless errors:      2
```

### Change Channel (to avoid WiFi)
```bash
set wireless_transceiver_channel = 100  # 2.500 GHz
save
reboot
```

### Change TX Power
```bash
set wireless_transceiver_power = 2  # -6 dBm (lower power)
save
reboot
```

### Set Custom Address
Edit your code in `src/main.cpp` or modify the config:
```cpp
// Both sender and receiver must use the SAME address
uint8_t address[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};
model.config.wireless_transceiver.setAddress(address, 5);
```

## Testing Communication

### Test 1: Single Module
Connect one NRF24L01 to your ESP32 and:
1. Upload firmware
2. Enable module via CLI
3. Check if it's detected: `status wireless`

### Test 2: Two Modules
If you have two ESP32s with NRF24L01:

**ESP32 #1 (Transmitter)**:
```cpp
// In your loop
uint8_t data[] = {0x48, 0x65, 0x6C, 0x6C, 0x6F};  // "Hello"
model.state.wireless.dev->send(data, 5);
delay(100);
```

**ESP32 #2 (Receiver)**:
```cpp
// In your loop
if (model.state.wireless.dev->isDataAvailable())
{
  uint8_t rxBuffer[32];
  uint8_t rxLength = 32;
  model.state.wireless.dev->receive(rxBuffer, &rxLength);
  // Process data...
}
```

## Default Configuration

| Setting | Default | Range |
|---------|---------|-------|
| Channel | 76 | 0-125 |
| TX Power | 3 (0dBm) | 0-3 |
| Data Rate | 2 Mbps | (fixed) |
| Address | 0xC2C2C2C2C2 | Any 5 bytes |
| Payload Size | 32 bytes | 1-32 |
| Enabled | No | Yes/No |

## Common Issues & Fixes

### "Module not detected" 
- ✓ Check power supply (need stable 3.3V)
- ✓ Add 10µF capacitor to VCC/GND
- ✓ Check SPI pin connections
- ✓ Reboot ESP32

### "Can't communicate between two modules"
- ✓ Set **same channel** on both
- ✓ Set **same 5-byte address** on both
- ✓ Check antenna is connected (PA+LNA version)
- ✓ Move modules closer together
- ✓ Check 3.3V power on both modules

### "Getting error packets"
- ✓ Reduce TX power: `set wireless_transceiver_power = 1`
- ✓ Change channel away from WiFi
- ✓ Check power supply quality (might need better PSU)
- ✓ Reduce distance between modules

## Specifications

**NRF24L01+PA+LNA:**
- Frequency: 2.400 - 2.525 GHz
- TX Power: 0 dBm (typical ~20 dBm with antenna for PA+LNA)
- Range: 100-300 meters (open air)
- Data Rate: 2 Mbps
- Channels: 125 (1 MHz each)
- Packet Size: 1-32 bytes

## Next Steps

1. Read the full guide: [NRF24L01_Integration.md](NRF24L01_Integration.md)
2. Implement your own telemetry protocol
3. Use for remote control or data logging
4. Experiment with different channels/power levels

## Get Help

- Check module is powered: Look for red LED on PA+LNA board
- Check SPI communication: See pin connections above
- Verify address matches: Make sure both devices use same address
- Test with simple hello world: Send 5 bytes, receive 5 bytes

---

Good luck with your wireless drone project! 🚁📡
