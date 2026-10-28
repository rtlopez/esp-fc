# ESP-FC Wireless functions

Espressif modules have a built-in WiFi module that can be used for configuration and control.

## WiFi configuration

> [!NOTE]
> To be able to configure device using WiFi connection, you need to enable `SOFTSERIAL` function. WiFi function can only be used to configure device. Whilst WiFi is active, it is not possible to ARM controller and reboot is required.

![Enable WiFi](/docs/images/espfc_wifi_ap_enable.png)

WiFi will start its own Access-Point if board will not detect receiver signal for at least 30 seconds (stay in failsafe mode since boot). Name of this AP is `ESP-FC` and it is open network. If you connect to this AP, then choose `Manual Selection` and type `tcp://192.168.4.1:1111` 

![Connect to ESP-FC AP](/docs/images/espfc_wifi_ap_connect.png)

You can also automatically connect ESP-FC to your home network. To achive that, go to the CLI tab, and enter your network name and secret.
```
set wifi_ssid MY-HOME-NET
set wifi_pass MY-HOME-PASS
```
> [!NOTE]
> Network name and pass must not contains spaces.

then you can check status of connection by typing `wifi` in CLI tab.
```
wifi 
ST IP4: tcp://0.0.0.0:1111
ST MAC: 30:30:F9:6E:10:74
AP IP4: tcp://192.168.4.1:1111
AP MAC: 32:30:F9:6E:10:74
```
In this mode you need to discover IP address granted by DHCP. In line `ST IP4` is the address that you need to use in configurator. If there is `0.0.0.0`, that means that FC was not able to connect to home network.

## ESP-NOW control

ESP-NOW is a proprietary wireless communication protocol defined by Espressif, which enables the direct, quick and low-power control of smart devices, without the need of a router. 

As there are no real transmitters usng this protocol on the market, you have to build your own transmitting module first. If you alredy own RC transmitter with JR bay, you can use another ESP32 module. In this case follow [espnow-rclink-tx](https://github.com/rtlopez/espnow-rclink-tx) instructions.

In ESP-FC you have to choose `SPI Rx (e.g. built-in Rx)` as Receiver mode in Receiver tab.

![ESP-FC ESP-NOW Reciever](/docs/images/espfc_receiver.png)

Transmitter and receiver binds automatically after power up, you don't need to do anything. Recomended startup procedure is:
1. turn on transmitter first
2. next power up receiver/flight controller
