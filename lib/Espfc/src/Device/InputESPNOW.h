#ifndef _INPUT_DEVICE_INPUT_ESPNOW_H_
#define _INPUT_DEVICE_INPUT_ESPNOW_H_

#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
  #include <WiFi.h>
  #include "esp_wifi.h"
  #include <esp_now.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#endif
#include "Device/InputDevice.h"
#include "Math/Utils.h"



namespace Espfc {

namespace Device {

struct ControllerData {
  uint8_t syncByte: 1;
  uint16_t A: 10;
  uint16_t E: 10;
  uint16_t R: 10;
  uint16_t T: 10;

  uint8_t ch1;
  uint8_t ch2;
  uint8_t ch3;
  uint8_t ch4;
}_data;
int8_t rssi;
struct TelemetryData {
  uint8_t syncByte: 1;

  uint8_t Battery_v; // int/10 -> fixed point 25.6v max
}TxData;
bool _new_data = false;
#define wifi_channel 4 // Set the wifi channel (1-13) -> TODO: scan and choose the best one
uint8_t TxMacAddr[] = {0xA8,0x42,0xE3,0xCD,0x5F,0x04}; //TODO: put it into model
bool pair_status = false; //TODO: should be save in model, can be changed using cli



#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
void IRAM_ATTR OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
#elif defined(ESP8266)
void IRAM_ATTR OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
#endif
{
  memcpy(&_data, incomingData, sizeof(_data));
  if(pair_status){
    for (int i = 0; i < 6; i++) {
      if (mac[i] != TxMacAddr[i]) return;
    }
    if (_data.syncByte) return; //ignore the second pair req
    _new_data = true;
    return;
  }

  switch (_data.syncByte) {
  case 0://TODO: telemetry
    break;

  case 1://pair
    memcpy(TxMacAddr, mac, 6);
    pair_status = true;

#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
    //TODO: move the following part away from this IRS function to avoid WDT
    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer 
    esp_now_add_peer(&peerInfo); // TODO: record errors to log
#elif defined(ESP8266)
    esp_now_add_peer(TxMacAddr, ESP_NOW_ROLE_COMBO, wifi_channel, NULL, 0);
#endif
    // Send pair request
    TxData.syncByte = 1;
    esp_now_send(TxMacAddr, (uint8_t *) &TxData, sizeof(TxData));

    //TODO: save
    break;
  }
}

#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  rssi = ppkt->rx_ctrl.rssi; // signed 8bits -128 - 128
}
#endif

class InputESPNOW: public InputDevice
{
  public:

    InputESPNOW() {}

    int begin(void)
    {
      WiFi.mode(WIFI_STA);
      WiFi.channel(wifi_channel);
      esp_now_init();
      esp_now_register_recv_cb(OnDataRecv);
#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
      esp_wifi_set_promiscuous(true); // rssi
      esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);// rssi
#endif
      return 1;
    }

    InputStatus update() override
    {
      if(_new_data) {
        _new_data = false;
        _channels[0]  = convert(_data.A * 175 / 128 + 800);
        _channels[1]  = convert(_data.E * 175 / 128 + 800);
        _channels[2]  = convert(_data.R * 175 / 128 + 800);
        _channels[3]  = convert(_data.T * 175 / 128 + 800);
        _channels[4]  = convert(_data.ch1 * 175 / 32 + 800);
        _channels[5]  = convert(_data.ch2 * 175 / 32 + 800);
        _channels[6]  = convert(_data.ch3 * 175 / 32 + 800);
        _channels[7]  = convert(_data.ch4 * 175 / 32 + 800);
        _channels[8]  = convert(((int)rssi + 128) * 175 /32 + 800);

        //if(millis() - lastRecvTime > 60) return INPUT_FAILSAFE;
        //if(millis() - lastRecvTime > 100) return INPUT_LOST;
        return INPUT_RECEIVED;
      }
      return INPUT_IDLE;
    }

    uint16_t get(uint8_t i) const override
    {
      return _channels[i];
    }

    void get(uint16_t * data, size_t len) const override
    {
      const uint16_t * src = _channels;
      while(len--)
      {
        *data++ = *src++;
      }
    }

    size_t getChannelCount() const override { return CHANNELS; }

    bool needAverage() const override { return false; }

    void print(char c) const
    {
      //Serial.write(c);
    }

  private:

  inline uint16_t convert(int v) {
    return Math::clamp(v, 800, 2200);
  }

  static const size_t CHANNELS = 9;
  uint16_t _channels[CHANNELS];
};

}

}

#endif
