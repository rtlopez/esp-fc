#ifndef _RC_CRSF_H_
#define _RC_CRSF_H_

#include "Math/Utils.h"
#include "Math/Crc.h"

namespace Espfc {

namespace Rc {

enum { CRSF_SYNC_BYTE = 0xC8 };
enum { CRSF_FRAME_SIZE_MAX = 64 }; // 62 bytes frame plus 2 bytes frame header(<length><type>)
enum { CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6 };

enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
};

enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
};

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */
struct CrsfLinkStats {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} __attribute__ ((__packed__));

struct CrsfData
{
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
} __attribute__ ((__packed__));

/*
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 */
struct CrsfMessage
{
  uint8_t addr; // CrsfAddress
  uint8_t size; // counts size after this byte, so it must be the payload size + 2 (type and crc)
  uint8_t type; // CrsfFrameType
  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];
} __attribute__ ((__packed__));

union CrsfFrame
{
  CrsfMessage message;
  uint8_t data[CRSF_FRAME_SIZE_MAX];
};

class Crsf
{
public:
  static void decodeRcData(uint16_t* channels, const CrsfData* frame)
  {
    channels[0]  = convert(frame->chan0);
    channels[1]  = convert(frame->chan1);
    channels[2]  = convert(frame->chan2);
    channels[3]  = convert(frame->chan3);
    channels[4]  = convert(frame->chan4);
    channels[5]  = convert(frame->chan5);
    channels[6]  = convert(frame->chan6);
    channels[7]  = convert(frame->chan7);
    channels[8]  = convert(frame->chan8);
    channels[9]  = convert(frame->chan9);
    channels[10] = convert(frame->chan10);
    channels[11] = convert(frame->chan11);
    channels[12] = convert(frame->chan12);
    channels[13] = convert(frame->chan13);
    channels[14] = convert(frame->chan14);
    channels[15] = convert(frame->chan15);
  }

  static void encodeRcData(CrsfFrame& frame, const CrsfData& data)
  {
    frame.message.addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame.message.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    frame.message.size = sizeof(data) + 2;
    memcpy(frame.message.payload, (void*)&data, sizeof(data));
    frame.message.payload[sizeof(data)] = crc(frame);
  }

  static inline uint16_t convert(int v)
  {
    /* conversion from RC value to PWM
     * for 0x16 RC frame
     *       RC     PWM
     * min  172 ->  988us
     * mid  992 -> 1500us
     * max 1811 -> 2012us
     * scale factor = (2012-988) / (1811-172) = 0.62477120195241    => 1024 / 1639 = 0.62477
     * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548   => 988 - 107.46 = 880.54
     */
    return ((v * 1024) / 1639) + 881;
    //return lrintf((0.62477120195241 * (float)v) + 880.54);
    //return Math::map(v, 172, 1811, 988, 2012);
    //return Math::mapi(v, 172, 1811, 988, 2012);
  }

  static uint8_t crc(const CrsfFrame& frame)
  {
    // CRC includes type and payload
    uint8_t crc = Math::crc8_dvb_s2(0, frame.message.type);
    for (int i = 0; i < frame.message.size - 2; i++) { // size includes type and crc
        crc = Math::crc8_dvb_s2(crc, frame.message.payload[i]);
    }
    return crc;
  }
};

}

}

#endif