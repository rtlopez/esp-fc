#include "Crsf.h"
#include "Utils/Crc.hpp"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"
#include <Arduino.h>
#include <cstring>

namespace Espfc::Rc {

void FAST_CODE_ATTR Crsf::decodeRcData(uint16_t* channels, const CrsfData* frame)
{
  channels[0] = convert(frame->chan0);
  channels[1] = convert(frame->chan1);
  channels[2] = convert(frame->chan2);
  channels[3] = convert(frame->chan3);
  channels[4] = convert(frame->chan4);
  channels[5] = convert(frame->chan5);
  channels[6] = convert(frame->chan6);
  channels[7] = convert(frame->chan7);
  channels[8] = convert(frame->chan8);
  channels[9] = convert(frame->chan9);
  channels[10] = convert(frame->chan10);
  channels[11] = convert(frame->chan11);
  channels[12] = convert(frame->chan12);
  channels[13] = convert(frame->chan13);
  channels[14] = convert(frame->chan14);
  channels[15] = convert(frame->chan15);
}

void FAST_CODE_ATTR Crsf::decodeRcDataShift8(uint16_t* channels, const CrsfData* frame)
{
  // 8-bit
  // 0....... ...1.... ......2. ........ .3...... ....4... .......5 ........ ..6..... .....7.. ........
  // 8....... ...9.... ......A. ........ .B...... ....C... .......D ........ ..E..... .....F.. ........
  const uint8_t* crsfData = reinterpret_cast<const uint8_t*>(frame);
  channels[0] = convert((crsfData[0] | crsfData[1] << 8) & 0x07FF);
  channels[1] = convert((crsfData[1] >> 3 | crsfData[2] << 5) & 0x07FF);
  channels[2] = convert((crsfData[2] >> 6 | crsfData[3] << 2 | crsfData[4] << 10) & 0x07FF);
  channels[3] = convert((crsfData[4] >> 1 | crsfData[5] << 7) & 0x07FF);

  channels[4] = convert((crsfData[5] >> 4 | crsfData[6] << 4) & 0x07FF);
  channels[5] = convert((crsfData[6] >> 7 | crsfData[7] << 1 | crsfData[8] << 9) & 0x07FF);
  channels[6] = convert((crsfData[8] >> 2 | crsfData[9] << 6) & 0x07FF);
  channels[7] = convert((crsfData[9] >> 5 | crsfData[10] << 3) & 0x07FF);

  channels[8] = convert((crsfData[11] | crsfData[12] << 8) & 0x07FF);
  channels[9] = convert((crsfData[12] >> 3 | crsfData[13] << 5) & 0x07FF);
  channels[10] = convert((crsfData[13] >> 6 | crsfData[14] << 2 | crsfData[15] << 10) & 0x07FF);
  channels[11] = convert((crsfData[15] >> 1 | crsfData[16] << 7) & 0x07FF);

  channels[12] = convert((crsfData[16] >> 4 | crsfData[17] << 4) & 0x07FF);
  channels[13] = convert((crsfData[17] >> 7 | crsfData[18] << 1 | crsfData[19] << 9) & 0x07FF);
  channels[14] = convert((crsfData[19] >> 2 | crsfData[20] << 6) & 0x07FF);
  channels[15] = convert((crsfData[20] >> 5 | crsfData[21] << 3) & 0x07FF);
}

/*void Crsf::decodeRcDataShift32(uint16_t* channels, const CrsfData* frame)
{
  // 32-bit
  // 0..........1..........2......... .3..........4..........5........ ..6..........7..........8.......
  // ...9..........A..........B...... ....C..........D..........E..... .....F..........
  const uint32_t * crsfData = reinterpret_cast<const uint32_t *>(frame);
  channels[0]  = convert((crsfData[0]) & 0x07FF);
  channels[1]  = convert((crsfData[0] >> 11) & 0x07FF);
  channels[2]  = convert((crsfData[0] >> 22 | crsfData[1] << 10) & 0x07FF);
  channels[3]  = convert((crsfData[1] >> 1) & 0x07FF);

  channels[4]  = convert((crsfData[1] >> 12) & 0x07FF);
  channels[5]  = convert((crsfData[1] >> 23 | crsfData[2] << 9)  & 0x07FF);
  channels[6]  = convert((crsfData[2] >> 2) & 0x07FF);
  channels[7]  = convert((crsfData[2] >> 13) & 0x07FF);

  channels[8]  = convert((crsfData[2] >> 24 | crsfData[3] << 8) & 0x07FF);
  channels[9]  = convert((crsfData[3] >> 3) & 0x07FF);
  channels[10] = convert((crsfData[3] >> 14) & 0x07FF);
  channels[11] = convert((crsfData[3] >> 25 | crsfData[4] << 7) & 0x07FF);

  channels[12] = convert((crsfData[4] >> 4) & 0x07FF);
  channels[13] = convert((crsfData[4] >> 15)  & 0x07FF);
  channels[14] = convert((crsfData[4] >> 26 | crsfData[5] << 6) & 0x07FF);
  channels[15] = convert((crsfData[5] >> 5) & 0x07FF);
}*/

void Crsf::encodeRcData(CrsfMessage& msg, const CrsfData& data)
{
  msg.addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  msg.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
  msg.size = sizeof(data) + 2;
  std::memcpy(msg.payload, (void*)&data, sizeof(data));
  msg.payload[sizeof(data)] = crc(msg);
}

const uint8_t* Crsf::encodeMspData(CrsfMessage& msg, uint8_t origin, uint8_t version, uint8_t seq, bool start,
                                   const uint8_t* begin, const uint8_t* end)
{
  auto fragmentEnd = std::min(begin + CRSF_PAYLOAD_SIZE_MAX - 1, end); // preserve space for crc

  uint8_t status = 0;
  status |= (seq & CRSF_MSP_STATUS_SEQ_MASK);                // sequence number
  status |= start ? (1 << 4) : 0;                            // start bit
  status |= ((version << 5) & CRSF_MSP_STATUS_VERSION_MASK); // msp version (1 or 2)

  msg.prepare(Rc::CRSF_FRAMETYPE_MSP_RESP);
  msg.writeU8(origin);
  msg.writeU8(Rc::CRSF_ADDRESS_FLIGHT_CONTROLLER);
  msg.writeU8(status);
  msg.write(begin, fragmentEnd - begin);
  msg.finalize();

  return fragmentEnd;
}

template<typename HeaderType>
static inline void fillMessage(const CrsfMessage& frame, Connect::MspMessage& m, Connect::MspVersion version)
{
  // Payload structure: [dst, origin, flags, msp_header, msp_data...]
  // Available MSP data (after header) = frame.size - 5 (type,dst,origin,flags,crc) - sizeof(header)
  const auto* hdr = reinterpret_cast<const HeaderType*>(frame.payload + 3);
  const size_t framePayloadSize = frame.size - 5 - sizeof(HeaderType);
  m.cmd = hdr->cmd;
  m.version = version;
  m.dir = Connect::MSP_TYPE_CMD;
  m.expected = hdr->size;
  m.append(frame.payload + 3 + sizeof(HeaderType),
           std::min(framePayloadSize, (size_t)hdr->size)); // skip dst, origin, status and msp header
  if (m.expected == m.received)
  {
    m.state = Connect::MSP_STATE_RECEIVED;
  }
}

int FAST_CODE_ATTR Crsf::decodeMsp(const CrsfMessage& frame, Connect::MspMessage& m, uint8_t& origin)
{
  // 0x7A, 0x7C
  //   CRSF frame which wraps MSP request (‘$M<’ or ‘$X<’)
  //   Supported by Betaflight devices
  //   Supported devices will respond with 0x7B

  // 0x7B
  //   CRSF frame which wraps MSP response (‘$M>’,’$X>’,‘$M!’,’$X!’)
  //   Supported by Betaflight devices
  //   Supported device will send this frame in response of MSP_Request (0x7A)

  // MSP frame over CRSF Payload packing:
  //   MSP frame is stripped from header ($ + M/X + [/]/!) and CRC
  //   Resulted MSP-body might be divided in chunks if it doesn't fit in one CRSF-frame.
  //   A ‘Status’ byte is put before MSP-body in each CRSF-frame.
  //   Status byte consists of three parts:
  //     bits 0-3 represent cyclic sequence number of the CRSF frame;
  //     bit 4 checks if current MSP chunk is the beginning (or only) of a new frame (1 if true);
  //     bits 5-6 represent the version number of MSP protocol (1 or 2 currently);
  //     bit 7 represents an error (for response only).
  //   Chunk size of the MSP-body is calculated from size of CRSF frame. But size of the MSP-body
  //     must be parsed from the MSP-body itself (with respect to MSP version and Jumbo-frame).
  //   The last/only CRSF-frame might be longer than needed. In such a case, the extra bytes must be ignored.
  //   Maximum chunk size is defined by maximum length of CRSF frame 64 bytes, therefore, maximum MSP-chunk length is 57
  //   bytes.
  //     Minimum chunk length might by anything, but the first chunk must consist of size and function ID (i.e., 5 bytes
  //     for MSPv2).
  //   CRC of the MSP frame is not sent because it’s already protected by CRC of CRSF. If MSP CRC is needed,
  //     it should be calculated at the receiving point.
  //   MSP-response must be sent to the origin of the MSP-request. It means that [destination] and [origin] bytes of
  //   CRSF-header
  //     in response must be the same as in request but swapped.
  // see https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/msp_shared.c#L175
  //     https://github.com/betaflight/betaflight/blob/538564bbe3eb226d5eac2e3387401bca7c6fdb90/src/main/telemetry/msp_shared.c#L175

  // frame: <sync><size><type>[<dst><origin><flags>{msp}]<crc>
  // uint8_t dst = msg.payload[0];
  origin = frame.payload[1];
  uint8_t status = frame.payload[2];

  uint8_t sequence = (status & CRSF_MSP_STATUS_SEQ_MASK);         // 00001111
  uint8_t start = (status & CRSF_MSP_STATUS_START_MASK) >> 4;     // 00010000
  uint8_t version = (status & CRSF_MSP_STATUS_VERSION_MASK) >> 5; // 01100000
  // uint8_t error    = (status & CRSF_MSP_STATUS_ERROR_MASK) >> 7; // 10000000

  if (start)
  {
    // reset message on start
    m.state = Connect::MSP_STATE_IDLE;
    m.received = 0;
    m.expected = 0;
    if (version == 1)
    {
      fillMessage<Connect::MspHeaderV1>(frame, m, Connect::MSP_V1);
    }
    else if (version == 2)
    {
      fillMessage<Connect::MspHeaderV2>(frame, m, Connect::MSP_V2);
    }
  }
  else
  {
    // next chunks - continuation of fragmented message
    if (sequence == ((m.sequence + 1) & CRSF_MSP_STATUS_SEQ_MASK))
    {
      size_t framePayloadSize = std::min(frame.size - 5, m.expected - m.received); // skip dst, origin, status;
      if (m.received + framePayloadSize <= Connect::MSP_BUF_SIZE)
      {
        m.append(frame.payload + 3, framePayloadSize);
        if (m.received == m.expected)
        {
          m.state = Connect::MSP_STATE_RECEIVED;
        }
      }
    }
    else
    {
      // sequence mismatch - reset message
      m.state = Connect::MSP_STATE_IDLE;
      m.received = 0;
      m.expected = 0;
    }
  }

  m.sequence = sequence;

  return m.state == Connect::MSP_STATE_RECEIVED ? 1 : 0;
}

uint16_t Crsf::convert(int v)
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
  // return lrintf((0.62477120195241 * (float)v) + 880.54);
  // return Utils::map(v, 172, 1811, 988, 2012);
  // return Utils::mapi(v, 172, 1811, 988, 2012);
}

uint8_t Crsf::crc(const CrsfMessage& msg)
{
  // CRC includes type and payload
  uint8_t crc = Utils::crc8_dvb_s2(0, msg.type);
  // size includes type and crc
  for (int i = 0; i < msg.size - 2; i++)
  {
    crc = Utils::crc8_dvb_s2(crc, msg.payload[i]);
  }
  return crc;
}

} // namespace Espfc::Rc
