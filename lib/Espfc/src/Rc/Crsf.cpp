
#include "Crsf.h"
#include "Math/Utils.h"
#include "Math/Crc.h"
#include "Utils/MemoryHelper.h"
#include <cstring>

namespace Espfc {

namespace Rc {

void FAST_CODE_ATTR Crsf::decodeRcData(uint16_t* channels, const CrsfData* frame)
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

void FAST_CODE_ATTR Crsf::decodeRcDataShift8(uint16_t* channels, const CrsfData* frame)
{
  // 8-bit
  // 0....... ...1.... ......2. ........ .3...... ....4... .......5 ........ ..6..... .....7.. ........ 
  // 8....... ...9.... ......A. ........ .B...... ....C... .......D ........ ..E..... .....F.. ........ 
  const uint8_t * crsfData = reinterpret_cast<const uint8_t *>(frame);
  channels[0]  = convert((crsfData[0]       | crsfData[1]  << 8) & 0x07FF);
  channels[1]  = convert((crsfData[1]  >> 3 | crsfData[2]  << 5) & 0x07FF);
  channels[2]  = convert((crsfData[2]  >> 6 | crsfData[3]  << 2 | crsfData[4] << 10) & 0x07FF);
  channels[3]  = convert((crsfData[4]  >> 1 | crsfData[5]  << 7) & 0x07FF);

  channels[4]  = convert((crsfData[5]  >> 4 | crsfData[6]  << 4) & 0x07FF);
  channels[5]  = convert((crsfData[6]  >> 7 | crsfData[7]  << 1 | crsfData[8] << 9)  & 0x07FF);
  channels[6]  = convert((crsfData[8]  >> 2 | crsfData[9]  << 6) & 0x07FF);
  channels[7]  = convert((crsfData[9]  >> 5 | crsfData[10] << 3) & 0x07FF);

  channels[8]  = convert((crsfData[11]      | crsfData[12] << 8) & 0x07FF);
  channels[9]  = convert((crsfData[12] >> 3 | crsfData[13] << 5) & 0x07FF);
  channels[10] = convert((crsfData[13] >> 6 | crsfData[14] << 2 | crsfData[15] << 10) & 0x07FF);
  channels[11] = convert((crsfData[15] >> 1 | crsfData[16] << 7) & 0x07FF);

  channels[12] = convert((crsfData[16] >> 4 | crsfData[17] << 4) & 0x07FF);
  channels[13] = convert((crsfData[17] >> 7 | crsfData[18] << 1 | crsfData[19] << 9)  & 0x07FF);
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

void Crsf::encodeRcData(CrsfFrame& frame, const CrsfData& data)
{
  frame.message.addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame.message.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
  frame.message.size = sizeof(data) + 2;
  std::memcpy(frame.message.payload, (void*)&data, sizeof(data));
  frame.message.payload[sizeof(data)] = crc(frame);
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
  //return lrintf((0.62477120195241 * (float)v) + 880.54);
  //return Math::map(v, 172, 1811, 988, 2012);
  //return Math::mapi(v, 172, 1811, 988, 2012);
}

uint8_t Crsf::crc(const CrsfFrame& frame)
{
  // CRC includes type and payload
  uint8_t crc = Math::crc8_dvb_s2(0, frame.message.type);
  for (int i = 0; i < frame.message.size - 2; i++) { // size includes type and crc
      crc = Math::crc8_dvb_s2(crc, frame.message.payload[i]);
  }
  return crc;
}

}

}
