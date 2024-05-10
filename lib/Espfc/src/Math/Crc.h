#pragma once

#include <cstdint>
#include <cstddef>

namespace Espfc {

namespace Math {

uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t a);
uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t *data, size_t len);
uint8_t crc8_xor(uint8_t checksum, const uint8_t a);
uint8_t crc8_xor(uint8_t checksum, const uint8_t *data, int len);

}

}
