#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Utils {

uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t a);
uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t* data, size_t len);
uint8_t crc8_xor(uint8_t checksum, const uint8_t a);
uint8_t crc8_xor(uint8_t checksum, const uint8_t* data, size_t len);

} // namespace Espfc::Utils
