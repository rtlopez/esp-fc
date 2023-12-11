#pragma once

namespace Espfc {

namespace Math {

  inline bool getBit(uint8_t data, uint8_t bitNum)
  {
    return data & (1 << bitNum);
  }

  inline uint8_t setBit(uint8_t data, uint8_t bitNum, bool bitVal)
  {
    data = (bitVal != 0) ? (data | (1 << bitNum)) : (data & ~(1 << bitNum));
    return data;
  }

  inline uint8_t getMaskMsb(uint8_t bitStart, uint8_t bitLen)
  {
    uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
    return mask;
  }

  inline uint8_t setMasked(uint8_t data, uint8_t mask, uint8_t newData)
  {
    newData &= mask; // zero all non-important bits in data
    data &= ~(mask); // zero all important bits in existing byte
    data |= newData; // combine data with existing byte
    return data;
  }

  /**
   * I2Cdev::readBits() compatible 
   * Read multiple bits from an 8-bit data
   * @param bitStart First bit position to read (0-7), Most Significant Bit
   * @param bitLen Number of bits to read (no more than 8, no more than start + 1)
   */
  inline uint8_t getBitsMsb(uint8_t data, uint8_t bitStart, uint8_t bitLen)
  {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
    data &= mask;
    data >>= (bitStart - bitLen + 1);
    return data;
  }

  inline uint8_t setBitsMsb(uint8_t data, uint8_t bitStart, uint8_t bitLen, uint8_t bitVal)
  {
    uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
    bitVal <<= (bitStart - bitLen + 1); // shift data into correct position
    bitVal &= mask; // zero all non-important bits in data
    data &= ~(mask); // zero all important bits in existing byte
    data |= bitVal; // combine data with existing byte
    return data;
  }

  inline uint8_t getMaskLsb(uint8_t bitStart, uint8_t bitLen)
  {
    uint8_t mask = ((1 << bitLen) - 1) << bitStart;
    return mask;
  }

  /**
   * Read multiple bits from an 8-bit data
   * @param bitStart Last bit position to read (0-7), Least Significant Bit
   * @param bitLen Number of bits to read (not more than 8 - bitStart)
   */
  inline uint8_t getBitsLsb(uint8_t data, uint8_t bitStart, uint8_t bitLen)
  {
    // 01101001 read byte
    // 76543210 bit numbers
    //   xxx   args: bitStart=4, length=3
    //   101    masked
    //   -> 101 shifted
    uint8_t mask = ((1 << bitLen) - 1);
    data >>= bitStart;
    data &= mask;
    return data;
  }

  inline uint8_t setBitsLsb(uint8_t data, uint8_t bitStart, uint8_t bitLen, uint8_t bitVal)
  {
    uint8_t mask = ((1 << bitLen) - 1) << bitStart;
    bitVal <<= bitStart; // shift data into correct position
    bitVal &= mask; // zero all non-important bits in data
    data &= ~(mask); // zero all important bits in existing byte
    data |= bitVal; // combine data with existing byte
    return data;
  }
}

}
