#pragma once

/*
  EspTwoWire.hpp - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified December 2014 by Ivan Grokhotkov (ivan@esp8266.com) - esp8266 support
  Modified April 2015 by Hrsto Gochkov (ficeto@ficeto.com) - alternative esp8266 support
*/

#include <cstddef>
#include <cstdint>

class EspTwoWire
{
public:
  void begin(int sda, int scl, uint32_t frequency = 0);
  void setClock(uint32_t);
  void setClockStretchLimit(uint32_t);
  void beginTransmission(uint8_t);
  void beginTransmission(int);
  uint8_t endTransmission(void);
  uint8_t endTransmission(uint8_t);
  size_t requestFrom(uint8_t address, size_t size, bool sendStop);
  uint8_t status();

  uint8_t requestFrom(uint8_t, uint8_t);
  uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
  uint8_t requestFrom(int, int);
  uint8_t requestFrom(int, int, int);

  size_t write(uint8_t);
  size_t write(const uint8_t*, size_t);
  int available(void);
  int read(void);
  int peek(void);
  void flush();

  inline size_t write(unsigned long n)
  {
    return write((uint8_t)n);
  }

  inline size_t write(long n)
  {
    return write((uint8_t)n);
  }

  inline size_t write(unsigned int n)
  {
    return write((uint8_t)n);
  }

  inline size_t write(int n)
  {
    return write((uint8_t)n);
  }

private:
  static constexpr size_t ESPWIRE_BUFFER_LENGTH = 64;

  uint8_t rxBuffer[ESPWIRE_BUFFER_LENGTH]{};
  size_t rxBufferIndex = 0;
  size_t rxBufferLength = 0;

  uint8_t txAddress = 0;
  uint8_t txBuffer[ESPWIRE_BUFFER_LENGTH]{};
  size_t txBufferIndex = 0;
  size_t txBufferLength = 0;

  uint8_t transmitting = 0;
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
extern EspTwoWire EspWire;
#endif
