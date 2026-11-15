/*
  TwoWire.cpp - TWI/I2C library for Arduino & Wiring
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

#if defined(ESP32) || defined(ESP8266)

#include "Hal/Detail/EspWire.hpp"
#include "esp_twi.hpp"
#include <Arduino.h>

// Initialize Class Variables //////////////////////////////////////////////////

// Constructors ////////////////////////////////////////////////////////////////

// Public Methods //////////////////////////////////////////////////////////////

void EspTwoWire::begin(int sda, int scl, uint32_t frequency)
{
  esp_twi_init(sda, scl);
  if (frequency) setClock(frequency);
  flush();
}

uint8_t EspTwoWire::status()
{
  return esp_twi_status();
}

void EspTwoWire::setClock(uint32_t frequency)
{
  esp_twi_setClock(frequency);
}

void EspTwoWire::setClockStretchLimit(uint32_t limit)
{
  esp_twi_setClockStretchLimit(limit);
}

size_t IRAM_ATTR EspTwoWire::requestFrom(uint8_t address, size_t size, bool sendStop)
{
  if (size > ESPWIRE_BUFFER_LENGTH)
  {
    size = ESPWIRE_BUFFER_LENGTH;
  }
  size_t read = (esp_twi_readFrom(address, rxBuffer, size, sendStop) == 0) ? size : 0;
  rxBufferIndex = 0;
  rxBufferLength = read;
  return read;
}

uint8_t IRAM_ATTR EspTwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  return requestFrom(address, static_cast<size_t>(quantity), static_cast<bool>(sendStop));
}

uint8_t IRAM_ATTR EspTwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom(address, static_cast<size_t>(quantity), true);
}

uint8_t EspTwoWire::requestFrom(int address, int quantity)
{
  return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), true);
}

uint8_t EspTwoWire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), static_cast<bool>(sendStop));
}

void IRAM_ATTR EspTwoWire::beginTransmission(uint8_t address)
{
  transmitting = 1;
  txAddress = address;
  txBufferIndex = 0;
  txBufferLength = 0;
}

void IRAM_ATTR EspTwoWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

uint8_t IRAM_ATTR EspTwoWire::endTransmission(uint8_t sendStop)
{
  int8_t ret = esp_twi_writeTo(txAddress, txBuffer, txBufferLength, sendStop);
  txBufferIndex = 0;
  txBufferLength = 0;
  transmitting = 0;
  return ret;
}

uint8_t IRAM_ATTR EspTwoWire::endTransmission(void)
{
  return endTransmission(true);
}

size_t IRAM_ATTR EspTwoWire::write(uint8_t data)
{
  if (transmitting)
  {
    if (txBufferLength >= ESPWIRE_BUFFER_LENGTH)
    {
      return 0;
    }
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    txBufferLength = txBufferIndex;
  }
  else
  {
    // i2c_slave_transmit(&data, 1);
  }
  return 1;
}

size_t IRAM_ATTR EspTwoWire::write(const uint8_t* data, size_t quantity)
{
  if (transmitting)
  {
    for (size_t i = 0; i < quantity; ++i)
    {
      if (!write(data[i])) return i;
    }
  }
  else
  {
    // i2c_slave_transmit(data, quantity);
  }
  return quantity;
}

int EspTwoWire::available(void)
{
  int result = rxBufferLength - rxBufferIndex;

  if (!result)
  {
// yielding here will not make more data "available",
// but it will prevent the system from going into WDT reset
#if !defined(UNIT_TEST) && !defined(ARCH_RP2040)
    optimistic_yield(1000);
#endif
  }

  return result;
}

int EspTwoWire::read(void)
{
  int value = -1;
  if (rxBufferIndex < rxBufferLength)
  {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }
  return value;
}

int EspTwoWire::peek(void)
{
  int value = -1;
  if (rxBufferIndex < rxBufferLength)
  {
    value = rxBuffer[rxBufferIndex];
  }
  return value;
}

void EspTwoWire::flush(void)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
EspTwoWire EspWire;
#endif

#endif
