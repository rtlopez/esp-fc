/*
  si2c.c - Software I2C library for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

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
*/
#include "esp_twi.h"
#if defined(ESP32C3)
  #include <soc/cpu.h>
#endif
#ifndef UNIT_TEST
#include <pins_arduino.h>
#include <wiring_private.h>
#else
#endif

uint16_t esp_twi_dcount = 18;
static unsigned char esp_twi_sda, esp_twi_scl;
static uint32_t esp_twi_clockStretchLimit;

#if defined(ESP8266)
  #define SDA_LOW()   (GPES = (1 << esp_twi_sda)) //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
  #define SDA_HIGH()  (GPEC = (1 << esp_twi_sda)) //Disable SDA (becomes input and since it has pullup it will go high)
  #define SDA_READ()  ((GPI & (1 << esp_twi_sda)) != 0)
  #define SCL_LOW()   (GPES = (1 << esp_twi_scl))
  #define SCL_HIGH()  (GPEC = (1 << esp_twi_scl))
  #define SCL_READ()  ((GPI & (1 << esp_twi_scl)) != 0)
#elif defined(ESP32C3)
  //#define SDA_LOW()   (GPIO.enable_w1ts.enable_w1ts = ((uint32_t) 1 << esp_twi_sda)) //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
  static inline void SDA_LOW() { GPIO.enable_w1ts.enable_w1ts = (uint32_t) 1 << esp_twi_sda; GPIO.out_w1tc.out_w1tc = (uint32_t) 1 << esp_twi_sda; }
  #define SDA_HIGH()  (GPIO.enable_w1tc.enable_w1tc = ((uint32_t) 1 << esp_twi_sda)) //Disable SDA (becomes input and since it has pullup it will go high)
  #define SDA_READ()  ((GPIO.in.val & ((uint32_t) 1 << esp_twi_sda)) != 0)
  #define SCL_LOW()   (GPIO.enable_w1ts.enable_w1ts = ((uint32_t) 1 << esp_twi_scl))
  #define SCL_HIGH()  (GPIO.enable_w1tc.enable_w1tc = ((uint32_t) 1 << esp_twi_scl))
  #define SCL_READ()  ((GPIO.in.val & ((uint32_t) 1 << esp_twi_scl)) != 0)
#elif defined(ESP32)
  //#define SDA_LOW()   (GPIO.enable_w1ts = (1 << esp_twi_sda)) //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
  static inline void SDA_LOW() { GPIO.enable_w1ts = 1 << esp_twi_sda; GPIO.out_w1tc = 1 << esp_twi_sda; }
  #define SDA_HIGH()  (GPIO.enable_w1tc = (1 << esp_twi_sda)) //Disable SDA (becomes input and since it has pullup it will go high)
  #define SDA_READ()  ((GPIO.in & (1 << esp_twi_sda)) != 0)
  #define SCL_LOW()   (GPIO.enable_w1ts = (1 << esp_twi_scl))
  #define SCL_HIGH()  (GPIO.enable_w1tc = (1 << esp_twi_scl))
  #define SCL_READ()  ((GPIO.in & (1 << esp_twi_scl)) != 0)
#elif defined(ARCH_RP2040)
  #define SDA_LOW()   (pinMode(esp_twi_sda, OUTPUT)) //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
  #define SDA_HIGH()  (pinMode(esp_twi_sda, INPUT_PULLUP))  //Disable SDA (becomes input and since it has pullup it will go high)
  #define SDA_READ()  (digitalRead(esp_twi_sda))
  #define SCL_LOW()   (pinMode(esp_twi_scl, OUTPUT))
  #define SCL_HIGH()  (pinMode(esp_twi_scl, INPUT_PULLUP))
  #define SCL_READ()  (digitalRead(esp_twi_scl))
#elif defined(UNIT_TEST)
  #define SDA_LOW()   //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
  #define SDA_HIGH()  //Disable SDA (becomes input and since it has pullup it will go high)
  #define SDA_READ()  (1)
  #define SCL_LOW()   
  #define SCL_HIGH()  
  #define SCL_READ()  (1)
#else
  #error "Unsupported platform"
#endif

#ifndef FCPU80
#define FCPU80 80000000L
#endif

#ifndef FCPU240
#define FCPU240 240000000L
#endif

#ifndef FCPU133
#define FCPU133 133000000L
#endif

#if F_CPU == FCPU80
#define TWI_CLOCK_STRETCH_MULTIPLIER 3
#elif F_CPU == FCPU240
#define TWI_CLOCK_STRETCH_MULTIPLIER 9
#elif F_CPU == FCPU133
#define TWI_CLOCK_STRETCH_MULTIPLIER 5
#else
#define TWI_CLOCK_STRETCH_MULTIPLIER 6
#endif

#if defined(ESP32) || defined(ARCH_RP2040)
#define TWI_PIN_MAX 32
#else
#define TWI_PIN_MAX 15
#endif

static unsigned int esp_twi_getDelay(unsigned int freq){
#if F_CPU == FCPU80
  if(freq < 100000) return  19;     // 100KHz
  else if(freq < 200000) return 8;  // 200KHz
  else if(freq < 300000) return 3;  // 300KHz
  else if(freq < 400000) return 1;  // 400KHz
  else return 0;                    // 400KHz
#elif F_CPU == FCPU240
  if(freq < 50000) return 300;       //  50kHz
  else if(freq < 100000) return 190; // 100kHz
  else if(freq < 150000) return 133; // 150kHz
  else if(freq < 200000) return 65;  // 200kHz
  else if(freq < 250000) return 50;  // 250kHz
  else if(freq < 300000) return 39;  // 300kHz
  else if(freq < 400000) return 27;  // 400kHz
  else if(freq < 500000) return 20;  // 500kHz
  else if(freq < 600000) return 15;  // 600kHz
  else if(freq < 700000) return 11;  // 700kHz
  else if(freq < 800000) return 8;   // 800kHz
  else if(freq < 900000) return 7;   // 900kHz
  else if(freq < 1000000) return 5;  // 1.0MHz
  else if(freq < 1100000) return 4;  // 1.1MHz
  else if(freq < 1200000) return 2;  // 1.2MHz
  else return 1;                     // 1.2MHz
#elif F_CPU == FCPU133 // 133 mhz
  if(freq < 50000) return 32;        // 50KHz
  else if(freq < 100000) return 16;  // 100KHz
  else if(freq < 200000) return 8;
  else if(freq < 400000) return 4;
  else if(freq < 600000) return 2;
  else if(freq < 800000) return 1;
  else return 0;                     // 700KHz
#else // 160 mhz
#if defined(ESP32C3)
  if(freq <= 50000) return 250;        // 50KHz
  else if(freq <= 100000) return 89;  // 100KHz.
  else if(freq <= 200000) return 39;  // 200KHz.
  else if(freq <= 300000) return 23;  // 300KHz.
  else if(freq <= 400000) return 15;  // 400KHz.
  else if(freq <= 500000) return 9;   // 500KHz.
  else if(freq <= 600000) return 6;   // 600KHz.
  else if(freq <= 800000) return 2;   // 800KHz.
  else if(freq <= 1000000) return 1;  // 900KHz.
  else return 0;                      // 1000KHz.
#else
  if(freq < 50000) return 64;        // 50KHz
  else if(freq < 100000) return 32;  // 100KHz
  else if(freq < 200000) return 16;  // 200KHz
  else if(freq < 300000) return 8;   // 300KHz
  else if(freq < 400000) return 5;   // 400KHz
  else if(freq < 500000) return 3;   // 500KHz
  else if(freq < 600000) return 2;   // 600KHz
  else if(freq < 800000) return 1;   // 800KHz
  else return 0;                     // 700KHz
#endif
#endif
}

void esp_twi_setClock(unsigned int freq){
  esp_twi_dcount = esp_twi_getDelay(freq);
}

void esp_twi_setClockStretchLimit(uint32_t limit){
  esp_twi_clockStretchLimit = limit * TWI_CLOCK_STRETCH_MULTIPLIER;
}

void esp_twi_init(unsigned char sda, unsigned char scl){
  if(sda > TWI_PIN_MAX || scl > TWI_PIN_MAX) return;
  esp_twi_sda = sda;
  esp_twi_scl = scl;
  pinMode(esp_twi_sda, INPUT_PULLUP);
  pinMode(esp_twi_scl, INPUT_PULLUP);
  esp_twi_setClock(100000);
  esp_twi_setClockStretchLimit(230); // default value is 230 uS
}

void esp_twi_stop(void){
  pinMode(esp_twi_sda, INPUT);
  pinMode(esp_twi_scl, INPUT);
}

static inline IRAM_ATTR unsigned int _getCycleCount()
{
#if defined(ESP32C3)
  return esp_cpu_get_ccount();
#elif defined(ESP32) || defined(ESP8266)
  unsigned int ccount = 0;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
#elif defined(ARCH_RP2040)
  return rp2040.getCycleCount();
#elif defined(UNIT_TEST)
  return 0;
#endif
}

static inline IRAM_ATTR void esp_twi_delay(unsigned int v)
{
  unsigned int end;
  int maxCount = 200;
  switch(v)
  {
    case 0: break;
    default:
      end = _getCycleCount() + (v << 3); // * 8
      while(_getCycleCount() <= end) {
        if(--maxCount == 0) break; // counter wrap protection
      }
  }
}

static IRAM_ATTR bool esp_twi_write_start(void) {
  SCL_HIGH();
  SDA_HIGH();
  if (SDA_READ() == 0) return false;
  esp_twi_delay(esp_twi_dcount);
  SDA_LOW();
  esp_twi_delay(esp_twi_dcount);
  return true;
}

static IRAM_ATTR bool esp_twi_write_stop(void){
  uint32_t i = 0;
  SCL_LOW();
  SDA_LOW();
  esp_twi_delay(esp_twi_dcount);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < esp_twi_clockStretchLimit); // Clock stretching
  esp_twi_delay(esp_twi_dcount);
  SDA_HIGH();
  esp_twi_delay(esp_twi_dcount);
  return true;
}

static IRAM_ATTR bool esp_twi_write_bit(bool bit) {
  uint32_t i = 0;
  SCL_LOW();
  if (bit) SDA_HIGH();
  else SDA_LOW();
  esp_twi_delay(esp_twi_dcount); //esp_twi_delay(esp_twi_dcount+1);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < esp_twi_clockStretchLimit);// Clock stretching
  esp_twi_delay(esp_twi_dcount);
  return true;
}

static IRAM_ATTR bool esp_twi_read_bit(void) {
  uint32_t i = 0;
  SCL_LOW();
  SDA_HIGH();
  esp_twi_delay(esp_twi_dcount); //esp_twi_delay(esp_twi_dcount+2);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < esp_twi_clockStretchLimit);// Clock stretching
  bool bit = SDA_READ();
  esp_twi_delay(esp_twi_dcount);
  return bit;
}

static IRAM_ATTR bool esp_twi_write_byte(unsigned char byte) {
  unsigned char bit;
  for (bit = 0; bit < 8; bit++) {
    esp_twi_write_bit(byte & 0x80);
    byte <<= 1;
  }
  return !esp_twi_read_bit();//NACK/ACK
}

static IRAM_ATTR unsigned char esp_twi_read_byte(bool nack) {
  unsigned char byte = 0;
  unsigned char bit;
  for (bit = 0; bit < 8; bit++) byte = (byte << 1) | esp_twi_read_bit();
  esp_twi_write_bit(nack);
  return byte;
}

unsigned char IRAM_ATTR esp_twi_writeTo(unsigned char address, unsigned char * buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!esp_twi_write_start()) return 4;//line busy
  if(!esp_twi_write_byte(((address << 1) | 0) & 0xFF)) {
    if (sendStop) esp_twi_write_stop();
    return 2; //received NACK on transmit of address
  }
  for(i=0; i<len; i++) {
    if(!esp_twi_write_byte(buf[i])) {
      if (sendStop) esp_twi_write_stop();
      return 3;//received NACK on transmit of data
    }
  }
  if(sendStop) esp_twi_write_stop();
  i = 0;
  while(SDA_READ() == 0 && (i++) < 10){
    SCL_LOW();
    esp_twi_delay(esp_twi_dcount);
    SCL_HIGH();
    esp_twi_delay(esp_twi_dcount);
  }
  return 0;
}

unsigned char IRAM_ATTR esp_twi_readFrom(unsigned char address, unsigned char* buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!esp_twi_write_start()) return 4;//line busy
  if(!esp_twi_write_byte(((address << 1) | 1) & 0xFF)) {
    if (sendStop) esp_twi_write_stop();
    return 2;//received NACK on transmit of address
  }
  for(i=0; i<(len-1); i++) buf[i] = esp_twi_read_byte(false);
  buf[len-1] = esp_twi_read_byte(true);
  if(sendStop) esp_twi_write_stop();
  i = 0;
  while(SDA_READ() == 0 && (i++) < 10){
    SCL_LOW();
    esp_twi_delay(esp_twi_dcount);
    SCL_HIGH();
    esp_twi_delay(esp_twi_dcount);
  }
  return 0;
}

uint8_t esp_twi_status(){
    if (SCL_READ()==0)     return I2C_SCL_HELD_LOW;       		//SCL held low by another device, no procedure available to recover
    int clockCount = 20;

    while (SDA_READ()==0 && clockCount>0){                      //if SDA low, read the bits slaves have to sent to a max
        esp_twi_read_bit();
        if (SCL_READ()==0) return I2C_SCL_HELD_LOW_AFTER_READ;  //I2C bus error. SCL held low beyond slave clock stretch time
    }

    if (SDA_READ()==0)     return I2C_SDA_HELD_LOW;       		//I2C bus error. SDA line held low by slave/another_master after n bits.

    if(!esp_twi_write_start()) return I2C_SDA_HELD_LOW_AFTER_INIT;  //line busy. SDA again held low by another device. 2nd master?
    else                   return I2C_OK;       				//all ok
}
