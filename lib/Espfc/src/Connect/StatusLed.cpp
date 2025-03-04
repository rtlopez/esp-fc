#include "StatusLed.hpp"
#include "Target/Target.h"
#include <Arduino.h>

#ifdef ESPFC_LED_WS2812
#include "driver/i2s.h"

static constexpr size_t LED_NUMBER = 1;
static constexpr size_t PIXEL_SIZE = 12; // each colour takes 4 bytes
static constexpr uint32_t SAMPLE_RATE = 93750;
static constexpr size_t ZERO_BUFFER = 48;
static constexpr i2s_port_t I2S_NUM = I2S_NUM_0;

typedef struct {
  uint8_t green;
  uint8_t red;
  uint8_t blue;
} ws2812_pixel_t;

static i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_MSB,
  .intr_alloc_flags = 0,
  .dma_buf_count = 4,
  .dma_buf_len = LED_NUMBER * PIXEL_SIZE,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0,
  .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
  .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
};

static i2s_pin_config_t pin_config = {
  .bck_io_num = -1,
  .ws_io_num = -1,
  .data_out_num = -1,
  .data_in_num = -1
};

static uint8_t out_buffer[LED_NUMBER * PIXEL_SIZE] = {0};
static uint8_t off_buffer[ZERO_BUFFER] = {0};
static uint16_t size_buffer = LED_NUMBER * PIXEL_SIZE;

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

static void ws2812_init(int8_t pin)
{
  pin_config.data_out_num = pin;
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
}

static void ws2812_update(const ws2812_pixel_t * pixels)
{
  size_t bytes_written = 0;

  for (uint16_t i = 0; i < LED_NUMBER; i++) {
    int loc = i * PIXEL_SIZE;

    out_buffer[loc]     = bitpatterns[pixels[i].green >> 6 & 0x03];
    out_buffer[loc + 1] = bitpatterns[pixels[i].green >> 4 & 0x03];
    out_buffer[loc + 2] = bitpatterns[pixels[i].green >> 2 & 0x03];
    out_buffer[loc + 3] = bitpatterns[pixels[i].green & 0x03];

    out_buffer[loc + 4] = bitpatterns[pixels[i].red >> 6 & 0x03];
    out_buffer[loc + 5] = bitpatterns[pixels[i].red >> 4 & 0x03];
    out_buffer[loc + 6] = bitpatterns[pixels[i].red >> 2 & 0x03];
    out_buffer[loc + 7] = bitpatterns[pixels[i].red & 0x03];

    out_buffer[loc + 8]  = bitpatterns[pixels[i].blue >> 6 & 0x03];
    out_buffer[loc + 9]  = bitpatterns[pixels[i].blue >> 4 & 0x03];
    out_buffer[loc + 10] = bitpatterns[pixels[i].blue >> 2 & 0x03];
    out_buffer[loc + 11] = bitpatterns[pixels[i].blue & 0x03];
  }

  i2s_write(I2S_NUM, out_buffer, size_buffer, &bytes_written, portMAX_DELAY);
  i2s_write(I2S_NUM, off_buffer, ZERO_BUFFER, &bytes_written, portMAX_DELAY);
  delay(1); // was 10
  i2s_zero_dma_buffer(I2S_NUM);
}

static const ws2812_pixel_t PIXEL_ON[] = {{127, 0, 0}};
static const ws2812_pixel_t PIXEL_OFF[] = {{0, 0, 0}};

#endif

// https://github.com/vunam/esp32-i2s-ws2812/blob/master/ws2812.c

namespace Espfc::Connect
{

static int LED_OFF_PATTERN[] = {0};
static int LED_OK_PATTERN[] = {100, 900, 0};
static int LED_ERROR_PATTERN[] = {100, 100, 100, 100, 100, 1500, 0};
static int LED_ON_PATTERN[] = {100, 0};

StatusLed::StatusLed() : _pin(-1), _invert(0), _status(LED_OFF), _next(0), _state(LOW), _step(0), _pattern(LED_OFF_PATTERN) {}

void StatusLed::begin(int8_t pin, uint8_t invert)
{
  if(pin == -1) return;
  _pin = pin;
  _invert = invert;

#ifdef ESPFC_LED_WS2812
  ws2812_init(_pin);
#else
  pinMode(_pin, OUTPUT);
#endif
  setStatus(LED_ON, true);
}

void StatusLed::setStatus(LedStatus newStatus, bool force)
{
  if(_pin == -1) return;
  if(!force && newStatus == _status) return;

  _status = newStatus;
  _state = LOW;
  _step = 0;
  _next = millis();

  switch (_status)
  {
    case LED_OK:
      _pattern = LED_OK_PATTERN;
      break;
    case LED_ERROR:
      _pattern = LED_ERROR_PATTERN;
      break;
    case LED_ON:
      _pattern = LED_ON_PATTERN;
      _state = HIGH;
      break;
    case LED_OFF:
    default:
      _pattern = LED_OFF_PATTERN;
      break;
  }
  _write(_state);
}

void StatusLed::update()
{
  if(_pin == -1 || !_pattern) return;
  
  uint32_t now = millis();
  
  if(now < _next) return;

  if (!_pattern[_step])
  {
    _step = 0;
    _next = now + 20;
    return;
  }

  _state = !(_step & 1);
  _write(_state);

  _next = now + _pattern[_step];
  _step++;
}

void StatusLed::_write(uint8_t val)
{
#ifdef ESPFC_LED_WS2812
  ws2812_update(val ? PIXEL_ON : PIXEL_OFF);
#else
  digitalWrite(_pin, val ^ _invert);
#endif
}

}
