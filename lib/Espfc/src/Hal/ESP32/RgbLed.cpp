#if defined(ESP32)

#include "Hal/RgbLed.hpp"
#include "driver/i2s.h"
#include <algorithm>

// https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32/api-reference/peripherals/i2s.html
// https://github.com/vunam/esp32-i2s-ws2812/blob/master/ws2812.c

namespace Espfc::Hal {

namespace {

constexpr size_t LED_NUMBER = 1;
constexpr size_t PIXEL_SIZE = 12; // each colour takes 4 bytes in buffer
constexpr size_t ZERO_BUFFER = 32;
constexpr size_t SIZE_BUFFER = LED_NUMBER * PIXEL_SIZE + ZERO_BUFFER;
constexpr uint32_t SAMPLE_RATE = 93750;
constexpr i2s_port_t I2S_NUM = I2S_NUM_0;

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 2,
    .dma_buf_len = SIZE_BUFFER / 2,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
    .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
};

i2s_pin_config_t pin_config = {.bck_io_num = -1, .ws_io_num = -1, .data_out_num = -1, .data_in_num = -1};

uint8_t out_buffer[SIZE_BUFFER] = {0};

const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

void ws2812_init(int8_t pin)
{
  pin_config.data_out_num = pin;
  i2s_driver_install(I2S_NUM, &i2s_config, 0, nullptr);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  std::fill_n(out_buffer, SIZE_BUFFER, 0);
}

void ws2812_write_pixel(uint8_t* buffer, const RgbColor& pixel)
{
  *buffer++ = bitpatterns[pixel.g >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.r >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.b >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 0 & 0x03];
}

void ws2812_update(const RgbColor& color)
{
  size_t bytes_written = 0;
  for (size_t i = 0; i < LED_NUMBER; i++)
  {
    ws2812_write_pixel(out_buffer + i * PIXEL_SIZE, color);
  }
  i2s_zero_dma_buffer(I2S_NUM);
  i2s_write(I2S_NUM, out_buffer, SIZE_BUFFER, &bytes_written, portMAX_DELAY);
}

} // namespace

void RgbLed::begin(int8_t pin)
{
  if (pin == -1) return;

  _pin = pin;
  ws2812_init(_pin);
}

void RgbLed::write(const RgbColor& color)
{
  if (_pin == -1) return;

  ws2812_update(color);
}

} // namespace Espfc::Hal

#endif
