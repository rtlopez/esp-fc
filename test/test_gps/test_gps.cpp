// test/test_gps/test_gps.cpp
// Tests for GPS distance and bearing math used in GpsSensor::calculateHomeVector()

#include <Gps.hpp>
#include <GpsParser.hpp>
#include <cmath>
#include <cstdint>
#include <unity.h>

static constexpr float toRad(float deg)
{
  return deg * (float)M_PI / 180.0f;
}

// ---------------------------------------------------------------------------

void test_distance_north()
{
  // 0.1 degree north from equator ≈ 11130 m (within uint16_t range)
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 1000000, 0);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 11130.0f, distance);
}

void test_distance_zero_at_same_position()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(377490000, -1224194000, 377490000, -1224194000);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, distance);
}

void test_bearing_north()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 10000000, 0);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, bearing);
}

void test_bearing_east()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 0, 10000000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(90.0f), bearing);
}

void test_bearing_south()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, -10000000, 0);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(180.0f), bearing);
}

void test_bearing_west()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 0, -10000000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(270.0f), bearing);
}

void test_bearing_northeast()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 10000000, 10000000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(45.0f), bearing);
}

void test_bearing_wraps_360()
{
  // West is 270°, not -90°
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 0, -5000000);
  TEST_ASSERT_TRUE(bearing >= 0.0f && bearing <= 2.0f * M_PI);
}

// Date line crossing tests
void test_date_line_crossing_east_to_west()
{
  // Home at 179° East, current at -179° (179° West)
  // Actual shortest distance: ~2° (2° * 111300m/deg ≈ 222600m)
  // Without date line handling, would calculate 358° (40067000m) - WRONG!
  const int32_t homeLon = 1790000000; // 179° * 1e7
  const int32_t curLon = -1790000000; // -179° * 1e7
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);

  // Distance should be ~2° (222600m) not 358° (40000000m+)
  TEST_ASSERT_FLOAT_WITHIN(100.0f, 222600.0f, distance);
}

void test_date_line_crossing_west_to_east()
{
  // Home at -179° (179° West), current at 179° East
  // Actual shortest distance: ~2°
  const int32_t homeLon = -1790000000; // -179° * 1e7
  const int32_t curLon = 1790000000;   // 179° * 1e7
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);

  TEST_ASSERT_FLOAT_WITHIN(100.0f, 222600.0f, distance);
}

void test_date_line_crossing_bearing()
{
  // Home at -179° West, current at 179° East
  // Shortest path: 2° west (across date line from 179°W → 179°E going west/backward)
  // Bearing should be ~270° (west)
  const int32_t homeLon = -1790000000;
  const int32_t curLon = 1790000000;
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(270.0f), bearing);
}

// ---------------------------------------------------------------------------
// UBX parser

static void ubxFeed(Gps::UbxParser& parser, Gps::UbxMessage& m, const uint8_t* data, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    parser.parse(data[i], m);
  }
}

static size_t ubxBuildFrame(uint8_t* out, uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
  size_t n = 0;
  out[n++] = Gps::UBX_SYNC0;
  out[n++] = Gps::UBX_SYNC1;
  out[n++] = cls;
  out[n++] = id;
  out[n++] = len & 0xff;
  out[n++] = len >> 8;
  for (uint16_t i = 0; i < len; i++)
  {
    out[n++] = payload[i];
  }
  const uint16_t crc = Gps::ubxChecksum(0, out + 2, n - 2);
  out[n++] = crc & 0xff;
  out[n++] = crc >> 8;
  return n;
}

void test_ubx_parse_ack_ack()
{
  // ACK-ACK for CFG-PRT
  const uint8_t frame[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x37};
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, frame, sizeof(frame));

  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_TRUE(m.isAck());
  TEST_ASSERT_EQUAL_UINT16(Gps::UBX_ACK_ACK, m.msgId);
  TEST_ASSERT_EQUAL_UINT16(2, m.length);
  TEST_ASSERT_EQUAL_UINT16(2, m.written);
  TEST_ASSERT_EQUAL_HEX8(0x06, m.payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, m.payload[1]);
  TEST_ASSERT_EQUAL_HEX16(0x370E, m.crc);
}

void test_ubx_parse_states()
{
  const uint8_t frame[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x37};
  const Gps::UbxState expected[] = {
      Gps::UBX_STATE_SYNC,    Gps::UBX_STATE_CLASS,   Gps::UBX_STATE_ID,   Gps::UBX_STATE_LEN0, Gps::UBX_STATE_LEN1,
      Gps::UBX_STATE_PAYLOAD, Gps::UBX_STATE_PAYLOAD, Gps::UBX_STATE_CRC0, Gps::UBX_STATE_CRC1, Gps::UBX_STATE_READY,
  };
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  for (size_t i = 0; i < sizeof(frame); i++)
  {
    parser.parse(frame[i], m);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected[i], m.status, "state mismatch");
  }
}

void test_ubx_parse_empty_payload()
{
  uint8_t frame[16];
  const size_t len = ubxBuildFrame(frame, Gps::UBX_MON, 0x04, nullptr, 0);
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, frame, len);

  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_TRUE(m.isResponse(Gps::UBX_MON_VER));
  TEST_ASSERT_EQUAL_UINT16(0, m.length);
}

void test_ubx_parse_skip_garbage_before_sync()
{
  const uint8_t garbage[] = {0x00, 0xFF, 0x62, '$', 0xB5, 0x00};
  const uint8_t frame[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x37};
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, garbage, sizeof(garbage));
  TEST_ASSERT_TRUE(m.isIdle());

  ubxFeed(parser, m, frame, sizeof(frame));
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_TRUE(m.isAck());
}

void test_ubx_parse_bad_sync1()
{
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  parser.parse(Gps::UBX_SYNC0, m);
  TEST_ASSERT_EQUAL_UINT8(Gps::UBX_STATE_SYNC, m.status);
  parser.parse(0x00, m);
  TEST_ASSERT_TRUE(m.isIdle());
}

void test_ubx_parse_bad_checksum()
{
  const uint8_t frame[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x38};
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, frame, sizeof(frame));

  TEST_ASSERT_FALSE(m.isReady());
  TEST_ASSERT_TRUE(m.isIdle());
}

void test_ubx_parse_length_too_big()
{
  const uint8_t header[] = {0xB5, 0x62, 0x01, 0x07, 0x01, 0x02};
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, header, sizeof(header));

  TEST_ASSERT_TRUE(m.isIdle());
}

void test_ubx_parse_max_payload()
{
  static uint8_t payload[512];
  static uint8_t frame[512 + 8];
  for (size_t i = 0; i < sizeof(payload); i++) payload[i] = i & 0xff;
  const size_t len = ubxBuildFrame(frame, Gps::UBX_NAV, 0x35, payload, sizeof(payload));
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, frame, len);

  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_UINT16(512, m.written);
  TEST_ASSERT_EQUAL_HEX8(payload[511], m.payload[511]);
}

void test_ubx_parse_recover_after_bad_checksum()
{
  const uint8_t bad[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x38};
  const uint8_t good[] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x06, 0x01, 0x0E, 0x33};
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, bad, sizeof(bad));
  TEST_ASSERT_TRUE(m.isIdle());

  ubxFeed(parser, m, good, sizeof(good));
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_TRUE(m.isNak());
  TEST_ASSERT_EQUAL_UINT16(2, m.written);
  TEST_ASSERT_EQUAL_HEX8(0x06, m.payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x01, m.payload[1]);
}

void test_ubx_parse_consecutive_frames()
{
  const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  uint8_t frame[32];
  const size_t len = ubxBuildFrame(frame, Gps::UBX_NAV, 0x07, payload, sizeof(payload));
  Gps::UbxParser parser;
  Gps::UbxMessage m;

  ubxFeed(parser, m, frame, len);
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_UINT16(Gps::UBX_NAV_PVT, m.msgId);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, m.payload, sizeof(payload));

  // without explicit message reset, as after checksum failure
  ubxFeed(parser, m, frame, len);
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_UINT16(sizeof(payload), m.written);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, m.payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// NMEA parser

static void nmeaFeed(Gps::NmeaParser& parser, Gps::NmeaMessage& m, const char* data)
{
  while (*data)
  {
    parser.parse((uint8_t)*data++, m);
  }
}

void test_nmea_parse_sentence()
{
  Gps::NmeaParser parser;
  Gps::NmeaMessage m;

  nmeaFeed(parser, m, "$GNTXT,01,01,01,PF=3E8*59\r\n");

  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_STRING("GNTXT,01,01,01,PF=3E8*59", m.payload);
  TEST_ASSERT_EQUAL(24, m.length);
}

void test_nmea_parse_states()
{
  Gps::NmeaParser parser;
  Gps::NmeaMessage m;

  parser.parse('x', m);
  TEST_ASSERT_TRUE(m.isIdle());
  parser.parse('$', m);
  TEST_ASSERT_EQUAL_UINT8(Gps::NMEA_STATE_PAYLOAD, m.status);
  parser.parse('A', m);
  TEST_ASSERT_EQUAL_UINT8(Gps::NMEA_STATE_PAYLOAD, m.status);
  parser.parse('\r', m);
  TEST_ASSERT_EQUAL_UINT8(Gps::NMEA_STATE_END, m.status);
  parser.parse('\n', m);
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_STRING("A", m.payload);
}

void test_nmea_parse_skip_garbage_before_start()
{
  Gps::NmeaParser parser;
  Gps::NmeaMessage m;

  nmeaFeed(parser, m, "garbage\r\n");
  TEST_ASSERT_TRUE(m.isIdle());
  TEST_ASSERT_EQUAL(0, m.length);

  nmeaFeed(parser, m, "$GPGGA\r\n");
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_STRING("GPGGA", m.payload);
}

void test_nmea_parse_lf_only_resets_payload()
{
  Gps::NmeaParser parser;
  Gps::NmeaMessage m;

  nmeaFeed(parser, m, "$GPGSV,1\n");
  TEST_ASSERT_TRUE(m.isIdle());

  nmeaFeed(parser, m, "$GNTXT\r\n");
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_STRING("GNTXT", m.payload);
  TEST_ASSERT_EQUAL(5, m.length);
}

void test_nmea_parse_consecutive_sentences()
{
  Gps::NmeaParser parser;
  Gps::NmeaMessage m;

  nmeaFeed(parser, m, "$GPGGA,123\r\n");
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_STRING("GPGGA,123", m.payload);

  nmeaFeed(parser, m, "$GPRMC\r\n");
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_STRING("GPRMC", m.payload);
}

void test_nmea_parse_payload_overflow()
{
  Gps::NmeaParser parser;
  Gps::NmeaMessage m;

  parser.parse('$', m);
  for (size_t i = 0; i < 600; i++)
  {
    parser.parse('A', m);
  }
  TEST_ASSERT_EQUAL(511, m.length);
  TEST_ASSERT_EQUAL_CHAR('\0', m.payload[511]);

  nmeaFeed(parser, m, "\r\n");
  TEST_ASSERT_TRUE(m.isReady());
}

// ---------------------------------------------------------------------------

int main()
{
  UNITY_BEGIN();

  RUN_TEST(test_distance_north);
  RUN_TEST(test_distance_zero_at_same_position);
  RUN_TEST(test_bearing_north);
  RUN_TEST(test_bearing_east);
  RUN_TEST(test_bearing_south);
  RUN_TEST(test_bearing_west);
  RUN_TEST(test_bearing_northeast);
  RUN_TEST(test_bearing_wraps_360);
  RUN_TEST(test_date_line_crossing_east_to_west);
  RUN_TEST(test_date_line_crossing_west_to_east);
  RUN_TEST(test_date_line_crossing_bearing);

  RUN_TEST(test_ubx_parse_ack_ack);
  RUN_TEST(test_ubx_parse_states);
  RUN_TEST(test_ubx_parse_empty_payload);
  RUN_TEST(test_ubx_parse_skip_garbage_before_sync);
  RUN_TEST(test_ubx_parse_bad_sync1);
  RUN_TEST(test_ubx_parse_bad_checksum);
  RUN_TEST(test_ubx_parse_length_too_big);
  RUN_TEST(test_ubx_parse_max_payload);
  RUN_TEST(test_ubx_parse_recover_after_bad_checksum);
  RUN_TEST(test_ubx_parse_consecutive_frames);

  RUN_TEST(test_nmea_parse_sentence);
  RUN_TEST(test_nmea_parse_states);
  RUN_TEST(test_nmea_parse_skip_garbage_before_start);
  RUN_TEST(test_nmea_parse_lf_only_resets_payload);
  RUN_TEST(test_nmea_parse_consecutive_sentences);
  RUN_TEST(test_nmea_parse_payload_overflow);

  return UNITY_END();
}
