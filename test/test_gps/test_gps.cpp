// test/test_gps/test_gps.cpp
// Tests for GPS distance and bearing math used in GpsSensor::calculateHomeVector()

#include <unity.h>
#include <cmath>
#include <cstdint>
#include <Gps.hpp>

static constexpr float toRad(float deg) { return deg * (float)M_PI / 180.0f; }

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
  const int32_t homeLon = 1790000000;   // 179° * 1e7
  const int32_t curLon = -1790000000;   // -179° * 1e7
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);
  
  // Distance should be ~2° (222600m) not 358° (40000000m+)
  TEST_ASSERT_FLOAT_WITHIN(100.0f, 222600.0f, distance);
}

void test_date_line_crossing_west_to_east()
{
  // Home at -179° (179° West), current at 179° East
  // Actual shortest distance: ~2°
  const int32_t homeLon = -1790000000;  // -179° * 1e7
  const int32_t curLon = 1790000000;    // 179° * 1e7
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

  return UNITY_END();
}
