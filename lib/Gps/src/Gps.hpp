#pragma once

#include "GpsProtocol.hpp"
#include "GpsParser.hpp"
#include <cmath>
#include <tuple>

namespace Gps {
   
/**
 * Calculates the distance between two GPS coordinates using the equirectangular approximation.
 * valid for short distances < few km
 * @param homeLat Home latitude in degrees * 1e7
 * @param homeLon Home longitude in degrees * 1e7
 * @param curLat Current latitude in degrees * 1e7
 * @param curLon Current longitude in degrees * 1e7
 * @return Tuple of (distance, bearing) where distance is in meters and bearing is in radians (0-2PI)
 */
inline std::tuple<float, float> calculateDistanceAndBearing(int32_t homeLat, int32_t homeLon, int32_t curLat, int32_t curLon)
{
  static constexpr float LAT_TO_M = 1.113e-2f; // deg*1e-7 → meters (111300 m/deg / 1e7)
  static constexpr int64_t LON_180 = 1800000000LL;   // 180 * 1e7
  static constexpr int64_t LON_360 = 3600000000LL;   // 360 * 1e7
  
  // Use int64_t to avoid overflow when subtracting longitudes across date line
  int64_t dlon_raw = (int64_t)curLon - (int64_t)homeLon;
  
  // Handle date line crossing: if difference > 180, take the shorter path
  if (dlon_raw > LON_180) dlon_raw -= LON_360;
  else if (dlon_raw < -LON_180) dlon_raw += LON_360;
  
  const float dlat = (curLat - homeLat) * LAT_TO_M;
  const float dlon = (float)dlon_raw * LAT_TO_M * cosf(homeLat * 1e-7f * (float)M_PI / 180.0f);

  const float distance = sqrtf(dlat * dlat + dlon * dlon);
  float bearing = atan2f(dlon, dlat);
  if (bearing < 0.0f) bearing += 2.0f * (float)M_PI; // normalize to [0, 2PI]

  return std::make_tuple(distance, bearing);
}

}
