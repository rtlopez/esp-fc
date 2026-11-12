#pragma once

#include <cstdint>
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
std::tuple<float, float> calculateDistanceAndBearing(int32_t homeLat, int32_t homeLon, int32_t curLat, int32_t curLon);

} // namespace Gps
