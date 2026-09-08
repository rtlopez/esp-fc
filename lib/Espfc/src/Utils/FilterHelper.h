#pragma once

#include "Utils/Filter.h"
#include <helper_3dmath.hpp>

namespace Espfc::Utils {

float applyFilter(Filter& filter, float sample);

VectorFloat applyFilter(Filter filters[3], const VectorFloat& samples);

} // namespace Espfc::Utils
