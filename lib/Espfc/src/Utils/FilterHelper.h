#pragma once

#include "Filter.h"
#include <helper_3dmath.h>

namespace Espfc {

namespace Utils {

float applyFilter(Filter& filter, float sample);

VectorFloat applyFilter(Filter filters[3], const VectorFloat& samples);

}

}
