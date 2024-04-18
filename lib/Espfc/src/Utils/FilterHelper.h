#pragma once

#include "Filter.h"
#include <helper_3dmath.h>
#include "MemoryHelper.h"

namespace Espfc {

namespace Utils {

float applyFilter(Filter& filter, float sample) FAST_CODE_ATTR;

VectorFloat applyFilter(Filter filters[3], const VectorFloat& samples) FAST_CODE_ATTR;

}

}
