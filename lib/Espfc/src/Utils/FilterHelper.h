#pragma once

#include "Filter.h"
#include "helper_3dmath.h"

namespace Espfc {

namespace Utils {

float applyFilter(Filter& filter, float sample)
{
    return filter.update(sample);
}

VectorFloat applyFilter(Filter * filters, const VectorFloat& samples)
{
    VectorFloat result;
    result.x = filters[0].update(samples.x);
    result.y = filters[1].update(samples.y);
    result.z = filters[2].update(samples.z);
    return result;
}

}

}
