#include "Debug_Espfc.hpp"
#include "Stream/NullWritable.hpp"
#include <utility>

namespace Espfc {

static Stream::NullWritable _nullWriter;
Stream::Printer _debugStream{_nullWriter};

void initDebugStream(Stream::Printer p)
{
  _debugStream = std::move(p);
}

} // namespace Espfc
