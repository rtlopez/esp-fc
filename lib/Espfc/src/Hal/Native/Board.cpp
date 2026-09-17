#if defined(UNIT_TEST)

#include "Hal/Board.hpp"

namespace Espfc::Hal {

uint32_t Board::getId0()
{
  return 0;
}

uint32_t Board::getId1()
{
  return 0;
}

uint32_t Board::getId2()
{
  return 0;
}

void Board::reset() {}

ResetReason Board::getResetReason()
{
  return ResetReason::UNKNOWN;
}

uint32_t Board::getCpuFreq()
{
  return 1;
}

uint32_t Board::getFreeHeap()
{
  return 1;
}

} // namespace Espfc::Hal

#endif
