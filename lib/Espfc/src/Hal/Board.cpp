#include "Hal/Board.hpp"

namespace Espfc::Hal {

bool isUnexpectedReset(ResetReason reason)
{
  switch (reason)
  {
    case ResetReason::SOFTWARE:
    case ResetReason::WATCHDOG:
    case ResetReason::PANIC:
      return true;
    default:
      return false;
  }
}

} // namespace Espfc::Hal