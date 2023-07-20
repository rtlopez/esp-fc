#if defined(ESP32)

#include "EscDriverEsp32.h"

bool EscDriverEsp32::_tx_end_installed = false;

EscDriverEsp32* EscDriverEsp32::instances[] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

#endif