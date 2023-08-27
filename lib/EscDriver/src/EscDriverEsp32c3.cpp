#if defined(ESP32_C3)

#include "EscDriverEsp32c3.h"

bool EscDriverEsp32c3::_tx_end_installed = false;

EscDriverEsp32c3* EscDriverEsp32c3::instances[] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

#endif