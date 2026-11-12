#pragma once

#include "Model.hpp"
#include <platform.h>
extern "C" {
#include <blackbox/blackbox.h>
#include <blackbox/blackbox_fielddefs.h>
bool blackboxShouldLogPFrame(void);
bool blackboxShouldLogIFrame(void);
}

void initBlackboxModel(Espfc::Model* m);
