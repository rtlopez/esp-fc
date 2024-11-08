#pragma once

#ifndef UNIT_TEST

#include <pgmspace.h>

#else

#ifndef PSTR
#define PSTR(s) (s)
#endif

#ifndef FPSTR
#define FPSTR(s) (s)
#endif

#ifndef F
#define F(s) (s)
#endif

#define PGM_P                 const char *
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

#endif // UNIT_TEST
