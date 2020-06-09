#ifndef _HAL_H_
#define _HAL_H_

#ifdef UNIT_TEST

#define PSTR(s) (s)

#ifndef F
#define F(s) (s)
#endif

#define PGM_P                 const char *
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

#endif // UNIT_TEST

#undef max
#undef min

#endif // _HAL_H_
