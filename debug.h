#ifndef __debug__h
#define __debug__h
#include "stm32f4xx.h"                  // Device header
#include "stdio.h"
void log_error(char *p);
void log_info(char *p);
void log_debug(char *p);
void log_debug_array(char const * const label, void const *array, uint16_t const len);



#endif