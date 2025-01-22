/*
 * light_printf.h
 *
 *  Created on: Sep 30, 2020
 *      Author: aluno
 */

#ifndef INC_LIGHT_PRINTF_H_
#define INC_LIGHT_PRINTF_H_

#include <stdarg.h>

int printf_(const char *format, ...);

int sprintf_(char *out, const char *format, ...);

int snprintf_( char *buf, unsigned int count, const char *format, ... );

#endif /* INC_LIGHT_PRINTF_H_ */
