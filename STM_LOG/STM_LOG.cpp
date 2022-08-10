/*
 * STM_LOG.cpp
 *
 *  Created on: 28 thg 7, 2022
 *      Author: A315-56
 */


#include "SYSCLK_F1.h"
#include "STM_LOG.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f1xx.h"

static const char *COLOR_END = "\033[0m";
static const char *LOG_COLOR[] = {
	"\033[0;30m",
	"\033[0;31m",
	"\033[0;32m",
	"\033[0;33m",
	"\033[0;34m",
	"\033[0;35m",
	"\033[0;36m",
	"\033[0;37m",

	// Bold
	"\033[1;30m",
	"\033[1;31m",
	"\033[1;32m",
	"\033[1;33m",
	"\033[1;34m",
	"\033[1;35m",
	"\033[1;36m",
	"\033[1;37m",

	// Italic
	"\033[4;30m",
	"\033[4;31m",
	"\033[4;32m",
	"\033[4;33m",
	"\033[4;34m",
	"\033[4;35m",
	"\033[4;36m",
	"\033[4;37m",

	// Background
	"\033[40m",
	"\033[41m",
	"\033[42m",
	"\033[43m",
	"\033[44m",
	"\033[45m",
	"\033[46m",
	"\033[47m",
};

void STM_LOG(log_type_t log_type, const char *tag, const char *format, ...){
	uint32_t time = GetTick();
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char));
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[log_type]);
	char *Output_buffer = (char *)malloc((color_start_length + 14 + strlen(tag) + 2 + length + 4) * sizeof(char));
	sprintf(Output_buffer, "%s[%lums]%s: %s%s\n\r", LOG_COLOR[log_type], time, tag, Temp_buffer, COLOR_END);

	STM_LOG_Print(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

__WEAK void STM_LOG_Print(char *log){
	printf("Print log cc, let write this function!");
}







