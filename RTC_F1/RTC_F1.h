/*
 * RTC_F1.h
 *
 *  Created on: Aug 1, 2022
 *      Author: A315-56
 */

#ifndef RTC_F1_H_
#define RTC_F1_H_

#include "stm32f1xx.h"
#include "stdio.h"
#include "MVA_DEF.h"

#ifdef __cplusplus
extern "C"{
#endif

#if !defined(LSE_VALUE)
  #define LSE_VALUE 32768U

#endif

#if !defined(LSI_VALUE)
  #define LSI_VALUE 40000U

#endif


typedef enum{
	RTC_ClockSource_LSI = RCC_BDCR_RTCSEL_LSI,
	RTC_ClockSource_LSE = RCC_BDCR_RTCSEL_LSE,
	RTC_ClockSource_HSE = RCC_BDCR_RTCSEL_HSE,
} RTC_ClockSource;

typedef struct{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
}RTC_Time;

typedef struct{
	uint8_t weekday;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} RTC_Date;

typedef enum{
	BCD_Format,
	DEC_Format,
}RTC_Format;

#define RTC_BKP_DR1  0x00000001U
#define RTC_BKP_DR2  0x00000002U
#define RTC_BKP_DR3  0x00000003U
#define RTC_BKP_DR4  0x00000004U
#define RTC_BKP_DR5  0x00000005U
#define RTC_BKP_DR6  0x00000006U
#define RTC_BKP_DR7  0x00000007U
#define RTC_BKP_DR8  0x00000008U
#define RTC_BKP_DR9  0x00000009U
#define RTC_BKP_DR10 0x0000000AU


uint32_t Read_CountTimeRegister(void);

Status_t RTC_Init(RTC_ClockSource CLKSource);

Status_t RTC_SetTime(RTC_Time *time, RTC_Format Format);
Status_t RTC_SetDate(RTC_Date *date, RTC_Format Format);

Status_t RTC_GetTime(RTC_Time *time, RTC_Format Format);
Status_t RTC_GetDate(RTC_Date *date, RTC_Format Format);

Status_t RTC_SetAlarm(RTC_Date *date, RTC_Time *time, RTC_Format Format);

uint32_t BKPUPRegister_Read(uint32_t BKPUPRegister);
void BKPUPRegister_Write(uint32_t BKPUPRegister, uint32_t Data);

#ifdef __cplusplus
}
#endif

#endif /* RTC_F1_H_ */
