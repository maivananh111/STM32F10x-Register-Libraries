/*
 * RTC_F1.c
 *
 *  Created on: Aug 1, 2022
 *      Author: A315-56
 */
#include "SYSCLK_F1.h"
#include "RTC_F1.h"


typedef struct{
	uint8_t weekday;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} DateUpdate;

static DateUpdate dateUpdate;

static Status_t EnterConfMode(void){
	// WAIT FOR RTC REGISTER WRITE FINISH.
	__IO uint32_t tick = GetTick();
	while(!(RTC -> CRL & RTC_CRL_RTOFF)){
		if(GetTick() - tick > DEFAULT_CONF_TIMEOUT) return TIMEOUT;
	}
	// ENTER IN CONFIGURATION MODE.
	RTC -> CRL |= RTC_CRL_CNF;
	return OKE;
}

static Status_t ExitConfMode(void){
	// EXIT IN CONFIGURATION MODE.
	RTC -> CRL &=~ RTC_CRL_CNF;
	// WAIT FOR RTC REGISTER WRITE FINISH.
	__IO uint32_t tick = GetTick();
	while(!(RTC -> CRL & RTC_CRL_RTOFF)){
		if(GetTick() - tick > DEFAULT_CONF_TIMEOUT) return TIMEOUT;
	}
	return OKE;
}

Status_t RTC_Init(RTC_ClockSource CLKSource){
	__IO uint32_t tick = 0;

	// ENABLE PWR CLK INTERFACE.
	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;
	// DISABLE BACKUP DOMAIN WRITE PROTECTION.
	PWR -> CR |= PWR_CR_DBP;
	tick = GetTick();
	while(!(PWR -> CR & PWR_CR_DBP)){
		if(GetTick() - tick > LSI_CONF_TIMEOUT) return TIMEOUT;
	}

	__IO uint32_t tmperg = RCC -> BDCR & RCC_BDCR_RTCSEL;
	if(tmperg != 0UL && tmperg != (CLKSource & RCC_BDCR_RTCSEL)){
		tmperg = RCC -> BDCR & (~RCC_BDCR_RTCSEL);

		// RESET BACKUP DOMAIN REGISTER.
		RCC -> BDCR |= RCC_BDCR_BDRST;
		RCC -> BDCR &=~ RCC_BDCR_BDRST;
		RCC -> BDCR = tmperg;

		// SETUP CLOCK SOURE FOR RTC.
		if(CLKSource == RTC_ClockSource_LSI){
			RCC -> CSR |= RCC_CSR_LSION;
			tick = GetTick();
			while(!(RCC -> CSR & RCC_CSR_LSIRDY)){
				if(GetTick() - tick > LSI_CONF_TIMEOUT) return TIMEOUT;
			}
		}
		else if(CLKSource == RTC_ClockSource_LSE){
			RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
			RCC -> BDCR |= RCC_BDCR_LSEON;
			tick = GetTick();
			while(!(RCC -> BDCR & RCC_BDCR_LSERDY)){
				if(GetTick() - tick > LSE_CONF_TIMEOUT) return TIMEOUT;
			}
		}
	}
	// SETUP RCC_BDCR REGISTER.
	RCC -> BDCR = (RCC -> BDCR & (~RCC_BDCR_RTCSEL)) | CLKSource;
	// ENABLE BACKUP DOMAIN.
	RCC -> APB1ENR |= RCC_APB1ENR_BKPEN;
	// ENABLE RTC.
	RCC -> BDCR |= RCC_BDCR_RTCEN;

	// CLEAR REGISTER SYNCHRONIZED FLAG.
	RTC -> CRL &=~ RTC_CRL_RSF;
	tick = GetTick();
	while(RTC -> CRL & RTC_CRL_RSF){
		if(GetTick() - tick > DEFAULT_CONF_TIMEOUT) return ERR;
	}

	// GO TO CONFIGURATION MODE.
	if(EnterConfMode() != OKE) return TIMEOUT;

	// CLEAR ALL CRL FLAG.
	RTC -> CRL &=~ (RTC_CRL_ALRF | RTC_CRL_OWF | RTC_CRL_SECF);

	// DISABLE SELECTED TAMPER PIN.
	BKP -> CR &=~ BKP_CR_TPE;
	BKP -> RTCCR |= BKP_RTCCR_ASOE;

	// SETUP RTC PRESCALER.
	uint32_t prescaler = LSI_VALUE - 1; // DEFAULT IS 39999 (LSI Frequency is 40KHz);
	if(((RCC -> BDCR & RCC_BDCR_RTCSEL_Msk) == RCC_BDCR_RTCSEL_LSI) && (RCC -> CSR & RCC_CSR_LSIRDY)){ // LSI.
		prescaler = LSI_VALUE - 1; // 39999 (LSI Frequency is 40KHz);
	}
	else if(((RCC -> BDCR & RCC_BDCR_RTCSEL_Msk) == RCC_BDCR_RTCSEL_LSE) && (RCC -> BDCR & RCC_BDCR_LSERDY)){ // LSE.
		prescaler = LSE_VALUE - 1; // 32767 (LSE Frequency is 32.768KHz);
	}
	else if(((RCC -> BDCR & RCC_BDCR_RTCSEL_Msk) == RCC_BDCR_RTCSEL_HSE) && (RCC -> CR & RCC_CR_HSERDY)){ // HSE.
		prescaler = (HSE_VALUE / 128U) - 1;
	}
	RTC -> PRLL = (uint32_t)(prescaler & 0xFFFF);
	RTC -> PRLH = (uint32_t)((prescaler >> 16) & 0xFFFF);

	// EXIT CONFIGURATION MODE.
	if(ExitConfMode() != OKE) return TIMEOUT;

	dateUpdate.date  = 1;
	dateUpdate.month = 1;
	dateUpdate.year  = 00;

	return OKE;
}

static uint32_t BCDtoDEC(uint8_t Value){
  uint32_t tmp = 0U;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> 4) * 10U;
  return (uint32_t)(tmp + (Value & (uint8_t)0x0F));
}

static uint32_t DECtoBCD(uint8_t Value){
  uint32_t bcdhigh = 0U;
  while (Value >= 10U){
    bcdhigh++;
    Value -= 10U;
  }
  return ((uint8_t)(bcdhigh << 4U) | Value);
}

static uint8_t WeekDayNum(uint32_t Day, uint8_t Month, uint8_t Year){
  uint32_t year = 0U, weekday = 0U;
  year = 2000U + Year;

  if (Month < 3U)
    /*D = { [(23 x month)/9] + day + 4 + year + [(year-1)/4] - [(year-1)/100] + [(year-1)/400] } mod 7*/
    weekday = (((23U * Month) / 9U) + Day + 4U + year + ((year - 1U) / 4U) - ((year - 1U) / 100U) + ((year - 1U) / 400U)) % 7U;

  else
    /*D = { [(23 x month)/9] + day + 4 + year + [year/4] - [year/100] + [year/400] - 2 } mod 7*/
    weekday = (((23U * Month) / 9U) + Day + 4U + year + (year / 4U) - (year / 100U) + (year / 400U) - 2U) % 7U;

  return (uint8_t)weekday;
}

static void Date_Update(uint32_t DayElapsed){
	  uint32_t year = 0U, month = 0U, day = 0U;
	  uint32_t loop = 0U;

	  year = dateUpdate.year;
	  month = dateUpdate.month;
	  day = dateUpdate.date;

	  for (loop = 0U; loop < DayElapsed; loop++){
	    if ((month == 1U) || (month == 3U) || (month == 5U) || (month == 7U) || (month == 8U) || (month == 10U) || (month == 12U)){ // Tháng 31 ngày.
	      if (day < 31U) day++; // Chưa phải ngày 31 thì tăng thêm 1 ngày
	      else{ // Là ngày 31.
	        if(month != 12U){ // Ko phải 31/21 thì tăng 1 tháng, ngày về 1.
	          month++;
	          day = 1U;
	        }
	        else{ // Là 31/12 thì tăng 1 năm, ngày tháng về 1.
	          month = 1U;
	          day = 1U;
	          year++;
	        }
	      }
	    }

	    else if ((month == 4U) || (month == 6U) || (month == 9U) || (month == 11U)){// Tháng 30 ngày.
	      if (day < 30U) day++; // Chưa phải ngày 30 thì tăng ngày.
	      else{ // Là nagyf 30 thì tăng 1 tháng, ngày về 1.
	        month++;
	        day = 1U;
	      }
	    }
	    else if (month == 2U){ // Tháng 2 đặc biệt chỉ có 28 ngày.
	      if (day < 28U) day++; // Chưa phải ngày 28 thì tăng 1 ngày.
	      else if (day == 28U){ // Nếu là ngày 28.
	        /* Năm nhuận */
	        if (((year % 4U) != 0U) || ((year % 100U) != 0U) || ((year % 400U) == 0U)) day++; // Năm nhuận có ngày 29.
	        else{ // Qua 01/03.
	          month++;
	          day = 1U;
	        }
	      }
	      else if (day == 29U){ // Ngày 29/02 (Chỉ đúng trong năm nhuận) thì lên 01/03.
	        month++;
	        day = 1U;
	      }
	    }
	  }

	  dateUpdate.year = year;
	  dateUpdate.month = month;
	  dateUpdate.date = day;

	  /* Update day of the week */
	  dateUpdate.weekday = WeekDayNum(day, month, year);
}

uint32_t Read_CountTimeRegister(void){
	uint32_t count_time = 0U, cnt_h1 = 0U, cnt_h2 = 0U, cnt_l = 0U;

	cnt_h1 = (uint32_t)(RTC -> CNTH & 0xFFFF);
	cnt_l  = (uint32_t)(RTC -> CNTL & 0xFFFF);
	cnt_h2 = (uint32_t)(RTC -> CNTH & 0xFFFF);
	if(cnt_h1 != cnt_h2) // Counter changer during reading register;
		count_time = (uint32_t)(cnt_h2 << 16U) | (uint32_t)(RTC -> CNTL & 0xFFFFU);
	else
		count_time = (uint32_t)(cnt_h1 << 16U) | cnt_l;

	return count_time;
}

static uint32_t Read_CountAlarmRegister(void){
	  uint16_t cnt_h = 0U, cnt_l = 0U;

	  cnt_h = (uint32_t)(RTC->ALRH & 0xFFFF);
	  cnt_l = (uint32_t)(RTC->ALRL & 0xFFFF);

	  return (((uint32_t) cnt_h << 16U) | cnt_l);
}

static Status_t Write_CountTimeRegister(uint32_t counter){
	if(EnterConfMode() != OKE) return TIMEOUT;
	RTC -> CNTH = (uint32_t)(counter >> 16);
	RTC -> CNTL = (uint32_t)(counter & 0xFFFF);
	if(ExitConfMode() != OKE) return TIMEOUT;

	return OKE;
}

static Status_t Write_CountAlarmRegister(uint32_t counter){
	if(EnterConfMode() != OKE) return TIMEOUT;
	RTC -> ALRH = (uint32_t)(counter >> 16);
	RTC -> ALRL = (uint32_t)(counter & 0xFFFF);
	if(ExitConfMode() != OKE) return TIMEOUT;

	return OKE;
}

/*	Trong thanh ghi CNT 3600 là 60s * 60p = 1h.
 *  3600 * 24 = 86400 = 1 ngày.
 *	Suy ra:
 *		- Số Ngày: Day = counter / (24*3600).
 *		- Số Giờ : Hour = counter / 3600.
 */

Status_t RTC_SetTime(RTC_Time *time, RTC_Format Format){
	if(Format == DEC_Format){
		if(time -> hour > 23)   return ERR;
		if(time -> minute > 59) return ERR;
		if(time -> second > 59) return ERR;
	}
	else if(Format == BCD_Format){
		if(BCDtoDEC(time -> hour) > 23)   return ERR;
		if(BCDtoDEC(time -> minute) > 59) return ERR;
		if(BCDtoDEC(time -> second) > 59) return ERR;
	}

	uint32_t count_time = 0U, count_alarm = 0U;
	if(Format == DEC_Format)
		count_time = time -> hour * 3600 + time -> minute * 60 + time -> second;
	else if(Format == BCD_Format)
		count_time = BCDtoDEC(time -> hour) * 3600 + BCDtoDEC(time -> minute) * 60 + BCDtoDEC(time -> second);

	if(Write_CountTimeRegister(count_time) != OKE) return ERR;

	// CLEAR OVERFLOW AND SECOND FLAG.
	RTC -> CRL &=~ (RTC_CRL_OWF | RTC_CRL_SECF);

	count_alarm = Read_CountAlarmRegister();
	if(count_alarm != 0xFFFFFFFFUL && count_alarm < count_time){
		count_alarm += (uint32_t)(24 * 3600U);
		if(Write_CountAlarmRegister(count_alarm) != OKE) return ERR;
	}

	return OKE;
}

Status_t RTC_GetTime(RTC_Time *time, RTC_Format Format){
	if(RTC -> CRL & RTC_CRL_OWF) return ERR;

	uint32_t count_time = 0U, count_alarm = 0;
	count_time = Read_CountTimeRegister();

	time -> minute = (uint8_t)((count_time % 3600U) / 60U);
	time -> second = (uint8_t)((count_time % 3600U) % 60U);
	uint32_t hour = count_time / 3600U;
	if(hour >= 24){
		uint32_t days_elapsed = hour / 24U;
		time -> hour = hour % 24U;
		count_alarm = Read_CountAlarmRegister();
	    if ((count_alarm != 0xFFFFFFFFU) && (count_alarm > count_time)) count_alarm -= count_time;
	    else count_alarm = 0xFFFFFFFFU;
	    count_time -= (days_elapsed * 24U * 3600U);

	    if(Write_CountTimeRegister(count_time) != OKE) return ERR;

	    if(count_alarm != 0xFFFFFFFFU){
	    	count_alarm += count_time;
	    	if(Write_CountAlarmRegister(count_alarm) != OKE) return ERR;
	    }
	    else{
	    	if(Write_CountAlarmRegister(count_alarm) != OKE) return ERR;
	    }
	    Date_Update(days_elapsed);
	}
	else{
		time -> hour = hour;
	}

	if(Format == BCD_Format){
		time -> hour   = (uint8_t)DECtoBCD(time -> hour);
		time -> minute = (uint8_t)DECtoBCD(time -> minute);
		time -> second = (uint8_t)DECtoBCD(time -> second);
	}

	return OKE;
}

Status_t RTC_SetDate(RTC_Date *date, RTC_Format Format){
	uint32_t count_time = 0U, count_alarm = 0U, hour = 0U;
	if(Format == DEC_Format){
		if(date -> date > 23)   return ERR;
		if(date -> month > 59) return ERR;
		if(date -> year > 59) return ERR;
		dateUpdate.date  = date -> date;
		dateUpdate.month = date -> month;
		dateUpdate.year  = date -> year;
	}
	else if(Format == BCD_Format){
		if(BCDtoDEC(date -> date)  < 1 || BCDtoDEC(date -> date) > 31)   return ERR;
		if(BCDtoDEC(date -> month) < 1 || BCDtoDEC(date -> month) > 12) return ERR;
		if(BCDtoDEC(date -> year) > 99) return ERR;
		dateUpdate.date  = BCDtoDEC(date -> date);
		dateUpdate.month = BCDtoDEC(date -> month);
		dateUpdate.year  = BCDtoDEC(date -> year);
	}
	dateUpdate.weekday = WeekDayNum(dateUpdate.date, dateUpdate.month, dateUpdate.year);

	count_time = Read_CountTimeRegister();
	hour = count_time / 3600U;
	if(hour > 24U){
		count_time -= ((hour / 24U) * 24U * 3600U);
		if(Write_CountTimeRegister(count_time) != OKE) return ERR;
		count_alarm = Read_CountAlarmRegister();
		if(count_alarm != 0xFFFFFFFFU){
			if(count_alarm < count_time){
				count_alarm += (uint32_t)(24U * 3600U);
				if(Write_CountAlarmRegister(count_alarm) != OKE) return ERR;
			}
		}
	}

	return OKE;
}

Status_t RTC_GetDate(RTC_Date *date, RTC_Format Format){
	RTC_Time time;
	if(RTC_GetTime(&time, DEC_Format) != OKE) return ERR;

	date -> weekday = dateUpdate.weekday;
	date -> date  = dateUpdate.date;
	date -> month = dateUpdate.month;
	date -> year  = dateUpdate.year;
	if(Format == BCD_Format){
		date -> date  = DECtoBCD(dateUpdate.date);
		date -> month = DECtoBCD(dateUpdate.month);
		date -> year  = DECtoBCD(dateUpdate.year);
	}

	return OKE;
}

uint32_t BKPUPRegister_Read(uint32_t BKPUPRegister){
	  uint32_t backupregister = 0U;
	  uint32_t pvalue = 0U;

	  backupregister = (uint32_t)BKP_BASE;
	  backupregister += (BKPUPRegister * 4U);

	  pvalue = (*(__IO uint32_t *)(backupregister)) & BKP_DR1_D;

	  return pvalue;
}

void BKPUPRegister_Write(uint32_t BKPUPRegister, uint32_t Data){
	  uint32_t tmp = 0U;

	  tmp = (uint32_t)BKP_BASE;
	  tmp += (BKPUPRegister * 4U);

	  *(__IO uint32_t *) tmp = (Data & BKP_DR1_D);
}





