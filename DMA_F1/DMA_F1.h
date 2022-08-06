/*
 * DMA_F1.h
 *
 *  Created on: Jun 4, 2022
 *      Author: A315-56
 */

#ifndef DMA_F1_H_
#define DMA_F1_H_

#include "stdio.h"
#include "stm32f1xx.h"


#ifdef __cplusplus
extern "C" {
#endif

#define USE_DMA1_Channel1
#define USE_DMA1_Channel2
#define USE_DMA1_Channel3
//#define USE_DMA1_Channel4
#define USE_DMA1_Channel5
//#define USE_DMA1_Channel6
//#define USE_DMA1_Channel7

typedef enum{
	DMA_Normal = 0x00UL,
	DMA_Circular = 0x20UL
} DMA_Mode;

typedef enum{
	DMA_Data8Bit = 0UL,
	DMA_Data16Bit,
	DMA_Data32Bit
} DMA_DataSize;

typedef enum{
	DMA_PERIPH_TO_MEM = 0x00UL,
	DMA_MEM_TO_PERIPH = 0x10UL,
	DMA_MEM_MEM = 0x4000UL
} DMA_DataDirection;

typedef enum{
	DMA_Priority_Low = 0,
	DMA_Priority_Medium,
	DMA_Priority_High,
	DMA_Priority_VeryHigh
} DMA_Priority;

typedef enum{
	DMA_State_Idle = 0,
	DMA_State_Busy
}DMA_State;

class DMA{
	public:
		/* Create new dma oop */
		DMA(DMA_Channel_TypeDef *DMA_CHANNEL, uint32_t INTRp);
		/* Dma init */
		void Init(DMA_Mode MODE, DMA_DataDirection DIR, DMA_DataSize SIZE, DMA_Priority PRIORITY);
		/* Dma configuration to use */
		void Start(uint32_t Source_Addr, uint32_t Dest_Addr, uint16_t Data_Size);

		void SetMode(DMA_Mode MODE);
		void SetDirection(DMA_DataDirection DIR);

		void Clear_Intr_Flag(void);
		void Enable(void);
		void Disable(void);
		void Enable_IT(uint32_t IT);
		void Disable_IT(uint32_t IT);
		void Stop(void);

	private:
		DMA_Channel_TypeDef *_dmach;
		DMA_TypeDef *_dma;
		IRQn_Type _IRQn;
		uint32_t _intr_priority;
		DMA_Mode _mode;
		DMA_DataDirection _dir;
		DMA_DataSize _size;
		DMA_Priority _priority;
		uint8_t _dma_channel_num = 0;

		DMA_State _state = DMA_State_Idle;
};
#ifdef USE_DMA1_Channel1
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel1_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA1_Channel2
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel2_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA1_Channel3
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel3_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA1_Channel4
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel4_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA1_Channel5
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel5_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA1_Channel6
void DMA1_Channel6_IRQHandler(void);
void DMA1_Channel6_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA1_Channel7
void DMA1_Channel7_IRQHandler(void);
void DMA1_Channel7_TxCplt_CallBack(void);
#endif









#ifdef __cplusplus
}
#endif

#endif /* DMA_F1_H_ */
