/*
 * DMA_F1.h
 *
 *  Created on: 12 thg 8, 2022
 *      Author: A315-56
*/

#ifndef DMA_F1_H_
#define DMA_F1_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f1xx.h"
#include "MVA_DEF.h"

#define USE_DMA1_Channel1
#define USE_DMA1_Channel2
#define USE_DMA1_Channel3
//#define USE_DMA1_Channel4
#define USE_DMA1_Channel5
//#define USE_DMA1_Channel6
//#define USE_DMA1_Channel7

//#define DMA2_AVAILABLE
#ifdef DMA2_AVAILABLE
//#define USE_DMA2_Channel1
//#define USE_DMA2_Channel2
//#define USE_DMA2_Channel3
//#define USE_DMA2_Channel4
//#define USE_DMA2_Channel5
//#define USE_DMA2_Channel6
//#define USE_DMA2_Channel7
#endif

typedef enum{
	DMA_PERIPH_TO_MEM = 0x00UL,
	DMA_MEM_TO_PERIPH = DMA_CCR_DIR,
} DMA_Direction;

typedef enum{
	DMA_NORMAL   = 0x00UL,
	DMA_CIRCULAR = DMA_CCR_CIRC
} DMA_Mode;

typedef enum{
	DMA_DATA8BIT  = 0x000UL,
	DMA_DATA16BIT = 0x100UL,
	DMA_DATA32BIT = 0x200UL
} DMA_DataSize;

typedef enum{
	DMA_CHANNEL_PRIORITY_LOW      = 0UL,
	DMA_CHANNEL_PRIORITY_MEDIUM,
	DMA_CHANNEL_PRIORITY_HIGH,
	DMA_CHANNEL_PRIORITY_VERYHIGH,
} DMA_ChannelPriority;

typedef enum{
	DMA_TRANSFER_COMPLETE_IE = DMA_CCR_TCIE,
	DMA_HALF_TRANSFER_IE     = DMA_CCR_HTIE,
	DMA_TRANSFER_ERROR_IE    = DMA_CCR_TEIE,
} DMA_INTRSelect;

typedef struct{
	DMA_Direction Direction = DMA_MEM_TO_PERIPH;
	DMA_Mode Mode = DMA_NORMAL;
	DMA_DataSize DataSize = DMA_DATA8BIT;
	DMA_ChannelPriority DMAChannelPriority = DMA_CHANNEL_PRIORITY_MEDIUM;
	DMA_INTRSelect DMAInterruptSelect = DMA_TRANSFER_COMPLETE_IE;
	uint32_t InterruptPriority = 0;
} DMA_Config;

class DMA {
	public:
		DMA(DMA_Channel_TypeDef *DmaChannel);
		void Init(DMA_Config DmaConf);
		Result_t Start(uint32_t Src_Address, uint32_t Dest_Address, uint32_t Number_Data);
		Result_t Stop(void);

	private:
		DMA_Channel_TypeDef *_dmachannel;
		DMA_TypeDef *_dma;
		DMA_Direction _dir;
		DMA_INTRSelect _intrsl;
		IRQn_Type _IRQn;
		uint8_t ChannelIndex = 0;
		Status_t _dmastate = READY;
};

#ifdef USE_DMA1_Channel1
	void DMA1_Channel1_IRQHandler(void);
	void DMA1_Channel1_HalfTx_CallBack(void);
	void DMA1_Channel1_TxCplt_CallBack(void);
	void DMA1_Channel1_TxError_CallBack(void);
#endif
#ifdef USE_DMA1_Channel2
	void DMA1_Channel2_IRQHandler(void);
	void DMA1_Channel2_HalfTx_CallBack(void);
	void DMA1_Channel2_TxCplt_CallBack(void);
	void DMA1_Channel2_TxError_CallBack(void);
#endif
#ifdef USE_DMA1_Channel3
	void DMA1_Channel3_IRQHandler(void);
	void DMA1_Channel3_HalfTx_CallBack(void);
	void DMA1_Channel3_TxCplt_CallBack(void);
	void DMA1_Channel3_TxError_CallBack(void);
#endif
#ifdef USE_DMA1_Channel4
	void DMA1_Channel4_IRQHandler(void);
	void DMA1_Channel4_HalfTx_CallBack(void);
	void DMA1_Channel4_TxCplt_CallBack(void);
	void DMA1_Channel4_TxError_CallBack(void);
#endif
#ifdef USE_DMA1_Channel5
	void DMA1_Channel5_IRQHandler(void);
	void DMA1_Channel5_HalfTx_CallBack(void);
	void DMA1_Channel5_TxCplt_CallBack(void);
	void DMA1_Channel5_TxError_CallBack(void);
#endif
#ifdef USE_DMA1_Channel6
	void DMA1_Channel6_IRQHandler(void);
	void DMA1_Channel6_HalfTx_CallBack(void);
	void DMA1_Channel6_TxCplt_CallBack(void);
	void DMA1_Channel6_TxError_CallBack(void);
#endif
#ifdef USE_DMA1_Channel7
	void DMA1_Channel7_IRQHandler(void);
	void DMA1_Channel7_HalfTx_CallBack(void);
	void DMA1_Channel7_TxCplt_CallBack(void);
	void DMA1_Channel7_TxError_CallBack(void);
#endif

#ifdef DMA2_AVAILABLE
#ifdef USE_DMA2_Channel1
	void DMA2_Channel1_IRQHandler(void);
	void DMA2_Channel1_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA2_Channel2
	void DMA2_Channel2_IRQHandler(void);
	void DMA2_Channel2_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA2_Channel3
	void DMA2_Channel3_IRQHandler(void);
	void DMA2_Channel3_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA2_Channel4
	void DMA2_Channel4_IRQHandler(void);
	void DMA2_Channel4_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA2_Channel5
	void DMA2_Channel5_IRQHandler(void);
	void DMA2_Channel5_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA2_Channel6
	void DMA2_Channel6_IRQHandler(void);
	void DMA2_Channel6_TxCplt_CallBack(void);
#endif
#ifdef USE_DMA2_Channel7
	void DMA2_Channel7_IRQHandler(void);
	void DMA2_Channel7_TxCplt_CallBack(void);
#endif
#endif


#ifdef __cplusplus
}
#endif

#endif /* DMA_F1_H_ */
