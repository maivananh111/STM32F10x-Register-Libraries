/*
 * DMA_F1.c
 *
 *  Created on: Jun 4, 2022
 *      Author: A315-56
 */

#include "DMA_F1.h"
#include "stm32f103xb.h"
#include "STM_LOG.h"


DMA::DMA(DMA_Channel_TypeDef *DMA_CHANNEL){
	_dmach = DMA_CHANNEL;
	_intr_priority = 0UL;

	if(DMA_CHANNEL == DMA1_Channel1 || DMA_CHANNEL == DMA1_Channel2 || DMA_CHANNEL == DMA1_Channel3 || DMA_CHANNEL == DMA1_Channel4 ||
	   DMA_CHANNEL == DMA1_Channel5 || DMA_CHANNEL == DMA1_Channel6 || DMA_CHANNEL == DMA1_Channel7){
		_dma = DMA1;
	}

	if(DMA_CHANNEL == DMA1_Channel1){
		_dma_channel_num = 0;
		_IRQn = DMA1_Channel1_IRQn;
	}
	else if(DMA_CHANNEL == DMA1_Channel2){
		_dma_channel_num = 4;
		_IRQn = DMA1_Channel2_IRQn;
	}
	else if(DMA_CHANNEL == DMA1_Channel3){
		_dma_channel_num = 8;
		_IRQn = DMA1_Channel3_IRQn;
	}
	else if(DMA_CHANNEL == DMA1_Channel4){
		_dma_channel_num = 12;
		_IRQn = DMA1_Channel4_IRQn;
	}
	else if(DMA_CHANNEL == DMA1_Channel5){
		_dma_channel_num = 16;
		_IRQn = DMA1_Channel5_IRQn;
	}
	else if(DMA_CHANNEL == DMA1_Channel6){
		_dma_channel_num = 20;
		_IRQn = DMA1_Channel6_IRQn;
	}
	else if(DMA_CHANNEL == DMA1_Channel7){
		_dma_channel_num = 24;
		_IRQn = DMA1_Channel7_IRQn;
	}
	_size = DMA_Data8Bit;
	_mode = DMA_Normal;
	_dir = DMA_MEM_TO_PERIPH;
	_priority = DMA_Priority_VeryHigh;
}

void DMA::Init(DMA_Config dma_conf){
	_mode = dma_conf.DMAMode;
	_dir  = dma_conf.Direction;
	_size = dma_conf.DataSize;
	_priority = dma_conf.DMAPriority;
	/* ENABLE DMA1 CLOCK */
	RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
	/* CONFIGURATION DMA */
	_dmach -> CCR |= _dir | _mode | DMA_CCR_MINC | (_priority << DMA_CCR_PL_Pos);
	_dmach -> CCR |= ((_size << DMA_CCR_PSIZE_Pos) | (_size << DMA_CCR_MSIZE_Pos));
	/* DMA INTERRUPT */
	__NVIC_SetPriority(_IRQn, dma_conf.INTRPriority);
	__NVIC_EnableIRQ(_IRQn);
	dma_status = READY;
}

void DMA::SetMode(DMA_Mode MODE){
	if(MODE == DMA_Normal)        _dmach -> CCR &=~ DMA_Circular;
	else if(MODE == DMA_Circular) _dmach -> CCR |=  DMA_Circular;
}

void DMA::SetDirection(DMA_DataDirection DIR){
	if(DIR == DMA_PERIPH_TO_MEM)      _dmach -> CCR &=~ DMA_MEM_TO_PERIPH;
	else if(DIR == DMA_MEM_TO_PERIPH) _dmach -> CCR |=  DMA_MEM_TO_PERIPH;
}

void DMA::Enable(void){
	/* ENABLE DMA */
	_dmach -> CCR |= DMA_CCR_EN;
}
void DMA::Disable(void){
	/* DISABLE DMA */
	_dmach -> CCR &=~ DMA_CCR_EN;
}

void DMA::Enable_IT(uint32_t IT){
	_dmach -> CCR |= IT;
}
void DMA::Disable_IT(uint32_t IT){
	_dmach -> CCR &=~ IT;
}

void DMA::Clear_Intr_Flag(void){
	/* CLEAR ALL DMAn INTERRUPT FLAG */
	_dma -> IFCR = (DMA_IFCR_CGIF1 << _dma_channel_num);
}

Result_t DMA::Start(uint32_t Source_Addr, uint32_t Dest_Addr, uint16_t Data_Size){
	Result_t res = {
		.Status = OKE,
	};
	if(dma_status == READY){
		Disable();
		Clear_Intr_Flag();

		_dmach -> CNDTR = Data_Size;

		if(_dir == DMA_MEM_TO_PERIPH){ // DMA FOR PERIPHERAL TRANSMIT
			_dmach -> CMAR = Source_Addr;
			_dmach -> CPAR = Dest_Addr;
		}
		else if(_dir == DMA_PERIPH_TO_MEM){ // DMA FOR PERIPHERAL RECEIVE
			_dmach -> CPAR = Source_Addr;
			_dmach -> CMAR = Dest_Addr;
		}

		Enable_IT(DMA_CCR_TCIE);
		Enable();

		dma_status = BUSY;

		res.Status = OKE;
	}
	else{
		res.Status = BUSY;
	}

	return res;
}

Result_t DMA::Stop(void){
	Disable_IT(DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
	Disable();
	Clear_Intr_Flag();
	dma_status = READY;

	return {OKE, 0, 0};
}


#ifdef USE_DMA1_Channel1
void DMA1_Channel1_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR = DMA_IFCR_CGIF1;
	/* CALL HANDLE */
	DMA1_Channel1_TxCplt_CallBack();
}
__WEAK void DMA1_Channel1_TxCplt_CallBack(void){

}
#endif

#ifdef USE_DMA1_Channel2
void DMA1_Channel2_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR = DMA_IFCR_CGIF2;
	/* CALL HANDLE */
	DMA1_Channel2_TxCplt_CallBack();
}
__WEAK void DMA1_Channel2_TxCplt_CallBack(void){

}
#endif

#ifdef USE_DMA1_Channel3
void DMA1_Channel3_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR = DMA_IFCR_CGIF3;
	/* CALL HANDLE */
	DMA1_Channel3_TxCplt_CallBack();
}
__WEAK void DMA1_Channel3_TxCplt_CallBack(void){

}
#endif

#ifdef USE_DMA1_Channel4
void DMA1_Channel4_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR |= DMA_IFCR_CGIF4;
	/* CALL HANDLE */
	DMA1_Channel4_TxCplt_CallBack();
}
__WEAK void DMA1_Channel4_TxCplt_CallBack(void){

}
#endif

#ifdef USE_DMA1_Channel5
void DMA1_Channel5_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR = DMA_IFCR_CGIF5;
	/* CALL HANDLE */
	DMA1_Channel5_TxCplt_CallBack();
}
__WEAK void DMA1_Channel5_TxCplt_CallBack(void){

}
#endif

#ifdef USE_DMA1_Channel6
void DMA1_Channel6_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR |= DMA_IFCR_CGIF6;
	/* CALL HANDLE */
	DMA1_Channel6_TxCplt_CallBack();
}
__WEAK void DMA1_Channel6_TxCplt_CallBack(void){

}
#endif

#ifdef USE_DMA1_Channel7
void DMA1_Channel7_IRQHandler(void){
	/* CLEAR ALL DMA INTERRUPT FLAG */
	DMA1 -> IFCR |= DMA_IFCR_CGIF7;
	/* CALL HANDLE */
	DMA1_Channel7_TxCplt_CallBack();
}
__WEAK void DMA1_Channel7_TxCplt_CallBack(void){

}
#endif








