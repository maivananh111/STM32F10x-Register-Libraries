/*
 * DMA_F1.c
 *
 *  Created on: Jun 4, 2022
 *      Author: A315-56
 */

#include "DMA_F1.h"

#include "stm32f103xb.h"


DMA::DMA(DMA_Channel_TypeDef *DMA_CHANNEL, uint32_t INTRp){
	_dmach = DMA_CHANNEL;
	_intr_priority = INTRp;

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
}

void DMA::Init(DMA_Mode MODE, DMA_DataDirection DIR, DMA_DataSize SIZE, DMA_Priority PRIORITY){
	_mode = MODE;
	_dir = DIR;
	_size = SIZE;
	_priority = PRIORITY;
	/* ENABLE DMA1 CLOCK */
	RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
	/* CONFIGURATION DMA */
	_dmach -> CCR |= _dir | _mode | DMA_CCR_MINC | (_priority << DMA_CCR_PL_Pos);
	_dmach -> CCR |= ((_size << DMA_CCR_PSIZE_Pos) | (_size << DMA_CCR_MSIZE_Pos));
	/* DMA INTERRUPT */
	__NVIC_SetPriority(_IRQn, _intr_priority);
	__NVIC_EnableIRQ(_IRQn);
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

void DMA::Start(uint32_t Source_Addr, uint32_t Dest_Addr, uint16_t Data_Size){
	if(_state == DMA_State_Idle){
		/* DISABLE DMA */
		Disable();
		/* CLEAR ALL DMAn INTERRUPT FLAG */
		Clear_Intr_Flag();
		/* SET DATA SIZE */
		_dmach -> CNDTR = Data_Size;
		/* SET DATA ADDRESS */
		if(_dir == DMA_MEM_TO_PERIPH){ // DMA FOR PERIPHERAL TRANSMIT
			_dmach -> CMAR = Source_Addr;
			_dmach -> CPAR = Dest_Addr;
		}
		else if(_dir == DMA_PERIPH_TO_MEM){ // DMA FOR PERIPHERAL RECEIVE
			_dmach -> CPAR = Source_Addr;
			_dmach -> CMAR = Dest_Addr;
		}
		/* ENABLE DMA INTERRUPT TRANSFER */
		Enable_IT(DMA_CCR_TCIE);
		/* ENABLE DMA */
		Enable();
		_state = DMA_State_Busy;
	}
}

void DMA::Stop(void){
	if(_state == DMA_State_Busy){
		Disable_IT(DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
		Disable();
		Clear_Intr_Flag();
		_state = DMA_State_Idle;
	}
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








