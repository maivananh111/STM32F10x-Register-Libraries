/*
 * DMA_F1.cpp
 *
 *  Created on: 12 thg 8, 2022
 *      Author: A315-56
*/

#include "DMA_F1.h"
#include "STM_LOG.h"

#define DMA_LOG_DEBUG_ENABLE

DMA::DMA(DMA_Channel_TypeDef *DmaChannel){
	_dmachannel = DmaChannel;
	if(_dmachannel == DMA1_Channel1 || _dmachannel == DMA1_Channel2 || _dmachannel == DMA1_Channel3 ||
	   _dmachannel == DMA1_Channel4 || _dmachannel == DMA1_Channel5 || _dmachannel == DMA1_Channel6 ||
	   _dmachannel == DMA1_Channel7){
		_dma = DMA1;
	}
#ifdef DMA2_AVAILABLE
	else if(_dmachannel == DMA2_Channel1 || _dmachannel == DMA2_Channel2 || _dmachannel == DMA2_Channel3 ||
	        _dmachannel == DMA2_Channel4 || _dmachannel == DMA2_Channel5 || _dmachannel == DMA2_Channel6 ||
	        _dmachannel == DMA2_Channel7){
		_dma = DMA2;
	}
#endif

	if(_dmachannel == DMA1_Channel1) {_IRQn = DMA1_Channel1_IRQn; ChannelIndex = 0;}
	if(_dmachannel == DMA1_Channel2) {_IRQn = DMA1_Channel2_IRQn; ChannelIndex = 1;}
	if(_dmachannel == DMA1_Channel3) {_IRQn = DMA1_Channel3_IRQn; ChannelIndex = 2;}
	if(_dmachannel == DMA1_Channel4) {_IRQn = DMA1_Channel4_IRQn; ChannelIndex = 3;}
	if(_dmachannel == DMA1_Channel5) {_IRQn = DMA1_Channel5_IRQn; ChannelIndex = 4;}
	if(_dmachannel == DMA1_Channel6) {_IRQn = DMA1_Channel6_IRQn; ChannelIndex = 5;}
	if(_dmachannel == DMA1_Channel7) {_IRQn = DMA1_Channel7_IRQn; ChannelIndex = 6;}
#ifdef DMA2_AVAILABLE
	if(_dmachannel == DMA2_Channel1) {_IRQn = DMA2_Channel1_IRQn; ChannelIndex = 0;}
	if(_dmachannel == DMA2_Channel2) {_IRQn = DMA2_Channel2_IRQn; ChannelIndex = 1;}
	if(_dmachannel == DMA2_Channel3) {_IRQn = DMA2_Channel3_IRQn; ChannelIndex = 2;}
	if(_dmachannel == DMA2_Channel4) {_IRQn = DMA2_Channel4_IRQn; ChannelIndex = 3;}
	if(_dmachannel == DMA2_Channel5) {_IRQn = DMA2_Channel5_IRQn; ChannelIndex = 4;}
	if(_dmachannel == DMA2_Channel6) {_IRQn = DMA2_Channel6_IRQn; ChannelIndex = 5;}
	if(_dmachannel == DMA2_Channel7) {_IRQn = DMA2_Channel7_IRQn; ChannelIndex = 6;}
#endif

}

void DMA::Init(DMA_Config DmaConf){
	_dir = DmaConf.Direction;
	_intrsl = DmaConf.DMAInterruptSelect;

	if(_dma == DMA1) RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
#ifdef DMA2_AVAILABLE
	else if(_dma == DMA2) RCC -> AHBENR |= RCC_AHBENR_DMA2EN;
#endif

	_dmachannel -> CCR |= DmaConf.Mode | DmaConf.Direction | (DmaConf.DMAChannelPriority << DMA_CCR_PL_Pos);
	_dmachannel -> CCR |= DMA_CCR_MINC | DmaConf.DataSize | (DmaConf.DataSize << 2U);

	__NVIC_SetPriority(_IRQn, DmaConf.InterruptPriority);
	__NVIC_EnableIRQ(_IRQn);
}

Result_t DMA::Start(uint32_t Src_Address, uint32_t Dest_Address, uint32_t Number_Data){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_dmastate == READY){
		_dmachannel -> CCR &=~ DMA_CCR_EN;
		_dma -> IFCR = (DMA_IFCR_CGIF1 << (ChannelIndex * 4U));

		_dmachannel -> CNDTR = Number_Data;

		if(_dir == DMA_MEM_TO_PERIPH){
			_dmachannel -> CMAR = Src_Address;
			_dmachannel -> CPAR = Dest_Address;
		}
		else{
			_dmachannel -> CMAR = Dest_Address;
			_dmachannel -> CPAR = Src_Address;
		}

		_dmachannel -> CCR |= _intrsl;
		_dmachannel -> CCR |= DMA_CCR_EN;
		_dmastate = BUSY;
	}
	else{
		res.CodeLine = __LINE__;
		res.Status = BUSY;
		_dmastate = READY;
#ifdef DMA_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "DMA", "Error DMA busy, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
	}
	return res;
}

Result_t DMA::Stop(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_dmastate == BUSY){
		_dmachannel -> CCR &=~ (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
		_dmachannel -> CCR &=~ DMA_CCR_EN;
		_dma -> IFCR = (DMA_IFCR_CGIF1 << (ChannelIndex * 4U));
		_dmastate = READY;
	}
	else{
		res.CodeLine = __LINE__;
		res.Status = ERR;
		_dmastate = BUSY;
#ifdef DMA_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "DMA", "Error DMA not started yet, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
	}
	return res;
}

#ifdef USE_DMA1_Channel1
void DMA1_Channel1_IRQHandler(void){
	if((DMA1_Channel1 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF1)){
		if(!(DMA1_Channel1 -> CCR & DMA_CCR_CIRC)) DMA1_Channel1 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF1;
		DMA1_Channel1_HalfTx_CallBack();
	}
	else if((DMA1_Channel1 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF1)){
		if(!(DMA1_Channel1 -> CCR & DMA_CCR_CIRC)) DMA1_Channel1 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF1;
		DMA1_Channel1_TxCplt_CallBack();
	}
	else if((DMA1_Channel1 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF1)){
		DMA1_Channel1 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF1;
		DMA1_Channel1_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel1_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel1_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel1_TxError_CallBack(void){}
#endif

#ifdef USE_DMA1_Channel2
void DMA1_Channel2_IRQHandler(void){
	if((DMA1_Channel2 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF2)){
		if(!(DMA1_Channel2 -> CCR & DMA_CCR_CIRC)) DMA1_Channel2 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF2;
		DMA1_Channel2_HalfTx_CallBack();
	}
	else if((DMA1_Channel2 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF2)){
		if(!(DMA1_Channel2 -> CCR & DMA_CCR_CIRC)) DMA1_Channel2 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF2;
		DMA1_Channel2_TxCplt_CallBack();
	}
	else if((DMA1_Channel2 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF2)){
		DMA1_Channel2 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF2;
		DMA1_Channel2_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel2_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel2_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel2_TxError_CallBack(void){}
#endif

#ifdef USE_DMA1_Channel3
void DMA1_Channel3_IRQHandler(void){
	if((DMA1_Channel3 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF3)){
		if(!(DMA1_Channel3 -> CCR & DMA_CCR_CIRC)) DMA1_Channel3 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF3;
		DMA1_Channel3_HalfTx_CallBack();
	}
	else if((DMA1_Channel3 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF3)){
		if(!(DMA1_Channel3 -> CCR & DMA_CCR_CIRC)) DMA1_Channel3 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF3;
		DMA1_Channel3_TxCplt_CallBack();
	}
	else if((DMA1_Channel3 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF3)){
		DMA1_Channel3 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF3;
		DMA1_Channel3_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel3_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel3_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel3_TxError_CallBack(void){}
#endif

#ifdef USE_DMA1_Channel4
void DMA1_Channel4_IRQHandler(void){
	if((DMA1_Channel4 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF4)){
		if(!(DMA1_Channel4 -> CCR & DMA_CCR_CIRC)) DMA1_Channel4 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF4;
		DMA1_Channel4_HalfTx_CallBack();
	}
	else if((DMA1_Channel4 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF4)){
		if(!(DMA1_Channel4 -> CCR & DMA_CCR_CIRC)) DMA1_Channel4 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF4;
		DMA1_Channel4_TxCplt_CallBack();
	}
	else if((DMA1_Channel4 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF4)){
		DMA1_Channel4 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF4;
		DMA1_Channel4_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel4_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel4_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel4_TxError_CallBack(void){}
#endif

#ifdef USE_DMA1_Channel5
void DMA1_Channel5_IRQHandler(void){
	if((DMA1_Channel5 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF5)){
		if(!(DMA1_Channel5 -> CCR & DMA_CCR_CIRC)) DMA1_Channel5 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF5;
		DMA1_Channel5_HalfTx_CallBack();
	}
	else if((DMA1_Channel5 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF5)){
		if(!(DMA1_Channel5 -> CCR & DMA_CCR_CIRC)) DMA1_Channel5 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF5;
		DMA1_Channel5_TxCplt_CallBack();
	}
	else if((DMA1_Channel5 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF5)){
		DMA1_Channel5 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF5;
		DMA1_Channel5_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel5_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel5_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel5_TxError_CallBack(void){}
#endif

#ifdef USE_DMA1_Channel6
void DMA1_Channel6_IRQHandler(void){
	if((DMA1_Channel6 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF6)){
		if(!(DMA1_Channel6 -> CCR & DMA_CCR_CIRC)) DMA1_Channel6 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF6;
		DMA1_Channel6_HalfTx_CallBack();
	}
	else if((DMA1_Channel6 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF6)){
		if(!(DMA1_Channel6 -> CCR & DMA_CCR_CIRC)) DMA1_Channel6 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF6;
		DMA1_Channel6_TxCplt_CallBack();
	}
	else if((DMA1_Channel6 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF6)){
		DMA1_Channel6 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF6;
		DMA1_Channel6_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel6_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel6_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel6_TxError_CallBack(void){}
#endif

#ifdef USE_DMA1_Channel7
void DMA1_Channel7_IRQHandler(void){
	if((DMA1_Channel7 -> CCR & DMA_CCR_HTIE) && (DMA1 -> ISR & DMA_ISR_HTIF7)){
		if(!(DMA1_Channel7 -> CCR & DMA_CCR_CIRC)) DMA1_Channel7 -> CCR &=~ DMA_CCR_HTIE;
		DMA1 -> IFCR = DMA_IFCR_CHTIF7;
		DMA1_Channel7_HalfTx_CallBack();
	}
	else if((DMA1_Channel7 -> CCR & DMA_CCR_TCIE) && (DMA1 -> ISR & DMA_ISR_TCIF7)){
		if(!(DMA1_Channel7 -> CCR & DMA_CCR_CIRC)) DMA1_Channel7 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE);
		DMA1 -> IFCR = DMA_IFCR_CTCIF7;
		DMA1_Channel7_TxCplt_CallBack();
	}
	else if((DMA1_Channel7 -> CCR & DMA_CCR_TEIE) && (DMA1 -> ISR & DMA_ISR_TEIF7)){
		DMA1_Channel7 -> CCR &=~ (DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_HTIE);
		DMA1 -> IFCR = DMA_IFCR_CGIF7;
		DMA1_Channel7_TxError_CallBack();
	}
}
__WEAK void DMA1_Channel7_HalfTx_CallBack(void){}
__WEAK void DMA1_Channel7_TxCplt_CallBack(void){}
__WEAK void DMA1_Channel7_TxError_CallBack(void){}
#endif













