/*
 * TFT_ST7735.h
 *
 *  Created on: 3 thg 6, 2022
 *      Author: A315-56
 */

#ifndef TFT_ST7735_H_
#define TFT_ST7735_H_

#pragma once

#include "stdio.h"
#include <stdbool.h>
#include "stm32f1xx.h"
#include "Font.h"
#include "TFT_Setup.h"

#include "SPI_F1.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#define USE_EX_FLASH

#ifdef USE_EX_FLASH
#include "W25Qxx.h"
#endif

#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFDA0      /* 255, 180,   0 */
#define GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define PINK        0xFC9F


extern volatile uint8_t dma_tft_flag;
extern volatile int8_t dma_tft_cnt;

typedef enum{
	COLOR_RGB = ST7735_MADCTL_RGB,
	COLOR_BGR = ST7735_MADCTL_BGR
} Color_Format;

class TFT_ST7735{
	public:
		const uint16_t COLOR[19] = {BLACK, NAVY, DARKGREEN, DARKCYAN, MAROON, PURPLE, OLIVE, LIGHTGREY, DARKGREY, BLUE, GREEN, CYAN, RED, MAGENTA, YELLOW, WHITE, ORANGE, GREENYELLOW, PINK};

		TFT_ST7735(GPIO_TypeDef *CS_PORT, uint16_t CS_PIN, GPIO_TypeDef *RST_PORT, uint16_t RST_PIN, GPIO_TypeDef *DC_PORT, uint16_t DC_PIN);
		void Init(SPI<uint8_t> *spi, Color_Format Format);
		void SetRotation(uint8_t m);
		void SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
		void WriteColor(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *Data, uint16_t size);
		void DrawPixel(uint16_t x, uint16_t y, uint16_t color);
		void FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
		void FillScreen(uint16_t color);
		void DrawRGBBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* Bitmap);
		void DrawMkrEBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t Color, uint16_t B_Color, const uint8_t* Bitmap);
		uint8_t DrawChar(uint16_t x, uint16_t y, Font Font, uint16_t Color, uint16_t B_Color, char *Char);
		void Print(uint16_t x, uint16_t y, Font Font, uint16_t Color, uint16_t B_Color, char *str);

#ifdef USE_EX_FLASH
		void WriteRGBBitmapToFlash(W25Q w25q, uint32_t Write_Address, uint16_t w, uint16_t h, const uint16_t *Bitmap);
		void DrawRGBBitmapInFlash(W25Q w25q, uint32_t Read_Address, uint16_t x, uint16_t y);

#endif
	private:
		GPIO_TypeDef *CS_Port, *RST_Port, *DC_Port;
		uint16_t CS_Pin, RST_Pin, DC_Pin;
		uint8_t X_Start, Y_Start;
		Color_Format _format = COLOR_RGB;

		void Active(void);
		void Idle(void);
		void EnableWrite(void);
		void Reset(void);
		void WriteCommand(uint8_t cmd);
		void WriteData(uint8_t* buff, size_t buff_size);
		void InitCommandList(const uint8_t *addr);
		uint8_t UTF8_GetAddr(char *utf8_char, uint8_t *char_offset);
};


#ifdef __cplusplus
}
#endif

#endif /* TFT_ST7735_H_ */
