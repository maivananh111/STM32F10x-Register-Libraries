/*
 * Font.h
 *
 *  Created on: 3 thg 6, 2022
 *      Author: A315-56
 */

#ifndef FONT_H_
#define FONT_H_


#include <stdint.h>

#define UTF8_start1 1
#define UTF8_end1   31
#define UTF8_start2 127
const uint32_t UTF8_table[] = {
	0x0000C380, // À
	0x0000C381, // Á
	0x00E1BAA2, // Ả
	0x0000C383, // Ã
	0x00E1BAA0, // Ạ
	0x0000C482, // Ă
	0x00E1BAB0, // Ằ
	0x00E1BAAE, // Ắ
	0x00E1BAB2, // Ẳ
	0x00E1BAB4, // Ẵ
	0x00E1BAB6, // Ặ
	0x0000C382, // Â
	0x00E1BAA6, // Ầ
	0x00E1BAA4, // Ấ
	0x00E1BAA8, // Ẩ
	0x00E1BAAA, // Ẫ
	0x00E1BAAC, // Ậ
	0x0000C490, // Đ
	0x0000C388, // È
	0x0000C389, // É
	0x00E1BABA, // Ẻ
	0x00E1BABC, // Ẽ
	0x00E1BAB8, // Ẹ
	0x0000C38A, // Ê
	0x00E1BB80, // Ề
	0x00E1BABE, // Ế
	0x00E1BB82, // Ể
	0x00E1BB84, // Ễ
	0x00E1BB86, // Ệ
	0x0000C38C, // Ì
	0x0000C38D, // Í
	0x00E1BB88, // Ỉ
	0x0000C4A8, // Ĩ
	0x00E1BB8A, // Ị
	0x0000C392, // Ò
	0x0000C393, // Ó
	0x00E1BB8E, // Ỏ
	0x0000C395, // Õ
	0x00E1BB8C, // Ọ
	0x0000C394, // Ô
	0x00E1BB92, // Ồ
	0x00E1BB90, // Ố
	0x00E1BB94, // Ổ
	0x00E1BB96, // Ỗ
	0x00E1BB98, // Ộ
	0x0000C6A0, // Ơ
	0x00E1BB9C, // Ờ
	0x00E1BB9A, // Ớ
	0x00E1BB9E, // Ở
	0x00E1BBA0, // Ỡ
	0x00E1BBA2, // Ợ
	0x0000C399, // Ù
	0x0000C39A, // Ú
	0x00E1BBA6, // Ủ
	0x0000C5A8, // Ũ
	0x00E1BBA4, // Ụ
	0x0000C6AF, // Ư
	0x00E1BBAA, // Ừ
	0x00E1BBA8, // Ứ
	0x00E1BBAC, // Ử
	0x00E1BBAE, // Ữ
	0x00E1BBB0, // Ự
	0x00E1BBB2, // Ỳ
	0x0000C39D, // Ý
	0x00E1BBB6, // Ỷ
	0x00E1BBB8, // Ỹ
	0x00E1BBB4, // Ỵ
	0x0000C3A0, // à
	0x0000C3A1, // á
	0x00E1BAA3, // ả
	0x0000C3A3, // ã
	0x00E1BAA1, // ạ
	0x0000C483, // ă
	0x00E1BAB1, // ằ
	0x00E1BAAF, // ắ
	0x00E1BAB3, // ẳ
	0x00E1BAB5, // ẵ
	0x00E1BAB7, // ặ
	0x0000C3A2, // â
	0x00E1BAA7, // ầ
	0x00E1BAA5, // ấ
	0x00E1BAA9, // ẩ
	0x00E1BAAB, // ẫ
	0x00E1BAAD, // ậ
	0x0000C491, // đ
	0x0000C3A8, // è
	0x0000C3A9, // é
	0x00E1BABB, // ẻ
	0x00E1BABD, // ẽ
	0x00E1BAB9, // ẹ
	0x0000C3AA, // ê
	0x00E1BB81, // ế
	0x00E1BABF, // ề
	0x00E1BB83, // ể
	0x00E1BB85, // ễ
	0x00E1BB87, // ệ
	0x0000C3AC, // ì
	0x0000C3AD, // í
	0x00E1BB89, // ỉ
	0x0000C4A9, // ĩ
	0x00E1BB8B, // ị
	0x0000C3B2, // ò
	0x0000C3B3, // ó
	0x00E1BB8F, // ỏ
	0x0000C3B5, // õ
	0x00E1BB8D, // ọ
	0x0000C3B4, // ô
	0x00E1BB93, // ố
	0x00E1BB91, // ồ
	0x00E1BB95, // ổ
	0x00E1BB97, // ỗ
	0x00E1BB99, // ộ
	0x0000C6A1, // ơ
	0x00E1BB9D, // ờ
	0x00E1BB9B, // ớ
	0x00E1BB9F, // ở
	0x00E1BBA1, // ỡ
	0x00E1BBA3, // ợ
	0x0000C3B9, // ù
	0x0000C3BA, // ú
	0x00E1BBA7, // ủ
	0x0000C5A9, // ũ
	0x00E1BBA5, // ụ
	0x0000C6B0, // ư
	0x00E1BBAB, // ừ
	0x00E1BBA9, // ứ
	0x00E1BBAD, // ử
	0x00E1BBAF, // ữ
	0x00E1BBB1, // ự
	0x00E1BBB3, // ỳ
	0x0000C3BD, // ý
	0x00E1BBB7, // ỷ
	0x00E1BBB9, // ỹ
	0x00E1BBB5, // ỵ
};

typedef struct {
	uint8_t  *bitmap;
	uint8_t  w;
    uint8_t  h;
	uint8_t  byte;
} Font;

extern const Font Viet_Terminal8x20;


#endif /* FONT_H_ */
