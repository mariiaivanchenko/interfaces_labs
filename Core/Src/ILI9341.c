/*
 * ILI9341.c
 *
 *  Created on: Oct 2, 2025
 *      Author: mariia
 */

#include "ILI9341.h"

float tCSS = 0.001;
float tCSH = 0.001;

uint32_t ILI9341_WIDTH = 240;
uint32_t ILI9341_HEIGHT = 320;

void Screen_Tiles_init(Screen *screen) {
	uint16_t start_x = 0;
	uint16_t start_y = 0;
	int idx = 0;
	for (int row = 0; row < 4; row++) {
		start_x = 0;
		for (int col = 0; col < 4; col++) {
			screen->tiles[idx].point_x = start_x;
			screen->tiles[idx].point_y = start_y;
			start_x += 60;
			idx++;
		}
		start_y += 80;
	}
}

void Screen_fill_tile(SPI_HandleTypeDef *hspi, Tile *tile, uint16_t color) {
	ILI9341_set_window(hspi, tile->point_x, tile->point_y, tile->point_x + 60 - 1, tile->point_y + 80 - 1);

	uint32_t total_pixels = 60 * 80;
	const uint32_t pixels_in_chunk = 128;
	uint8_t chunk[pixels_in_chunk * 2];
	uint8_t hi = color >> 8;
	uint8_t lo = color & 0xFF;

	for (uint32_t i = 0; i < pixels_in_chunk; ++i) {
		chunk[2*i] = hi;
		chunk[2*i + 1] = lo;
	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // High -> data
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // CS -> Low
	HAL_Delay(tCSS);

	uint32_t sent_pixels = 0;
	while (sent_pixels < total_pixels) {
		uint32_t remain = total_pixels - sent_pixels;
		uint32_t to_send = (remain >= pixels_in_chunk) ? pixels_in_chunk : remain;
		HAL_SPI_Transmit_DMA(hspi, chunk, to_send * 2);
		while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
		sent_pixels += to_send;
	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // CS -> High
	HAL_Delay(tCSH);
}


void ILI9341_send_command(SPI_HandleTypeDef *hspi, uint8_t cmd) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // Low -> command
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // CS -> Low
    HAL_Delay(tCSS); // 1000ns -> 0.001 ms
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // CS -> High
    HAL_Delay(tCSH);
}

void ILI9341_send_data(SPI_HandleTypeDef *hspi, uint8_t *buff, size_t buff_size) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // High -> data
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // CS -> Low
    HAL_Delay(tCSS);
    HAL_SPI_Transmit_DMA(hspi, buff, buff_size);
	while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // CS -> High
    HAL_Delay(tCSH);
}

void ILI9341_reset(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(20);
}

void ILI9341_init(SPI_HandleTypeDef *hspi) {
    ILI9341_reset();

    // software reset
    ILI9341_send_command(hspi, 0x01);
    HAL_Delay(5);

    // display OFF
    ILI9341_send_command(hspi, 0x28);

    // setting pixel format
    ILI9341_send_command(hspi, 0x3A);

    uint8_t data = 0x55; // 16-bit per pixel, 5-6-5 RGB
    ILI9341_send_data(hspi, &data, 1);

    // waking up the display
    ILI9341_send_command(hspi, 0x11);
    HAL_Delay(120);

    // display ON
    ILI9341_send_command(hspi, 0x29);
    HAL_Delay(20);
}

void ILI9341_set_window(SPI_HandleTypeDef *hspi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    // setting column address
    ILI9341_send_command(hspi, 0x2A);
    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;
    ILI9341_send_data(hspi, data, 4);

    // setting page address
    ILI9341_send_command(hspi, 0x2B);
    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;
    ILI9341_send_data(hspi, data, 4);

    // Memory write
    ILI9341_send_command(hspi, 0x2C);
}

void ILI9341_fill_screen(SPI_HandleTypeDef *hspi, Screen *screen, uint16_t color) {
	 for (int i = 0; i < 16; i++) {
		 Screen_fill_tile(hspi, &screen->tiles[i], color);
	 }
	 HAL_Delay(1000);
}

uint8_t get_char_index(char c) {
    switch(c) {
        case 'A': return 0;
        case 'L': return 1;
        case 'I': return 2;
        case 'D': return 3;
        case ':': return 4;
        case 'T': return 5;
        case 'O': return 6;
        case 'N': return 7;
        case 'U': return 8;
        case 'C': return 9;
        case ' ': return 10;
        case '0': return 11;
        case '1': return 12;
        case '2': return 13;
        case '3': return 14;
        case '4': return 15;
        case '5': return 16;
        case '6': return 17;
        case '7': return 18;
        case '8': return 19;
        case '9': return 20;
        case '.': return 21;
        case '-': return 22;
        case 'x': return 23;
        default:  return 0;
    }
}



void ILI9341_draw_char(SPI_HandleTypeDef *hspi, char c, uint16_t x, uint16_t y, uint16_t color) {

	const uint8_t *bitmap = font6x8[get_char_index(c)];
	const uint8_t width = 6;
	const uint8_t height = 8;

	uint16_t buf[width * height];

	    for (uint8_t col = 0; col < height; col++) {
	        for (uint8_t row = 0; row < width; row++) {
	            if (bitmap[col] & (1 << row)) {
	                buf[col * width + row] = color; // pixel ON
	            } else {
	                buf[col * width + row] = 0x0000; // pixel OFF (black)
	            }
	        }
	    }

	ILI9341_set_window(hspi, x, y, x + width - 1, y + height - 1);
	ILI9341_send_data(hspi, buf, width * height * 2);
}

void ILI9341_draw_text(SPI_HandleTypeDef *hspi, Screen *screen, uint16_t x, uint16_t y, const char *str, uint16_t color) {
    uint16_t cursor_x = x;
    while (*str) {
    	ILI9341_draw_char(hspi, *str, cursor_x, y, color);
    	cursor_x -= 7;
        str++;
    }
}


