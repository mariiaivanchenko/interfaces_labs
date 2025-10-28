/*
 * ILI9341.h
 *
 *  Created on: Oct 2, 2025
 *      Author: mariia
 */

#ifndef SRC_ILI9341_H_
#define SRC_ILI9341_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "fonts.h"

extern float tCSS;
extern float tCSH;

extern uint32_t ILI9341_WIDTH;
extern uint32_t ILI9341_HEIGHT;

typedef struct {
	uint16_t point_x;
	uint16_t point_y;

} Tile;


typedef struct {
	Tile tiles[16];

} Screen;

void Screen_Tiles_init(Screen *screen);
void Screen_fill_tile(SPI_HandleTypeDef *hspi, Tile *tile, uint16_t color);

void ILI9341_send_command(SPI_HandleTypeDef *hspi, uint8_t cmd);
void ILI9341_send_data(SPI_HandleTypeDef *hspi, uint8_t *buff, size_t buff_size);

void ILI9341_reset(void);
void ILI9341_init(SPI_HandleTypeDef *hspi);

void ILI9341_draw_char(SPI_HandleTypeDef *hspi, char c, uint16_t x, uint16_t y, uint16_t color);
void ILI9341_draw_text(SPI_HandleTypeDef *hspi, Screen *screen, uint16_t x, uint16_t y, const char *str, uint16_t color);
void ILI9341_set_window(SPI_HandleTypeDef *hspi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9341_fill_screen(SPI_HandleTypeDef *hspi, Screen *screen, uint16_t color);


#endif /* SRC_ILI9341_H_ */
