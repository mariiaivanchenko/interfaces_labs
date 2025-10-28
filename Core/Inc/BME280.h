/*
 * BME280.h
 *
 *  Created on: Oct 25, 2025
 *      Author: mariia
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define BME280_addr 0x76

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t	 dig_H1;
    int16_t	 dig_H2;
    uint8_t	 dig_H3;
    int16_t	 dig_H4;
    int16_t	 dig_H5;
    int8_t	 dig_H6;
} BME280_CalibData;

void BME280_init(I2C_HandleTypeDef *hi2c);
void BME280_scan_address(I2C_HandleTypeDef *hi2c);
void BME280_read_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n);
void BME280_write_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n);

void BME280_get_raw_data(I2C_HandleTypeDef *hi2c, int32_t *raw_temp, int32_t *raw_pres, int32_t *raw_hum);
void BME280_calculate_T(I2C_HandleTypeDef *hi2c, int32_t *t_fineG, int32_t raw_temp, float *T);
void BME280_calculate_P(I2C_HandleTypeDef *hi2c, int32_t t_fine, int32_t raw_pres, float *P);
void BME280_calculate_H(I2C_HandleTypeDef *hi2c, int32_t t_fine, int32_t raw_hum, float *H);

#endif /* INC_BME280_H_ */
