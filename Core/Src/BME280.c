/*
 * BME280.c
 *
 *  Created on: Oct 17, 2025
 *      Author: mariia
 */

#include "BME280.h"

static BME280_CalibData calib_data;
uint8_t message[48];

void BME280_init(I2C_HandleTypeDef *hi2c) {
	uint8_t calib[26];
    uint8_t calib2[7];
	BME280_read_reg(hi2c, BME280_addr, 0x88, calib, 26);
    BME280_read_reg(hi2c, BME280_addr, 0xA1, &calib_data.dig_H1, 1);
    BME280_read_reg(hi2c, BME280_addr, 0xE1, calib2, 7);

	calib_data.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
	calib_data.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
	calib_data.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
	calib_data.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
	calib_data.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
	calib_data.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
	calib_data.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
	calib_data.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
	calib_data.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
	calib_data.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
	calib_data.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
	calib_data.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
	calib_data.dig_H2 = (int16_t)((calib2[1] << 8) | calib2[0]);
	calib_data.dig_H3 = calib2[2];
	calib_data.dig_H4 = (int16_t)((calib2[3] << 4) | (calib2[4] & 0x0F));
	calib_data.dig_H5 = (int16_t)((calib2[5] << 4) | ((calib2[4] >> 4) & 0x0F));
	calib_data.dig_H6 = (int8_t)calib2[6];

	uint8_t config    = 0x90; // t_sb=500ms, filter=8
	uint8_t ctrl_hum  = 0x01; // humidity oversampling x1
	uint8_t ctrl_meas = 0x4B; // temp×2, press×2, normal mode

	HAL_I2C_Mem_Write(hi2c, BME280_addr<<1, 0xF5, 1, &config, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(hi2c, BME280_addr<<1, 0xF2, 1, &ctrl_hum, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_I2C_Mem_Write(hi2c, BME280_addr<<1, 0xF4, 1, &ctrl_meas, 1, HAL_MAX_DELAY);

	HAL_Delay(10);
}

void BME280_scan_address(I2C_HandleTypeDef *hi2c) {
	uint8_t message[32] = {'\0'};
	send_msg_uart("Scanning I2C bus...\r\n", 23);

	for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
		if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 100) == HAL_OK) { // ACK
		  int len = sprintf(message, "I2C Device: 0x%02X\r\n", addr);
		  send_msg_uart(message, len);
		  HAL_Delay(500);

		}
	}
	send_msg_uart("Scan complete.\r\n", 17);
}

void BME280_read_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n) {
    if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 100) == HAL_OK) { // device ready
    	HAL_I2C_Mem_Read(hi2c, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 100);
    }
}


void BME280_write_reg(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n) {
    if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 100) == HAL_OK) { // device ready
    	HAL_I2C_Mem_Write(hi2c, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 100);
    }
}

void BME280_get_raw_data(I2C_HandleTypeDef *hi2c, int32_t *raw_temp, int32_t *raw_pres, int32_t *raw_hum) {

	uint8_t temp_bytes[3];
	BME280_read_reg(hi2c, BME280_addr, 0xFA, temp_bytes, 3);
	*raw_temp = ((int32_t)temp_bytes[0] << 12) |
				((int32_t)temp_bytes[1] << 4) |
				((int32_t)temp_bytes[2] >> 4);

	uint8_t pres_bytes[3];
	BME280_read_reg(hi2c, BME280_addr, 0xF7, pres_bytes, 3);
	*raw_pres = ((int32_t)pres_bytes[0] << 12) |
				((int32_t)pres_bytes[1] << 4) |
				((int32_t)pres_bytes[2] >> 4);

	uint8_t hum_bytes[2];
	BME280_read_reg(hi2c, BME280_addr, 0xFD, hum_bytes, 2);
	*raw_hum = ((int32_t)hum_bytes[0] << 8) | (int32_t)hum_bytes[1];
}

void BME280_calculate_T(I2C_HandleTypeDef *hi2c, int32_t *t_fineG, int32_t raw_temp, float *T) {

	int32_t var1, var2, t_fine;
	var1 = ((((raw_temp >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
	var2 = (((((raw_temp >> 4) - ((int32_t)calib_data.dig_T1)) * ((raw_temp >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
	t_fine = var1 + var2;
	*t_fineG = t_fine;

	int32_t T_int = (t_fine * 5 + 128) >> 8;
	*T = T_int / 100.0f;
}


void BME280_calculate_P(I2C_HandleTypeDef *hi2c, int32_t t_fine, int32_t raw_pres, float *P) {

	int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + ((int64_t)calib_data.dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)calib_data.dig_P1) >> 33;

    if (var1 == 0) {  // avoid division by zero
        *P = 0;
        return;
    }

    p = 1048576 - raw_pres;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)calib_data.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)calib_data.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)calib_data.dig_P7 << 4);

    *P = (float)p / 25600.0f;  // Pressure in hPa
}

void BME280_calculate_H(I2C_HandleTypeDef *hi2c, int32_t t_fine, int32_t raw_hum, float *H) {
	int32_t v_x1;

	v_x1 = t_fine - 76800;

	int32_t v1 = ((raw_hum << 14) - ((int32_t)calib_data.dig_H4 << 20) - ((int32_t)calib_data.dig_H5 * v_x1) + 16384) >> 15;
	int32_t v2 = (((((v_x1 * calib_data.dig_H6) >> 10) * ((v_x1 * calib_data.dig_H3) >> 11)) + 32768) >> 10) + 2097152;
	v2 = ((v2 * calib_data.dig_H2) + 8192) >> 14;

	v_x1 = v1 * v2;

	v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * calib_data.dig_H1) >> 4);

	if (v_x1 < 0) v_x1 = 0;
	if (v_x1 > 419430400) v_x1 = 419430400;

	*H = (uint32_t)(v_x1 >> 12) / 1024.0f; // Q22.10 format
}


