//
// Created by Hina on 2025/7/7.
//
#include "ds1307.h"

#include <stdint.h>

#define DS1307_ADDRESS   (0x68 << 1)  // HAL 要求左移

static I2C_HandleTypeDef *ds1307_i2c;

static uint8_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcd2dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

void DS1307_Init(I2C_HandleTypeDef *hi2c) {
    ds1307_i2c = hi2c;
}

HAL_StatusTypeDef DS1307_SetTime(const DS1307_TimeTypeDef *time) {
    uint8_t data[7];
    data[0] = dec2bcd(time->seconds);
    data[1] = dec2bcd(time->minutes);
    data[2] = dec2bcd(time->hours);
    data[3] = dec2bcd(time->dayOfWeek);
    data[4] = dec2bcd(time->day);
    data[5] = dec2bcd(time->month);
    data[6] = dec2bcd(time->year);

    return HAL_I2C_Mem_Write(ds1307_i2c, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data, 7, HAL_MAX_DELAY);
}

HAL_StatusTypeDef DS1307_GetTime(DS1307_TimeTypeDef *time) {
    uint8_t data[7];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(ds1307_i2c, DS1307_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data, 7, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    time->seconds    = bcd2dec(data[0] & 0x7F);  // bit7 是 CH（时钟停止位）
    time->minutes    = bcd2dec(data[1]);
    time->hours      = bcd2dec(data[2] & 0x3F);  // 24小时制
    time->dayOfWeek  = bcd2dec(data[3]);
    time->day        = bcd2dec(data[4]);
    time->month      = bcd2dec(data[5]);
    time->year       = bcd2dec(data[6]);

    return HAL_OK;
}
