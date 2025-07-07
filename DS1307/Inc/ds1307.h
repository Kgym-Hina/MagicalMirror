#ifndef __DS1307_H
#define __DS1307_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_i2c.h"

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t dayOfWeek;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} DS1307_TimeTypeDef;

void DS1307_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef DS1307_SetTime(const DS1307_TimeTypeDef *time);
HAL_StatusTypeDef DS1307_GetTime(DS1307_TimeTypeDef *time);

#endif
