#ifndef MAX30102_FOR_STM32_HAL_H
#define MAX30102_FOR_STM32_HAL_H

#define MAX30102_I2C_ADDR 0x57
#define MAX30102_I2C_TIMEOUT 1000

#include "../Inc/main.h"
#include <stdint.h>
#include <string.h>

typedef struct max30102_t
{
    I2C_HandleTypeDef *ui2c;
} max30102_t;

void max30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c); 
void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, size_t buflen);
void max30102_read(max30102_t *obj, uint8_t *buf, size_t buflen);

#endif