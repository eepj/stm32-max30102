#ifndef MAX30102_FOR_STM32_HAL_H
#define MAX30102_FOR_STM32_HAL_H

#define MAX30102_I2C_ADDR 0x57

typedef struct max30102
{
    
} max30102;

void max30102_write(uint8_t reg, uint8_t *payload);
void max30102_read(uint8_t reg, uint8_t *payload);

#endif