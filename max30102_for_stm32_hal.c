#include "max30102_for_stm32_hal.h"

void max30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c)
{
    obj->ui2c = hi2c;
}

void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, size_t buflen)
{
    uint8_t *payload = (uint8_t *)malloc((buflen + 1) * sizeof(uint8_t));
    *payload = reg;
    if (buf != NULL && buflen != 0)
        memcpy(payload + 1, buf, buflen);
    HAL_I2C_Master_Transmit(obj->ui2c, MAX30102_I2C_ADDR << 1, payload, buflen + 1, 1000);
    free(payload);
}

void max30102_read(max30102_t *obj, uint8_t *buf, size_t buflen)
{
    HAL_I2C_Master_Receive(obj->ui2c, MAX30102_I2C_ADDR << 1, buf, buflen, 1000);
}