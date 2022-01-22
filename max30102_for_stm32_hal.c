#include "max30102_for_stm32_hal.h"

void max30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c)
{
    obj->ui2c = hi2c;
}

void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t *payload = (uint8_t *)malloc((buflen + 1) * sizeof(uint8_t));
    *payload = reg;
    if (buf != NULL && buflen != 0)
        memcpy(payload + 1, buf, buflen);
    HAL_I2C_Master_Transmit(obj->ui2c, MAX30102_I2C_ADDR << 1, payload, buflen + 1, MAX30102_I2C_TIMEOUT);
    free(payload);
}

void max30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t reg_addr = reg;
    HAL_I2C_Master_Transmit(obj->ui2c, MAX30102_I2C_ADDR << 1, &reg_addr, 1, MAX30102_I2C_TIMEOUT);
    HAL_I2C_Master_Receive(obj->ui2c, MAX30102_I2C_ADDR << 1, buf, buflen, MAX30102_I2C_TIMEOUT);
}

void max30102_enable_interrupt(max30102_t *obj, uint8_t a_full, uint8_t ppg_rdy, uint8_t alc_ovf, uint8_t die_temp_rdy)
{
    uint8_t interrupt_enabled[2] = {0x00};
    interrupt_enabled[0] |= 1 << MAX30102_INTERRUPT_A_FULL;
    interrupt_enabled[0] |= 1 << MAX30102_INTERRUPT_PPG_RDY;
    interrupt_enabled[0] |= 1 << MAX30102_INTERRUPT_ALC_OVF;
    interrupt_enabled[1] |= 1 << MAX30102_INTERRUPT_DIE_TEMP_RDY;
    max30102_write(obj, MAX30102_INTERRUPT_ENABLE_1, interrupt_enabled, 2);
}

void max30102_interrupt_handler(max30102_t *obj)
{
    uint8_t interrupt[2] = {0x00};
    max30102_read(obj, MAX30102_INTERRUPT_STATUS_1, interrupt, 2);
    // To be implemented
}

void max30102_shutdown(max30102_t *obj, uint8_t shdn)
{
    uint8_t config;
    max30102_read(obj, MAX30102_MODE_CONFIG, &config, 1);
    config = (config & ~(1 << MAX30102_MODE_SHDN)) | (shdn << MAX30102_MODE_SHDN);
    max30102_write(obj, MAX30102_MODE_CONFIG, &config, 1);
}

void max30102_set_mode(max30102_t *obj, max30102_mode_t mode)
{
    uint8_t config;
    obj->_mode = mode;
    max30102_read(obj, MAX30102_MODE_CONFIG, &config, 1);
    config = (config & 0xf8) | mode;
    max30102_write(obj, MAX30102_MODE_CONFIG, &config, 1);
}

void max30102_set_spo2_sampling_rate(max30102_t *obj, max30102_spo2_sr_t sr)
{
    uint8_t config;
    max30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x63) << MAX30102_SPO2_SR;
    max30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);
}

void max30102_set_spo2_led_pulse_width(max30102_t *obj, max30102_spo2_led_pw_t pw)
{
    uint8_t config;
    max30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x7c) | (pw << MAX30102_SPO2_LEW_PW);
    max30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);
}

void max30102_set_spo2_adc_resolution(max30102_t *obj, max30102_spo2_adc_t adc)
{
    uint8_t config;
    max30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x1f) | (adc << MAX30102_SPO2_ADC_RGE);
    max30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);
}

void max30102_set_led_current_1(max30102_t *obj, float ma)
{
    if (ma < 0)
        ma = 0;
    else if (ma > 51.0)
        ma = 51.0;
    uint8_t pa = ma / 0.2;
    max30102_write(obj, MAX30102_LED_IR_PA1, &pa, 1);
}

void max30102_set_led_current_2(max30102_t *obj, float ma)
{
    if (ma < 0)
        ma = 0;
    else if (ma > 51.0)
        ma = 51.0;
    uint8_t pa = ma / 0.2;
    max30102_write(obj, MAX30102_LED_RED_PA2, &pa, 1);
}

void max30102_set_fifo_config(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full){
    uint8_t config = 0x00;
    config |= smp_ave << MAX30102_FIFO_CONFIG_SMP_AVE;
    config |= ((roll_over_en & 0x01) << MAX30102_FIFO_CONFIG_ROLL_OVER_EN);
    config |= ((fifo_a_full & 0x0f) << MAX30102_FIFO_CONFIG_FIFO_A_FULL);
    max30102_write(obj, MAX30102_FIFO_CONFIG, &config, 1);
}

void max30102_clear_fifo(max30102_t *obj)
{
    uint8_t val = 0x00;
    max30102_write(obj, MAX30102_FIFO_WR_PTR, &val, 1);
    max30102_write(obj, MAX30102_FIFO_RD_PTR, &val, 1);
    max30102_write(obj, MAX30102_OVF_COUNTER, &val, 1);
}

void max30102_read_fifo(max30102_t *obj)
{
    uint8_t ptr[3] = {0x00};
    max30102_read(obj, MAX30102_FIFO_WR_PTR, ptr, 1);

    uint8_t number_of_samples = ptr[2] - ptr[0];
    uint8_t fifo_data[64] = {0};
    max30102_read(obj, MAX30102_FIFO_DATA, fifo_data, number_of_samples);

    for (uint8_t i = 0; i < number_of_samples; i += 4)
    {
        obj->_ir_sample[i / 4] = (fifo_data[i] << 8) | fifo_data[i + 1];
        obj->_red_sample[i / 4] = (fifo_data[i + 2] << 8) | fifo_data[i + 3];
    }
}
