#ifndef MAX30102_FOR_STM32_HAL_H
#define MAX30102_FOR_STM32_HAL_H

#include "../Inc/main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#define MAX30102_I2C_ADDR 0x57
#define MAX30102_I2C_TIMEOUT 1000

#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL           7
#define MAX30102_INTERRUPT_PPG_RDY          6
#define MAX30102_INTERRUPT_ALC_OVF          5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY     1

#define MAX30102_FIFO_WR_PTR        0x04
#define MAX30102_OVF_COUNTER        0x05
#define MAX30102_FIFO_RD_PTR        0x06

#define MAX30102_FIFO_DATA          0x07

#define MAX30102_MODE_CONFIG        0x09
#define MAX30102_MODE_SHDN                  7
#define MAX30102_MODE_RESET                 6
#define MAX30102_MODE_MODE                  0

#define MAX30102_SPO2_CONFIG        0x0a
#define MAX30102_SPO2_ADC_RGE               5
#define MAX30102_SPO2_SR                    2
#define MAX30102_SPO2_LEW_PW                0

#define MAX30102_LED_IR_PA1         0x0c
#define MAX30102_LED_RED_PA2        0x0d

#define MAX30102_MULTI_LED_CTRL_1   0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2       4
#define MAX30102_MULTI_LED_CTRL_SLOT1       0
#define MAX30102_MULTI_LED_CTRL_2   0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4       4
#define MAX30102_MULTI_LED_CTRL_SLOT3       0


typedef enum max30102_mode_t {
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

typedef enum max30102_spo2_sr_t {
    max30102_spo2_50,
    max30102_spo2_100,
    max30102_spo2_200,
    max30102_spo2_400,
    max30102_spo2_800,
    max30102_spo2_1000,
    max30102_spo2_1600,
    max30102_spo2_3200
} max30102_spo2_sr_t;

typedef enum max30102_spo2_led_pw_t {
    max30102_spo2_15_bit,
    max30102_spo2_16_bit,
    max30102_spo2_17_bit,
    max30102_spo2_18_bit
} max30102_spo2_led_pw_t;

typedef enum max30102_spo2_adc_t {
    max30102_spo2_adc_00,
    max30102_spo2_adc_01,
    max30102_spo2_adc_10,
    max30102_spo2_adc_11
} max30102_spo2_adc_t;

typedef enum max30102_multi_led_ctrl_t {
    max30102_led_off,
    max30102_led_red,
    max30102_led_ir
} max30102_multi_led_ctrl_t;

typedef struct max30102_t
{
    I2C_HandleTypeDef *ui2c;
    max30102_mode_t _mode;
    uint16_t _ir_sample[16];
    uint16_t _red_sample[16];
} max30102_t;

void max30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c);
void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, size_t buflen);
void max30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, size_t buflen);

#endif