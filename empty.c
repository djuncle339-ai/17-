/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "Drivers/detector.h"

#define ENABLE_OLED_DEBUG 0

#if ENABLE_OLED_DEBUG
#include "Drivers/OLED_Hardware_I2C/oled_hardware_i2c.h"
#include <stdio.h>
#endif

#define motor_bianma 260  // r/s
#define motor_wheel 45    // mm
#define Pi 3.14f
#define ENCODER_X4_MULTIPLIER   4.0f

#define MOTOR_PI_DT_S           0.01f
#define MOTOR_SPEED_SCALE       (1.0f / MOTOR_PI_DT_S)
#define MOTOR_PWM_MAX           3200U
#define MOTOR_PWM_LIMIT         2560U
#define MOTOR_PWM_MIN_START     150U
#define MOTOR_TARGET_DEFAULT    500.0f
#define MOTOR_PI_KP_DEFAULT     1.0f
#define MOTOR_PI_KI_DEFAULT     2.0f

/* MOTOR2 编码器 A/B 相已改由 SysConfig 配置。
 * 如果生成文件还没有同步出 DC_motorB_BA_* 宏，则临时保留 PA25 的手动配置兜底。
 */
#ifndef DC_motorB_BA_PIN
#define MOTORB_BA_MANUAL_CONFIG 1
#define DC_motorB_BA_PORT   GPIOA
#define DC_motorB_BA_PIN    DL_GPIO_PIN_25
#define DC_motorB_BA_IOMUX  IOMUX_PINCM55
#define DC_motorB_BA_IIDX   DL_GPIO_IIDX_DIO25
#else
#define MOTORB_BA_MANUAL_CONFIG 0
#endif

#ifndef DC_motorB_BB_PIN
#define MOTORB_BB_MANUAL_CONFIG 1
#define DC_motorB_BB_PORT   GPIOA
#define DC_motorB_BB_PIN    DL_GPIO_PIN_14
#define DC_motorB_BB_IOMUX  IOMUX_PINCM36
#define DC_motorB_BB_IIDX   DL_GPIO_IIDX_DIO14
#else
#define MOTORB_BB_MANUAL_CONFIG 0
#endif

/* 如果工程没有把 Drivers/MSPM0/clock.c 加进编译，这些 weak 函数也能让 OLED
 * 驱动使用 SysTick 毫秒延时；如果 clock.c 已经参与编译，会自动使用 clock.c 中的实现。
 */
SYSCONFIG_WEAK volatile unsigned long tick_ms;

SYSCONFIG_WEAK int mspm0_delay_ms(unsigned long num_ms)
{
    unsigned long start_time = tick_ms;
    while ((tick_ms - start_time) < num_ms) {
    }
    return 0;
}

SYSCONFIG_WEAK int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count) {
        return 1;
    }
    count[0] = tick_ms;
    return 0;
}

SYSCONFIG_WEAK void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

SYSCONFIG_WEAK void SysTick_Handler(void)
{
    tick_ms++;
}

volatile int32_t counter1A = 0, counter1B = 0, counter2A = 0, counter2B = 0;
volatile float speed1 = 0.0f, speed2 = 0.0f;
volatile float motor1_target_mm_s = MOTOR_TARGET_DEFAULT;
volatile float motor2_target_mm_s = MOTOR_TARGET_DEFAULT;
volatile uint16_t duty1 = 0, duty2 = 0;

static uint8_t encoder1_last_state;
static uint8_t encoder2_last_state;

typedef struct {
    float kp;
    float ki;
    float target_mm_s;
    float integral;
    float output;
    uint16_t duty;
    uint16_t out_max;
    uint16_t min_start;
    uint8_t enabled;
} MotorPI;

static MotorPI motor1_pi;
static MotorPI motor2_pi;

static float MotorPI_ClampFloat(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static void MotorPI_Reset(MotorPI *pi)
{
    pi->integral = 0.0f;
    pi->output = 0.0f;
    pi->duty = 0;
}

static void MotorPI_SetTarget(MotorPI *pi, float target_mm_s)
{
    pi->target_mm_s = target_mm_s;
    if (target_mm_s <= 0.0f) {
        MotorPI_Reset(pi);
    }
}

static void MotorPI_Init(MotorPI *pi, float kp, float ki, float target_mm_s, uint16_t out_max, uint16_t min_start)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->out_max = out_max;
    pi->min_start = min_start;
    pi->enabled = 1;
    MotorPI_Reset(pi);
    MotorPI_SetTarget(pi, target_mm_s);
}

static uint16_t MotorPI_Update(MotorPI *pi, float measured_mm_s)
{
    float error;
    float output;

    if (!pi->enabled || pi->target_mm_s <= 0.0f) {
        MotorPI_Reset(pi);
        return 0;
    }

    error = pi->target_mm_s - measured_mm_s;
    pi->integral += pi->ki * error * MOTOR_PI_DT_S;
    pi->integral = MotorPI_ClampFloat(pi->integral, 0.0f, (float)pi->out_max);

    output = pi->kp * error + pi->integral;
    output = MotorPI_ClampFloat(output, 0.0f, (float)pi->out_max);

    if (output > 0.0f && output < (float)pi->min_start) {
        output = (float)pi->min_start;
    }

    pi->output = output;
    pi->duty = (uint16_t)(output + 0.5f);
    return pi->duty;
}

#if ENABLE_OLED_DEBUG
static uint32_t speed_to_uint(float speed)
{
    if (speed <= 0.0f) {
        return 0;
    }
    if (speed > 99999.0f) {
        return 99999;
    }
    return (uint32_t)(speed + 0.5f);
}

static uint32_t duty_to_uint(uint16_t duty)
{
    if (duty > 9999U) {
        return 9999U;
    }
    return duty;
}

void oled_show_speed(float motor1_speed, float motor2_speed)
{
    OLED_ShowString(0, 0, (uint8_t *)"M1:", 16);
    OLED_ShowNum(24, 0, speed_to_uint(motor1_speed), 5, 16);
    OLED_ShowString(64, 0, (uint8_t *)"D", 16);
    OLED_ShowNum(72, 0, duty_to_uint(duty1), 4, 16);

    OLED_ShowString(0, 2, (uint8_t *)"M2:", 16);
    OLED_ShowNum(24, 2, speed_to_uint(motor2_speed), 5, 16);
    OLED_ShowString(64, 2, (uint8_t *)"D", 16);
    OLED_ShowNum(72, 2, duty_to_uint(duty2), 4, 16);
}

static char line_turn_char(LineTurnDirection direction)
{
    switch (direction)
    {
    case LINE_TURN_LEFT:
        return 'L';
    case LINE_TURN_RIGHT:
        return 'R';
    case LINE_TURN_STRAIGHT:
        return 'S';
    case LINE_TURN_LOST:
    default:
        return 'X';
    }
}

static void oled_clear_8px_line(uint8_t y)
{
    OLED_ShowString(0, y, (uint8_t *)"                     ", 8);
}

static void oled_show_signed_num8(uint8_t x, uint8_t y, int32_t value, uint8_t len)
{
    uint32_t abs_value;

    if (value < 0)
    {
        OLED_ShowChar(x, y, '-', 8);
        abs_value = (uint32_t)(-value);
    }
    else
    {
        OLED_ShowChar(x, y, '+', 8);
        abs_value = (uint32_t)value;
    }

    OLED_ShowNum(x + 6, y, abs_value, len, 8);
}

static void oled_show_error_x10(uint8_t x, uint8_t y, int16_t value_x10)
{
    uint32_t abs_value;

    if (value_x10 < 0)
    {
        OLED_ShowChar(x, y, '-', 8);
        abs_value = (uint32_t)(-value_x10);
    }
    else
    {
        OLED_ShowChar(x, y, '+', 8);
        abs_value = (uint32_t)value_x10;
    }

    OLED_ShowNum(x + 6, y, abs_value / 10U, 1, 8);
    OLED_ShowChar(x + 18, y, '.', 8);
    OLED_ShowNum(x + 24, y, abs_value % 10U, 1, 8);
}

static void oled_show_sensor_mask(uint8_t x, uint8_t y, uint8_t mask)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        OLED_ShowChar(x + i * 6, y, (mask & (1U << i)) ? '1' : '0', 8);
    }
}

static void oled_show_line_follow(const LineFollowState *state)
{
    oled_clear_8px_line(4);
    OLED_ShowString(0, 4, (uint8_t *)"DIR:", 8);
    OLED_ShowChar(24, 4, line_turn_char(state->direction), 8);
    OLED_ShowString(42, 4, (uint8_t *)"W:", 8);
    oled_show_error_x10(54, 4, state->error_x10);

    oled_clear_8px_line(5);
    OLED_ShowString(0, 5, (uint8_t *)"SUM:", 8);
    oled_show_signed_num8(24, 5, state->weighted_sum, 3);
    OLED_ShowString(66, 5, (uint8_t *)"CNT:", 8);
    OLED_ShowNum(90, 5, state->active_count, 1, 8);

    oled_clear_8px_line(6);
    OLED_ShowString(0, 6, (uint8_t *)"L:", 8);
    OLED_ShowNum(12, 6, state->left_target_mm_s, 4, 8);
    OLED_ShowString(48, 6, (uint8_t *)"R:", 8);
    OLED_ShowNum(60, 6, state->right_target_mm_s, 4, 8);
    OLED_ShowString(96, 6, (uint8_t *)"D:", 8);
    oled_show_signed_num8(108, 6, state->delta_mm_s, 2);

    oled_clear_8px_line(7);
    OLED_ShowString(0, 7, (uint8_t *)"MSK:", 8);
    oled_show_sensor_mask(24, 7, state->sensor_mask);
}
#endif

void motor_set_speed(uint8_t id, uint16_t speed)
{
    if (id == 1)
    {
        DL_TimerG_setCaptureCompareValue(PWMA_INST, speed, GPIO_PWMA_C0_IDX);
    }
    if (id == 2)
    {
        DL_TimerG_setCaptureCompareValue(PWMA_INST, speed, GPIO_PWMA_C1_IDX);
    }
}

static uint8_t encoder_get_state(GPIO_Regs *port, uint32_t pin_a, uint32_t pin_b)
{
    uint32_t pins = DL_GPIO_readPins(port, pin_a | pin_b);

    return ((pins & pin_a) ? 0x02U : 0U) | ((pins & pin_b) ? 0x01U : 0U);
}

static int8_t encoder_x4_delta(uint8_t last_state, uint8_t current_state)
{
    static const int8_t delta_table[16] = {
        0,  1, -1,  0,
       -1,  0,  0,  1,
        1,  0,  0, -1,
        0, -1,  1,  0
    };

    return delta_table[((last_state & 0x03U) << 2) | (current_state & 0x03U)];
}

static void encoder_update(volatile int32_t *counter, uint8_t *last_state, uint8_t current_state)
{
    *counter += encoder_x4_delta(*last_state, current_state);
    *last_state = current_state;
}

void GROUP1_IRQHandler()
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        DC_motorA_AA_PIN | DC_motorA_AB_PIN | DC_motorB_BA_PIN | DC_motorB_BB_PIN);

    if (pending & (DC_motorA_AA_PIN | DC_motorA_AB_PIN))
    {
        encoder_update(&counter1A, &encoder1_last_state,
            encoder_get_state(DC_motorA_AA_PORT, DC_motorA_AA_PIN, DC_motorA_AB_PIN));
    }
    if (pending & (DC_motorB_BA_PIN | DC_motorB_BB_PIN))
    {
        encoder_update(&counter2A, &encoder2_last_state,
            encoder_get_state(DC_motorB_BA_PORT, DC_motorB_BA_PIN, DC_motorB_BB_PIN));
    }

    DL_GPIO_clearInterruptStatus(GPIOA, pending);
}

void to_speed(uint8_t id)
{
    int32_t count;

    if(id==1)
    {
        count = counter1A;
        counter1A=0;
        if (count < 0) {
            count = -count;
        }
        speed1=(float)count/(motor_bianma*ENCODER_X4_MULTIPLIER)*Pi*motor_wheel*MOTOR_SPEED_SCALE; // mm/s
    }
    if(id==2)
    {
        count = counter2A;
        counter2A=0;
        if (count < 0) {
            count = -count;
        }
        speed2=(float)count/(motor_bianma*ENCODER_X4_MULTIPLIER)*Pi*motor_wheel*MOTOR_SPEED_SCALE; // mm/s
    }
}

void motor_pid_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(motor_pid_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        to_speed(1);
        to_speed(2);
        if (line_open_loop_active)
        {
            MotorPI_Reset(&motor1_pi);
            MotorPI_Reset(&motor2_pi);
            break;
        }
        MotorPI_SetTarget(&motor1_pi, motor1_target_mm_s);
        MotorPI_SetTarget(&motor2_pi, motor2_target_mm_s);
        duty1 = MotorPI_Update(&motor1_pi, speed1);
        duty2 = MotorPI_Update(&motor2_pi, speed2);
        motor_set_speed(1, duty1);
        motor_set_speed(2, duty2);
        break;

    default:
        break;
    }
}

static void encoder_init_state(void)
{
    encoder1_last_state = encoder_get_state(DC_motorA_AA_PORT, DC_motorA_AA_PIN, DC_motorA_AB_PIN);
    encoder2_last_state = encoder_get_state(DC_motorB_BA_PORT, DC_motorB_BA_PIN, DC_motorB_BB_PIN);
}

static void motorB_encoder_init(void)
{
#if MOTORB_BA_MANUAL_CONFIG
    DL_GPIO_initDigitalInputFeatures(DC_motorB_BA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setUpperPinsPolarity(DC_motorB_BA_PORT, DL_GPIO_PIN_25_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(DC_motorB_BA_PORT, DC_motorB_BA_PIN);
    DL_GPIO_enableInterrupt(DC_motorB_BA_PORT, DC_motorB_BA_PIN);
#endif
#if MOTORB_BB_MANUAL_CONFIG
    DL_GPIO_initDigitalInputFeatures(DC_motorB_BB_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setLowerPinsPolarity(DC_motorB_BB_PORT, DL_GPIO_PIN_14_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(DC_motorB_BB_PORT, DC_motorB_BB_PIN);
    DL_GPIO_enableInterrupt(DC_motorB_BB_PORT, DC_motorB_BB_PIN);
#endif
}

void motor_init()
{
    DL_GPIO_clearPins(DC_motorA_AIN1_PORT, DC_motorA_AIN1_PIN);
    DL_GPIO_setPins(DC_motorA_AIN2_PORT, DC_motorA_AIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C0_IDX);

    DL_GPIO_clearPins(DC_motorB_BIN1_PORT, DC_motorB_BIN1_PIN);
    DL_GPIO_setPins(DC_motorB_BIN2_PORT, DC_motorB_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C1_IDX);

    motorB_encoder_init();
    encoder_init_state();
    DL_Timer_startCounter(motor_pid_INST);
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    motor_init();
    MotorPI_Init(&motor1_pi, MOTOR_PI_KP_DEFAULT, MOTOR_PI_KI_DEFAULT,
        MOTOR_TARGET_DEFAULT, MOTOR_PWM_LIMIT, MOTOR_PWM_MIN_START);
    MotorPI_Init(&motor2_pi, MOTOR_PI_KP_DEFAULT, MOTOR_PI_KI_DEFAULT,
        MOTOR_TARGET_DEFAULT, MOTOR_PWM_LIMIT, MOTOR_PWM_MIN_START);
    motor_set_speed(1, 0);
    motor_set_speed(2, 0);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(motor_pid_INST_INT_IRQN);

    while (1) {
        adjust_motor();
        mspm0_delay_ms(10);
    }
}
