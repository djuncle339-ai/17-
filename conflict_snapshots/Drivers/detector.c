#include "detector.h"
#include "ti_msp_dl_config.h"

<<<<<<< HEAD
#define LINE_BASE_SPEED_MM_S            120.0f
#define LINE_TURN_DEADBAND_X10          5
#define LINE_OPEN_BASE_PWM              700.0f
#define LINE_OPEN_MIN_PWM               0.0f
#define LINE_OPEN_MAX_PWM               1200.0f
#define LINE_OPEN_KP_PWM_PER_ERR        35.0f
#define LINE_OPEN_KD_PWM_PER_ERR_S      0.08f
#define LINE_OPEN_OUTPUT_LIMIT_PWM      350.0f
#define LINE_OPEN_D_TERM_LIMIT_PWM      80.0f
#define LINE_OPEN_D_FILTER_ALPHA        0.75f
#define LINE_OPEN_MIN_TURN_PWM          80.0f
#define LINE_OPEN_DT_S                  0.010f

/* 置信度/丢线确认逻辑：
 * 八路都读 0 时，不代表车真的偏出黑线了——也可能是线正好压在
 * 两个探头之间的盲区。这里给"丢线"加一个连续确认次数，
 * 在确认次数之内，先沿用上一次的有效误差继续开环转向（相当于
 * "相信"车还在线上），只有连续 LINE_LOST_CONFIRM_COUNT 次
 * (adjust_motor 约 10ms 跑一次，这里约 50ms) 都读不到，才真正
 * 判定丢线、切回基速直行闭环。 */
#define LINE_LOST_CONFIRM_COUNT         5
=======
#define LINE_BASE_SPEED_MM_S            200.0f
#define LINE_MIN_TARGET_MM_S            60.0f
#define LINE_TURN_DEADBAND_X10          5
#define LINE_PD_KP_MM_S_PER_ERR         10.0f
#define LINE_PD_KD_MM_S_PER_ERR_S       0.025f
#define LINE_PD_DT_S                    0.010f
#define LINE_PD_OUTPUT_LIMIT_MM_S       75.0f
#define LINE_PD_D_TERM_LIMIT_MM_S       10.0f
#define LINE_PD_D_FILTER_ALPHA          0.75f
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa

uint8_t detector_value[]={0,0,0,0,0,0,0,0};
volatile uint8_t line_open_loop_active = 0;
static LineFollowState line_state;
static float line_pd_prev_error;
static float line_pd_d_term_mm_s;
static uint8_t line_pd_prev_valid;
static float line_last_valid_error;
static uint8_t line_last_valid_error_valid;
<<<<<<< HEAD
static uint8_t line_lost_count;
=======
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa

static float LineControl_ClampFloat(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

uint8_t getstatus(GPIO_Regs *gpio_port, uint32_t pin)
{
    if(DL_GPIO_readPins(gpio_port, pin)& pin)
        return 1;
    else
        return 0;
}

const LineFollowState *line_follow_get_state(void)
{
    return &line_state;
}

void detector_get_value(void)
{
    detector_value[7]=getstatus(detector_PORT,detector_DETECTORL1_PIN);   // 最左传感器
    detector_value[6]=getstatus(detector_PORT,detector_DETECTORL2_PIN);
    detector_value[5]=getstatus(detector_PORT,detector_DETECTORL3_PIN);
    detector_value[4]=getstatus(detector_PORT,detector_DETECTORL4_PIN);
    detector_value[3]=getstatus(detector_PORT,detector_DETECTORR1_PIN);
    detector_value[2]=getstatus(detector_PORT,detector_DETECTORR2_PIN);
    detector_value[1]=getstatus(detector_PORT,detector_DETECTORR3_PIN);
    detector_value[0]=getstatus(detector_PORT,detector_DETECTORR4_PIN);     // 最右传感器
}

static uint8_t LineDecodeError(uint8_t sensor_mask, float *error)
{
    switch (sensor_mask)
    {
    case 0x80: *error = -7.0f; return 1;
    case 0xC0: *error = -6.0f; return 1;
    case 0x40: *error = -5.0f; return 1;
    case 0x60: *error = -4.0f; return 1;
    case 0x20: *error = -3.0f; return 1;
    case 0x30: *error = -2.0f; return 1;
    case 0x10: *error = -1.0f; return 1;
    case 0x18: *error = 0.0f; return 1;
    case 0x08: *error = 1.0f; return 1;
    case 0x0C: *error = 2.0f; return 1;
    case 0x04: *error = 3.0f; return 1;
    case 0x06: *error = 4.0f; return 1;
    case 0x02: *error = 5.0f; return 1;
    case 0x03: *error = 6.0f; return 1;
    case 0x01: *error = 7.0f; return 1;
    default: return 0;
    }
}

<<<<<<< HEAD
static void LineResetOpenLoop(void)
=======
static void LineResetClosedLoop(void)
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa
{
    line_pd_prev_error = 0.0f;
    line_pd_d_term_mm_s = 0.0f;
    line_pd_prev_valid = 0;
}

<<<<<<< HEAD
static void LineApplyOpenLoop(float error, float *left_pwm, float *right_pwm)
{
    float derivative = 0.0f;
    float d_raw = 0.0f;
    float turn_pwm;

    if (line_pd_prev_valid)
    {
        derivative = (error - line_pd_prev_error) / LINE_OPEN_DT_S;
        d_raw = LINE_OPEN_KD_PWM_PER_ERR_S * derivative;
        d_raw = LineControl_ClampFloat(d_raw,
            -LINE_OPEN_D_TERM_LIMIT_PWM, LINE_OPEN_D_TERM_LIMIT_PWM);
        line_pd_d_term_mm_s = LINE_OPEN_D_FILTER_ALPHA * line_pd_d_term_mm_s +
            (1.0f - LINE_OPEN_D_FILTER_ALPHA) * d_raw;
    }
    else
    {
        line_pd_d_term_mm_s = 0.0f;
        line_pd_prev_valid = 1;
    }

    line_pd_prev_error = error;

    turn_pwm = LINE_OPEN_KP_PWM_PER_ERR * error + line_pd_d_term_mm_s;
    if (turn_pwm > 0.0f)
    {
        turn_pwm += LINE_OPEN_MIN_TURN_PWM;
    }
    else if (turn_pwm < 0.0f)
    {
        turn_pwm -= LINE_OPEN_MIN_TURN_PWM;
    }
    turn_pwm = LineControl_ClampFloat(turn_pwm,
        -LINE_OPEN_OUTPUT_LIMIT_PWM, LINE_OPEN_OUTPUT_LIMIT_PWM);

    *left_pwm = LineControl_ClampFloat(LINE_OPEN_BASE_PWM + turn_pwm,
        LINE_OPEN_MIN_PWM, LINE_OPEN_MAX_PWM);
    *right_pwm = LineControl_ClampFloat(LINE_OPEN_BASE_PWM - turn_pwm,
        LINE_OPEN_MIN_PWM, LINE_OPEN_MAX_PWM);
}

static void LineSteerFromError(float error, int16_t *weighted_sum,
    int16_t *error_x10, float *left_output, float *right_output,
    float *turn_delta, uint8_t *open_loop_active)
{
    extern void motor_set_speed(uint8_t id, uint16_t speed);

    *weighted_sum = (int16_t)error;
    *error_x10 = (int16_t)(error * 10.0f);

    if (*error_x10 > LINE_TURN_DEADBAND_X10)
    {
        line_state.direction = LINE_TURN_RIGHT;
    }
    else if (*error_x10 < -LINE_TURN_DEADBAND_X10)
    {
        line_state.direction = LINE_TURN_LEFT;
    }
    else
    {
        LineResetOpenLoop();
        line_state.direction = LINE_TURN_STRAIGHT;
    }

    LineApplyOpenLoop(error, left_output, right_output);
    *turn_delta = *left_output - LINE_OPEN_BASE_PWM;
    *open_loop_active = 1;

    motor_set_speed(1, (uint16_t)(*right_output + 0.5f));
    motor_set_speed(2, (uint16_t)(*left_output + 0.5f));
}

=======
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa
void adjust_motor()
{
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;
    uint8_t sensor_mask = 0;
    uint8_t valid_error = 0;
<<<<<<< HEAD
    uint8_t open_loop_active = 0;
    int16_t error_x10 = 0;
    float error = 0.0f;
    float turn_delta = 0.0f;
    float left_output = LINE_BASE_SPEED_MM_S;
    float right_output = LINE_BASE_SPEED_MM_S;
=======
    int16_t error_x10 = 0;
    float error = 0.0f;
    float effective_error = 0.0f;
    float derivative = 0.0f;
    float d_raw = 0.0f;
    float turn_delta = 0.0f;
    float left_speed = LINE_BASE_SPEED_MM_S;
    float right_speed = LINE_BASE_SPEED_MM_S;
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa

    extern volatile float motor1_target_mm_s;
    extern volatile float motor2_target_mm_s;

    detector_get_value();
    line_open_loop_active = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (detector_value[i])
        {
            sensor_mask |= (uint8_t)(1U << i);
            active_count++;
        }
    }

    if (active_count)
    {
<<<<<<< HEAD
        line_lost_count = 0;

=======
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa
        valid_error = LineDecodeError(sensor_mask, &error);
        if (!valid_error && line_last_valid_error_valid)
        {
            error = line_last_valid_error;
            valid_error = 1;
<<<<<<< HEAD
            LineResetOpenLoop();
=======
            LineResetClosedLoop();
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa
        }

        if (valid_error)
        {
            line_last_valid_error = error;
            line_last_valid_error_valid = 1;
<<<<<<< HEAD
            LineSteerFromError(error, &weighted_sum, &error_x10,
                &left_output, &right_output, &turn_delta, &open_loop_active);
        }
        else
        {
            LineResetOpenLoop();
            line_state.direction = LINE_TURN_STRAIGHT;
        }
    }
    else
    {
        if (line_last_valid_error_valid && line_lost_count < LINE_LOST_CONFIRM_COUNT)
        {
            /* 八路瞬间全 0，但次数还没到确认阈值——大概率是黑线正好
             * 卡在两个探头缝隙之间，车其实还压着线。先沿用上一次的
             * 有效误差继续开环转向，而不是立刻回落到基速直行。 */
            line_lost_count++;
            error = line_last_valid_error;
            valid_error = 1;
            LineSteerFromError(error, &weighted_sum, &error_x10,
                &left_output, &right_output, &turn_delta, &open_loop_active);
        }
        else
        {
            /* 连续多次都读不到才真正判定丢线 */
            LineResetOpenLoop();
            line_last_valid_error_valid = 0;
            line_lost_count = LINE_LOST_CONFIRM_COUNT;
            line_state.direction = LINE_TURN_LOST;
        }
    }

    if (!open_loop_active)
    {
        motor1_target_mm_s = LINE_BASE_SPEED_MM_S;
        motor2_target_mm_s = LINE_BASE_SPEED_MM_S;
        left_output = LINE_BASE_SPEED_MM_S;
        right_output = LINE_BASE_SPEED_MM_S;
    }
    line_open_loop_active = open_loop_active;
=======
            weighted_sum = (int16_t)error;
            error_x10 = (int16_t)(error * 10.0f);

            if (error_x10 > LINE_TURN_DEADBAND_X10)
            {
                effective_error = error;
                line_state.direction = LINE_TURN_RIGHT;
            }
            else if (error_x10 < -LINE_TURN_DEADBAND_X10)
            {
                effective_error = error;
                line_state.direction = LINE_TURN_LEFT;
            }
            else
            {
                LineResetClosedLoop();
                line_state.direction = LINE_TURN_STRAIGHT;
            }
        }
        else
        {
            LineResetClosedLoop();
            line_state.direction = LINE_TURN_STRAIGHT;
        }

        if (effective_error != 0.0f)
        {
            if (line_pd_prev_valid)
            {
                derivative = (effective_error - line_pd_prev_error) / LINE_PD_DT_S;
                d_raw = LINE_PD_KD_MM_S_PER_ERR_S * derivative;
                d_raw = LineControl_ClampFloat(d_raw,
                    -LINE_PD_D_TERM_LIMIT_MM_S, LINE_PD_D_TERM_LIMIT_MM_S);
                line_pd_d_term_mm_s = LINE_PD_D_FILTER_ALPHA * line_pd_d_term_mm_s +
                    (1.0f - LINE_PD_D_FILTER_ALPHA) * d_raw;
            }
            else
            {
                line_pd_d_term_mm_s = 0.0f;
                line_pd_prev_valid = 1;
            }

            line_pd_prev_error = effective_error;
        }

        turn_delta = LINE_PD_KP_MM_S_PER_ERR * effective_error + line_pd_d_term_mm_s;
        turn_delta = LineControl_ClampFloat(turn_delta,
            -LINE_PD_OUTPUT_LIMIT_MM_S, LINE_PD_OUTPUT_LIMIT_MM_S);

        left_speed = LINE_BASE_SPEED_MM_S + turn_delta;
        right_speed = LINE_BASE_SPEED_MM_S - turn_delta;

        if (left_speed < LINE_MIN_TARGET_MM_S) left_speed = LINE_MIN_TARGET_MM_S;
        if (right_speed < LINE_MIN_TARGET_MM_S) right_speed = LINE_MIN_TARGET_MM_S;
    }
    else
    {
        LineResetClosedLoop();
        line_last_valid_error_valid = 0;
        line_state.direction = LINE_TURN_LOST;
    }

    motor1_target_mm_s = right_speed;  // M1 是右轮
    motor2_target_mm_s = left_speed;   // M2 是左轮
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa

    line_state.sensor_mask = sensor_mask;
    line_state.active_count = active_count;
    line_state.weighted_sum = weighted_sum;
    line_state.error_x10 = error_x10;
    line_state.delta_mm_s = (int16_t)turn_delta;
<<<<<<< HEAD
    line_state.left_target_mm_s = (uint16_t)(left_output + 0.5f);
    line_state.right_target_mm_s = (uint16_t)(right_output + 0.5f);
=======
    line_state.left_target_mm_s = (uint16_t)(left_speed + 0.5f);
    line_state.right_target_mm_s = (uint16_t)(right_speed + 0.5f);
>>>>>>> fec2fb861346cc9f8ffafd1890805f55bed6d7fa
}
