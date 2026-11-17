#include "detector.h"
#include "ti_msp_dl_config.h"

#define LINE_BASE_SPEED_MM_S            200.0f
#define LINE_MIN_TARGET_MM_S            60.0f
#define LINE_TURN_DEADBAND_X10          5
#define LINE_PD_KP_MM_S_PER_ERR         10.0f
#define LINE_PD_KD_MM_S_PER_ERR_S       0.025f
#define LINE_PD_DT_S                    0.010f
#define LINE_PD_OUTPUT_LIMIT_MM_S       75.0f
#define LINE_PD_D_TERM_LIMIT_MM_S       10.0f
#define LINE_PD_D_FILTER_ALPHA          0.75f

uint8_t detector_value[]={0,0,0,0,0,0,0,0};
volatile uint8_t line_open_loop_active = 0;
static LineFollowState line_state;
static float line_pd_prev_error;
static float line_pd_d_term_mm_s;
static uint8_t line_pd_prev_valid;
static float line_last_valid_error;
static uint8_t line_last_valid_error_valid;

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

static void LineResetClosedLoop(void)
{
    line_pd_prev_error = 0.0f;
    line_pd_d_term_mm_s = 0.0f;
    line_pd_prev_valid = 0;
}

void adjust_motor()
{
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;
    uint8_t sensor_mask = 0;
    uint8_t valid_error = 0;
    int16_t error_x10 = 0;
    float error = 0.0f;
    float effective_error = 0.0f;
    float derivative = 0.0f;
    float d_raw = 0.0f;
    float turn_delta = 0.0f;
    float left_speed = LINE_BASE_SPEED_MM_S;
    float right_speed = LINE_BASE_SPEED_MM_S;

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
        valid_error = LineDecodeError(sensor_mask, &error);
        if (!valid_error && line_last_valid_error_valid)
        {
            error = line_last_valid_error;
            valid_error = 1;
            LineResetClosedLoop();
        }

        if (valid_error)
        {
            line_last_valid_error = error;
            line_last_valid_error_valid = 1;
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

    line_state.sensor_mask = sensor_mask;
    line_state.active_count = active_count;
    line_state.weighted_sum = weighted_sum;
    line_state.error_x10 = error_x10;
    line_state.delta_mm_s = (int16_t)turn_delta;
    line_state.left_target_mm_s = (uint16_t)(left_speed + 0.5f);
    line_state.right_target_mm_s = (uint16_t)(right_speed + 0.5f);
}
