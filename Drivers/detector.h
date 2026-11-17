#ifndef __DETECTOR_H
#define __DETECTOR_H

#include <stdint.h>

typedef enum {
    LINE_TURN_LOST = 0,
    LINE_TURN_STRAIGHT,
    LINE_TURN_LEFT,
    LINE_TURN_RIGHT
} LineTurnDirection;

typedef struct {
    uint8_t sensor_mask;
    uint8_t active_count;
    int16_t weighted_sum;
    int16_t error_x10;
    int16_t delta_mm_s;
    uint16_t left_target_mm_s;
    uint16_t right_target_mm_s;
    LineTurnDirection direction;
} LineFollowState;

extern volatile uint8_t line_open_loop_active;

void detector_init(void);
uint8_t detector_get_status(void);
void detector_get_value(void);
void adjust_motor(void);
const LineFollowState *line_follow_get_state(void);

#endif /* __DETECTOR_H */
