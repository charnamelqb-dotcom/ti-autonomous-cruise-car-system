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
#include "motor.h"
#include "encoder.h"
#include "uart.h"
#include "pid.h"
#include "stdio.h"
#include "string.h"
#include "delay.h"
#include "oled.h"
#include "bmp.h"
#include "wit.h"

/* ================= User tuning area =================
 * Speed unit: encoder-count target used by the speed PID.
 * Angle unit: degree.
 * Distance unit: mm, converted to encoder counts below.
 */
#define cicir  260
#define OLED_STATUS_ENABLE 1
#define BLACK_LINE_STOP_COUNT 4
#define TRACE_STOP_ANGLE_DEG 175.0f
#define TASK1_ALERT_TICKS 13
#define TASK1_ALERT_SIGNAL_TICKS 10
#define TASK2_ALERT_SIGNAL_TICKS 10
#define TASK345_ALERT_SIGNAL_TICKS 10
#define TASK_LINE_STOP_ALERT_SIGNAL_TICKS 10
#define TASK_LINE_STOP_DETECT_TICKS 10

/* PID output limits */
#define SPEED_PID_OUTPUT_LIMIT 60.0f
#define SPEED_PID_INTEGRAL_LIMIT 180.0f
#define TRACE_SPEED_PID_INTEGRAL_LIMIT 80.0f
#define TRACE_GRAY_PID_OUTPUT_LIMIT 16.0f
#define GYRO_PID_INTEGRAL_LIMIT 160.0f
#define GYRO_RATE_PID_INTEGRAL_LIMIT 100.0f

/* Old path mode parameters */
#define MODE3_STRAIGHT_COUNTS 2600
#define MODE3_ARC_ANGLE_DEG 175.0f
#define MODE3_STRAIGHT_SPEED 20.0f
#define MODE3_ARC_INNER_SPEED 12.0f
#define MODE3_ARC_OUTER_SPEED 28.0f

/* Task 1 speed:
 * STRAIGHT: gyro PID straight before black line.
 * TRACE: line tracing after black line is detected.
 */
#define TASK1_STRAIGHT_SPEED 26.0f
#define TASK1_TRACE_SPEED 26.0f

/* Task 2 speed:
 * STRAIGHT: gyro PID straight, including reverse straight.
 * TRACE: line tracing after black line is detected.
 * REVERSE_STRAIGHT_COMP_DEG: target-angle offset for the reverse straight.
 */
#define TASK2_STRAIGHT_SPEED 30.0f
#define TASK2_TRACE_SPEED 30.0f
#define TASK2_REVERSE_STRAIGHT_COMP_DEG 2.5f
#define TASK2_SPEED_BALANCE_OUTPUT_LIMIT 5.0f
#define TASK2_SPEED_BALANCE_INTEGRAL_LIMIT 40.0f
#define TASK2_TRACE_START_FORWARD_TICKS 30

/* Button1 line-stop task */
#define TASK_LINE_STOP_STRAIGHT_SPEED 30.0f
#define TASK_LINE_STOP_SPEED_BALANCE_OUTPUT_LIMIT 5.0f
#define TASK_LINE_STOP_SPEED_BALANCE_INTEGRAL_LIMIT 40.0f

/* Task 3/4/5 turn direction:
 * FIRST_TURN_DIR: first turn direction.
 * SECOND_TURN_DIR: second turn direction.
 * Change sign if turn direction is reversed.
 */
#define MODE6_FIRST_TURN_DIR -1.0f
#define MODE6_SECOND_TURN_DIR 1.0f

/* Task 3/4/5 cycle 1 parameters.
 * Task3 runs cycle 1.
 * Task4 runs cycle 1 + cycle 2 + cycle 3 + cycle 4.
 * Task5 runs cycle 1 + cycle 2 + cycle 3.
 */
#define CYCLE1_FIRST_TURN_DEG 42.5f
#define CYCLE1_FIRST_DISTANCE_MM 1150.0f
#define CYCLE1_SECOND_TURN_DEG 42.5f
#define CYCLE1_SECOND_DISTANCE_MM 1160.0f
#define CYCLE1_INITIAL_STRAIGHT_COMP_DEG 0.0f
#define CYCLE1_REVERSE_STRAIGHT_COMP_DEG -1.0f

/* Task 3/4/5 cycle 2 parameters */
#define CYCLE2_FIRST_TURN_DEG 35.5f//41.5
#define CYCLE2_FIRST_DISTANCE_MM 1100.0f
#define CYCLE2_SECOND_TURN_DEG 48.5f
#define CYCLE2_SECOND_DISTANCE_MM 1110.0f
#define CYCLE2_INITIAL_STRAIGHT_COMP_DEG 3.0f
#define CYCLE2_REVERSE_STRAIGHT_COMP_DEG -4.1f

/* Task 3/4/5 cycle 3 parameters */
#define CYCLE3_FIRST_TURN_DEG 35.0f//47.5
#define CYCLE3_FIRST_DISTANCE_MM 1080.0f
#define CYCLE3_SECOND_TURN_DEG 43.5f
#define CYCLE3_SECOND_DISTANCE_MM 1120.0f
#define CYCLE3_INITIAL_STRAIGHT_COMP_DEG 2.0f
#define CYCLE3_REVERSE_STRAIGHT_COMP_DEG -5.0f

/* Task 4 cycle 4 parameters */
#define CYCLE4_FIRST_TURN_DEG 35.0f
#define CYCLE4_FIRST_DISTANCE_MM 1080.0f
#define CYCLE4_SECOND_TURN_DEG 37.5f
#define CYCLE4_SECOND_DISTANCE_MM 1100.0f
#define CYCLE4_INITIAL_STRAIGHT_COMP_DEG 2.0f
#define CYCLE4_REVERSE_STRAIGHT_COMP_DEG -5.0f

/* Task 3/4/5 turn control */
#define MODE6_TURN_SPEED 12.0f
#define MODE6_TURN_TOLERANCE_DEG 2.0f
#define MODE6_TURN_MIN_SPEED 8.0f
#define MODE6_FIRST_TURN_MAX_TICKS 50
#define MODE6_TURN_BEFORE_STRAIGHT_MM 100.0f  /* 转弯前直行距离10cm */

/* Task 3/4/5 speed:
 * TRACE: line tracing speed.
 * ENCODER_STRAIGHT: fixed-distance straight speed.
 * FIND_LINE_STRAIGHT: straight speed while searching for black line.
 */
#define TASK345_TRACE_SPEED 34.0f
#define TASK345_ENCODER_STRAIGHT_SPEED 38.0f
#define TASK345_FIND_LINE_STRAIGHT_SPEED 26.0f
/* Direct PWM duty for in-place correction when only the outer sensor sees line. */
#define TRACE_LEFT_OUTER_PIVOT_PWM 12.0f
#define TRACE_RIGHT_OUTER_PIVOT_PWM 10.0f
#define TRACE_OUTER_RECOVER_STOP_TICKS 2
#define MODE6_TRACE_SPEED TASK345_TRACE_SPEED
#define MODE6_ENCODER_STRAIGHT_SPEED TASK345_ENCODER_STRAIGHT_SPEED
#define MODE6_FIND_LINE_STRAIGHT_SPEED TASK345_FIND_LINE_STRAIGHT_SPEED

/* Encoder and distance conversion */
#define MODE7_WHEEL_DIAMETER_MM 65.0f
#define MODE7_ENCODER_LINES 13.0f
#define MODE7_REDUCTION_RATIO 28.0f
#define MODE7_ENCODER_MULTIPLE 1.0f
#define MODE7_TARGET_DISTANCE_MM 1100.0f
#define DISTANCE_MM_TO_COUNTS(distance_mm) ((int32_t)(((distance_mm) * MODE7_ENCODER_LINES * MODE7_REDUCTION_RATIO * MODE7_ENCODER_MULTIPLE) / (3.1415926f * MODE7_WHEEL_DIAMETER_MM) + 0.5f))
#define MODE7_TARGET_COUNTS DISTANCE_MM_TO_COUNTS(MODE7_TARGET_DISTANCE_MM)
#define CYCLE1_FIRST_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE1_FIRST_DISTANCE_MM)
#define CYCLE1_SECOND_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE1_SECOND_DISTANCE_MM)
#define CYCLE2_FIRST_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE2_FIRST_DISTANCE_MM)
#define CYCLE2_SECOND_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE2_SECOND_DISTANCE_MM)
#define CYCLE3_FIRST_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE3_FIRST_DISTANCE_MM)
#define CYCLE3_SECOND_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE3_SECOND_DISTANCE_MM)
#define CYCLE4_FIRST_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE4_FIRST_DISTANCE_MM)
#define CYCLE4_SECOND_DISTANCE_COUNTS DISTANCE_MM_TO_COUNTS(CYCLE4_SECOND_DISTANCE_MM)
#define MODE7_STRAIGHT_SPEED 20.0f

/* Black-line detection filter:
 * DETECT_CONFIRM: consecutive black-line samples before entering tracing.
 * LOST_CONFIRM: consecutive no-line samples before leaving tracing.
 * ACTIVE_MAX: max valid active grayscale sensors, filters bad all-active cases.
 */
#define LINE_DETECT_CONFIRM_COUNT 1
#define LINE_LOST_CONFIRM_COUNT 1
#define LINE_ACTIVE_MAX 5
#define GYRO_STRAIGHT_LINE_ACTIVE_MIN 1
#define GYRO_STRAIGHT_LINE_CONFIRM_COUNT 1
#define TRACE_MODE_PROTECT_TICKS 20
#define FINAL_RETURN_ANGLE_DEG 8.0f

/* Internal task id */
#define TASK_ID_NONE 0
#define TASK_ID_1 1
#define TASK_ID_2 2
#define TASK_ID_3 3
#define TASK_ID_4 4
#define TASK_ID_5 5
#define TASK_ID_LINE_STOP 6
#define MODE_TASK1_ALERT 9
#define MODE_TASK_LINE_STOP 10
/* ================= End user tuning area ================= */
// volatile unsigned char uart_data [40];
volatile int32_t Left_Count=0 ,Left_Count_Sum=0 ,Right_Count=0 ,Right_Count_Sum=0;
volatile int32_t NEW_Count=0 ,NEW_Count_Sum=0 , NEW_Speed=0;
volatile int32_t Left_Speed=0 ,Right_Speed=0;
volatile int motor_test_active = 0;
volatile int flag_en = 0;       // 使能标志 0-未使能 1-使能

// extern float wit_data.yaw;                                       // 陀螺仪航向角（小车当前朝向）

int16_t rxbuf = 0, cx = 160;
int16_t baseSpeed = 26;
uint8_t oled_buffer[32];
extern PID PosionPID;
extern PID cicriPID;
extern PID PosionPIDR;
extern PID cicriPIDR;
extern PID addPID;
extern PID grayPID;
extern PID gyroPID;
char txbuffer[50];
char run=0;
char count=0;
volatile char mode=0;
int16_t count_turn=0;
char quan=0,qunanend=1;
char Key_Last=0,Key_Now=0;
char Key2_Last=0,Key2_Now=0;
char Key3_Last=0,Key3_Now=0;
float_t a;
extern volatile unsigned char uart_data,uart_data2;
extern volatile unsigned char uart_data_temp[40];
extern int turn;
extern int L1,L2,L3,L4,R1,R2,R3,R4;
extern int add,stop;



float mainspeed;
float c;
uint32_t gpioB=0;
extern float angle[];
static int gray_error_value = 0;
static int gray_active_count = 0;
static int gray_last_error = 20;
static int gray_line_is_high = 0;
static int trace_outer_pivot_active = 0;
static int trace_outer_recover_ticks = 0;
static uint16_t oled_tick = 0;
static int button1_pressed = 0;
static int button2_pressed = 0;
static int button3_pressed = 0;
static int key_locked = 0;
static int task_select_mode = 0;
static int active_task_id = TASK_ID_NONE;
static float straight_yaw_target = 0.0f;
static float trace_start_yaw = 0.0f;
static int trace_stop_after_180 = 0;
static int mode3_step = 0;
static int32_t mode3_segment_counts = 0;
static float mode3_step_yaw = 0.0f;
static volatile int mode6_step = 0;
static volatile float mode6_initial_yaw = 0.0f;
static volatile float mode6_start_yaw = 0.0f;
static volatile int mode6_line_armed = 0;
static volatile int mode6_first_turn_ticks = 0;
static volatile float mode6_first_turn_entry_yaw = 0.0f;
static volatile float mode6_first_turn_last_yaw = 0.0f;
static volatile float mode6_first_turn_angle = 0.0f;
static volatile int mode6_task_id = 3;
static volatile int mode6_repeat_target = 1;
static volatile int mode6_repeat_count = 0;
static volatile float mode6_first_turn_deg = CYCLE1_FIRST_TURN_DEG;
static volatile float mode6_second_turn_deg = CYCLE1_SECOND_TURN_DEG;
static volatile float mode6_initial_straight_comp_deg = CYCLE1_INITIAL_STRAIGHT_COMP_DEG;
static volatile float mode6_reverse_straight_comp_deg = CYCLE1_REVERSE_STRAIGHT_COMP_DEG;
static volatile int32_t mode6_first_distance_counts = MODE7_TARGET_COUNTS;
static volatile int32_t mode6_second_distance_counts = MODE7_TARGET_COUNTS;
static volatile int32_t mode6_distance_counts = 0;
static volatile int32_t mode6_turn_before_distance_counts = 0;
static volatile int32_t mode6_left_counts = 0;
static volatile int32_t mode6_right_counts = 0;
static int32_t mode7_distance_counts = 0;
static int32_t mode7_left_counts = 0;
static int32_t mode7_right_counts = 0;
static float mode7_start_yaw = 0.0f;
static int reverse_line_armed = 0;
static int final_trace_stop_at_initial = 0;
static float initial_yaw_target = 0.0f;
static int gyro_straight_line_count = 0;
static int line_stop_detect_ticks = 0;
static int task2_trace_start_forward_ticks = 0;
static int line_detect_count = 0;
static int line_lost_count = 0;
static int line_state_detected = 0;
static volatile int trace_mode_protect_ticks = 0;
static volatile int task1_alert_ticks = 0;
static volatile int alert_signal_ticks = 0;
// void motor_speed(uint8_t side,int8_t duty);
void uart0_send_char(char ch); //串口0发送单个字符
void uart0_send_string(char* str); //串口0发送字符串
int fputc(int ch, FILE *f);
int fputs(const char* restrict s, FILE* restrict stream);
int puts(const char* _ptr);
static void stop_trace(void);
static float yaw_error(float target, float now);
static float normalize_yaw(float yaw);
static void reset_pid_state(PID *pid);
static void reset_line_state(void);
static void update_line_state(void);
static int line_detected_stable(void);
static int line_lost_stable(void);
static int line_active_valid(void);
static void start_trace_protect(void);
static void enter_trace_mode(void);
static void alert_output_set(int enable);
static void alert_signal_start(int ticks);
static void alert_signal_update(void);
static void task1_alert_start(void);
static void update_task1_alert_pause(void);
static void trace_outer_pivot_drive(int pivot_dir);
static float straight_speed_balance_bias(float output_limit, float integral_limit);
static void mode6_start(int repeat_target);
static void mode6_restart_next_cycle(void);
static void mode6_load_cycle_config(void);
static void update_mode6_turn_then_straight(void);
static int mode6_update_distance_1100(void);
static int32_t mode6_current_distance_target(void);
static int mode6_update_return_to_yaw(float target_yaw);
static float mode6_initial_straight_target(void);
static float mode6_reverse_straight_target(void);
static void mode6_reset_distance(void);
static void mode7_start(void);
static void update_mode7_distance_straight(void);
static void task1_start(void);
static void task2_start(void);
static void task_line_stop_start(void);
static void update_task_line_stop_straight(void);
static void task3_start(void);
static void task4_start(void);
static void task5_start(void);

static void read_gray_sensors(void)
{
    L4 = DL_GPIO_readPins(GPIO_Traking_left4_PORT, GPIO_Traking_left4_PIN) ? 1 : 0;
    L3 = DL_GPIO_readPins(GPIO_Traking_left3_PORT, GPIO_Traking_left3_PIN) ? 1 : 0;
    L2 = DL_GPIO_readPins(GPIO_Traking_left2_PORT, GPIO_Traking_left2_PIN) ? 1 : 0;
    L1 = DL_GPIO_readPins(GPIO_Traking_left1_PORT, GPIO_Traking_left1_PIN) ? 1 : 0;
    R1 = DL_GPIO_readPins(GPIO_Traking_right1_PORT, GPIO_Traking_right1_PIN) ? 1 : 0;
    R2 = DL_GPIO_readPins(GPIO_Traking_right2_PORT, GPIO_Traking_right2_PIN) ? 1 : 0;
    R3 = DL_GPIO_readPins(GPIO_Traking_right3_PORT, GPIO_Traking_right3_PIN) ? 1 : 0;
    R4 = DL_GPIO_readPins(GPIO_Traking_right4_PORT, GPIO_Traking_right4_PIN) ? 1 : 0;
}

int get_gray_error(void)
{
    return gray_error_value;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int line_active_valid(void)
{
    return (gray_active_count > 0) && (gray_active_count <= LINE_ACTIVE_MAX);
}

static void reset_line_state(void)
{
    line_detect_count = 0;
    line_lost_count = 0;
    line_state_detected = 0;
}

static void update_line_state(void)
{
    if(line_active_valid()) {
        if(line_detect_count < LINE_DETECT_CONFIRM_COUNT) {
            line_detect_count++;
        }
        line_lost_count = 0;
    } else {
        if(line_lost_count < LINE_LOST_CONFIRM_COUNT) {
            line_lost_count++;
        }
        line_detect_count = 0;
    }

    if(line_detect_count >= LINE_DETECT_CONFIRM_COUNT) {
        line_state_detected = 1;
    } else if(line_lost_count >= LINE_LOST_CONFIRM_COUNT) {
        line_state_detected = 0;
    }
}

static int line_detected_stable(void)
{
    return line_state_detected;
}

static int line_lost_stable(void)
{
    return (line_lost_count >= LINE_LOST_CONFIRM_COUNT);
}

static void start_trace_protect(void)
{
    trace_mode_protect_ticks = TRACE_MODE_PROTECT_TICKS;
}

static void enter_trace_mode(void)
{
    mode = 5;
    start_trace_protect();
}

static void update_gray_error(void)
{
    int raw[8] = {L4, L3, L2, L1, R1, R2, R3, R4};
    int weight[8] = {48, 34, 20, 10, -10, -20, -34, -48};
    int high_count = 0;
    int sum = 0;
    int active = 0;

    for(int i = 0; i < 8; i++) {
        high_count += raw[i] ? 1 : 0;
    }

    if((high_count > 0) && (high_count < 8)) {
        gray_line_is_high = (high_count <= 4);
    }

    for(int i = 0; i < 8; i++) {
        int is_line = gray_line_is_high ? raw[i] : !raw[i];
        if(is_line) {
            sum += weight[i];
            active++;
        }
    }

    gray_active_count = active;
    update_line_state();
    if(active == 0) {
        gray_error_value = gray_last_error;
        return;
    }

    gray_error_value = sum / active;
    if(gray_error_value != 0) {
        gray_last_error = gray_error_value;
    }
}

static void trace_outer_pivot_drive(int pivot_dir)
{
    float duty = (pivot_dir < 0) ? TRACE_LEFT_OUTER_PIVOT_PWM : TRACE_RIGHT_OUTER_PIVOT_PWM;

    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
    PosionPID.target_val = 0;
    PosionPIDR.target_val = 0;

    if(pivot_dir < 0) {
        motor_speed(0, duty);
        motor_speed(1, -duty);
    } else {
        motor_speed(0, -duty);
        motor_speed(1, duty);
    }
}

static void update_line_trace(void)
{
    float gray_bias;
    float gyro_bias;
    float turn_bias;
    int left_outer_line;
    int right_outer_line;
    int left_inner_line;
    int right_inner_line;
    int trace_center_line;
    int center_line;
    int trace_start_forward_active;

    read_gray_sensors();
    update_gray_error();
    xunjiopen();
    left_outer_line = gray_line_is_high ? L4 : !L4;
    right_outer_line = gray_line_is_high ? R4 : !R4;
    left_inner_line = gray_line_is_high ? (L1 || L2) : (!L1 || !L2);
    right_inner_line = gray_line_is_high ? (R1 || R2) : (!R1 || !R2);
    trace_center_line = gray_line_is_high ? (L1 || R1) : (!L1 || !R1);
    center_line = left_inner_line || right_inner_line;
    trace_start_forward_active = ((active_task_id == TASK_ID_2) &&
                                  (mode == 1) &&
                                  (task2_trace_start_forward_ticks > 0));
    if(task2_trace_start_forward_ticks > 0) {
        task2_trace_start_forward_ticks--;
    }

    if((mode == 5) && (trace_mode_protect_ticks == 0) && line_lost_stable()) {
        if(active_task_id == TASK_ID_1) {
            alert_signal_start(TASK1_ALERT_SIGNAL_TICKS);
            mode = 0;
            stop_trace();
        } else if(active_task_id == TASK_ID_2) {
            alert_signal_start(TASK2_ALERT_SIGNAL_TICKS);
            mode = 0;
            stop_trace();
        } else if(mode6_step == 3) {
            alert_signal_start(TASK345_ALERT_SIGNAL_TICKS);
            mode = 6;
            mode6_step = 4;
            straight_yaw_target = normalize_yaw(mode6_start_yaw + 180.0f +
                                                MODE6_SECOND_TURN_DIR * mode6_second_turn_deg);
            reset_line_state();
            reset_pid_state(&grayPID);
            reset_pid_state(&gyroPID);
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
        } else if(mode6_step == 6) {
            alert_signal_start(TASK345_ALERT_SIGNAL_TICKS);
            mode6_repeat_count++;
            if(mode6_repeat_count < mode6_repeat_target) {
                mode6_restart_next_cycle();
            } else {
                mode = 0;
                stop_trace();
            }
        } else {
            mode = 0;
            stop_trace();
        }
        return;
    }

    if((mode == 1) && (trace_mode_protect_ticks == 0) && line_lost_stable()) {
        if(active_task_id == TASK_ID_2) {
            alert_signal_start(TASK2_ALERT_SIGNAL_TICKS);
        }
        mode = 4;
        mainspeed = TASK2_STRAIGHT_SPEED;
        reverse_line_armed = 0;
        task2_trace_start_forward_ticks = 0;
        reset_line_state();
        straight_yaw_target = normalize_yaw(initial_yaw_target + 180.0f +
                                            TASK2_REVERSE_STRAIGHT_COMP_DEG);
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        reset_pid_state(&addPID);
        return;
    }

    if(trace_outer_pivot_active && trace_center_line) {
        trace_outer_pivot_active = 0;
        trace_outer_recover_ticks = TRACE_OUTER_RECOVER_STOP_TICKS;
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
    }

    if(trace_outer_recover_ticks > 0) {
        trace_outer_recover_ticks--;
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        reset_pid_state(&PosionPID);
        reset_pid_state(&PosionPIDR);
        PosionPID.target_val = mainspeed * 0.5f;
        PosionPIDR.target_val = mainspeed * 0.5f;
        motor_speed(0, mainspeed * 0.5f);
        motor_speed(1, mainspeed * 0.5f);
        return;
    }

    if(gray_active_count == 0) {
        trace_outer_pivot_active = 0;
        trace_outer_recover_ticks = 0;
        gray_bias = 0.0f;
        gyroPID.target_val = 0.0f;
        gyro_bias = PID_realize_limited(&gyroPID,
                                        -yaw_error(straight_yaw_target, wit_data.yaw),
                                        15.0f, GYRO_PID_INTEGRAL_LIMIT);
        turn_bias = gyro_bias;
    } else if(!trace_start_forward_active && !center_line &&
              ((left_outer_line && !right_outer_line && (gray_active_count == 1)) ||
               (trace_outer_pivot_active == -1))) {
        trace_outer_pivot_active = -1;
        trace_outer_recover_ticks = 0;
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        trace_outer_pivot_drive(-1);
        return;
    } else if(!trace_start_forward_active && !center_line &&
              ((right_outer_line && !left_outer_line && (gray_active_count == 1)) ||
               (trace_outer_pivot_active == 1))) {
        trace_outer_pivot_active = 1;
        trace_outer_recover_ticks = 0;
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        trace_outer_pivot_drive(1);
        return;
    } else {
        trace_outer_pivot_active = 0;
        trace_outer_recover_ticks = 0;
        gray_bias = PID_realize_limited(&grayPID, get_gray_error(),
                                        TRACE_GRAY_PID_OUTPUT_LIMIT, 80.0f);
        gyroPID.target_val = 0.0f;
        gyro_bias = PID_realize_limited(&gyroPID, wit_data.gz, 6.0f,
                                        GYRO_RATE_PID_INTEGRAL_LIMIT);
        turn_bias = -gray_bias - gyro_bias;
    }

    PosionPID.target_val = mainspeed + turn_bias;
    PosionPIDR.target_val = mainspeed - turn_bias;

}

static float yaw_error(float target, float now)
{
    float error = target - now;

    while(error > 180.0f) {
        error -= 360.0f;
    }
    while(error < -180.0f) {
        error += 360.0f;
    }

    return error;
}

static float normalize_yaw(float yaw)
{
    while(yaw > 180.0f) {
        yaw -= 360.0f;
    }
    while(yaw < -180.0f) {
        yaw += 360.0f;
    }

    return yaw;
}

static void reset_pid_state(PID *pid)
{
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->integral = 0.0f;
    pid->output_val = 0.0f;
}

static void speed_loop_control(void)
{
    float integral_limit = ((mode == 5) || (mode == 1)) ?
                           TRACE_SPEED_PID_INTEGRAL_LIMIT :
                           SPEED_PID_INTEGRAL_LIMIT;
    float left_pwm = PID_realize_limited(&PosionPID, Left_Speed,
                                         SPEED_PID_OUTPUT_LIMIT,
                                         integral_limit);
    float right_pwm = PID_realize_limited(&PosionPIDR, Right_Speed,
                                          SPEED_PID_OUTPUT_LIMIT,
                                          integral_limit);

    motor_speed(0, left_pwm);
    motor_speed(1, right_pwm);
}

static void speed_loop_stop(void)
{
    motor_speed(0, 0);
    motor_speed(1, 0);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
}

static float straight_speed_balance_bias(float output_limit, float integral_limit)
{
    float speed_diff = (float)(abs_i32(Left_Speed) - abs_i32(Right_Speed));

    addPID.target_val = 0.0f;
    return PID_realize_limited(&addPID, speed_diff, output_limit, integral_limit);
}

static void alert_output_set(int enable)
{
    if(enable) {
        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_LED_PIN);
        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_BEEP_PIN);
    } else {
        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_LED_PIN);
        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_BEEP_PIN);
    }
}

static void alert_signal_start(int ticks)
{
    alert_signal_ticks = ticks;
    alert_output_set(1);
}

static void alert_signal_update(void)
{
    if(alert_signal_ticks > 0) {
        alert_signal_ticks--;
        if(alert_signal_ticks <= 0) {
            alert_output_set(0);
        }
    }
}

static void task1_alert_start(void)
{
    mode = MODE_TASK1_ALERT;
    task1_alert_ticks = TASK1_ALERT_TICKS;
    alert_signal_ticks = 0;
    gyro_straight_line_count = 0;
    reset_line_state();
    reset_pid_state(&grayPID);
    reset_pid_state(&gyroPID);
    speed_loop_stop();
    alert_output_set(1);
}

static void update_task1_alert_pause(void)
{
    speed_loop_stop();

    if(task1_alert_ticks > 0) {
        task1_alert_ticks--;
    }

    if(task1_alert_ticks <= (TASK1_ALERT_TICKS - TASK1_ALERT_SIGNAL_TICKS)) {
        alert_output_set(0);
    }

    if(task1_alert_ticks <= 0) {
        alert_output_set(0);
        enter_trace_mode();
        mainspeed = TASK1_TRACE_SPEED;
        trace_stop_after_180 = 0;
        trace_start_yaw = wit_data.yaw;
        reset_line_state();
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        reset_pid_state(&PosionPID);
        reset_pid_state(&PosionPIDR);
        update_line_trace();
    }
}

static void point_prompt(void)
{
}

static void mode3_next_step(void)
{
    mode3_step++;
    mode3_segment_counts = 0;
    mode3_step_yaw = wit_data.yaw;
    reset_pid_state(&gyroPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
    point_prompt();
}

static void mode3_start(void)
{
    flag_en = 1;
    mode = 3;
    mainspeed = MODE3_STRAIGHT_SPEED;
    mode3_step = 1;
    mode3_segment_counts = 0;
    mode3_step_yaw = wit_data.yaw;
    reset_pid_state(&gyroPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
    point_prompt();
}

static void update_mode3_path(void)
{
    float angle_bias;
    float turn_bias;

    mode3_segment_counts += (abs_i32(Left_Speed) + abs_i32(Right_Speed)) / 2;

    switch(mode3_step) {
        case 1:
        case 3:
            angle_bias = yaw_error(mode3_step_yaw, wit_data.yaw);
            gyroPID.target_val = 0.0f;
            turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 12.0f,
                                            GYRO_PID_INTEGRAL_LIMIT);
            PosionPID.target_val = MODE3_STRAIGHT_SPEED + turn_bias;
            PosionPIDR.target_val = MODE3_STRAIGHT_SPEED - turn_bias;
            if(mode3_segment_counts >= MODE3_STRAIGHT_COUNTS) {
                mode3_next_step();
            }
            break;

        case 2:
            PosionPID.target_val = MODE3_ARC_OUTER_SPEED;
            PosionPIDR.target_val = MODE3_ARC_INNER_SPEED;
            if(abs_float(yaw_error(mode3_step_yaw, wit_data.yaw)) >= MODE3_ARC_ANGLE_DEG) {
                mode3_next_step();
            }
            break;

        case 4:
            PosionPID.target_val = MODE3_ARC_OUTER_SPEED;
            PosionPIDR.target_val = MODE3_ARC_INNER_SPEED;
            if(abs_float(yaw_error(mode3_step_yaw, wit_data.yaw)) >= MODE3_ARC_ANGLE_DEG) {
                mode = 0;
                stop_trace();
                point_prompt();
            }
            break;

        default:
            stop_trace();
            break;
    }
}

static void mode6_start(int repeat_target)
{
    flag_en = 0;
    mode = 0;
    mode6_step = 0;
    mode6_repeat_target = repeat_target;
    mode6_repeat_count = 0;
    mode6_load_cycle_config();
    mainspeed = MODE6_ENCODER_STRAIGHT_SPEED;
    mode6_initial_yaw = wit_data.yaw;
    mode6_start_yaw = mode6_initial_yaw;
    mode6_line_armed = 0;
    mode6_first_turn_ticks = 0;
    mode6_first_turn_entry_yaw = wit_data.yaw;
    mode6_first_turn_last_yaw = wit_data.yaw;
    mode6_first_turn_angle = 0.0f;
    mode6_reset_distance();
    mode6_turn_before_distance_counts = DISTANCE_MM_TO_COUNTS(MODE6_TURN_BEFORE_STRAIGHT_MM);
    straight_yaw_target = mode6_start_yaw;
    reset_line_state();
    reset_pid_state(&gyroPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
    mode6_step = 1;
    mode = 6;
    flag_en = 1;
}

static void mode6_restart_next_cycle(void)
{
    mode = 0;
    mode6_step = 0;
    mode6_load_cycle_config();
    mainspeed = MODE6_ENCODER_STRAIGHT_SPEED;
    mode6_start_yaw = mode6_initial_yaw;
    mode6_line_armed = 0;
    mode6_first_turn_ticks = 0;
    mode6_first_turn_entry_yaw = wit_data.yaw;
    mode6_first_turn_last_yaw = wit_data.yaw;
    mode6_first_turn_angle = 0.0f;
    mode6_reset_distance();
    mode6_turn_before_distance_counts = DISTANCE_MM_TO_COUNTS(MODE6_TURN_BEFORE_STRAIGHT_MM);
    straight_yaw_target = mode6_start_yaw;
    reset_line_state();
    reset_pid_state(&grayPID);
    reset_pid_state(&gyroPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
    mode6_step = 1;
    mode = 6;
}

static void mode6_load_cycle_config(void)
{
    if(mode6_repeat_count == 0) {
        mode6_first_turn_deg = CYCLE1_FIRST_TURN_DEG;
        mode6_second_turn_deg = CYCLE1_SECOND_TURN_DEG;
        mode6_initial_straight_comp_deg = CYCLE1_INITIAL_STRAIGHT_COMP_DEG;
        mode6_reverse_straight_comp_deg = CYCLE1_REVERSE_STRAIGHT_COMP_DEG;
        mode6_first_distance_counts = CYCLE1_FIRST_DISTANCE_COUNTS;
        mode6_second_distance_counts = CYCLE1_SECOND_DISTANCE_COUNTS;
    } else if(mode6_repeat_count == 1) {
        mode6_first_turn_deg = CYCLE2_FIRST_TURN_DEG;
        mode6_second_turn_deg = CYCLE2_SECOND_TURN_DEG;
        mode6_initial_straight_comp_deg = CYCLE2_INITIAL_STRAIGHT_COMP_DEG;
        mode6_reverse_straight_comp_deg = CYCLE2_REVERSE_STRAIGHT_COMP_DEG;
        mode6_first_distance_counts = CYCLE2_FIRST_DISTANCE_COUNTS;
        mode6_second_distance_counts = CYCLE2_SECOND_DISTANCE_COUNTS;
    } else if(mode6_repeat_count == 2) {
        mode6_first_turn_deg = CYCLE3_FIRST_TURN_DEG;
        mode6_second_turn_deg = CYCLE3_SECOND_TURN_DEG;
        mode6_initial_straight_comp_deg = CYCLE3_INITIAL_STRAIGHT_COMP_DEG;
        mode6_reverse_straight_comp_deg = CYCLE3_REVERSE_STRAIGHT_COMP_DEG;
        mode6_first_distance_counts = CYCLE3_FIRST_DISTANCE_COUNTS;
        mode6_second_distance_counts = CYCLE3_SECOND_DISTANCE_COUNTS;
    } else {
        mode6_first_turn_deg = CYCLE4_FIRST_TURN_DEG;
        mode6_second_turn_deg = CYCLE4_SECOND_TURN_DEG;
        mode6_initial_straight_comp_deg = CYCLE4_INITIAL_STRAIGHT_COMP_DEG;
        mode6_reverse_straight_comp_deg = CYCLE4_REVERSE_STRAIGHT_COMP_DEG;
        mode6_first_distance_counts = CYCLE4_FIRST_DISTANCE_COUNTS;
        mode6_second_distance_counts = CYCLE4_SECOND_DISTANCE_COUNTS;
    }
}

static void mode6_reset_distance(void)
{
    mode6_distance_counts = 0;
    mode6_turn_before_distance_counts = 0;
    mode6_left_counts = 0;
    mode6_right_counts = 0;
}

static int32_t mode6_current_distance_target(void)
{
    if(mode6_step == 5) {
        return mode6_second_distance_counts;
    }

    return mode6_first_distance_counts;
}

static float mode6_initial_straight_target(void)
{
    return normalize_yaw(mode6_start_yaw + mode6_initial_straight_comp_deg);
}

static float mode6_reverse_straight_target(void)
{
    return normalize_yaw(mode6_start_yaw + 180.0f + mode6_reverse_straight_comp_deg);
}

static int mode6_update_distance_1100(void)
{
    float angle_bias;
    float turn_bias;
    int32_t target_counts = mode6_current_distance_target();

    mode6_left_counts += abs_i32(Left_Speed);
    mode6_right_counts += abs_i32(Right_Speed);
    mode6_distance_counts = (mode6_left_counts + mode6_right_counts) / 2;

    if(mode6_distance_counts >= target_counts) {
        mode6_reset_distance();
        reset_pid_state(&gyroPID);
        reset_pid_state(&PosionPID);
        reset_pid_state(&PosionPIDR);
        PosionPID.target_val = 0;
        PosionPIDR.target_val = 0;
        return 1;
    }

    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);

    PosionPID.target_val = mainspeed + turn_bias;
    PosionPIDR.target_val = mainspeed - turn_bias;
    return 0;
}

static int mode6_update_return_to_yaw(float target_yaw)
{
    float angle_bias;
    float turn_bias;

    straight_yaw_target = normalize_yaw(target_yaw);
    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    if(abs_float(angle_bias) <= MODE6_TURN_TOLERANCE_DEG) {
        reset_line_state();
        reset_pid_state(&gyroPID);
        reset_pid_state(&PosionPID);
        reset_pid_state(&PosionPIDR);
        PosionPID.target_val = 0;
        PosionPIDR.target_val = 0;
        return 1;
    }

    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);
    if((turn_bias > 0.0f) && (turn_bias < MODE6_TURN_MIN_SPEED)) {
        turn_bias = MODE6_TURN_MIN_SPEED;
    } else if((turn_bias < 0.0f) && (turn_bias > -MODE6_TURN_MIN_SPEED)) {
        turn_bias = -MODE6_TURN_MIN_SPEED;
    }

    PosionPID.target_val = turn_bias;
    PosionPIDR.target_val = -turn_bias;
    return 0;
}

static void mode7_start(void)
{
    flag_en = 1;
    mode = 7;
    mainspeed = MODE7_STRAIGHT_SPEED;
    mode7_distance_counts = 0;
    mode7_left_counts = 0;
    mode7_right_counts = 0;
    mode7_start_yaw = wit_data.yaw;
    straight_yaw_target = mode7_start_yaw;
    reset_pid_state(&gyroPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
}

static void update_mode7_distance_straight(void)
{
    float angle_bias;
    float turn_bias;

    mode7_left_counts += abs_i32(Left_Speed);
    mode7_right_counts += abs_i32(Right_Speed);
    mode7_distance_counts = (mode7_left_counts + mode7_right_counts) / 2;

    if(mode7_distance_counts >= MODE7_TARGET_COUNTS) {
        mode = 0;
        stop_trace();
        return;
    }

    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);

    PosionPID.target_val = mainspeed + turn_bias;
    PosionPIDR.target_val = mainspeed - turn_bias;
}

static void update_mode6_turn_then_straight(void)
{
    float angle_bias;
    float turn_bias;

    /* step 0: 转弯前直行10cm */
    if(mode6_step == 0) {
        mode6_left_counts += abs_i32(Left_Speed);
        mode6_right_counts += abs_i32(Right_Speed);
        mode6_distance_counts = (mode6_left_counts + mode6_right_counts) / 2;

        angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
        gyroPID.target_val = 0.0f;
        turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                        GYRO_PID_INTEGRAL_LIMIT);

        PosionPID.target_val = mainspeed + turn_bias;
        PosionPIDR.target_val = mainspeed - turn_bias;

        if(mode6_distance_counts >= mode6_turn_before_distance_counts) {
            mode6_step = 1;
            mode6_reset_distance();
            reset_pid_state(&gyroPID);
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
            PosionPID.target_val = 0;
            PosionPIDR.target_val = 0;
        }
        return;
    }

    if(mode6_step == 1) {
        float first_turn_target = normalize_yaw(mode6_start_yaw +
                                                MODE6_FIRST_TURN_DIR * mode6_first_turn_deg);
        float angle_bias_to_target = yaw_error(first_turn_target, wit_data.yaw);
        float yaw_delta = yaw_error(wit_data.yaw, mode6_first_turn_last_yaw);
        float step_turned_angle;
        float turn_output;

        mode6_first_turn_angle += yaw_delta * MODE6_FIRST_TURN_DIR;
        mode6_first_turn_last_yaw = wit_data.yaw;
        if(mode6_first_turn_angle < 0.0f) {
            mode6_first_turn_angle = 0.0f;
        }
        step_turned_angle = mode6_first_turn_angle;
        mode6_first_turn_angle = step_turned_angle;
        mode6_first_turn_ticks++;
        if(mode6_first_turn_ticks > MODE6_FIRST_TURN_MAX_TICKS) {
            mode6_step = 2;
            straight_yaw_target = first_turn_target;
            mainspeed = MODE6_ENCODER_STRAIGHT_SPEED;
            mode6_line_armed = 0;
            mode6_first_turn_ticks = 0;
            mode6_first_turn_angle = step_turned_angle;
            mode6_reset_distance();
            reset_pid_state(&gyroPID);
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
            PosionPID.target_val = 0;
            PosionPIDR.target_val = 0;
            return;
        }

        if((step_turned_angle >= (mode6_first_turn_deg - MODE6_TURN_TOLERANCE_DEG)) ||
           (abs_float(angle_bias_to_target) <= MODE6_TURN_TOLERANCE_DEG)) {
            mode6_step = 2;
            straight_yaw_target = first_turn_target;
            mainspeed = MODE6_ENCODER_STRAIGHT_SPEED;
            mode6_line_armed = 0;
            mode6_first_turn_ticks = 0;
            mode6_reset_distance();
            reset_pid_state(&gyroPID);
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
            PosionPID.target_val = 0;
            PosionPIDR.target_val = 0;
            return;
        }

        turn_output = MODE6_FIRST_TURN_DIR * MODE6_TURN_SPEED;
        PosionPID.target_val = turn_output;
        PosionPIDR.target_val = -turn_output;
        return;
    }

    if(mode6_step == 2) {
        if(mode6_update_distance_1100()) {
            mode6_step = 7;
        }
        return;
    }

    if(mode6_step == 7) {
        if(mode6_update_return_to_yaw(mode6_initial_straight_target())) {
            mode6_step = 8;
            mainspeed = MODE6_FIND_LINE_STRAIGHT_SPEED;
            mode6_line_armed = 0;
            reset_line_state();
        }
        return;
    }

    if(mode6_step == 8) {
        read_gray_sensors();
        update_gray_error();
        if(!mode6_line_armed) {
            if(line_lost_stable()) {
                mode6_line_armed = 1;
            }
        } else if(line_detected_stable()) {
            enter_trace_mode();
            alert_signal_start(TASK345_ALERT_SIGNAL_TICKS);
            mainspeed = MODE6_TRACE_SPEED;
            mode6_step = 3;
            mode6_line_armed = 0;
            reset_line_state();
            reset_pid_state(&grayPID);
            reset_pid_state(&gyroPID);
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
            update_line_trace();
            return;
        }
    }

    if(mode6_step == 4) {
        if(mode6_update_return_to_yaw(straight_yaw_target)) {
            mode6_step = 5;
            straight_yaw_target = wit_data.yaw;
            mainspeed = MODE6_ENCODER_STRAIGHT_SPEED;
            mode6_reset_distance();
        }
        return;
    }

    if(mode6_step == 5) {
        if(mode6_update_distance_1100()) {
            mode6_step = 9;
        }
        return;
    }

    if(mode6_step == 9) {
        if(mode6_update_return_to_yaw(mode6_reverse_straight_target())) {
            mode6_step = 10;
            mainspeed = MODE6_FIND_LINE_STRAIGHT_SPEED;
            mode6_line_armed = 0;
            reset_line_state();
        }
        return;
    }

    if(mode6_step == 10) {
        read_gray_sensors();
        update_gray_error();
        if(!mode6_line_armed) {
            if(line_lost_stable()) {
                mode6_line_armed = 1;
            }
        } else if(line_detected_stable()) {
            enter_trace_mode();
            alert_signal_start(TASK345_ALERT_SIGNAL_TICKS);
            mainspeed = MODE6_TRACE_SPEED;
            mode6_step = 6;
            mode6_line_armed = 0;
            reset_line_state();
            reset_pid_state(&grayPID);
            reset_pid_state(&gyroPID);
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
            update_line_trace();
            return;
        }
    }

    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);

    PosionPID.target_val = mainspeed + turn_bias;
    PosionPIDR.target_val = mainspeed - turn_bias;
}

static void update_gyro_straight(void)
{
    float angle_bias;
    float turn_bias;
    float speed_balance = 0.0f;

    read_gray_sensors();
    update_gray_error();

    if((gray_active_count >= GYRO_STRAIGHT_LINE_ACTIVE_MIN) &&
       (gray_active_count <= LINE_ACTIVE_MAX)) {
        if(gyro_straight_line_count < GYRO_STRAIGHT_LINE_CONFIRM_COUNT) {
            gyro_straight_line_count++;
        }
    } else {
        gyro_straight_line_count = 0;
    }

    if(gyro_straight_line_count >= GYRO_STRAIGHT_LINE_CONFIRM_COUNT) {
        if((active_task_id == TASK_ID_1) && (mode == 8)) {
            task1_alert_start();
            return;
        }

        if(mode == 8) {
            enter_trace_mode();
        } else {
            mode = 1;
            start_trace_protect();
        }
        if(active_task_id == TASK_ID_1) {
            mainspeed = TASK1_TRACE_SPEED;
        } else if(active_task_id == TASK_ID_2) {
            alert_signal_start(TASK2_ALERT_SIGNAL_TICKS);
            mainspeed = TASK2_TRACE_SPEED;
            task2_trace_start_forward_ticks = TASK2_TRACE_START_FORWARD_TICKS;
        }
        trace_stop_after_180 = 0;
        trace_start_yaw = wit_data.yaw;
        gyro_straight_line_count = 0;
        reset_line_state();
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        if(active_task_id != TASK_ID_2) {
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
        }
        update_line_trace();
        return;
    }

    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);
    if((active_task_id == TASK_ID_2) && (mode == 2)) {
        speed_balance = straight_speed_balance_bias(TASK2_SPEED_BALANCE_OUTPUT_LIMIT,
                                                    TASK2_SPEED_BALANCE_INTEGRAL_LIMIT);
    }

    PosionPID.target_val = mainspeed + turn_bias + speed_balance;
    PosionPIDR.target_val = mainspeed - turn_bias - speed_balance;
}

static void update_gyro_reverse_straight(void)
{
    float angle_bias;
    float turn_bias;
    float speed_balance;

    read_gray_sensors();
    update_gray_error();

    if(reverse_line_armed && line_detected_stable()) {
        enter_trace_mode();
        if(active_task_id == TASK_ID_2) {
            alert_signal_start(TASK2_ALERT_SIGNAL_TICKS);
            mainspeed = TASK2_TRACE_SPEED;
        }
        final_trace_stop_at_initial = 0;
        reset_line_state();
        reset_pid_state(&grayPID);
        reset_pid_state(&gyroPID);
        if(active_task_id != TASK_ID_2) {
            reset_pid_state(&PosionPID);
            reset_pid_state(&PosionPIDR);
        }
        update_line_trace();
        return;
    }

    if(line_lost_stable()) {
        reverse_line_armed = 1;
    }

    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);
    speed_balance = straight_speed_balance_bias(TASK2_SPEED_BALANCE_OUTPUT_LIMIT,
                                                TASK2_SPEED_BALANCE_INTEGRAL_LIMIT);

    PosionPID.target_val = mainspeed + turn_bias + speed_balance;
    PosionPIDR.target_val = mainspeed - turn_bias - speed_balance;
}

static void update_task_line_stop_straight(void)
{
    float angle_bias;
    float turn_bias;
    float speed_balance;

    read_gray_sensors();
    update_gray_error();

    if(line_active_valid()) {
        if(line_stop_detect_ticks < TASK_LINE_STOP_DETECT_TICKS) {
            line_stop_detect_ticks++;
        }
    } else {
        line_stop_detect_ticks = 0;
    }

    if(line_stop_detect_ticks >= TASK_LINE_STOP_DETECT_TICKS) {
        alert_signal_start(TASK_LINE_STOP_ALERT_SIGNAL_TICKS);
        mode = 0;
        stop_trace();
        return;
    }

    angle_bias = yaw_error(straight_yaw_target, wit_data.yaw);
    gyroPID.target_val = 0.0f;
    turn_bias = PID_realize_limited(&gyroPID, -angle_bias, 15.0f,
                                    GYRO_PID_INTEGRAL_LIMIT);
    speed_balance = straight_speed_balance_bias(TASK_LINE_STOP_SPEED_BALANCE_OUTPUT_LIMIT,
                                                TASK_LINE_STOP_SPEED_BALANCE_INTEGRAL_LIMIT);

    PosionPID.target_val = mainspeed + turn_bias + speed_balance;
    PosionPIDR.target_val = mainspeed - turn_bias - speed_balance;
}

static void stop_trace(void)
{
    flag_en = 0;
    task_select_mode = 0;
    active_task_id = TASK_ID_NONE;
    trace_stop_after_180 = 0;
    task1_alert_ticks = 0;
    if(alert_signal_ticks <= 0) {
        alert_output_set(0);
    }
    mode6_step = 0;
    mode6_line_armed = 0;
    mode6_first_turn_ticks = 0;
    mode6_first_turn_entry_yaw = 0.0f;
    mode6_first_turn_last_yaw = 0.0f;
    mode6_first_turn_angle = 0.0f;
    mode6_task_id = 3;
    mode6_repeat_target = 1;
    mode6_repeat_count = 0;
    mode6_reset_distance();
    mode7_distance_counts = 0;
    mode7_left_counts = 0;
    mode7_right_counts = 0;
    reverse_line_armed = 0;
    final_trace_stop_at_initial = 0;
    trace_outer_pivot_active = 0;
    trace_outer_recover_ticks = 0;
    gyro_straight_line_count = 0;
    task2_trace_start_forward_ticks = 0;
    line_stop_detect_ticks = 0;
    reset_line_state();
    PosionPID.target_val = 0;
    PosionPIDR.target_val = 0;
    speed_loop_stop();
}

static void oled_show_status(void)
{
#if OLED_STATUS_ENABLE
    char line[32];
    char sensor[9];

    sensor[0] = L4 ? '1' : '0';
    sensor[1] = L3 ? '1' : '0';
    sensor[2] = L2 ? '1' : '0';
    sensor[3] = L1 ? '1' : '0';
    sensor[4] = R1 ? '1' : '0';
    sensor[5] = R2 ? '1' : '0';
    sensor[6] = R3 ? '1' : '0';
    sensor[7] = R4 ? '1' : '0';
    sensor[8] = '\0';

    OLED_Clear();

    if(motor_test_active) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:MTEST", 16, 1);
    } else if(flag_en && mode == 6) {
        if(mode6_step == 1) {
            snprintf(line, sizeof(line), "MODE:C%dT1", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 2) {
            snprintf(line, sizeof(line), "MODE:C%dD1", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 7) {
            snprintf(line, sizeof(line), "MODE:C%dR1", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 8) {
            snprintf(line, sizeof(line), "MODE:C%dF1", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 4) {
            snprintf(line, sizeof(line), "MODE:C%dT2", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 5) {
            snprintf(line, sizeof(line), "MODE:C%dD2", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 9) {
            snprintf(line, sizeof(line), "MODE:C%dR2", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else if(mode6_step == 10) {
            snprintf(line, sizeof(line), "MODE:C%dF2", mode6_repeat_count + 1);
            OLED_ShowString(0, 0, (uint8_t *)line, 16, 1);
        } else {
            OLED_ShowString(0, 0, (uint8_t *)"MODE:M6LINE", 16, 1);
        }
    } else if(flag_en && mode == 7) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:D1100", 16, 1);
    } else if(flag_en && mode == MODE_TASK_LINE_STOP) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:LSTOP", 16, 1);
    } else if(flag_en && mode == MODE_TASK1_ALERT) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:T1BEEP", 16, 1);
    } else if(flag_en && mode == 8) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:T1GYR", 16, 1);
    } else if(flag_en && mode == 3) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:PATH3", 16, 1);
    } else if(flag_en && mode == 2) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:GYRO ", 16, 1);
    } else if(flag_en && mode == 5 && active_task_id == TASK_ID_1) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:T1TRC", 16, 1);
    } else if(flag_en && mode == 5) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:FTRC ", 16, 1);
    } else if(flag_en && mode == 4) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:REV  ", 16, 1);
    } else if(flag_en) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:TRACE", 16, 1);
    } else if(task_select_mode) {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:SELECT", 16, 1);
    } else {
        OLED_ShowString(0, 0, (uint8_t *)"MODE:IDLE ", 16, 1);
    }

    if(flag_en && mode == 6) {
        snprintf(line, sizeof(line), "Y:%d T:%d", (int)wit_data.yaw, (int)straight_yaw_target);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        snprintf(line, sizeof(line), "C:%ld/%ld", (long)mode6_distance_counts,
                 (long)mode6_current_distance_target());
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        if(mode6_step == 1) {
            snprintf(line, sizeof(line), "N:%d/%d T:%d A:%d", mode6_repeat_count + 1,
                     mode6_repeat_target, mode6_first_turn_ticks,
                     (int)mode6_first_turn_angle);
        } else {
            snprintf(line, sizeof(line), "N:%d/%d A:%d D:%d", mode6_repeat_count + 1,
                     mode6_repeat_target, mode6_line_armed, (int)mode6_first_turn_deg);
        }
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        return;
    }

    if(flag_en && mode == 7) {
        snprintf(line, sizeof(line), "C:%ld/%ld", (long)mode7_distance_counts, (long)MODE7_TARGET_COUNTS);
        OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

        snprintf(line, sizeof(line), "E:%ld/%ld", (long)mode7_left_counts, (long)mode7_right_counts);
        OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

        snprintf(line, sizeof(line), "V:%ld/%ld", (long)Left_Speed, (long)Right_Speed);
        OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);
        OLED_Refresh();
        return;
    }

    snprintf(line, sizeof(line), "G:%s", sensor);
    OLED_ShowString(0, 16, (uint8_t *)line, 16, 1);

    snprintf(line, sizeof(line), "E:%d A:%d F:%d M:%d", gray_error_value, gray_active_count, flag_en, mode);
    OLED_ShowString(0, 32, (uint8_t *)line, 16, 1);

    snprintf(line, sizeof(line), "K:%d%d%d L:%d R:%d", button1_pressed, button2_pressed, button3_pressed,
             (int)PosionPID.target_val, (int)PosionPIDR.target_val);
    OLED_ShowString(0, 48, (uint8_t *)line, 16, 1);

    OLED_Refresh();
#endif
}

static void motor_test(void)
{
    stop_trace();
    motor_test_active = 1;
    oled_show_status();

    motor_speed(0, 20);
    motor_speed(1, 0);
    delay_ms(800);

    motor_speed(0, -20);
    motor_speed(1, 0);
    delay_ms(800);

    motor_speed(0, 0);
    motor_speed(1, 20);
    delay_ms(800);

    motor_speed(0, 0);
    motor_speed(1, -20);
    delay_ms(800);

    motor_speed(0, 20);
    motor_speed(1, 20);
    delay_ms(800);

    motor_speed(0, -20);
    motor_speed(1, -20);
    delay_ms(800);

    motor_speed(0, 0);
    motor_speed(1, 0);
    motor_test_active = 0;
    oled_show_status();
}

static void task1_start(void)
{
    flag_en = 1;
    active_task_id = TASK_ID_1;
    mode = 8;
    mainspeed = TASK1_STRAIGHT_SPEED;
    straight_yaw_target = initial_yaw_target;
    trace_stop_after_180 = 0;
    mode6_step = 0;
    reverse_line_armed = 0;
    final_trace_stop_at_initial = 0;
    task1_alert_ticks = 0;
    alert_signal_ticks = 0;
    alert_output_set(0);
    gyro_straight_line_count = 0;
    task2_trace_start_forward_ticks = 0;
    reset_line_state();
    reset_pid_state(&grayPID);
    reset_pid_state(&gyroPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
}

static void task2_start(void)
{
    flag_en = 1;
    active_task_id = TASK_ID_2;
    mode = 2;
    mainspeed = TASK2_STRAIGHT_SPEED;
    straight_yaw_target = initial_yaw_target;
    trace_stop_after_180 = 0;
    reverse_line_armed = 0;
    final_trace_stop_at_initial = 0;
    task1_alert_ticks = 0;
    alert_signal_ticks = 0;
    alert_output_set(0);
    gyro_straight_line_count = 0;
    reset_line_state();
    reset_pid_state(&gyroPID);
    reset_pid_state(&addPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
}

static void task_line_stop_start(void)
{
    flag_en = 1;
    active_task_id = TASK_ID_LINE_STOP;
    mode = MODE_TASK_LINE_STOP;
    mainspeed = TASK_LINE_STOP_STRAIGHT_SPEED;
    straight_yaw_target = initial_yaw_target;
    trace_stop_after_180 = 0;
    mode6_step = 0;
    reverse_line_armed = 0;
    final_trace_stop_at_initial = 0;
    task1_alert_ticks = 0;
    alert_signal_ticks = 0;
    alert_output_set(0);
    gyro_straight_line_count = 0;
    line_stop_detect_ticks = 0;
    reset_line_state();
    reset_pid_state(&grayPID);
    reset_pid_state(&gyroPID);
    reset_pid_state(&addPID);
    reset_pid_state(&PosionPID);
    reset_pid_state(&PosionPIDR);
}

static void task3_start(void)
{
    active_task_id = TASK_ID_3;
    mode6_task_id = 3;
    mode6_start(1);
    trace_stop_after_180 = 0;
}

static void task4_start(void)
{
    active_task_id = TASK_ID_4;
    mode6_task_id = 4;
    mode6_start(4);
    trace_stop_after_180 = 0;
}

static void task5_start(void)
{
    active_task_id = TASK_ID_5;
    mode6_task_id = 5;
    mode6_start(3);
    trace_stop_after_180 = 0;
}

int main(void)
{
    SYSCFG_DL_init();
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_OpenMv_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(UART_OpenMv_INST_INT_IRQN);
    
    PID_param_init();
    // uart0_send_string("uart0 start work\r\n");
#if OLED_STATUS_ENABLE
    OLED_Init();
#endif
    WIT_Init();
    initial_yaw_target = wit_data.yaw;
    straight_yaw_target = initial_yaw_target;
    read_gray_sensors();
    update_gray_error();
    oled_show_status();
    // motor_speed(1,0);
    
    char t=10;
    // uartOpenMv_send_char('3');
    // uart0_send_char('1');
    mainspeed=baseSpeed;
    // addPID.target_val=mainspeed;
    PosionPID.target_val=mainspeed;
    PosionPIDR.target_val=mainspeed;
    mode=0;
    while (1)
    {
        gpioB = DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON1_PIN);
        Key_Now = ((gpioB & GPIO_BUTTON_PIN_BUTTON1_PIN) == 0);
        button1_pressed = Key_Now ? 1 : 0;

        if((key_locked == 0) && Key_Now && !Key_Last) {
            delay_ms(20);
            if((DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON1_PIN) & GPIO_BUTTON_PIN_BUTTON1_PIN) == 0) {
                if(task_select_mode) {
                    task3_start();
                    task_select_mode = 0;
                } else {
                    task_line_stop_start();
                }
                key_locked = 1;
            }
        }
        Key_Last = Key_Now;

#ifdef GPIO_BUTTON_PIN_BUTTON3_PIN
        Key3_Now = ((DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON3_PIN) & GPIO_BUTTON_PIN_BUTTON3_PIN) == 0);
        button3_pressed = Key3_Now ? 1 : 0;
        if((key_locked == 0) && Key3_Now && !Key3_Last) {
            delay_ms(20);
            if((DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON3_PIN) & GPIO_BUTTON_PIN_BUTTON3_PIN) == 0) {
                if(task_select_mode) {
                    task5_start();
                    task_select_mode = 0;
                    key_locked = 1;
                } else {
                    task_select_mode = 1;
                    oled_show_status();
                }
            }
        }
        Key3_Last = Key3_Now;
#else
        button3_pressed = 0;
#endif

        Key2_Now = ((DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON2_PIN) & GPIO_BUTTON_PIN_BUTTON2_PIN) == 0);
        button2_pressed = Key2_Now ? 1 : 0;
        if((key_locked == 0) && Key2_Now && !Key2_Last) {
            delay_ms(20);
            if((DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON2_PIN) & GPIO_BUTTON_PIN_BUTTON2_PIN) == 0) {
                if(task_select_mode) {
                    task4_start();
                    task_select_mode = 0;
                } else {
                    task2_start();
                }
                key_locked = 1;
            }
        }
        Key2_Last = Key2_Now;

        if(flag_en) {
            if(mode == 0) {
                mode = 1;
            }
        } else {
            read_gray_sensors();
            update_gray_error();
            PosionPID.target_val = 0;
            PosionPIDR.target_val = 0;
        }

        if(++oled_tick >= 2000) {
            oled_tick = 0;
            oled_show_status();
        }
    }
}

#if 0
    while (1)
    {   //OLED_Refresh();//刷新oled
        // motor_speed(3,30);//云台电机
        // OLED_ShowChar(1,1,'qunanend',16,1);
        // OLED_ShowNum(1,1,qunanend,2,16,1);
        // motor_speed(0,-10);
        // delay_ms(100);
                gpioB = DL_GPIO_readPins(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN_BUTTON1_PIN);
            if (DL_GPIO_readPins(GPIO_BUTTON_PORT,GPIO_BUTTON_PIN_BUTTON2_PIN)==0) {
            delay_ms(80);
            if (DL_GPIO_readPins(GPIO_BUTTON_PORT,GPIO_BUTTON_PIN_BUTTON2_PIN)==0)
            {
                mode = (mode + 1) % 5;
            }
            }
            if ((gpioB & GPIO_BUTTON_PIN_BUTTON1_PIN)==0) //按下按键
        {            
            DL_GPIO_togglePins(GPIO_LED_PORT,GPIO_LED_LED_PIN);
            flag_en = 1 - flag_en;
            // mainspeed=30;
            
            // mode=1;//1
            // mainspeed=30;
            // mainspeed=30;
            // PosionPID.target_val=8;
            // PosionPIDR.target_val=8;
        }
        while (mode==1 && flag_en==1) //寻竖线
        {
            // cicriPID.target_val=cicir;
            // cicriPIDR.target_val=-cicir;//向右转90
            mainspeed=30;
            
            read_sensors();
            xunjiopen();   
            add=0;
            float bias1, bias2, bias;                       // 循迹、角度偏差
            // bias = GYRO_Control(wit_data.yaw,0);
            // PosionPID.target_val=mainspeed-add;
            // PosionPIDR.target_val=mainspeed+add;
            bias=0;
                PosionPID.target_val=mainspeed-bias;
                PosionPIDR.target_val=mainspeed+bias;


            // if (stop==1) {
            //     add=0;
            //     PosionPID.target_val=mainspeed-add;
            //     PosionPIDR.target_val=mainspeed+add;
            //     delay_ms(500);

            //     mode=2;
            //     stop=0;
            // }
        

    }
}
}


#endif

void TIMER_0_INST_IRQHandler(void)
{

    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:
             count++;

            if(flag_en && (mode == MODE_TASK_LINE_STOP)) {
                update_task_line_stop_straight();
                if(!flag_en) {
                    speed_loop_stop();
                }
            } else if(flag_en && ((mode == 5) || (mode == 1))) {
                update_line_trace();
                if(!flag_en) {
                    speed_loop_stop();
                }
            }

            if (count==10)
            {

                Left_Speed=Left_Count;
                Left_Count_Sum+=Left_Count;
                Left_Count=0;

                Right_Speed=Right_Count;
                Right_Count_Sum+=Right_Count;
                Right_Count=0;

                if(trace_mode_protect_ticks > 0) {
                    trace_mode_protect_ticks--;
                }
                alert_signal_update();

                if(flag_en) {
                    if(mode == 7) {
                        update_mode7_distance_straight();
                    } else if(mode == 6) {
                        update_mode6_turn_then_straight();
                    } else if(mode == 3) {
                        update_mode3_path();
                    } else if(mode == MODE_TASK1_ALERT) {
                        update_task1_alert_pause();
                    } else if(mode == MODE_TASK_LINE_STOP) {
                        /* Line-stop straight is updated every timer tick for 0.1s line timing. */
                    } else if((mode == 5) || (mode == 1)) {
                        /* Tracing is updated every timer tick for faster response. */
                    } else if(mode == 4) {
                        update_gyro_reverse_straight();
                    } else if((mode == 2) || (mode == 8)) {
                        update_gyro_straight();
                    } else {
                        update_line_trace();
                    }

                    if(flag_en) {
                        if((mode != MODE_TASK1_ALERT) && !trace_outer_pivot_active) {
                            speed_loop_control();
                        }
                    } else {
                        speed_loop_stop();
                    }
                } else if(!motor_test_active) {
                    speed_loop_stop();
                }
                // NEW_Speed=NEW_Count;
                // NEW_Count_Sum+=NEW_Count;
                // NEW_Count=0;



                // c=PosionPID_realize(&PosionPIDR,Right_Speed);
                // motor_speed(0,addPID_realize(&addPID,Left_Speed));
            //     // printf("%d,%f\r\n",Left_Speed,PosionPID.target_val);//发给上危机
                // motor_speed(1,PosionPID_realize(&PosionPIDR,Right_Speed));
            //     // motor_speed(0,PosionPID_realize(&PosionPID,Left_Speed));
            //     // printf("%d,%d,%f\r\n",Right_Speed,Left_Speed,PosionPIDR.target_val);//发给上危机
            //     // sprintf(txbuffer, "%d\r\n",Left_Speed);
            //     // uart0_send_string(txbuffer);//可以映射到不同uart
                
            //     // DL_GPIO_togglePins(GPIO_LED_PORT,GPIO_LED_LED_PIN);
            //     if (mode==1) {//按下按钮循迹
            //         // motor_speed(0,PosionPID_realize(&cicriPID,Left_Count_Sum));
                    
            //         motor_speed(0,PosionPID_realize(&PosionPID,Left_Speed));
            //         // printf("%d,%f\r\n",Left_Speed,PosionPID.target_val);//发给上危机
            //         // printf("%d\r\n",Left_Count_Sum);//左边调参

            //         // motor_speed(1,PosionPID_realize(&cicriPID,Left_Count_Sum));
            //         motor_speed(1,PosionPID_realize(&PosionPIDR,Right_Speed));
            //         // printf("%d,%f\r\n",Right_Speed,PosionPIDR.target_val);//发给上危机
            //         // printf("%d\r\n",Left_Count_Sum);//右边调参
                    

            //     }
            //     if (mode==2) {
                    
                    
            //         // motor_speed(0,PosionPID_realize(&cicriPID,Left_Count_Sum));
            //         // motor_speed(1,PosionPID_realize(&cicriPIDR,Right_Count_Sum));
            //         motor_speed(0,PosionPID_realize(&PosionPID,Left_Speed));
            //         motor_speed(1,PosionPID_realize(&PosionPIDR,Right_Speed));//改为慢转
            //     }
            //     if (mode==3) {
            //         motor_speed(0,PosionPID_realize(&PosionPID,Left_Speed));
            //         motor_speed(1,PosionPID_realize(&PosionPIDR,Right_Speed));
            //     }
            //     if (mode==4) {
            //         motor_speed(0,PosionPID_realize(&cicriPID,Left_Count_Sum));
            //         motor_speed(1,PosionPID_realize(&cicriPIDR,Right_Count_Sum));
            //     }
            //     if (mode==5) {
            //         motor_speed(0,PosionPID_realize(&PosionPID,Left_Speed));
            //         motor_speed(1,PosionPID_realize(&PosionPIDR,Right_Speed));
            //     }
            //     if (mode==6) {
            //         motor_speed(0,PosionPID_realize(&cicriPID,Left_Count_Sum));
            //         motor_speed(1,PosionPID_realize(&cicriPIDR,Right_Count_Sum));
            //     }
            //     if (mode==7) {
            //         motor_speed(0,PosionPID_realize(&PosionPID,Left_Speed));
            //         motor_speed(1,PosionPID_realize(&PosionPIDR,Right_Speed));
            //     }
                count=0;
            }
            break;
        default:
            break;
    }

}
int fputc(int ch, FILE *f)
{
        DL_UART_transmitDataBlocking(UART_0_INST, ch);
        return (ch);
}

// 重定向fputs函数

int fputs(const char* restrict s, FILE* restrict stream) {
    uint16_t i,len;
    len = strlen(s);
    for(i=0;i<len;i++)
    {
        DL_UART_transmitDataBlocking(UART_0_INST, s[i]);
    }
    return len;
}



// 重定向puts函数

int puts(const char* _ptr)
{
    int count = fputs(_ptr,stdout);
    count += fputs("\n",stdout);
    return count;
}
