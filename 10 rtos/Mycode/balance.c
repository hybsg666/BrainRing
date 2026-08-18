// balance.c
#include "balance.h"
#include "servo.h"
#include "3TOF3Motor.h"
#include <stdio.h>
#include <math.h>

extern float current_roll;

// PID 结构体（保留但未使用，可删除）
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output_limit;
    float integral_limit;
} PID_t;

static PID_t pid;
static uint8_t focused_state;      // 0=放松, 1=专注
static uint32_t last_swing_time;
static int swing_direction;
static uint32_t last_vibration_time;
static uint8_t vibration_active;
static uint16_t target_angle;

// 用于状态机历史记录
static uint8_t att_history[10];
static uint8_t hist_index = 0;
static uint8_t hist_filled = 0;

// 用于 TOF 无效值保持（未使用，保留）
static float last_left_valid = 0, last_right_valid = 0;
static float left_filt = 0, right_filt = 0;

static void PID_Init(PID_t *p, float kp, float ki, float kd, float out_lim, float int_lim)
{
    p->Kp = kp; p->Ki = ki; p->Kd = kd;
    p->integral = 0; p->prev_error = 0;
    p->output_limit = out_lim; p->integral_limit = int_lim;
}

void Balance_Init(void)
{
    focused_state = 0;
    last_swing_time = 0;
    swing_direction = 1;
    last_vibration_time = 0;
    vibration_active = 0;
    target_angle = 90;
    hist_index = 0;
    hist_filled = 0;
    last_left_valid = 0;
    last_right_valid = 0;
    left_filt = 0;
    right_filt = 0;
    PID_Init(&pid, 1.2f, 0.08f, 0.15f, 30.0f, 50.0f);
    printf("Balance module initialized\r\n");
}

void Balance_Update(uint8_t attention, float left_dist, float right_dist)
{
    const uint32_t vibration_duration_ms = 150;
    uint32_t now = xTaskGetTickCount();

    // ---------- 距离有效性检查（仅用于碰撞检测） ----------
    int left_valid = (left_dist > 10 && left_dist < 4000);
    int right_valid = (right_dist > 10 && right_dist < 4000);
    float left = left_valid ? left_dist : 0;
    float right = right_valid ? right_dist : 0;

    // ---------- 状态机（基于历史窗口） ----------
    static uint8_t att_history[10];
    static uint8_t hist_index = 0, hist_filled = 0;
    att_history[hist_index] = attention;
    hist_index = (hist_index + 1) % 10;
    if (hist_filled < 10) hist_filled++;

    uint8_t high_cnt = 0, low_cnt = 0;
    for (int i = 0; i < hist_filled; i++) {
        if (att_history[i] > 70) high_cnt++;
        else if (att_history[i] < 30) low_cnt++;
    }
    uint8_t high_ratio = (high_cnt * 100) / hist_filled;
    uint8_t low_ratio  = (low_cnt * 100) / hist_filled;

    static uint8_t focused_state = 0;
    if (high_ratio > 60 && focused_state == 0) {
        focused_state = 1;
        printf("[CTRL] Enter FOCUS mode (ratio=%d%%)\r\n", high_ratio);
        Servo_SmoothToAngle(90, 5);
    } else if (low_ratio > 60 && focused_state == 1) {
        focused_state = 0;
        printf("[CTRL] Enter RELAX mode (ratio=%d%%)\r\n", low_ratio);
    }

    // ---------- 震动控制（放松模式碰撞检测） ----------
    static uint32_t last_vibration_time = 0;
    static uint8_t vibration_active = 0;
    if (vibration_active) {
        if (now - last_vibration_time >= pdMS_TO_TICKS(vibration_duration_ms)) {
            SetMotorDutyByDistance(0, 1);
            SetMotorDutyByDistance(0, 3);
            vibration_active = 0;
        }
    } else {
        if (focused_state == 0) {
            if (left_valid && left < 50.0f) {
                SetMotorDutyByDistance(100, 1);
                last_vibration_time = now;
                vibration_active = 1;
                printf("Left collision!\r\n");
            } else if (right_valid && right < 50.0f) {
                SetMotorDutyByDistance(100, 3);
                last_vibration_time = now;
                vibration_active = 1;
                printf("Right collision!\r\n");
            }
        }
    }

    // ---------- 舵机控制 ----------
    static uint16_t target_angle = 90;
    if (focused_state == 1) {
        // 专注模式：立即水平 + 极慢速 MPU6050 修正 + 极慢速 TOF 修正
        static uint32_t last_correction = 0;
        static uint16_t current_angle = 90;
        static uint8_t first_run = 1;
        
        if (first_run) {
            first_run = 0;
            current_angle = 90;
            Servo_SmoothToAngle(current_angle, 5);
            printf("[FOCUS] Set angle to 90\n");
        }
        
        if (now - last_correction >= pdMS_TO_TICKS(10000)) {
            last_correction = now;
            float total_step = 0.0f;
            
            // 1. MPU6050 修正（基于 Roll 角）
            float error_roll = -current_roll;
            float step_roll = error_roll * 0.05f;
            if (step_roll > 0.5f) step_roll = 0.5f;
            if (step_roll < -0.5f) step_roll = -0.5f;
            total_step += step_roll;
            
            // 2. TOF 修正（基于左右距离差，死区30mm，比例系数 0.01，最大步长0.3度）
            if (left_valid && right_valid) {
                float error_tof = right - left;   // 正值表示球偏左，需向右倾
                if (fabsf(error_tof) > 30.0f) {
                    float step_tof = error_tof * 0.01f;
                    if (step_tof > 0.3f) step_tof = 0.3f;
                    if (step_tof < -0.3f) step_tof = -0.3f;
                    total_step += step_tof;
                }
            }
            
            // 限制总步长在 ±0.8 度内，避免突变
            if (total_step > 0.8f) total_step = 0.8f;
            if (total_step < -0.8f) total_step = -0.8f;
            int16_t step = (int16_t)(total_step);
            if (step != 0) {
                current_angle += step;
                if (current_angle > 180) current_angle = 180;
                if (current_angle < 0) current_angle = 0;
                Servo_SmoothToAngle(current_angle, 10);
                printf("[SLOW CORRECTION] Roll=%.2f error_tof=%.1f step=%d angle=%d\n",
                       current_roll, (left_valid&&right_valid)?(right-left):0, step, current_angle);
            }
        }
        // 每 3 秒保持
        static uint32_t last_keep = 0;
        if (now - last_keep >= pdMS_TO_TICKS(3000)) {
            last_keep = now;
            Servo_SmoothToAngle(current_angle, 10);
        }
    } else {
        // 放松模式：大幅摆动（幅度50°，半周期1秒）
        static uint32_t last_swing_time = 0;
        static int swing_dir = 1;
        if (now - last_swing_time >= pdMS_TO_TICKS(1000)) {
            last_swing_time = now;
            swing_dir = -swing_dir;
            float angle = 90 + swing_dir * 50.0f;
            if (angle > 180) angle = 180;
            if (angle < 0) angle = 0;
            target_angle = (uint16_t)angle;
            Servo_SmoothToAngle(target_angle, 5);
            printf("SWING angle=%.0f\n", angle);
        }
    }

    // 状态打印
    static uint32_t last_print = 0;
    if (now - last_print >= pdMS_TO_TICKS(300)) {
        last_print = now;
        const char* state_str = (focused_state == 1) ? "FOCUS" : "RELAX";
        printf("[%s] ATT=%3d | L=%3.0f R=%3.0f | angle=%3d\r\n",
               state_str, attention, left, right, target_angle);
    }
}


uint16_t Balance_GetTargetAngle(void)
{
    return target_angle;
}
