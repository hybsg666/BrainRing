//#include "attention.h"
//#include <math.h>
//#include <string.h>

//static uint16_t g_adc_offset = 32768;

//static uint8_t att_median_buf[MEDIAN_WINDOW];
//static int att_median_idx = 0;
//static uint8_t att_smooth_buf[SMOOTH_WINDOW];
//static int att_smooth_idx = 0;

//void attention_init(void)
//{
//    for (int i = 0; i < MEDIAN_WINDOW; i++) att_median_buf[i] = 50;
//    for (int i = 0; i < SMOOTH_WINDOW; i++) att_smooth_buf[i] = 50;
//    att_median_idx = 0;
//    att_smooth_idx = 0;
//}

//void attention_set_offset(uint16_t offset)
//{
//    g_adc_offset = offset;
//}

//static uint8_t median_filter(uint8_t new_value)
//{
//    att_median_buf[att_median_idx++] = new_value;
//    if (att_median_idx >= MEDIAN_WINDOW) att_median_idx = 0;

//    uint8_t sorted[MEDIAN_WINDOW];
//    memcpy(sorted, att_median_buf, MEDIAN_WINDOW);
//    for (int i = 0; i < MEDIAN_WINDOW-1; i++) {
//        for (int j = i+1; j < MEDIAN_WINDOW; j++) {
//            if (sorted[i] > sorted[j]) {
//                uint8_t tmp = sorted[i];
//                sorted[i] = sorted[j];
//                sorted[j] = tmp;
//            }
//        }
//    }
//    return sorted[MEDIAN_WINDOW/2];
//}

//static uint8_t moving_average(uint8_t new_value)
//{
//    att_smooth_buf[att_smooth_idx++] = new_value;
//    if (att_smooth_idx >= SMOOTH_WINDOW) att_smooth_idx = 0;

//    uint32_t sum = 0;
//    for (int i = 0; i < SMOOTH_WINDOW; i++) sum += att_smooth_buf[i];
//    return (uint8_t)(sum / SMOOTH_WINDOW);
//}

//uint8_t compute_attention_from_adc(const uint16_t *adc_buffer)
//{
//    // 1. 计算实际 RMS（不减去偏置，因为噪声本身就是信号）
//    float sum_sq = 0.0f;
//    for (int i = 0; i < FFT_SIZE; i++) {
//        float val = (float)adc_buffer[i];
//        sum_sq += val * val;
//    }
//    float rms = sqrtf(sum_sq / FFT_SIZE);

//    // 2. 根据你的数据范围，rms 大约在 5000~18000 之间波动
//    //    将 rms 线性映射到 0~100，低 rms 对应放松，高 rms 对应专注
//    float att_f = (rms - 5000.0f) / 13000.0f;  // 假设最小5000，最大18000
//    if (att_f < 0) att_f = 0;
//    if (att_f > 1) att_f = 1;
//    uint8_t att = (uint8_t)(att_f * 100);
//    return att;
//}


#include "attention.h"
#include <math.h>

static float last_rms = 0;
static uint8_t high_counter = 0;
static uint8_t exceed_count = 0;

// 可选择对原始数据做简单平滑（可选，不必须）
static float smooth_buffer[FFT_SIZE];
void smooth_data(const uint16_t *src, float *dst, int len)
{
    // 简单滑动平均，窗口3
    for (int i = 0; i < len; i++) {
        float sum = src[i];
        if (i > 0) sum += src[i-1];
        if (i < len-1) sum += src[i+1];
        dst[i] = sum / (1 + (i>0) + (i<len-1));
    }
}

void attention_init(void)
{
    last_rms = 0;
    high_counter = 0;
    exceed_count = 0;
}

void attention_set_offset(uint16_t offset)
{
    (void)offset;
}



// 配置参数（最终稳定版）
#define RMS_FOCUS_HIGH       750.0f   // RMS ≤ 750 线性映射 85~100
#define RMS_RELAX_LOW       1000.0f   // 放松时 RMS 映射基准：1000→20, 2000→0
#define RMS_EXIT_FOCUS      1150.0f   // 退出专注需要 RMS ≥ 1150
#define DELTA_ACTIVE_THRESH   40.0f
#define ACTIVITY_WINDOW        15
#define FOCUS_ACTIVITY_RATIO   0.5f
#define RELAX_ACTIVITY_RATIO   0.1f   // 退出专注要求活动率 ≤ 0.1
#define EXIT_FOCUS_CONSEC      7      // 退出专注需要连续7帧满足条件
#define ENTER_FOCUS_CONSEC     2

// 参数配置
#define RMS_FOCUS_MAX     800.0f   // RMS <= 800 时专注度线性增加
#define RMS_RELAX_MIN    1100.0f   // RMS >= 1100 时专注度线性减少
#define DELTA_MAP_MAX     80.0f    // delta 超过 80 贡献全分
#define WINDOW_SIZE        7       // 平滑窗口（帧数）
#define SMOOTH_FACTOR     0.3f     // 一阶低通系数

uint8_t compute_attention_from_adc(const uint16_t *adc_buffer)
{
    static float last_rms = 0;
    static float delta_buf[WINDOW_SIZE];
    static float rms_buf[WINDOW_SIZE];
    static int idx = 0;
    static int filled = 0;
    static float smooth_att = 50.0f;
    
    // 1. 计算当前帧 RMS
    float sum_sq = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) {
        float val = (float)adc_buffer[i];
        sum_sq += val * val;
    }
    float rms = sqrtf(sum_sq / FFT_SIZE);
    
    // 2. 计算当前帧 delta
    float delta = (last_rms != 0.0f) ? fabsf(rms - last_rms) : 0.0f;
    last_rms = rms;
    
    // 3. 强干扰剔除（单帧 delta > 500 丢弃）
    if (delta > 500.0f) {
        static uint8_t last_att = 50;
        return last_att;
    }
    
    // 4. 存入环形缓冲区
    delta_buf[idx] = delta;
    rms_buf[idx] = rms;
    idx = (idx + 1) % WINDOW_SIZE;
    if (filled < WINDOW_SIZE) filled++;
    
    // 5. 计算窗口内的平均 delta 和平均 RMS
    float avg_delta = 0.0f, avg_rms = 0.0f;
    for (int i = 0; i < filled; i++) {
        avg_delta += delta_buf[i];
        avg_rms += rms_buf[i];
    }
    avg_delta /= filled;
    avg_rms /= filled;
    
    // 6. 基于 RMS 的专注度贡献（线性映射：rms=800 -> 100, rms=1100 -> 0）
    float rms_att;
    if (avg_rms <= RMS_FOCUS_MAX) {
        rms_att = 100.0f;
    } else if (avg_rms >= RMS_RELAX_MIN) {
        rms_att = 0.0f;
    } else {
        rms_att = 100.0f - (avg_rms - RMS_FOCUS_MAX) * (100.0f / (RMS_RELAX_MIN - RMS_FOCUS_MAX));
    }
    
    // 7. 基于 delta 的专注度贡献（线性映射：delta=0 -> 0, delta=80 -> 100）
    float delta_att;
    if (avg_delta <= 0) delta_att = 0;
    else if (avg_delta >= DELTA_MAP_MAX) delta_att = 100;
    else delta_att = avg_delta * (100.0f / DELTA_MAP_MAX);
    
    // 8. 融合：优先采用 RMS 贡献，但 delta 贡献作为加分（不超过100）
    //    当 RMS 很低时，delta 贡献权重降低，因为已经足够专注；当 RMS 中等时，delta 贡献权重增加
    float weight_rms = (avg_rms <= RMS_FOCUS_MAX) ? 1.0f :
                       (avg_rms >= RMS_RELAX_MIN) ? 0.0f :
                       1.0f - (avg_rms - RMS_FOCUS_MAX) / (RMS_RELAX_MIN - RMS_FOCUS_MAX);
    float weight_delta = 1.0f - weight_rms * 0.5f;  // delta 最大权重0.5
    
    float raw_att = rms_att * weight_rms + delta_att * weight_delta;
    if (raw_att > 100) raw_att = 100;
    
    // 9. 低通平滑
    smooth_att = smooth_att * (1 - SMOOTH_FACTOR) + raw_att * SMOOTH_FACTOR;
    uint8_t att = (uint8_t)smooth_att;
    
    // 调试打印（可取消注释观察）
     //printf("rms=%.1f d=%.1f rms_att=%.1f d_att=%.1f att=%d\n", avg_rms, avg_delta, rms_att, delta_att, att);
    
    return att;
}





















