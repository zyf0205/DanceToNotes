#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // IMU数据结构
    typedef struct
    {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float mag_x, mag_y, mag_z;
    } imu_data_t;

    // 欧拉角结构
    typedef struct
    {
        float roll, pitch, yaw;
    } imu_euler_t;

    // 简单动作类型
    typedef enum
    {
        ACTION_TILT_UP = 0,
        ACTION_TILT_DOWN,
        HAND_DOWN,
        ACTION_NONE
    } simple_action_t;

    // 音符时长类型
    typedef enum
    {
        NOTE_SIXTEENTH = 125,
        NOTE_EIGHTH = 250,
        NOTE_QUARTER = 500,
        NOTE_HALF = 1000
    } note_duration_t;

    // 基础IMU函数
    void imu_task(void *parameter);
    int imu_get_data(imu_data_t *data);
    void imu_calc_euler_smart(const imu_data_t *raw, imu_euler_t *euler);
    void imu_calc_euler_optimized(const imu_data_t *raw, imu_euler_t *euler);

    // 工具函数
    note_duration_t match_note_duration(uint32_t duration_ms);
    const char *get_action_name(simple_action_t action);

    // 三点检测算法
    simple_action_t detect_three_point_action(const imu_euler_t *euler, uint32_t *execution_time, note_duration_t *note_type);
    void print_three_point_status(const imu_euler_t *euler);
    void reset_three_point_detector(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_H