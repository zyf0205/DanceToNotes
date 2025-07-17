#include "imu.h"
#include "M5Unified.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "math.h"
#include <string.h>

static imu_data_t latest_data;
static SemaphoreHandle_t data_mutex = NULL;

// ============= IMU数据读取功能 =============

void imu_task(void *parameter)
{
    // 创建互斥锁
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL)
    {
        printf("创建互斥锁失败\r\n");
        return;
    }

    m5::imu_data_t m5_data;
    printf("IMU任务开始运行 (50Hz)\r\n");

    while (1)
    {
        // 更新M5设备和IMU
        M5.update();

        if (M5.Imu.update())
        {
            // 读取IMU数据
            m5_data = M5.Imu.getImuData();

            // 更新最新数据
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                latest_data.accel_x = m5_data.accel.x;
                latest_data.accel_y = m5_data.accel.y;
                latest_data.accel_z = m5_data.accel.z;

                latest_data.gyro_x = m5_data.gyro.x;
                latest_data.gyro_y = m5_data.gyro.y;
                latest_data.gyro_z = m5_data.gyro.z;

                latest_data.mag_x = m5_data.mag.x;
                latest_data.mag_y = m5_data.mag.y;
                latest_data.mag_z = m5_data.mag.z;

                xSemaphoreGive(data_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

int imu_get_data(imu_data_t *data)
{
    if (data == NULL)
    {
        return 0;
    }
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        *data = latest_data;
        xSemaphoreGive(data_mutex);
        return 1;
    }
    return 0;
}

// ============= 欧拉角计算 =============

// 简单的低通滤波器
typedef struct
{
    float alpha;
    float prev_value;
    bool initialized;
} low_pass_filter_t;

// 滤波器实例
static low_pass_filter_t accel_x_filter = {.alpha = 0.85f, .prev_value = 0, .initialized = false};
static low_pass_filter_t accel_y_filter = {.alpha = 0.85f, .prev_value = 0, .initialized = false};
static low_pass_filter_t accel_z_filter = {.alpha = 0.85f, .prev_value = 0, .initialized = false};
static low_pass_filter_t mag_x_filter = {.alpha = 0.7f, .prev_value = 0, .initialized = false};
static low_pass_filter_t mag_y_filter = {.alpha = 0.7f, .prev_value = 0, .initialized = false};
static low_pass_filter_t mag_z_filter = {.alpha = 0.7f, .prev_value = 0, .initialized = false};

// 低通滤波函数
float apply_low_pass(low_pass_filter_t *filter, float new_value)
{
    if (!filter->initialized)
    {
        filter->prev_value = new_value;
        filter->initialized = true;
        return new_value;
    }

    filter->prev_value = filter->alpha * new_value + (1.0f - filter->alpha) * filter->prev_value;
    return filter->prev_value;
}

// 角度标准化到 [-180, 180]
float normalize_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// 优化的欧拉角计算
void imu_calc_euler_optimized(const imu_data_t *raw, imu_euler_t *euler)
{
    // 滤波处理
    float ax = apply_low_pass(&accel_x_filter, raw->accel_x);
    float ay = apply_low_pass(&accel_y_filter, raw->accel_y);
    float az = apply_low_pass(&accel_z_filter, raw->accel_z);

    float mx = apply_low_pass(&mag_x_filter, raw->mag_x);
    float my = apply_low_pass(&mag_y_filter, raw->mag_y);
    float mz = apply_low_pass(&mag_z_filter, raw->mag_z);

    // 归一化加速度
    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    if (a_norm > 0.01f)
    {
        ax /= a_norm;
        ay /= a_norm;
        az /= a_norm;
    }
    else
    {
        ax = 0;
        ay = 0;
        az = 1;
    }

    // 计算Roll和Pitch
    euler->roll = atan2(ay, az) * 57.2958f;
    float pitch_val = fmaxf(-1.0f, fminf(1.0f, -ax));
    euler->pitch = asin(pitch_val) * 57.2958f;

    // 归一化磁力计
    float m_norm = sqrt(mx * mx + my * my + mz * mz);
    if (m_norm > 0.01f)
    {
        mx /= m_norm;
        my /= m_norm;
        mz /= m_norm;
    }
    else
    {
        return; // 磁力计数据无效
    }

    // 磁力计倾斜补偿
    float cr = cos(euler->roll * 0.017453f);
    float sr = sin(euler->roll * 0.017453f);
    float cp = cos(euler->pitch * 0.017453f);
    float sp = sin(euler->pitch * 0.017453f);

    float mx_comp = mx * cp + mz * sp;
    float my_comp = mx * sr * sp + my * cr - mz * sr * cp;

    // 计算Yaw并处理连续性
    float raw_yaw = atan2(-my_comp, mx_comp) * 57.2958f;

    static float prev_yaw = 0;
    static bool init = false;

    if (!init)
    {
        prev_yaw = raw_yaw;
        init = true;
    }
    else
    {
        float diff = raw_yaw - prev_yaw;
        if (diff > 180.0f)
            raw_yaw -= 360.0f;
        else if (diff < -180.0f)
            raw_yaw += 360.0f;
    }

    // Yaw滤波
    static float filt_yaw = 0;
    static bool yaw_init = false;

    if (!yaw_init)
    {
        filt_yaw = raw_yaw;
        yaw_init = true;
    }
    else
    {
        filt_yaw = 0.8f * raw_yaw + 0.2f * filt_yaw;
    }

    euler->yaw = normalize_angle(filt_yaw);
    prev_yaw = euler->yaw;
}

// 带运动检测的智能姿态计算
void imu_calc_euler_smart(const imu_data_t *raw, imu_euler_t *euler)
{
    // 检测是否在运动中
    static float prev_accel[3] = {0};
    static bool motion_detected = false;

    float accel_change = sqrt(
        (raw->accel_x - prev_accel[0]) * (raw->accel_x - prev_accel[0]) +
        (raw->accel_y - prev_accel[1]) * (raw->accel_y - prev_accel[1]) +
        (raw->accel_z - prev_accel[2]) * (raw->accel_z - prev_accel[2]));

    motion_detected = (accel_change > 0.1f); // 运动阈值

    if (motion_detected)
    {
        // 运动时使用较强的滤波
        accel_x_filter.alpha = 0.7f;
        accel_y_filter.alpha = 0.7f;
        accel_z_filter.alpha = 0.7f;
    }
    else
    {
        // 静止时使用较弱的滤波，提高响应性
        accel_x_filter.alpha = 0.9f;
        accel_y_filter.alpha = 0.9f;
        accel_z_filter.alpha = 0.9f;
    }

    // 使用优化的算法
    imu_calc_euler_optimized(raw, euler);

    // 更新上次加速度
    prev_accel[0] = raw->accel_x;
    prev_accel[1] = raw->accel_y;
    prev_accel[2] = raw->accel_z;
}

// ============= 工具函数 =============

// 根据时长匹配最接近的音符
note_duration_t match_note_duration(uint32_t duration_ms)
{
    uint32_t durations[] = {NOTE_SIXTEENTH, NOTE_EIGHTH, NOTE_QUARTER, NOTE_HALF};
    const char *names[] = {"十六分音符", "八分音符", "四分音符", "二分音符"};

    int best_match = 0;
    uint32_t min_diff = abs((int)duration_ms - (int)durations[0]);

    for (int i = 1; i < 4; i++)
    {
        uint32_t diff = abs((int)duration_ms - (int)durations[i]);
        if (diff < min_diff)
        {
            min_diff = diff;
            best_match = i;
        }
    }

    printf("音符: %s\n", names[best_match]);

    return (note_duration_t)durations[best_match];
}

// 简化的动作名称
const char *get_action_name(simple_action_t action)
{
    switch (action)
    {
    case ACTION_TILT_UP:
        return "向上倾斜";
    case ACTION_TILT_DOWN:
        return "向下倾斜";
    default:
        return "无动作";
    }
}

// ============= 三点检测算法 =============

// 三个特征点定义
typedef struct
{
    float roll;
    float pitch;
    float tolerance;
    const char *name;
} feature_point_t;

// 动作模板
typedef struct
{
    feature_point_t point1;
    feature_point_t point2;
    feature_point_t point3;
    uint32_t max_duration_ms; // 最大完成时间
    simple_action_t action_id;
    const char *action_name;
} three_point_template_t;

// 定义动作模板 - 只保留这一个，删除第539行的重复定义
static const three_point_template_t three_point_templates[] = {

    // 向下倾斜：roll 0° -> -25° -> -50°
    {
        .point1 = {0.0f, 0.0f, 25.0f, "起始点"},   // 增大容差
        .point2 = {-25.0f, 0.0f, 20.0f, "中间点"}, // 增大容差
        .point3 = {-50.0f, 0.0f, 20.0f, "结束点"}, // 增大容差
        .max_duration_ms = 1000,                   // 保留但仅用于显示
        .action_id = ACTION_TILT_DOWN,
        .action_name = "向下倾斜"},

    // 向上倾斜：roll -50° -> -25° -> 0°
    {
        .point1 = {-50.0f, 0.0f, 25.0f, "起始点"},
        .point2 = {-25.0f, 0.0f, 20.0f, "中间点"},
        .point3 = {0.0f, 0.0f, 20.0f, "结束点"},
        .max_duration_ms = 1000,
        .action_id = ACTION_TILT_UP,
        .action_name = "向上倾斜"}};

// 检测状态
typedef enum
{
    POINT_STATE_IDLE,     // 空闲，等待第一个点
    POINT_STATE_POINT1,   // 已检测到第一个点
    POINT_STATE_POINT2,   // 已检测到第二个点
    POINT_STATE_COMPLETED // 动作完成
} point_state_t;

// 三点检测器
typedef struct
{
    point_state_t state;
    int current_template;    // 当前匹配的模板索引
    uint32_t start_time;     // 动作开始时间
    uint32_t point1_time;    // 第一个点时间
    uint32_t point2_time;    // 第二个点时间
    uint32_t point3_time;    // 第三个点时间
    uint32_t last_detection; // 上次检测完成时间
} three_point_detector_t;

static three_point_detector_t detector = {
    .state = POINT_STATE_IDLE,
    .current_template = -1,
    .start_time = 0,
    .point1_time = 0,
    .point2_time = 0,
    .point3_time = 0,
    .last_detection = 0};

// 改进的点匹配函数 - 只保留这一个，删除第560行的重复定义
bool matches_point(const imu_euler_t *euler, const feature_point_t *point)
{
    float roll_diff = fabs(euler->roll - point->roll);
    float pitch_diff = fabs(euler->pitch - point->pitch);

    // 对Roll轴给予更大的权重，因为主要动作是Roll变化
    float weighted_distance = sqrt(
        (roll_diff * roll_diff) +
        (pitch_diff * pitch_diff * 0.5f) // Pitch误差权重降低
    );

    return weighted_distance <= point->tolerance;
}

// 三点检测主函数
simple_action_t detect_three_point_action(const imu_euler_t *euler, uint32_t *execution_time, note_duration_t *note_type)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    const int num_templates = sizeof(three_point_templates) / sizeof(three_point_templates[0]);

    switch (detector.state)
    {
    case POINT_STATE_IDLE:
        // 寻找第一个点的匹配
        for (int i = 0; i < num_templates; i++)
        {
            if (matches_point(euler, &three_point_templates[i].point1))
            {
                detector.state = POINT_STATE_POINT1;
                detector.current_template = i;
                detector.start_time = current_time;
                detector.point1_time = current_time;

                printf("🎯 第1点: %s - %s (R=%.1f°)\n",
                       three_point_templates[i].action_name,
                       three_point_templates[i].point1.name,
                       euler->roll);
                break;
            }
        }
        break;

    case POINT_STATE_POINT1:
    {
        const three_point_template_t *action_template = &three_point_templates[detector.current_template];

        // 第一阶段不设超时，给足够时间到达第二个点
        uint32_t elapsed_from_start = current_time - detector.start_time;

        // 如果在第一个点停留太久（比如5秒），重置
        if (elapsed_from_start > 5000)
        {
            printf("⏰ 第1点停留过久，重置\n");
            detector.state = POINT_STATE_IDLE;
            break;
        }

        // 检查第二个点
        if (matches_point(euler, &action_template->point2))
        {
            detector.state = POINT_STATE_POINT2;
            detector.point2_time = current_time; // 从第二个点开始计时！

            printf("🎯 第2点: %s (R=%.1f°) - 开始计时\n",
                   action_template->point2.name,
                   euler->roll);
        }
        break;
    }

    case POINT_STATE_POINT2:
    {
        const three_point_template_t *action_template = &three_point_templates[detector.current_template];

        // 从第二个点开始计算超时时间（1秒）
        uint32_t elapsed_from_point2 = current_time - detector.point2_time;

        // 超时检查：从第二个点开始1秒内必须完成
        if (elapsed_from_point2 > 1000)
        {
            printf("⏰ 从第2点超时1秒，重置\n");
            detector.state = POINT_STATE_IDLE;
            break;
        }

        // 检查第三个点
        if (matches_point(euler, &action_template->point3))
        {
            detector.point3_time = current_time;

            // 计算从第二个点到第三个点的时间作为执行时间
            uint32_t execution_duration = current_time - detector.point2_time;
            uint32_t total_time = current_time - detector.start_time;

            // 动作完成！
            *execution_time = execution_duration;
            *note_type = match_note_duration(execution_duration);

            printf("🎯 第3点: %s (R=%.1f°)\n",
                   action_template->point3.name, euler->roll);
            printf("✅ %s 完成! 执行时间: %lums (总时间: %lums)\n",
                   action_template->action_name, execution_duration, total_time);

            detector.state = POINT_STATE_COMPLETED;
            detector.last_detection = current_time;

            return action_template->action_id;
        }
        break;
    }

    case POINT_STATE_COMPLETED:
        // 立即重置，支持连续检测
        detector.state = POINT_STATE_IDLE;
        printf("🔄 立即准备下一个动作检测\n");
        break;
    }

    return ACTION_NONE;
}

// 显示三点检测状态
void print_three_point_status(const imu_euler_t *euler)
{
    printf("当前姿态: Roll=%.1f° Pitch=%.1f°\n", euler->roll, euler->pitch);

    const char *state_names[] = {"空闲", "等待第2点", "等待第3点", "已完成"};
    printf("检测状态: %s\n", state_names[detector.state]);

    if (detector.current_template >= 0)
    {
        const three_point_template_t *action_template = &three_point_templates[detector.current_template];
        printf("当前动作: %s\n", action_template->action_name);

        if (detector.state == POINT_STATE_POINT1)
        {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - detector.start_time;
            printf("第1点已用时: %lums (无超时限制)\n", elapsed);
            printf("目标: 第2点 Roll=%.1f° (容差±%.1f°)\n",
                   action_template->point2.roll, action_template->point2.tolerance);
        }
        else if (detector.state == POINT_STATE_POINT2)
        {
            uint32_t elapsed_from_point2 = (esp_timer_get_time() / 1000) - detector.point2_time;
            uint32_t remaining = elapsed_from_point2 < 1000 ? 1000 - elapsed_from_point2 : 0;

            printf("第2点已用时: %lums / 1000ms (剩余: %lums)\n",
                   elapsed_from_point2, remaining);
            printf("目标: 第3点 Roll=%.1f° (容差±%.1f°)\n",
                   action_template->point3.roll, action_template->point3.tolerance);
        }
    }

    // 显示当前位置匹配情况
    const int num_templates = sizeof(three_point_templates) / sizeof(three_point_templates[0]);
    for (int i = 0; i < num_templates; i++)
    {
        const three_point_template_t *action_template = &three_point_templates[i];

        if (matches_point(euler, &action_template->point1))
        {
            printf("可启动: %s (第1点匹配)\n", action_template->action_name);
        }
    }
}

// 重置三点检测器
void reset_three_point_detector(void)
{
    detector.state = POINT_STATE_IDLE;
    detector.current_template = -1;
    detector.last_detection = 0;
    printf("三点检测器已重置\n");
}
