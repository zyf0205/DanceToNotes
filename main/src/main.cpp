#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "M5Unified.h"
#include "initDevice/initDevice.h"
#include "imu/imu.h"
#include "nvs_flash.h"

TaskHandle_t imu_handle = NULL;

// 用于检测数值变化的变量
static float last_displayed_roll = 999.0f;
static float last_displayed_pitch = 999.0f;

// 初始化屏幕显示
void init_display()
{
    M5.Display.clear(BLACK);
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.setTextSize(1);
}

// 更新角度显示（只刷新变化的数值）
void update_angles_display(const imu_euler_t *euler)
{
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(WHITE, BLACK);

    // 检查Roll角是否变化（精度到0.1度）
    if (fabs(euler->roll - last_displayed_roll) >= 0.1f)
    {
        // 清除Roll角显示区域
        M5.Display.fillRect(0, 20, 320, 40, BLACK);

        // 显示新的Roll角
        M5.Display.setCursor(10, 30);
        M5.Display.printf("R: %6.1f", euler->roll);

        last_displayed_roll = euler->roll;
    }

    // 检查Pitch角是否变化（精度到0.1度）
    if (fabs(euler->pitch - last_displayed_pitch) >= 0.1f)
    {
        // 清除Pitch角显示区域
        M5.Display.fillRect(0, 80, 320, 40, BLACK);

        // 显示新的Pitch角
        M5.Display.setCursor(10, 90);
        M5.Display.printf("P: %6.1f", euler->pitch);

        last_displayed_pitch = euler->pitch;
    }
}

extern "C" void app_main(void)
{
    // 初始化M5设备
    init_m5atoms3r();
    printf("🚀 设备初始化完成\r\n");

    // 初始化屏幕显示
    init_display();

    // 启动IMU任务
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, &imu_handle);

    // 主循环变量
    imu_data_t data;
    imu_euler_t euler;
    uint32_t execution_time;
    note_duration_t note_type;

    // 音符频率 (两个动作对应两个音符)
    const float action_frequencies[] = {
        523.25f, // Do高 - 向上倾斜
        329.63f  // Mi - 向下倾斜
    };

    printf("🎼 三点检测系统启动\n");
    printf("支持动作:\n");
    printf("  向上倾斜: Roll 0° → 25° → 50° (1秒内)\n");
    printf("  向下倾斜: Roll 0° → -25° → -50° (1秒内)\n\n");

    while (1)
    {
        M5.update();

        if (imu_get_data(&data))
        {
            // 计算欧拉角
            imu_calc_euler_smart(&data, &euler);

            // 更新屏幕角度显示
            update_angles_display(&euler);

            // 使用三点检测
            simple_action_t action = detect_three_point_action(&euler, &execution_time, &note_type);

            // 处理检测结果
            if (action != ACTION_NONE)
            {
                printf("🎵 播放音符: %s -> %.1fHz (%dms)\n",
                       get_action_name(action),
                       action_frequencies[action],
                       note_type);

                // 这里可以添加你的音频播放函数
                // play_tone(action_frequencies[action], note_type);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz刷新率
    }
}
