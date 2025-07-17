#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "M5Unified.h"
#include "initDevice/initDevice.h"
#include "imu/imu.h"
#include "nvs_flash.h"

TaskHandle_t imu_handle = NULL;

extern "C" void app_main(void)
{
    // 初始化M5设备
    init_m5atoms3r();
    printf("🚀 设备初始化完成\r\n");

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

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
