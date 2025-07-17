#include "initDevice.h"
#include "M5Unified.h"
#include "esp_log.h"

static const char *TAG = "INIT_DEVICE";

esp_err_t init_m5atoms3r(void)
{
    // 配置M5Unified
    auto cfg = M5.config();

    // M5AtomS3r特定配置
    cfg.clear_display = true; // 清空显示
    cfg.output_power = true;  // 启用输出电源
    cfg.internal_imu = true;  // 启用内部IMU传感器
    cfg.led_brightness = 64;  // LED亮度 (0-255)

    M5.begin(cfg);
    ESP_LOGI(TAG, "M5AtomS3r设备初始化完成\r\n");
    return ESP_OK;
}
