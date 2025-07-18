#include "initDevice.h"
#include "M5Unified.h"
#include "esp_log.h"
#include "connection/connection.h"
#include "esp_wifi.h"
#include "esp_spiffs.h" // 改为这个
#include "webPage/webPage.h"

void mount_spiffs()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        printf("SPIFFS 挂载失败\n");
    }
}

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

    mount_spiffs(); // 挂载SPIFFS文件系统

    wifi_config_t sta_config = {};
    esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &sta_config);
    if (err == ESP_OK)
    {
        printf("找到wifi配证，开始连接...\n");
        wifi_connect_sta((const char *)sta_config.sta.ssid, (const char *)sta_config.sta.password);
    }
    else
    {
        printf("未找到已保存的WiFi配置,进入ap模式\n");
        wifi_init_softap("M5AtomS3R", "12345678");
    }

    start_web_server();

    printf("M5AtomS3r设备初始化完成\r\n");
    return ESP_OK;
}
