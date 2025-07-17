#ifndef INIT_DEVICE_H
#define INIT_DEVICE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化M5AtomS3r设备
     * @return ESP_OK 成功，其他值表示错误
     */
    esp_err_t init_m5atoms3r(void);

#ifdef __cplusplus
}
#endif

#endif // INIT_DEVICE_H