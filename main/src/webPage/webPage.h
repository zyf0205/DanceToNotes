#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 启动网页服务器
     * @param mode 模式选择："ap" = AP模式(配网页面), "sta" = STA模式(功能页面)
     */
    void start_web_server();

    /**
     * @brief 停止网页服务器
     */
    void stop_web_server(void);

    /**
     * @brief 更新传感器数据
     * @param accel_x 加速度X轴
     * @param accel_y 加速度Y轴
     * @param accel_z 加速度Z轴
     * @param gyro_x 陀螺仪X轴
     * @param gyro_y 陀螺仪Y轴
     * @param gyro_z 陀螺仪Z轴
     */
    void update_sensor_data(float accel_x, float accel_y, float accel_z,
                            float gyro_x, float gyro_y, float gyro_z);

#ifdef __cplusplus
}
#endif