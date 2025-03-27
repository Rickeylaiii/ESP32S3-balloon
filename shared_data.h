// 文件: shared_data.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// 传感器数据结构
typedef struct {
    float temperature;
    float pressure;
    float rawAltitude;
    float filteredAltitude;
    unsigned long lastReadTime;
} SensorData_t;

// 电机控制数据结构
typedef struct {
    int motor1Speed;    // 左电机速度(-100到100)
    int motor2Speed;    // 右电机速度(-100到100)
    int motor3Speed;    // 上升/下降电机速度(-100到100)
    bool autoControlActive; // 自动控制模式激活标志
} MotorControl_t;

// 全局共享数据
extern SensorData_t g_sensorData;
extern MotorControl_t g_motorControl;

// 互斥锁
extern SemaphoreHandle_t g_sensorMutex;
extern SemaphoreHandle_t g_motorMutex;
extern SemaphoreHandle_t g_wifiMutex;

// 消息队列
extern QueueHandle_t g_altitudeQueue;
extern QueueHandle_t g_motorCmdQueue;
extern QueueHandle_t g_webCmdQueue;

// 初始化共享资源
void initSharedResources();

#endif // SHARED_DATA_H