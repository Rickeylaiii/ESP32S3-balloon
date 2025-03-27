// �ļ�: shared_data.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ���������ݽṹ
typedef struct {
    float temperature;
    float pressure;
    float rawAltitude;
    float filteredAltitude;
    unsigned long lastReadTime;
} SensorData_t;

// ����������ݽṹ
typedef struct {
    int motor1Speed;    // �����ٶ�(-100��100)
    int motor2Speed;    // �ҵ���ٶ�(-100��100)
    int motor3Speed;    // ����/�½�����ٶ�(-100��100)
    bool autoControlActive; // �Զ�����ģʽ�����־
} MotorControl_t;

// ȫ�ֹ�������
extern SensorData_t g_sensorData;
extern MotorControl_t g_motorControl;

// ������
extern SemaphoreHandle_t g_sensorMutex;
extern SemaphoreHandle_t g_motorMutex;
extern SemaphoreHandle_t g_wifiMutex;

// ��Ϣ����
extern QueueHandle_t g_altitudeQueue;
extern QueueHandle_t g_motorCmdQueue;
extern QueueHandle_t g_webCmdQueue;

// ��ʼ��������Դ
void initSharedResources();

#endif // SHARED_DATA_H