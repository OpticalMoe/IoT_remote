/*!
 * @file        SmartRemote.ino
 * @brief       红外遥控 - 格力空调
 * @copyright   Copyright (c) 2021 ChenYuanliang
 * @licence     CC-BY-NC-SA 3.0，知识共享许可协议-署名-非商业使用-相同方式共享。
 * @author      ChenYuanliang
 * @version     V1.0
 * @date        2021-11-01
 * @url         https://github.com/OpticalMoe/IoT_remote
 */

 /*
    本程序及需要魔改的库中涉及个人隐私信息部分已用"xxxx"打码（长度是对的），需要修改为自己的信息，否则无法通过编译或运行。
 */

//硬件版本 V1.0
/*
 * 需要库 
 * DFRobot_SHT3x-master   https://github.com/DFRobot/DFRobot_SHT3x
 * 以下库可在Arduino库管理添加
 * AliyunIoTSDK   库需要魔改，https://github.com/yu-tou/arduino-aliyun-iot-sdk
 * ArduinoJson
 * PubSubClient
 * Crypto
 * 
 * WiFi                   
 * Wire
 */
#include <WiFi.h>
#include "irRemote.h"

// 两台设备通过这里的开关控制编译的
#define   GD001   1
#define   GD002   0

//******** Queue ********//
QueueHandle_t xQueue;
struct Sensor
{
  float Temperature = 0.0;
  float Humidity = 0.0;
  int AmbientLight = 0;
  char BatteryState = 0;
  char BatteryPercentage = 0;
};

// * Mode:        A -> 自动, C -> 制冷, D -> 抽湿, H -> 制热, F -> 送风
// * windSpeed:   A -> 自动, L -> 低风, M -> 中风, H -> 高风, F -> 固定风
struct AirConditioner
{
  bool Power = 0;
  int TargetTemperature = 26;
  int Mode = 0;
  int WindSpeed = 0;
  bool FlagRemote = 0;
  bool FlagUpload = 0;
  long TimeStamp = 0;
} airConditioner;

//******** IoT ********//
static WiFiClient espClient;
#include <AliyunIoTSDK.h>   // 引入阿里云 IoT SDK
// 设置产品和设备的信息，从阿里云设备信息里查看
#if GD001
#define PRODUCT_KEY "a11jwuxxxxx" // 已打码 （长度是对的）
#define DEVICE_NAME "GD001"
#define DEVICE_SECRET "21a4a593xxxxxxxxxxxxxxx6f14d40c1"  // 已打码 （长度是对的）
#elif GD002
#define PRODUCT_KEY "a11jwuxxxxx" // 已打码 （长度是对的）
#define DEVICE_NAME "GD002"
#define DEVICE_SECRET "56f1f0xxxxxxxxxxxxxxxaac609109da"  // 已打码 （长度是对的）
#endif
#define REGION_ID "cn-shanghai"
// 设置 wifi 信息
#define WIFI_SSID "IoT"
#define WIFI_PASSWD "yuanliang"

//******** SHT30 ********//
#include <DFRobot_SHT3x.h>
DFRobot_SHT3x   sht3x;

//******** GPIO ********//
#define LED_R 33
#define LED_G 25
#define LED_B 26
#define AMBIENT_LIGHT  34

//******** 声明 ********//
void TaskBlink( void *pvParameters );
void TaskAnalogReadA3( void *pvParameters );

//******** 任务句柄 ********//
TaskHandle_t xHandleBlink = NULL;
TaskHandle_t xHandleCloudReport = NULL;
TaskHandle_t xHandleCloudLoop = NULL;
TaskHandle_t xHandleRemote = NULL;
TaskHandle_t xHandleSensor = NULL;
TaskHandle_t xHandleACStatus = NULL;

//******** #define ********//
//Core 0
#define CLOUD_REPORT  1
#define CLOUD_LOOP    1
#define SENSOR        1
//Core 1
#define BLINK         1
#define REMOTE        1
#define AC_STATUS     1

void setup() {
  Serial.begin(115200);

  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, HIGH);

  //******** RTOS ********//
  //核心0：阿里云相关；属性上报，上云LOOP
  //核心1：Remote；传感器采集；空调状态；

#if BLINK
  xTaskCreatePinnedToCore(
    TaskBlink
    ,  "TaskBlink"   // A name just for humans
    ,  4096   // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1      // Priority, 7 being the highest, and 0 being the lowest.
    ,  &xHandleBlink  //任务句柄，删除等操作需要用到
    ,  1);    //运行在 核心1 上
#endif

  //******** Core 0 ********//
#if CLOUD_REPORT    //传感器数据上报
  xTaskCreatePinnedToCore(
    TaskCloudReport
    ,  "TaskCloudReport"
    ,  20480  // Stack size
    ,  NULL
    ,  4  // Priority
    ,  &xHandleCloudReport
    ,  0);
  vTaskSuspend(xHandleCloudReport);   //挂起
  //    vTaskResume(xHandleCloudReport);   //恢复
#endif

#if CLOUD_LOOP
  xTaskCreatePinnedToCore(
    TaskCloudLoop
    ,  "TaskCloudLoop"
    ,  20480  // Stack size
    ,  NULL
    ,  6  // Priority
    ,  &xHandleCloudLoop
    ,  0);
  //  vTaskSuspend(xHandleCloudLoop);   //挂起
  //  //    vTaskResume(xHandleCloudLoop);   //恢复
#endif

#if AC_STATUS     //空调状态上报
  xTaskCreatePinnedToCore(
    TaskACStatus
    ,  "TaskACStatus"
    ,  10240  // Stack size
    ,  NULL
    ,  5  // Priority
    ,  &xHandleACStatus
    ,  1);
  vTaskSuspend(xHandleACStatus);   //挂起
  //    vTaskResume(xHandleACStatus);   //恢复
#endif

  //******** Core 1 ********//
#if REMOTE
  xTaskCreatePinnedToCore(
    TaskRemote
    ,  "TaskRemote"
    ,  10240  // Stack size
    ,  NULL
    ,  7  // Priority
    ,  &xHandleRemote
    ,  1);
  vTaskSuspend(xHandleRemote);   //挂起
  //    vTaskResume(xHandleRemote);   //恢复
#endif

#if SENSOR
  xTaskCreatePinnedToCore(
    TaskSensor
    ,  "TaskSensor"
    ,  10240  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  &xHandleSensor
    ,  1);
  vTaskSuspend(xHandleSensor);   //挂起
  //    vTaskResume(xHandleSensor);   //恢复
#endif


}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*********************** Tasks **********************/
void TaskBlink(void *pvParameters)
{
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, LOW);
  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_B, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_B, LOW);
    vTaskDelay(500);
  }
}

//7
void TaskRemote(void *pvParameters)
{
  (void) pvParameters;
  // * Mode:        A -> 自动, C -> 制冷, D -> 抽湿, H -> 制热, F -> 送风
  char arrayMode[5] = {'A', 'C', 'D', 'H', 'F'};
  // * windSpeed:   A -> 自动, L -> 低风, M -> 中风, H -> 高风, F -> 固定风
  char arraysWind[5] = {'A', 'L', 'M', 'H', 'F'};
  vTaskDelay(2000);  //不要试图调短时间
  irRemoteInit(18);
  for (;;)
  {
    if (millis() - airConditioner.TimeStamp >= 1000)  //1s内，暂不执行
    {
      if (airConditioner.FlagRemote)
      {
        bool flagRemote = 0;
        airConditioner.FlagRemote = 0;   //清除标志位
        digitalWrite(LED_G, LOW);
        if (airConditioner.Power)
        {
          flagRemote |= irRemoteAdjustments(ID, airConditioner.TargetTemperature, \
                                            arrayMode[airConditioner.Mode], arraysWind[airConditioner.WindSpeed]);
          Serial.printf("Remote: T>%d, M>%c, W>%c\r\n", \
                        airConditioner.TargetTemperature, arrayMode[airConditioner.Mode], arraysWind[airConditioner.WindSpeed]);
        }
        else
        {
          flagRemote |= irRemoteOff(ID);
          Serial.println("Remote: OFF");
        }
        airConditioner.FlagUpload = flagRemote;
        delay(100);
        digitalWrite(LED_G, HIGH);
      }
    }
    vTaskDelay(500);
  }
}

//6
void TaskCloudLoop(void *pvParameters)
{
  (void) pvParameters;
  vTaskDelay(50);
  Serial.println("CloudLoopStart");
  wifiInit(WIFI_SSID, WIFI_PASSWD);
  AliyunIoTSDK::begin(espClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, REGION_ID);  // 初始化 iot，需传入 wifi 的 client，和设备产品信息

#if 1
  long timeStamp = millis();
  while (AliyunIoTSDK::mqttConnectSuccess() != 0) //0 成功
  {
    if ((millis() - timeStamp) > 30000) //30s超时自动重启
      ESP.restart();    //软重启
    digitalWrite(LED_G, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_G, LOW);
    vTaskDelay(500);
  }
#endif

#if CLOUD_REPORT
  vTaskResume(xHandleCloudReport);    //恢复 属性上报 任务
#endif
#if REMOTE
  vTaskResume(xHandleRemote);       //恢复 遥控  任务
#endif
#if SENSOR
  vTaskResume(xHandleSensor);       //恢复 传感器  任务
#endif
#if AC_STATUS
  vTaskResume(xHandleACStatus);       //恢复 空调状态  任务
#endif

  AliyunIoTSDK::bindData("LEDSwitch", LEDCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  AliyunIoTSDK::bindData("powerstate", ACPowerCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  AliyunIoTSDK::bindData("targetTemperature", ACTempCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  AliyunIoTSDK::bindData("mode", ACModeCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  AliyunIoTSDK::bindData("windspeed", ACWindCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback

  //新增
  AliyunIoTSDK::bindData("TempUp", TempUpCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  AliyunIoTSDK::bindData("TempDown", TempDownCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  for (;;)
  {
    //    Serial.println("CloudLoop");
    AliyunIoTSDK::loop();   //心跳30s
    vTaskDelay(1000);
  }
}

//5
void TaskCloudReport(void *pvParameters)
{
  (void) pvParameters;
  vTaskDelay(100);
  Sensor sensorUpload;
  Sensor sensorAverage;

  long TimeStamp = 0;
  for (;;)
  {
    TimeStamp =  millis();
    //求均值
    char count = 0;
    for (char i = 0; i < 10; i++)
    {
      if (xQueueReceive( xQueue, &sensorAverage, 100 ) == pdPASS)
      {
        count++;
        sensorUpload.Temperature += sensorAverage.Temperature;
        sensorUpload.Humidity += sensorAverage.Humidity;
        sensorUpload.AmbientLight += sensorAverage.AmbientLight;

        sensorUpload.BatteryState += sensorAverage.BatteryState;
        sensorUpload.BatteryPercentage += sensorAverage.BatteryPercentage;
      }
    }

    //有传感器数据
    if (count)
    {
      sensorUpload.Temperature /= count;
      sensorUpload.Humidity /= count;
      sensorUpload.AmbientLight /= count;
      //      sensorUpload.Temperature = sensorUpload.Temperature > 27 ? 26.4 : sensorUpload.Temperature;   //记得删除

      AliyunIoTSDK::send("IndoorTemperature", sensorUpload.Temperature);
      AliyunIoTSDK::send("IndoorHumidity", sensorUpload.Humidity);
      AliyunIoTSDK::send("IndoorBrightness", sensorUpload.AmbientLight);

      sensorUpload.BatteryState /= count;
      sensorUpload.BatteryPercentage /= count;
      AliyunIoTSDK::send("BatteryState", sensorUpload.BatteryState);
      AliyunIoTSDK::send("BatteryPercentage", sensorUpload.BatteryPercentage);

      Serial.printf("Sensor: T>%.1f, H>%.1f, B>%d\r\n", \
                    sensorUpload.Temperature, sensorUpload.Humidity, sensorUpload.AmbientLight);
    }
    vTaskDelay(10000 - (millis() - TimeStamp));
  }
}

//4 上传的数据并不会改变web端???
void TaskACStatus(void *pvParameters)
{
  (void) pvParameters;
  char uploadCount = 5;
  for (;;)
  {
    //    if (airConditioner.FlagUpload)
    //    {
    //      airConditioner.FlagUpload = 0;
    //      AliyunIoTSDK::send("powerstate", airConditioner.Power);
    //      AliyunIoTSDK::send("targetTemperature", airConditioner.TargetTemperature);
    //      AliyunIoTSDK::send("mode", airConditioner.Mode);
    //      AliyunIoTSDK::send("windspeed", airConditioner.WindSpeed);
    //
    //      Serial.printf("ACStatus: P>%x, T>%d, M>%d, W>%d\r\n", \
    //                    airConditioner.Power, airConditioner.TargetTemperature, airConditioner.Mode, airConditioner.WindSpeed);

    AliyunIoTSDK::send("TempDispaly", airConditioner.TargetTemperature);
    //    }

    //如果有数据，短时间内1Hz上传5次。
    //若新数据，0.1Hz上传
    if (airConditioner.FlagUpload)
    {
      airConditioner.FlagUpload = 0;
      uploadCount = 5;
    }

    if (uploadCount)
    {
      uploadCount--;
      vTaskDelay(1000);
    }
    else
      vTaskDelay(10000);
  }
}

//3
void TaskSensor(void *pvParameters)
{
  (void) pvParameters;
  vTaskDelay(3000);
  pinMode(AMBIENT_LIGHT, INPUT);
  pinMode(4, INPUT);              //多开一路ADC2，ADC1才能采出来数据。

#define BATTERY_STDBY  27
  pinMode(BATTERY_STDBY, INPUT);     //电池充满指示，充电指示引脚被断开

  Sensor sensorData;
  long TimeStamp = 0;         //时间戳

  //***** SHT30
  while (sht3x.begin() != 0)
    vTaskDelay(500);
  Serial.print("Chip serial number: ");
  Serial.println(sht3x.readSerialNumber());
  if (!sht3x.softReset())
    Serial.println("Failed to Initialize the chip....");

  //***** Queue
  xQueue = xQueueCreate(10, sizeof(Sensor));
  if (xQueue != NULL )
    Serial.println("Success: Creat Queue.");
  else
    Serial.println("Error: Creat Queue.");

  for (;;)
  {
    TimeStamp = millis();
    sensorData.AmbientLight = analogRead(AMBIENT_LIGHT);
    sensorData.Temperature = sht3x.getTemperatureC();
    sensorData.Humidity = sht3x.getHumidityRH();

    sensorData.BatteryState = (digitalRead(BATTERY_STDBY) == HIGH ? 2 : 0);   //充满为低，未满为高。
    sensorData.BatteryPercentage = (sensorData.BatteryState == 2 ? 100 : 95); //满电为100，否则为95

    //    Serial.printf("Sensor: %d,\t %.2f,\t%.2f\r\n", \
    sensorData.AmbientLight, sensorData.Temperature, sensorData.Humidity);
    xQueueSend( xQueue, &sensorData, 500);
    vTaskDelay(1000 - (millis() - TimeStamp));    //补足1s
  }
}



/*********************** WIFI **********************/
// 初始化 wifi 连接
void wifiInit(const char *ssid, const char *passphrase)
{
  long timeStamp = millis();
  digitalWrite(LED_R, LOW);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passphrase);
  //  WiFi.setAutoReconnect(true);
  vTaskDelay(3000);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not Connect");
    if ((millis() - timeStamp) > 20000) //10s超时自动重启
      ESP.restart();    //软重启
    digitalWrite(LED_R, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_R, LOW);
    vTaskDelay(500);
  }
  Serial.println("Connected to AP");
  digitalWrite(LED_R, HIGH);
}

//控制类
// LED修改的回调函数
void LEDCallback(JsonVariant p)
{
  bool ledSwitch = p["LEDSwitch"];
  if (ledSwitch == 1)
    digitalWrite(LED_R, LOW);
  else
    digitalWrite(LED_R, HIGH);
}

// 空调控制 回调函数
void ACPowerCallback(JsonVariant p)
{
  bool powerstate = p["powerstate"];
  Serial.printf("ACPower: %x\r\n", powerstate);
  airConditioner.Power = powerstate;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
  //  AliyunIoTSDK::send("powerstate", airConditioner.Power);
}

void ACTempCallback(JsonVariant p)
{
  int targetTemperature = p["targetTemperature"];
  Serial.printf("ACTemp: %d\r\n", targetTemperature);
  airConditioner.TargetTemperature = targetTemperature;
  //  airConditioner.Power = 1;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
  //  AliyunIoTSDK::send("targetTemperature", airConditioner.TargetTemperature);
}

void ACModeCallback(JsonVariant p)
{
  int acMode = p["mode"];
  acMode = (acMode >= 4 ? 4 : acMode);
  acMode = (acMode <= 0 ? 0 : acMode);
  Serial.printf("ACMode: %d\r\n", acMode);
  airConditioner.Mode = acMode;
  //  airConditioner.Power = 1;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
  //  AliyunIoTSDK::send("mode", airConditioner.Mode);
}

void ACWindCallback(JsonVariant p)
{
  int windspeed = p["windspeed"];
  windspeed = (windspeed >= 4 ? 4 : windspeed);
  windspeed = (windspeed <= 0 ? 0 : windspeed);
  Serial.printf("ACWind: %d\r\n", windspeed);
  airConditioner.WindSpeed = windspeed;
  //  airConditioner.Power = 1;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
  //  AliyunIoTSDK::send("windspeed", airConditioner.WindSpeed);
}

//新增
void TempUpCallback(JsonVariant p)
{
  int tempUp = p["TempUp"];
  airConditioner.TargetTemperature += tempUp;
  airConditioner.TargetTemperature = airConditioner.TargetTemperature >= 30 ? 30 : airConditioner.TargetTemperature;
  //  airConditioner.Power = 1;
  airConditioner.FlagRemote = 1;
  Serial.printf("TempUp: %d\tTemp: %d\r\n", tempUp, airConditioner.TargetTemperature);
  AliyunIoTSDK::send("TempDispaly", airConditioner.TargetTemperature);
}

void TempDownCallback(JsonVariant p)
{
  int tempDown = p["TempDown"];
  airConditioner.TargetTemperature -= tempDown;
  airConditioner.TargetTemperature = airConditioner.TargetTemperature <= 17 ? 17 : airConditioner.TargetTemperature;
  //  airConditioner.Power = 1;
  airConditioner.FlagRemote = 1;
  Serial.printf("TempDown: %d\tTemp: %d\r\n", tempDown, airConditioner.TargetTemperature);
  AliyunIoTSDK::send("TempDispaly", airConditioner.TargetTemperature);
}
