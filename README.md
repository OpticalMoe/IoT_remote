**中央空调管理**

> 该项目目前仅完成功能验证，由于暂不知晓所使用的库使用的开源协议，故暂不开源；
>
> 待全部完成后将使用 **GPL** 协议开源，且不可用于商业。

> **Copyright (c) 2021 ChenYuanliang.**

---

该项目分为物联网云平台和IoT设备端；

1. 物联网云平台：阿里云物联网云平台，可通过网页向IoT设备端发送指令，接收IoT设备上传的状态、温湿度、电量等信息；
2. IoT设备端：通过ESP32连接校园WIFI，可接收云平台下发的指令，上传房间空调状态、温湿度、设备电量等信息；

------

# **0. 目录**

[TOC]

# **1. 硬件**

## **1.1 光电楼中央空调控制流程**
光电楼现有的中央空调控制流程如下：
> 全程采用**红外**单向通信，左侧 **<u>智能遥控</u>** 是本项目设备，右侧三个设备为光电楼现有设备。

![光电楼现有中央空调控制流程](Image/%E5%B9%BB%E7%81%AF%E7%89%871.jpg)
这是光电楼现有的线控器，型号**KJR-29A**，按键型，无背光。

<img src="Image/1635173161792.jpg" alt="光电楼现有空调线控器" style="zoom: 25%;" />

下图是线控器的反面:

> 自上向下线序：C(5V), D(GND), A(IR+), B(IR-)。

<img src="Image/1635173161759.jpg" alt="线控器反面" style="zoom: 25%;" />

下图是接收器，型号：**KJR-02B**

> 左边自上向下线序：黄(5V)，黑(GND)，棕(IR+)，红(IR-)，白(RUN)；（其中，RUN表示空调运行指示，来自室内机。）

<img src="Image/1635173162093.jpg" alt="1635173162093" style="zoom:67%;" />

<img src="Image/1635173161977.jpg" alt="接收器" style="zoom:33%;" />

右边自上向下线序如下图：

> 线序：E->GND; SW->手动开关; BUZ->蜂鸣器; AL->报警指示灯; M.F.->制热指示灯; TIME->定时指示灯; RUN->运行指示灯; REV->直连红外接收头；

<img src="Image/1635173161943.jpg" alt="接收器右边线序" style="zoom: 50%;" />

下图是室内机，型号：**MDV-D90**

![室内机](Image/image-20211107093418380.png)

<img src="Image/%E7%A9%BA%E8%B0%83%E9%93%AD%E7%89%8C.jpg" alt="空调铭牌" style="zoom:33%;" />

下图是室外机：

![室外机](Image/image-20211107093453817.png)

## **1.2 硬件方案**

<img src="Image/1635173161777.jpg" alt="1635173161777" style="zoom:50%;" />

硬件方案采用：

1. ESP32模组做主控；
2. 四颗侧发光红外LED和一颗正发光红外LED做红外遥控发射；
3. GXHT30温湿度传感器采集环境数据；
4. ALS-PT19-315C/L177/TR8环境光传感器采集环境光数据；
5. TC4056A + DW03D + CW2015构成**锂电池充电管理 + 过充过放保护 + 电量统计**功能；
5. 板载TYPE-C接口，串口，自动下载电路。

## **1.3 原理图**

**硬件资料： ../1.Hardware**

使用 **立创EDA** 软件绘制，工程源码： **../1.Hardware\工程源码**

### 1.3.1 主控

ESP32-WROOM模组，双核MCU，板载WIFI、蓝牙。可替换为ESP32其他版本。

> 立创商城编号：C503587；数据手册： **../1.Hardware\datasheet\esp32_datasheet_cn.pdf & esp32-wroom-32_datasheet_cn.pdf**
>
> 开发板原理图： **../1.Hardware\datasheet\SchematicsforESP32.pdf**

> 部分引脚因功能冲突或设计失误，需要在V1.1版修正。
>
> 热释电功能取消；串口TX/RX画反；AmbientLight调至IO34；

![image-20211108194420726](Image/image-20211108194420726.png)

### 1.3.2 USB

USB部分包括TYPE-C、串口和自动下载电路。

R8、R9电阻防止TYPE-C接口触发快充。

> **串口** 立创商城编号：C84681；数据手册： **../1.Hardware\datasheet\CH340C.PDF**
>
> **TYPE-C** 立创商城编号：C165948；数据手册：  **../1.Hardware\datasheet\TYPE-C-31-M-12母座贴片.PDF**

> DP&DN画反，正确接法DP->D+, DN->D-；
>
> 串口电源改为3.3V，否则电流倒灌会导致ESP32的WIFI无法正常工作。

![image-20211108195307956](Image/image-20211108195307956.png)

### 1.3.3 锂电池充电

TC4056A，可更换为其他锂电池充电芯片。

锂电池400mah，安全充电电流0.5C=200mA，建议充电电流300mA，一部分给降压。

> **TC4056A** 立创商城编号：C84051；数据手册：  **../1.Hardware\datasheet\TC4056A.PDF**

> 删除R14

![image-20211108195706174](Image/image-20211108195706174.png)

### 1.3.4 降压

ME6217C33M5G，降压。只要输出3.3V，电流大于300mA即可。记得加0.1uF，100uF或更大的电容。

> **ME6217C33M5G** 立创商城编号：C427602；数据手册：  **../1.Hardware\datasheet\ME6217C33M5G.PDF**

![image-20211108194537862](Image/image-20211108194537862.png)

### 1.3.5 锂电池保护

DW03D，锂电池过充过放保护芯片。

> **DW03D** 立创商城编号：C82200；数据手册：  **../1.Hardware\datasheet\DW03D.PDF**

> 改变开关位置至降压前；
>
> GND和BAT-间添加0R电阻，当电池自带保护电路时，可选择该0R电阻跳过板载锂电池保护电路。

![image-20211108200218900](Image/image-20211108200218900.png)



### 1.3.6 红外

红外部分使用四颗侧发光红外LED和一颗正发光红外LED组成。侧发光LED互90度安装，保证360度覆盖。正发光LED保证正面120度覆盖。

> **红外发射** 立创商城编号：C273626；数据手册：  **../1.Hardware\datasheet\1206灯珠侧发红光.PDF**
>
> **红外接收** 立创商城编号：C390037；数据手册：  **../1.Hardware\datasheet\插件短脚接收头38KHZ遥控接收头.PDF**

> LED限流电阻27R 0603，5颗LED均需单独串接限流电阻，不可共用。

![image-20211108200543094](Image/image-20211108200543094.png)

### 1.3.7 温湿度传感器

目前选用GXHT30，可测温湿度，IIC通信，精度较高。可更换为DS18B20或其他廉价温度传感器，改一下程序即可。

>  **GXHT30** 立创商城编号：C2758005；数据手册：  **../1.Hardware\datasheet\温湿度传感器.PDF**

> AIFRT_GXHT30和GXHT30_RST引脚可删除；0R电阻删除；PCB板四周开槽；

![image-20211108194611140](Image/image-20211108194611140.png)

### 1.3.8 环境光传感器

ALS-PT19-315C/L177/TR8。搭配电阻10K。输出电压信号，使用ADC采集即可。

>  **ALS-PT19-315C/L177/TR8** 立创商城编号：C146233；数据手册：  **../1.Hardware\datasheet\ALS-PT19-315C%2FL177%2FTR8.PDF**

> （注：ESP32的WIFI和ADC2不能同时工作，但是和ADC1能同时工作，ADC1引脚包括：SENSOR_VP/SENSOR_VN/GPIO_34/GPIO_35）

![image-20211108194709502](Image/image-20211108194709502.png)

###  1.3.9 外设

按键为侧按键，按键行程0.4mm。

>  **按键** 立创商城编号：C393942；数据手册：  **../1.Hardware\datasheet\2_4_3.5半包小贝贝TS-018小侧按键M165蓝牙耳机轻触开关.PDF**

![image-20211108201028694](Image/image-20211108201028694.png)

# **2.程序**

程序采用Arduino（https://www.arduino.cc/en/software）平台，需要安装：

* ESP32支持包： **../2.Software\SoftwarePackage\SoftwarePackage\32_package_1.0.6_arduino.cn.exe**

* CH340驱动：**../2.Software\SoftwarePackage\SoftwarePackage\CH341SER.EXE**

包含库：**../2.Software\库**

 * DFRobot_SHT3x-master   https://github.com/DFRobot/DFRobot_SHT3x

 * AliyunIoTSDK   库需要魔改，https://github.com/yu-tou/arduino-aliyun-iot-sdk

 * ArduinoJson

 * Crypto

以下库无需安装：

 * WiFi                   

 * Wire

 * PubSubClient

> 可直接将 **DFRobot_SHT3x-master、AliyunIoTSDK、ArduinoJson、Crypto** 库拷贝到Arduino库文件夹中。
>
> 一般的路径： **C:\Users\电脑用户名\Documents\Arduino\libraries**

## 2.1 FreeRTOS 

ESP32在Arduino下自带FreeRTOS，只需使用以下函数即可创建新任务。
```C++
TaskHandle_t xHandleBlink = NULL;
xTaskCreatePinnedToCore(				//写在Setup函数中
    TaskBlink
    ,  "TaskBlink"   //任务名，便于记忆的
    ,  4096   // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1      // 优先级，7最高，0最低；
    ,  &xHandleBlink  //任务句柄，挂起、恢复、删除等操作需要用到
    ,  1);    //运行在 核心1 上
```

使用以下函数即可挂起、恢复任务。
```C++
vTaskSuspend(xHandleCloudReport);  //挂起
vTaskResume(xHandleCloudReport);   //恢复
```

使用以下代码编写任务执行部分代码：
```C++
void TaskBlink(void *pvParameters)
{
  (void) pvParameters;
  for(,,)
  {
      vTaskDelay(1000);
  }
}
```

>  RTOS尽可能避免使用全局变量传递信息，使用队列、邮箱等替代。（我没严格遵守）

## 2.2 红外遥控

红外遥控部分需要添加irRemote.c&.h文件。

**红外编码见 ../2.Software\R05d电控功能说明书.pdf  >  六、编码规范 > 1&9部分** 

> irRemote.c&.h文件主要负责红外部分编码和发送；
>
> 发送部分采用硬件LEDC产生38KHz方波，硬件TIM控制红外发送引脚搭接到方波时长。
>
> irRemote.c&.h文件见 **../2.Software\SmartRemote_Vx.x\SmartRemote\irRemote.cpp & irRemote.h**。

```C++
#include "irRemote.h"
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
TaskHandle_t xHandleRemote = NULL;		//全局
xTaskCreatePinnedToCore(				//写在Setup函数中
    TaskRemote
    ,  "TaskRemote"
    ,  10240  // Stack size
    ,  NULL
    ,  7  // Priority
    ,  &xHandleRemote
    ,  1);
vTaskSuspend(xHandleRemote);   //挂起，写在Setup函数中
  
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
          flagRemote |= irRemoteAdjustments(ID, airConditioner.TargetTemperature, arrayMode[airConditioner.Mode], arraysWind[airConditioner.WindSpeed]);
          Serial.printf("Remote: T>%d, M>%c, W>%c\r\n", airConditioner.TargetTemperature, arrayMode[airConditioner.Mode], arraysWind[airConditioner.WindSpeed]);
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
```

## 2.3 上云

上云部分使用github开源库AliyunIoTSDK，可快速上云。
https://github.com/yu-tou/arduino-aliyun-iot-sdk

> AliyunIoTSDK库需要魔改，具体魔改见github或自行搜索该库。
> 也可将 **../2.Software\库** 的 **AliyunIoTSDK、ArduinoJson、Crypto** 文件夹拷贝进Arduino库路径。
>
> **一般路径为：  C:\Users\电脑用户名\Documents\Arduino\libraries**。

> 以下代码块部分内容已打码

```C++
//******** IoT基本信息 ********//
static WiFiClient espClient;
#include <AliyunIoTSDK.h>   // 引入阿里云 IoT SDK
// 设置产品和设备的信息，从阿里云设备信息里查看
#define PRODUCT_KEY "a11xxxxxxxx"		//产品密钥 ProductKey（已打码）
#define DEVICE_NAME "GD507"				//设备名称，可在云端自定义。DeviceName
#define DEVICE_SECRET "21a4xxxxxxxxxxxxxxxxxxxxxxx"	//DeviceSecret （已打码）
#define REGION_ID "cn-shanghai"
// 设置 wifi 信息
#define WIFI_SSID "IoT"			//wifi名称
#define WIFI_PASSWD "xxx"		//wifi密码（已打码）

TaskHandle_t xHandleCloudLoop = NULL;
xTaskCreatePinnedToCore(				//写在Setup函数中
    TaskCloudLoop
    ,  "TaskCloudLoop"
    ,  20480  // Stack size
    ,  NULL
    ,  6  // Priority
    ,  &xHandleCloudLoop
    ,  0);

void TaskCloudLoop(void *pvParameters)
{
  (void) pvParameters;
  vTaskDelay(50);
  Serial.println("CloudLoopStart");
  wifiInit(WIFI_SSID, WIFI_PASSWD);
  AliyunIoTSDK::begin(espClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, REGION_ID);  // 初始化 iot，需传入 wifi 的 client，和设备产品信息
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
  vTaskResume(xHandleCloudReport);    //恢复 属性上报 任务
  vTaskResume(xHandleRemote);       //恢复 遥控  任务
  vTaskResume(xHandleSensor);       //恢复 传感器  任务
  vTaskResume(xHandleACStatus);       //恢复 空调状态  任务
  AliyunIoTSDK::bindData("LEDSwitch", LEDCallback);  // 绑定一个设备属性回调，当远程修改此属性，会触发 xxxCallback
  AliyunIoTSDK::bindData("powerstate", ACPowerCallback);  
  AliyunIoTSDK::bindData("targetTemperature", ACTempCallback);  
  AliyunIoTSDK::bindData("mode", ACModeCallback); 
  AliyunIoTSDK::bindData("windspeed", ACWindCallback); 
  for (;;)
  {
    AliyunIoTSDK::loop();   //心跳30s
    vTaskDelay(1000);
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

// 空调控制 回调函数
void ACPowerCallback(JsonVariant p)
{
  bool powerstate = p["powerstate"];
  Serial.printf("ACPower: %x\r\n", powerstate);
  airConditioner.Power = powerstate;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
}

void ACTempCallback(JsonVariant p)
{
  int targetTemperature = p["targetTemperature"];
  Serial.printf("ACTemp: %d\r\n", targetTemperature);
  airConditioner.TargetTemperature = targetTemperature;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
}

void ACModeCallback(JsonVariant p)
{
  int acMode = p["mode"];
  acMode = (acMode >= 4 ? 4 : acMode);
  acMode = (acMode <= 0 ? 0 : acMode);
  Serial.printf("ACMode: %d\r\n", acMode);
  airConditioner.Mode = acMode;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
}

void ACWindCallback(JsonVariant p)
{
  int windspeed = p["windspeed"];
  windspeed = (windspeed >= 3 ? 3 : windspeed);
  windspeed = (windspeed <= 0 ? 0 : windspeed);
  Serial.printf("ACWind: %d\r\n", windspeed);
  airConditioner.WindSpeed = windspeed;
  airConditioner.TimeStamp = millis();
  airConditioner.FlagRemote = 1;
}
```

## 2.4 环境数据上报

从队列读数据，求均值，上传。

```C++
QueueHandle_t xQueue;
struct Sensor
{
  float Temperature = 0.0;
  float Humidity = 0.0;
  int AmbientLight = 0;
  char BatteryState = 0;
  char BatteryPercentage = 0;
};

TaskHandle_t xHandleCloudReport = NULL;
xTaskCreatePinnedToCore(				//写在Setup函数中
    TaskCloudReport
    ,  "TaskCloudReport"
    ,  20480  // Stack size
    ,  NULL
    ,  4  // Priority
    ,  &xHandleCloudReport
    ,  0);
vTaskSuspend(xHandleCloudReport);   //挂起，写在Setup函数中

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
      }
    }
    //有传感器数据
    if (count)
    {
      sensorUpload.Temperature /= count;
      sensorUpload.Humidity /= count;
      sensorUpload.AmbientLight /= count;
      AliyunIoTSDK::send("IndoorTemperature", sensorUpload.Temperature);
      AliyunIoTSDK::send("IndoorHumidity", sensorUpload.Humidity);
      AliyunIoTSDK::send("IndoorBrightness", sensorUpload.AmbientLight);
      Serial.printf("Sensor: T>%.1f, H>%.1f, B>%d\r\n", \
                    sensorUpload.Temperature, sensorUpload.Humidity, sensorUpload.AmbientLight);
    }
    vTaskDelay(10000 - (millis() - TimeStamp));
  }
}
```

## 2.5 空调状态上传

> 注意：这里只上传上次红外发送的状态值或初始化后的状态值，并不是空调的实际值；因为全程是红外单向通信，并不能获取到空调实际状态。

> 云端好像并没有显示这个任务的任何值，可删除。
```C++
TaskHandle_t xHandleACStatus = NULL;
xTaskCreatePinnedToCore(				//写在Setup函数中
    TaskACStatus
    ,  "TaskACStatus"
    ,  10240  // Stack size
    ,  NULL
    ,  5  // Priority
    ,  &xHandleACStatus
    ,  1);
vTaskSuspend(xHandleACStatus);   //挂起，写在Setup函数中

void TaskACStatus(void *pvParameters)
{
  (void) pvParameters;
  char uploadCount = 5;
  for (;;)
  {
    AliyunIoTSDK::send("TempDispaly", airConditioner.TargetTemperature);
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
```

## 2.6 环境数据采集

环境数据采集主要负责温湿度和环境光数据采集，采集到的数据压入队列，每秒采集一次。

> 注意：ADC2和wifi不能同时工作，但是ADC1可以，再但是你需要同时开ADC1&2才能在ADC1采出来数据。

```C++
//******** GPIO ********//
#define LED_R 33
#define LED_G 25
#define LED_B 26
#define AMBIENT_LIGHT  34
//******** SHT30 ********//
#include <DFRobot_SHT3x.h>
DFRobot_SHT3x   sht3x;
TaskHandle_t xHandleSensor = NULL;
xTaskCreatePinnedToCore(				//写在Setup函数中
    TaskSensor
    ,  "TaskSensor"
    ,  10240  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  &xHandleSensor
    ,  1);
vTaskSuspend(xHandleSensor);   //挂起，写在Setup函数中

void TaskSensor(void *pvParameters)
{
  (void) pvParameters;
  vTaskDelay(3000);
  pinMode(AMBIENT_LIGHT, INPUT);
  pinMode(4, INPUT);              //多开一路ADC2，ADC1才能采出来数据。
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
    sensorData.AmbientLight, sensorData.Temperature, sensorData.Humidity);
    xQueueSend( xQueue, &sensorData, 500);
    vTaskDelay(1000 - (millis() - TimeStamp));    //补足1s
  }
}
```

## +1 接入 eduroam wifi网络

通常，物联网设备只能接入使用WPA2-PSK的wifi（只需要SSID和password的），而一般的校园采用的是WPA3-Enterprise方式的wifi，需要SSID、用户名、密码才可接入。以下程序将使IoT设备可接入校园wifi。

**../2.Software\ESP32-eduroam\experimental_example\experimental_example.ino**

该程序为github开源库例程。可直接搜索 **esp eduroam** 获得。

https://github.com/martinius96/ESP32-Eduroam

> 部分内容已打码

> **该部分内容未加入项目中，如有需要，可自行移植。 加油！**

```C++
#include <WiFi.h> //Wifi library
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks

#define EAP_ANONYMOUS_IDENTITY "anonymous@tuke.sk" //anonymous@example.com, or you can use also nickname@example.com
#define EAP_IDENTITY "xxxxx@xxx.edu.cn" //学号@学校域名。域名可在学校官网获得 (已打码)
#define EAP_PASSWORD "xxx" //信息门户密码 (已打码)
//SSID NAME
const char* ssid = "eduroam"; // eduroam SSID

//Root CA cert (DigiCert Assured ID Root CA) in .pem format from:
//https://uvt.tuke.sk/wps/portal/uv/sluzby/bezdrotove-siete-wifi-na-tuke/prirucka-pouzivatela-bezdrotovej-siete-eduroam
const static char* test_root_ca PROGMEM = \
    "-----BEGIN CERTIFICATE-----\n" \
    "MIIDtzCCAp+gAwIBAgIQDOfg5RfYRv6P5WD8G/AwOTANBgkqhkiG9w0BAQUFADBl\n" \
    "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
    "d3cuZGlnaWNlcnQuY29tMSQwIgYDVQQDExtEaWdpQ2VydCBBc3N1cmVkIElEIFJv\n" \
    "b3QgQ0EwHhcNMDYxMTEwMDAwMDAwWhcNMzExMTEwMDAwMDAwWjBlMQswCQYDVQQG\n" \
    "EwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNl\n" \
    "cnQuY29tMSQwIgYDVQQDExtEaWdpQ2VydCBBc3N1cmVkIElEIFJvb3QgQ0EwggEi\n" \
    "MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCtDhXO5EOAXLGH87dg+XESpa7c\n" \
    "JpSIqvTO9SA5KFhgDPiA2qkVlTJhPLWxKISKityfCgyDF3qPkKyK53lTXDGEKvYP\n" \
    "mDI2dsze3Tyoou9q+yHyUmHfnyDXH+Kx2f4YZNISW1/5WBg1vEfNoTb5a3/UsDg+\n" \
    "wRvDjDPZ2C8Y/igPs6eD1sNuRMBhNZYW/lmci3Zt1/GiSw0r/wty2p5g0I6QNcZ4\n" \
    "VYcgoc/lbQrISXwxmDNsIumH0DJaoroTghHtORedmTpyoeb6pNnVFzF1roV9Iq4/\n" \
    "AUaG9ih5yLHa5FcXxH4cDrC0kqZWs72yl+2qp/C3xag/lRbQ/6GW6whfGHdPAgMB\n" \
    "AAGjYzBhMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW\n" \
    "BBRF66Kv9JLLgjEtUYunpyGd823IDzAfBgNVHSMEGDAWgBRF66Kv9JLLgjEtUYun\n" \
    "pyGd823IDzANBgkqhkiG9w0BAQUFAAOCAQEAog683+Lt8ONyc3pklL/3cmbYMuRC\n" \
    "dWKuh+vy1dneVrOfzM4UKLkNl2BcEkxY5NM9g0lFWJc1aRqoR+pWxnmrEthngYTf\n" \
    "fwk8lOa4JiwgvT2zKIn3X/8i4peEH+ll74fg38FnSbNd67IJKusm7Xi+fT8r87cm\n" \
    "NW1fiQG2SVufAQWbqz0lwcy2f8Lxb4bG+mRo64EtlOtCt/qMHt1i8b5QZ7dsvfPx\n" \
    "H2sMNgcWfzd8qVttevESRmCD1ycEvkvOl77DZypoEd+A5wwzZr8TDRRu838fYxAe\n" \
    "+o0bJW1sj6W3YQGx0qMmoRBxna3iw/nDmVG3KwcIzi7mULKn+gpFL6Lw8g==\n" \
    "-----END CERTIFICATE-----\n";

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.print(F("Connecting to network: "));
  Serial.println(ssid);
  WiFi.disconnect(true);  //disconnect form wifi to set new wifi connection
  WiFi.mode(WIFI_STA); //init wifi mode
  esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t *)test_root_ca, strlen(test_root_ca) + 1);
  //esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t *)test_root_ca, strlen(test_root_ca));
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ANONYMOUS_IDENTITY, strlen(EAP_ANONYMOUS_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
  esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function
  WiFi.begin(ssid); //connect to wifi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println("");
  Serial.println(F("WiFi is connected!"));
  Serial.println(F("IP address set: "));
  Serial.println(WiFi.localIP()); //print LAN IP
}

void loop() {
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}
```



# **3.阿里云-飞燕-生活物联网平台**

https://living.aliyun.com/

按照文档中心教程走一遍即可。

## 3.1 创建新项目

点击右上角“**创建新项目**”，输入名称，类型选择 **自有品牌项目**。

## 3.2创建新产品

点击“**创建新产品**”， 输入产品名称，所属品类选择 **环境电器/空调”**，其他默认。

## 3.3 功能定义

按照下表添加功能定义。以下“**功能定义**”均为 **属性**。

| 名称           | 标识符            | 数据类型 | 数据定义                     |
| -------------- | ----------------- | -------- | ---------------------------- |
| 开关           | powerstate        | bool     | 0-关闭/1-打开                |
| 目标温度       | targetTemperature | double   | 17~30                        |
| 模式           | mode              | enum     | 0~4:自动/制冷/除湿/制热/送风 |
| 风速           | windspeed         | enum     | 0~3:自动/低风/中风/高风      |
| 室内湿度       | IndoorHumidity    | float    | 0~100                        |
| 室内亮度       | IndoorBrightness  | int32    | 0~4095                       |
| 室内温度       | IndoorTemperature | float    | -15~50                       |
| 电池状态       | BatteryState      | enum     | 0~2:放电/充电/充满           |
| 电池电量百分比 | BatteryPercentage | int32    | 0~100                        |
| LED开关        | LEDSwitch         | bool     | 0-关闭/1-打开                |

点击“下一步”。

## 3.4 人机交互

主要是手机APP端显示的内容，需要配网才能显示。我们不使用APP端显示，可以随便操作，不重要。

> APP端只能显示固定房间设备信息，不能在一个页面选择不同房间设备，且需要扫码配网才能添加设备，十分繁琐，故不使用。

点击“下一步”。

## 3.5 设备调试

划拉到最下面，点击”**新增测试设备**“，填入设备名称（只支持英文）。保存好 **ProductKey、DeviceName、 DeviceSecret** 。这三个值至关重要，需要填入程序中，不要泄露哦。

## 3.6 在线调试

点击最后一个标签”在线调试“，可在左侧选择”调试真实设备“、”调试虚拟设备“。右侧”实时日志“可看到经过IoT的数据。

* **调试真实设备**：可向设备下发数据，也可向设备请求数据（设备端没写请求数据对应部分）；
* **调试虚拟设备**：模拟设备向IoT后台发送数据。

# **4.web应用开发**

网址：https://studio.iot.aliyun.com/

web应用开发部分坑比较多，或者是我不会用，这部分会写的详细一点。

> 室内温湿度数据来源于远古时期调试数据，不是当时室内实际温湿度。

![image-20211109091107086](Image/image-20211109091107086.png)



## 4.1 新建

划拉到下面，找到**web应用**标签卡，点击下方**新建**，输入 应用名称，选择所属项目，根据需要填写描述即可。

<img src="Image/image-20211109091333924.png" alt="image-20211109091333924" style="zoom: 67%;" />

## 4.2 编写页面UI

### 4.2.1 导航布局

点击左侧 **导航布局** 标签，选择合适的样式（本项目使用第二种）。点击下面的**配置**，可在右侧编辑显示的logo、颜色、字体。

<img src="Image/image-20211109110912042.png" alt="image-20211109110912042" style="zoom: 70%;" /><img src="Image/image-20211109110926767.png" alt="image-20211109110926767" style="zoom:50%;" />



### 4.2.2  页面

点击左侧**页面**，点击**新建**，选择**空白**或其他合适的页面模板；点击右侧**首页**可将该页面作为首页；

<img src="Image/image-20211109111333412.png" alt="image-20211109111333412" style="zoom:65%;" />

### 4.2.3 组件树

点击最左侧第二个方块图标后，可拖拽组件到页面。

| 页面显示           | 所属     | 组件       |
| ------------------ | -------- | ---------- |
| 页面中所有文字部分 | 基础     | 文字       |
| 房间号             | 表单     | 下拉框     |
| 温度滑条           | 控制     | 滑条       |
| 模式/风速          | 控制     | 按钮标签组 |
| 室内温湿度数字     | 基础     | 卡片       |
| 更新&群控          | 控制     | 按钮       |
| 群控 二次确认      | 弹窗容器 | 弹窗       |
| 空调开关           | 控制     | 开关       |

<img src="Image/image-20211109112111233.png" alt="image-20211109112111233" style="zoom:67%;" />

## 4.3 功能

有些功能按正常思维无法实现，所以这里另辟蹊径，会显得复杂点，但总算可以实现。

编辑过程中请随时随手点击 **保存** 按钮，在页面最上面的右侧，<img src="Image/image-20211109165147098.png" alt="image-20211109165147098" style="zoom: 50%;" />

编辑过程中也可随时点击 **预览** ，预览最终效果。<img src="Image/image-20211109165300898.png" alt="image-20211109165300898" style="zoom:50%;" />

### 4.3.1. 房间号 下拉框

* 右侧 **样式** 选项卡

​		组件名称：房间号（或其他自定义名称，便于后面搜索）

​		选择产品：空调遥控（来自IoT部分定义的产品名称）

​		默认值：自定义（可自定义，也可默认选择列表第一项）

> 第一次点击**选择产品**时，弹出的窗口内容为空，需要主动关联IoT设备。
>
> 点击左下角**产品管理**，在弹出的先页面中点击**关联物联网平台产品**，勾选需要关联的产品即可。返回原浏览器页面继续操作。

<img src="Image/image-20211109155606484.png" alt="image-20211109155606484" style="zoom:50%;" />

### 4.3.2. 空调开关

* 右侧 **样式** 选项卡

​		组件名称：电源开关（或其他自定义名称，便于后面搜索）

​		开关数据：留空，不要配置

​		其他根据需要配置。

* 右侧 **互动** 选项卡

​		点击 **新增交互** ，事件 -> 点击；动作 -> 赋值给变量；点击**配置 > 管理变量 > 新增变量** 

| 变量名     | 描述     | 默认值 |
| ---------- | -------- | ------ |
| TargetTemp | 目标温度 | 26     |
| Mode       | 模式     | 0      |
| WindSpeed  | 风速     | 0      |
| Power      | 电源     | 0      |

点击 **赋值**，选择 **value**，选择 **Power**;

<img src="Image/image-20211109160636242.png" alt="image-20211109160636242" style="zoom: 80%;" />

### 4.3.3. 温度滑块

* 右侧 **样式** 选项卡

​		数值范围：17~30，步长1

* 右侧 **交互** 选项卡

  事件：值改变

  动作：赋值给变量

  变量：TargetTemp / value / 当前值

  <img src="Image/image-20211109161626062.png" alt="image-20211109161626062" style="zoom: 67%;" /><img src="Image/image-20211109161643080.png" alt="image-20211109161643080" style="zoom: 80%;" />

### 4.3.4. 模式 按钮标签组

  **样式** 默认选项卡设置：依次填入：**自动、制冷、除湿、制热、送风** 注意顺序；字体、颜色等根据需要改变。

  <img src="Image/image-20211109161855791.png" alt="image-20211109161855791" style="zoom:67%;" />

  **交互** 共5个交互，大体相同。

* 事件：切换选项卡

* 切换选项卡：**自动** （依次选择 **自动、制冷、除湿、制热、送风**） 

* 动作：赋值给变量；变量名 Mode；**值 0**（依次填入 0~4）

  <img src="Image/image-20211109162832476.png" alt="image-20211109162832476" style="zoom:67%;" />

### 4.3.5. 风速 按钮标签组

**样式** 默认选项卡设置：依次填入：**自动、低风、中风、高风** 注意顺序；字体、颜色等根据需要改变。

**交互** 共4个交互，大体相同。

* 事件：切换选项卡

* 切换选项卡：**自动** （依次选择 **自动、低风、中风、高风**） 

* 动作：赋值给变量；变量名 WindSpeed；**值 0**（依次填入 0~3）

<img src="Image/image-20211109163137702.png" alt="image-20211109163137702" style="zoom:67%;" />

### 4.3.6. 室内温湿度 卡片数字

展示数据：点击**配置数据源**

* 选择数据源：设备

* 产品：空调遥控（取决于IoT部分填写的产品名称）

* 设备：**动态设备 > 房间号**

* 属性：室内温度

**交互**部分不用操作。

<img src="Image/image-20211109163843469.png" alt="image-20211109163843469" style="zoom:67%;" />

### 4.3.7. 更新 按钮

**样式** 页面根据自己喜好配置。

**交互** 页面：（共4个交互：依次选择 **开关、温度、模式、风速**）

* 事件：点击；

* 动作：设备设备属性

* **产品：空调遥控**

* 设备：动态设备 > 房间号

* 属性：**开关**（4个交互 依次填入 **开关、温度、模式、风速**）

* 属性值：动态来源

产品选项中：

* 产品：空调遥控
* 设备：动态设备
* 属性：开关
* 设置值：变量 > Power

<img src="Image/image-20211109164729344.png" alt="image-20211109164729344" style="zoom:67%;" />

### 4.3.8. 群控 按钮

**交互** ：

* 事件：点击
* 动作：打开弹窗容器
* 选择组件：弹窗-群控确认（来自之前设置的弹窗）
* 弹窗数据源：静态数据

<img src="Image/image-20211109164952118.png" alt="image-20211109164952118" style="zoom:67%;" />

**弹窗**

点击左侧下方 **组件树** 下的 **弹框>弹窗-xxxx** ，编辑页面将显示弹窗。

在右侧，勾选 **样式** 下的 **蒙版**，编辑 **标题&操作按钮** 等。

<img src="Image/image-20211111103120625.png" alt="image-20211111103120625" style="zoom:67%;" />

最终效果：

<img src="Image/image-20211111103151405.png" alt="image-20211111103151405" style="zoom:50%;" />

点击保存，预览最终效果。在预览界面您可直接操作页面按钮滑块等。如果一切顺利，您可在IoT调试界面看到web下发的数据内容。

# 5.测试

**../0.Docs\测试视频.mp4**

下图来自视频截图。

1. 点击页面按钮 **除湿** ，页面上方显示 **设置设备属性值成功**；
2. 设备收到IoT下发信息，使用 **红外** 对线控器发送信息，同时绿灯闪烁；
3. 线控器收到 **红外** 信号，解码后将信息显示在界面上，同时通过有线方式将信息转发给接收器；（线控器℃图标上方形似wifi图标的标志表示正在向接收器发送信息）

![studio_video_1635244709967-00.00.07.133](Image/studio_video_1635244709967-00.00.07.133.png)

------

**至此，全部内容已完成。**

**调试部分需要自己发现问题，解决问题。这里不列举**

