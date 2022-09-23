/*!
 * @file        irRemote.cpp
 * @brief       红外遥控 - 格力空调 
 * @copyright   Copyright (c) 2021 ChenYuanliang
 * @licence     CC-BY-NC-SA 3.0，知识共享许可协议-署名-非商业使用-相同方式共享。
 * @author      ChenYuanliang
 * @version     V1.0
 * @date        2021-10-01
 * @url         https://github.com/OpticalMoe/IoT_remote
 */

#include "irRemote.h"
#include <Arduino.h>

/**
   L, A, A', B, B', C, C',| S,| L, A, A', B, B', C, C',| E

   L: 4400 + 4400 us
   S: 540 + 5220 us
   E: 540 us
   0: 540 + 540 us
   1: 540 + 1620 us

   A, B, C: 数据
   A', B', C': A, B, C的反码

   1 tab = 4 space

   TIM3 Channel 2 发送 GPIO B.5
*/

//****************************** 宏定义 **************************************//
#define BUFFER_LENGTH   (2 * ONE_PACK_LENGTH + INTERVAL_LENGTH + END_LENGTH)
#define ONE_PACK_LENGTH (INTRODUCTION_LENGTH + DATA_LENGTH * 6)
//长度
#define INTRODUCTION_LENGTH 1 * 2   //引导码
#define DATA_LENGTH     8 * 2       //数据A、B、C等长，用一个表示
#define INTERVAL_LENGTH 1 * 2       //间隔码
#define END_LENGTH      1           //终止符
//偏移
#define INTRODUCTION_OFFSET 0                               //引导
#define DATA_A_OFFSET   INTRODUCTION_LENGTH                 //数据A
#define DATA_B_OFFSET   (DATA_A_OFFSET + DATA_LENGTH * 2)
#define DATA_C_OFFSET   (DATA_B_OFFSET + DATA_LENGTH * 2)
#define DATA_AI_OFFSET  (DATA_A_OFFSET + DATA_LENGTH)       //数据A的反码
#define DATA_BI_OFFSET  (DATA_B_OFFSET + DATA_LENGTH)
#define DATA_CI_OFFSET  (DATA_C_OFFSET + DATA_LENGTH)
#define INTERVAL_OFFSET (DATA_C_OFFSET + DATA_LENGTH * 2)   //间隔码
#define TWO_PACK_OFFSET (INTERVAL_OFFSET + INTERVAL_LENGTH) //第二包
#define END_OFFSET  (TWO_PACK_OFFSET + ONE_PACK_LENGTH)     //终止符

//********** 各码时长
#define INTRODUCTION_HIGH   4400 //4.4ms     引导码
#define INTRODUCTION_LOW    4400 //4.4ms
#define INTERVAL_HIGH       540  //0.54ms    间隔码
#define INTERVAL_LOW        5220 //5.22ms
#define END_HIGH    540  //0.54ms            终止符
#define ZERO_HIGH   540  //0.54ms  
#define ZERO_LOW    540  //0.54ms
#define ONE_HIGH    540  //0.54ms
#define ONE_LOW     1620 //1.62ms

//********** other
unsigned char pinRemote = 0;
hw_timer_t *Tim = NULL;

//********** DEBUG
#define DEBUG   0

//****************************** 变量 ****************************************//
//[x][0]: 电平 -> 高电平/低电平; [x][1]: 时长 -> 脉冲/低电平持续时长
unsigned short int buffer[BUFFER_LENGTH][2] = {0};
unsigned char bufferPosition = 0;
unsigned short int timerCount = 0;
unsigned char timerEnableFlag = 0;

//****************************** 声明 ****************************************//
void TIM_IRQHandler(void);
unsigned char irRemoteEncode(unsigned char dataA, unsigned char dataB, \
                             unsigned char dataC);
unsigned char irRemoteDebug(unsigned char dataA, unsigned char dataB, \
                            unsigned char dataC);

//********************************* Init ***********************************//
/**
   初始化
*/
void irRemoteInit(unsigned char pin)
{
  unsigned char i = 0;
  pinRemote = pin;

  ledcSetup(0, 38000, 2);   //通道，频率，分辨率(2^2)
  //  ledcAttachPin(pinRemote, 0);  //连接到 pin
  ledcWrite(0, 2);          //通道 ch 写 高电平占空比
  //  ledcDetachPin(pinRemote); //取消连接到 pin

  Tim = timerBegin(0, 80, true);   //初始化；通道，分频(80MHz/x)，累加模式
  //timerEnd(Tim);   //取消定时器
  timerAttachInterrupt(Tim, TIM_IRQHandler, true);    //配置定时器中断；定时器，回调函数，中断边沿触发
  //timerDetachInterrupt(Tim);   //取消定时器中断
  timerAlarmWrite(Tim, 540, true);  //定时器设置；定时器，计数上限，是否重载
  timerAlarmEnable(Tim);   //使能定时器
  //timerAlarmDisable(Tim);  //失能定时器
  //bool timerAlarmEnabled(Tim);  //判断定时器是否启动

  for (i = 0; i < BUFFER_LENGTH; i++)
  {
    buffer[i][0] = 0, buffer[i][1] = 0;
  }

  bufferPosition = 0;
  timerCount = 0;
  timerEnableFlag = 0;
}

//***************************** Adjustments **********************************//
/**
   调整
   温度 | 模式 | 风速
   temp:        17 - 30 'C
   Mode:        A -> 自动, C -> 制冷, D -> 抽湿, H -> 制热, F -> 送风
   windSpeed:   A -> 自动, L -> 低风, M -> 中风, H -> 高风, F -> 固定风

   dataA:   ID = 1011 0010 = 0xB2
   dataB:   B7 B6 B5 B4  B3 B2 B1 B0
             风  速   1   1  1  1  1
   dataC:   C7 C6 C5 C4  C3 C2 C1 C0
             温     度    模式  0  0
   return:  0 -> busy,  1 -> OK
*/
unsigned char irRemoteAdjustments(unsigned char id, unsigned char temp, \
                                  char mode, char windSpeed)
{
  unsigned char dataB = 0, dataC = 0;
  unsigned char error = 0;

  if (timerEnableFlag == 1) return 0;     //判忙

again:
  //***** 系统限制
  //在 抽湿 或 自动模式 下，风量应为固定风(B000x xxxx)
  if ((mode == 'A') | (mode == 'D'))
    windSpeed = 'F';
  //送风 和 抽湿模式 代码一样 ,但抽湿模式有 温度代码 ,而送风模式无(B1110 xxxx)
  else if (mode == 'F')
    temp = 'N';     //无定义

  //***** 参数设定
  error = 0;
  switch (windSpeed)
  { //[此处 x 填充 1]
    case 'A':   dataB = 0xBF;   break;  //自动    B101x xxxx
    case 'L':   dataB = 0x9F;   break;  //低风    B100x xxxx
    case 'M':   dataB = 0x5F;   break;  //中风    B010x xxxx
    case 'H':   dataB = 0x3F;   break;  //高风    B001x xxxx
    case 'F':   dataB = 0x1F;   break;  //固定风  B000x xxxx
    default :   windSpeed = WIND_SPEED; error++;    //其他 默认值
  }

  switch (temp)
  { //[此处 x 留空]
    case 17:    dataC = 0x00;   break;  //B0000 xxxx
    case 18:    dataC = 0x10;   break;  //B0001 xxxx
    case 19:    dataC = 0x30;   break;  //B0011 xxxx
    case 20:    dataC = 0x20;   break;  //B0010 xxxx
    case 21:    dataC = 0x60;   break;  //B0110 xxxx
    case 22:    dataC = 0x70;   break;  //B0111 xxxx
    case 23:    dataC = 0x50;   break;  //B0101 xxxx
    case 24:    dataC = 0x40;   break;  //B0100 xxxx
    case 25:    dataC = 0xC0;   break;  //B1100 xxxx
    case 26:    dataC = 0xD0;   break;  //B1101 xxxx
    case 27:    dataC = 0x90;   break;  //B1001 xxxx
    case 28:    dataC = 0x80;   break;  //B1000 xxxx
    case 29:    dataC = 0xA0;   break;  //B1010 xxxx
    case 30:    dataC = 0xB0;   break;  //B1011 xxxx
    case 'N':   dataC = 0xE0;   break;  //B1110 xxxx    无定义
    default:    temp = TEMPERATURE; error++;    //其他 默认值
  }

  switch (mode)
  { //[此处 x 填充 0]
    case 'A':   dataC |= 0x08;  break;  //自动    B10xx
    case 'C':   dataC |= 0x00;  break;  //制冷    B00xx
    case 'D':   dataC |= 0x04;  break;  //抽湿    B01xx
    case 'H':   dataC |= 0x0C;  break;  //制热    B11xx
    case 'F':   dataC |= 0x04;  break;  //送风    B01xx   同 抽湿 代码
    default:    mode = MODE;    error++;    //其他 默认值
  }

  //传入的实参存在未定义数据，需改为默认值，并重新设置。
  if (error != 0)
    goto again;

  irRemoteEncode(id, dataB, dataC);       //编码

  //DEBUG
#if DEBUG
  printf("ID: 0x%x\t Temp: %d \t Mode: %c \t Wind: %c \r\n", \
         id, temp, mode, windSpeed);
#endif
  //    irRemoteDebug(id, dataB, dataC);

  //置位标志位
  bufferPosition = 0;
  timerCount = 0;
  timerEnableFlag = 1;

  return 1;
}

/**
   默认参数

   return: 0 -> busy, 1 -> OK
*/
unsigned char irRemoteDefault(void)
{
  return irRemoteAdjustments(ID, TEMPERATURE, MODE, WIND_SPEED);
}

/**
   关机

   return: 0 -> busy, 1 -> OK
*/
unsigned char irRemoteOff(unsigned char id)
{
  if (timerEnableFlag == 1) return 0;     //判忙

  irRemoteEncode(id, 0x7B, 0xE0);
  irRemoteDebug(id, 0x7B, 0xE0);

  //置位标志位
  bufferPosition = 0;
  timerCount = 0;
  timerEnableFlag = 1;

  return 1;
}

//******************************* Encode *************************************//
/**
   编码

   L, A, A', B, B', C, C',| S,| L, A, A', B, B', C, C',| E
  引导                    间隔 第二包（同第一包数据）  终止符
*/
unsigned char irRemoteEncode(unsigned char dataA, unsigned char dataB, \
                             unsigned char dataC)
{
  unsigned char i = 0, j = 0, k = 0;

  //全部置为 0码
  for (i = 0; i < BUFFER_LENGTH; i++)
  {
    if (i % 2 == 0)
      buffer[i][0] = 1, buffer[i][1] = ZERO_HIGH;
    else
      buffer[i][0] = 0, buffer[i][1] = ZERO_LOW;
  }

  //引导位
  buffer[INTRODUCTION_OFFSET][1] = INTRODUCTION_HIGH;
  buffer[INTRODUCTION_OFFSET + 1][1] = INTRODUCTION_LOW;

  //数据
  for (j = 0; j < 3; j++) //分三次编码 data A, B, C
  {
    if (j == 0) i = DATA_A_OFFSET, k = dataA;
    else if (j == 1) i = DATA_B_OFFSET, k = dataB;
    else i = DATA_C_OFFSET, k = dataC;

    while (k != 0)
    {
      if ((k & 0x80) == 0x80)         //MSB在先，LSB在后
        buffer[i + 1][1] = ONE_LOW; //仅修改一码，零码不用改
      i += 2;                         //仅修改低电平脉宽
      k = k << 1;
    }
  }

  //数据 反码
  for (j = 0; j < 3; j++)
  {
    if (j == 0) i = DATA_AI_OFFSET;
    else if (j == 1) i = DATA_BI_OFFSET;
    else i = DATA_CI_OFFSET;

    for (k = 0; k < 8; k++, i += 2)
    {
      if (buffer[i + 1 - DATA_LENGTH][1] == ZERO_LOW) //若 原码 为 零
        buffer[i + 1][1] = ONE_LOW;                 //则 反码 为 壹
    }
  }

  //间隔码
  buffer[INTERVAL_OFFSET][1] = INTERVAL_HIGH;
  buffer[INTERVAL_OFFSET + 1][1] = INTERVAL_LOW;

  //第二包数据
  for (i = INTRODUCTION_OFFSET, j = TWO_PACK_OFFSET; j < END_OFFSET; \
       i++, j++)
  {
    buffer[j][1] = buffer[i][1];
  }

  //终止符
  buffer[END_OFFSET][1] = END_HIGH;   //终止符 仅包含脉冲，不包含低电平

  return 1;
}

//******************************** DEBUG *************************************//
/**
   DEBUG
*/
unsigned char irRemoteDebug(unsigned char dataA, unsigned char dataB, \
                            unsigned char dataC)
{
#if DEBUG
  unsigned char i = 0;

  printf("\r\n");
  printf("Data:\t0x%x\t0x%x\t0x%x\r\n", dataA, dataB, dataC);
  for (i = 0; i < BUFFER_LENGTH; )
  {
    switch (i)
    {
      case INTRODUCTION_OFFSET:   printf("Introduction \r\n"); break;
      case DATA_A_OFFSET:         printf("Data_A \r\n");      break;
      case DATA_AI_OFFSET:        printf("Data_AI \r\n");     break;
      case DATA_B_OFFSET:         printf("Data_B \r\n");      break;
      case DATA_BI_OFFSET:        printf("Data_BI \r\n");     break;
      case DATA_C_OFFSET:         printf("Data_C \r\n");      break;
      case DATA_CI_OFFSET:        printf("Data_CI \r\n");     break;
      case INTERVAL_OFFSET:       printf("Interval \r\n");    break;
      case TWO_PACK_OFFSET:       printf("Two Pack \r\n");    break;
      default:         ;
    }

    printf("No %d:\t%d\t%d\t||\t%d\t%d\r\n", \
           i / 2, buffer[i][0], buffer[i][1], \
           buffer[i + 1][0], buffer[i + 1][1]);//1, 脉冲个数, 0, 低电平个数
    i += 2;
  }
#endif
  return 1;
}

//****************************** Interrupt ***********************************//
/**
   定时器中断服务程序
*/
void TIM_IRQHandler(void)   //TIM中断
{
  if (timerEnableFlag)
  {
    if (bufferPosition == BUFFER_LENGTH) //全部跑完，退出
    {
      ledcDetachPin(pinRemote); //取消连接到 pin - > 拉低
      timerEnableFlag = 0;
      return;
    }
    //没有全部跑完，切换下一位数据
    if (buffer[bufferPosition][0] != 0)
      ledcAttachPin(pinRemote, 0);  //连接到 pin
    else
      ledcDetachPin(pinRemote); //取消连接到 pin - > 拉低
    timerAlarmWrite(Tim, buffer[bufferPosition][1], true);  //定时器设置；定时器，计数上限，是否重载
    bufferPosition++;
  }
}

/********************************* END OF FILE ********************************/
