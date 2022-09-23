/*!
 * @file        irRemote.h
 * @brief       红外遥控 - 格力空调
 * @copyright   Copyright (c) 2021 ChenYuanliang
 * @licence     CC-BY-NC-SA 3.0，知识共享许可协议-署名-非商业使用-相同方式共享。
 * @author      ChenYuanliang
 * @version     V1.0
 * @date        2021-10-01
 * @url         https://github.com/OpticalMoe/IoT_remote
 */

#ifndef __IRREMOTE_H
#define __IRREMOTE_H

//********** 默认值
#define ID  0xB2
#define WIND_SPEED  'A' //自动
#define TEMPERATURE 26  //26'C
#define MODE        'A' //自动

/**
 * 初始化
 */
void irRemoteInit(unsigned char pin);

/**
 * 调整 
 * 温度 | 模式 | 风速
 * temp:        17 - 30 'C
 * Mode:        A -> 自动, C -> 制冷, D -> 抽湿, H -> 制热, F -> 送风
 * windSpeed:   A -> 自动, L -> 低风, M -> 中风, H -> 高风, F -> 固定风
 * return:      0 -> busy, 1 -> OK
 * 实参超出以上范围会被改为默认值
 */
unsigned char irRemoteAdjustments(unsigned char id, unsigned char temp, \
                                char mode, char windSpeed);
/**
 * 默认参数
 * return: 0 -> busy, 1 -> OK
 */
unsigned char irRemoteDefault(void);
/**
 * 关机
 * return: 0 -> busy, 1 -> OK
 */
unsigned char irRemoteOff(unsigned char id);

#endif
