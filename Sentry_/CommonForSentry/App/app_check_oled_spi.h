/**
 * @file      app_check_oled.h
 * @brief     离线检测·OLED液晶屏幕显示支持包
 * @details   
 * @author   ThunderDoge
 * @date      2020-4-19
 * @version   v0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: utf-8
 */


#ifndef __APP_CHECK_OLED_SPI_H
#define __APP_CHECK_OLED_SPI_H
#include "bsp_stddef.h"
#include "bsp_oled.h"
#include "app_check.h"

#define __OLED_SELECT_LAST_BOX ((int)(-1))
#define __OLED_SELECT_ALL_BOX ((int)(-2))

#define __OLED_BOX_LIST_LENGTH 10

typedef struct __app_check_oled_check_box{
    uint8_t* DeviceName;
    uint8_t* isOfflineVar;
}app_check_oled_check_box;

typedef struct __app_check_oled_check_box_list{
    app_check_oled_check_box Box[__OLED_BOX_LIST_LENGTH];
    uint16_t BitOfBoxInUse;
}app_check_oled_check_box_list;



extern app_check_oled_check_box_list GlobalBoxList;

void app_check_oled_Init();
HAL_StatusTypeDef app_check_oled_EnableDevice(DeviceIdEnum device_id, uint8_t* device_name,int position_at_list);
void app_check_oled_DisableDevice(int position_at_list);
void app_check_oled_Show_BoxList();

#endif // __APP_CHECK_OLED_SPI_H
