/**
 * @file      app_can_operator.hpp
 * @brief     class CanOperator 用于处理CAN_ID发送接收和处理的类
 * @details
 * @author   ThunderDoge
 * @date     2020-2-27
 * @version   0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
                           Using encoding: gb2312
 */
#ifndef __APP_CAN_OPERATOR_
#define __APP_CAN_OPERATOR_

#include "stm32f4xx_hal.h"
#include "app_mode.hpp"
#include <string.h>

class CanOperator;
typedef void (*pvCanOpCallBack)(CanOperator*)

class CanOperator /// CanOperator 用于处理CAN_ID发送接收和处理的类
{
public:
    CanOperator(uint32_t cmd_id, pvCanOpCallBack pfunc = nullptr,
                void *pData_read_1 = nullptr,
                void *pData_write_1 = nullptr,
                uint8_t data_lenght_1 = 1,
                void *pData_read_2 = nullptr,
                void *pData_write_2 = nullptr,
                uint8_t data_lenght_1 = 1)
        : TriggerId(cmd_id),
          TrigCallBack(pfunc),
          pReadData[0](pData_read_1),
          pWriteData[0](pData_write_1),
          pReadData[1](pData_read_2),
          pWriteData[1](pData_write_2),
          DataLength[0](data_lenght_1),
          DataLength[1](data_lenght_2)  {}

    uint8_t TryTrigger(uint32_t cmd_id);
    void ForceTrigger();

    //状态变量
    uint32_t TriggerId;
    pvCanOpCallBack TrigCallBack;
    //   uint8_t DataCommand;
    void *pReadData[2];
    void *pWriteData[2];
    uint8_t DataLength[2];
    uint32_t TriggedTimestamp;
};

void app_can_operator_StdSendCmdCallBack(CanOperator* self);

#endif // !__APP_CAN_OPERATOR_
