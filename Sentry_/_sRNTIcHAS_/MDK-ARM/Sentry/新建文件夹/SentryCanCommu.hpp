#ifndef __SENTRY_CLOUD_CAN_COMMU_HPP_
#define __SENTRY_CLOUD_CAN_COMMU_HPP_
/**
  * @brief    ÉÚ±øÔÆÌ¨CANÍ¨ÐÅ
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/7
  * @version  v0.0.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "can.h"
#include "bsp_can.hpp"
#include "SentryChassisLogic.hpp"
#include <string.h>

#define CAN_INTERBOARD hcan2



HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, uint8_t *ptrData);
HAL_StatusTypeDef SentryCanSend(CAN_HandleTypeDef *_hcan, uint32_t command_id, float argu1, float argu2);
#endif // __SENTRY_CLOUD_CAN_COMMU_HPP_
