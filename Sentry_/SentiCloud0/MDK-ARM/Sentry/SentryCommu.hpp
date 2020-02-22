/**
  * @file  SentryCommu.hpp
  * @brief    哨兵视觉、CAN通讯标识符/函数汇总
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/18
  * @version  v1.0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#ifndef __SENTRY_COMMU_HPP_
#define __SENTRY_COMMU_HPP_

//外设定义


// #ifndef __CLOUD_MODE_DEF
// #define __CLOUD_MODE_DEF
// enum _cloud_ctrl_mode:uint8_t
// {
//     absolute_cloud = 0x01,
//     relative_cloud = 0x02,
//     save_cloud = 0x00,
// };
// #endif

#endif

#ifndef __CHASSIS_MODE_DEF
#endif	//__CHASSIS_MODE_DEF

void CHASSIS_SUPERIOR_ALL_CanRx(uint32_t StdId, uint8_t *ptrData);


//全局接收变量
extern CanCommuRecv_t CanRx, CanTx;
// #ifdef USE_VISION
// extern sentry_vision_data VisionRx,VisionTx;
// #endif
#ifdef CLOUD_COMMU
#endif
#ifdef CHASSIS_COMMU
void ChassisCanCommuRoutine(void);
#endif
#ifdef USE_VISION
#endif
//CAN接收回调自动接收函数
#ifdef CLOUD_COMMU
#endif
void CanRxCpltCallBack_ChassisCommuUpdata(CAN_HandleTypeDef *_hcan, CAN_RxHeaderTypeDef *RxHead, uint8_t *Data);
//CAN信息底盘托管控制程序
#ifdef USE_VISION
//视觉串口中断接收函数
void SentryVisionUartRxAll(uint8_t* Vision_Rxbuffer);

//视觉信息控制托管程序

#endif

//废案。
//void UP_CLOUD_STATES_CanRx(int StdId, uint8_t *ptrData);
//void UP_CLOUD_STATES_CanTx();
//void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData);
//void DOWN_CLOUD_STATES_CanTx();
//void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData);
//void CHASSIS_STATES_CanTx();
//void UP_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
//void UP_FEED_CanTx();
//void DOWN_FEED_CanRx(uint32_t StdId, uint8_t *ptrData);
//void DOWN_FEED_CanTx();
//void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_UP_RELATIVE_CMD_CanTx();
//void SUPERIOR_UP_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_UP_ABSOLUTE_CMD_CanTx();
//void SUPERIOR_DOWN_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_DOWN_RELATIVE_CMD_CanTx();
//void SUPERIOR_DOWN_ABSOLUTE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_DOWN_ABSOLUTE_CMD_CanTx();
//void SUPERIOR_CHASSIS_MOVE_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_CHASSIS_MOVE_CanTx();
//void SUPERIOR_CHASSIS_SET_LOACTION_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_CHASSIS_SET_LOACTION_CanTx();
//void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_CHASSIS_SET_LOACTION_LIMIT_SPEED_CanTx();
//void SUPERIOR_SAFE_CanRx(uint32_t StdId, uint8_t *ptrData);
//void SUPERIOR_SAFE_CanTx();



#endif // __SENTRY_COMMU_HPP_
