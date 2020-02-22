/**
  * @file  SentryCommu.hpp
  * @brief    哨兵视觉、CAN通讯标识符/函数汇总
  * @details  
  * @author   ThunderDoge
  * @date     2019/12/18
  * @version  v1.0.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
  */

#include "SentryCommu.hpp"

// Infomation Storage
CanCommuRecv_t CanInfo, CanRx, CanTx;
#ifdef USE_VISION
Sentry_vision_data VisionRx, VisionTx;
#endif



