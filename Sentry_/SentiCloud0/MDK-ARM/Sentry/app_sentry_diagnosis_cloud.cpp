/**
 * @file      app_sentry_diagnosis_cloud.cpp
 * @brief     
 * @details   
 * @author   ThunderDoge
 * @date      
 * @version   
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
                         Using encoding: gb2312
*/
#include "app_sentry_diagnosis_cloud.hpp"

void app_sentry_diag_cloud_Init()
{
    memset(&OfflineListBytes, 0, APP_SENTRY_DIAG_OFFLINE_LIST_LENGTH_IN_BYTE);
}
static manager *diag_motorCheckList[5] = {  //将要检查的DJI_CAN电机的列表
    &CloudEntity.PitchMotor,
    &CloudEntity.YawMotor,
    &CloudEntity.Feed2nd,
    &CloudEntity.FricLeftMotor,
    &CloudEntity.FricRightMotor};
static uint8_t diag_motorCheckDcode[5] = {  //与每个电机对应的DCODE。序号相同。
    DCODE_UPCLOUD_PITCH,
    DCODE_UPCLOUD_YAW,
    DCODE_UPCLOUD_FEEDER,
    DCODE_UPCLOUD_FRIC_LEFT,
    DCODE_UPCLOUD_FRIC_RIGHT,
} void
app_sentry_diag_cloud_Handle()
{
    //检查电机掉线
    for (size_t i = 0; i < 5; i++)
    {
        app_sentry_diag_SetDevice(
            diag_motorCheckDcode[i],
            diag_motorCheckList[i]->Is_Offline() );
    }
    app_sentry_diag_SetDevice(DCODE_UPCLOUD_NUC, APP_SENTRY_DIAG_IS_OFFLINE_TIMEOUT(VisionRx.UpdateTime) );
    app_sentry_diag_SetDevice(DCODE_UPCLOUD_IMU,APP_SENTRY_DIAG_IS_OFFLINE_TIMEOUT(tNow) );
}