#include "SentryCommu.hpp"

// Infomation Storage
CanCommuRecv_t CanInfo;
bsp_vision_data VisionInfo;

// CAN TxRx Functions

void UP_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == UP_CLOUD_STATES)
        memcpy(&CanInfo.UpCloudPitchYaw, ptrData, 8);
}
void UP_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, UP_CLOUD_STATES, CanInfo.UpCloudPitchYaw[0],
                  CanInfo.UpCloudPitchYaw[1]);
}

void DOWN_CLOUD_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == DOWN_CLOUD_STATES)
        memcpy(&CanInfo.DownCloudPitchYaw, ptrData, 8);
}
void DOWN_CLOUD_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, DOWN_CLOUD_STATES,
                  CanInfo.DownCloudPitchYaw[0], CanInfo.DownCloudPitchYaw[1]);
}
void CHASSIS_STATES_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == CHASSIS_STATES)
        memcpy(&CanInfo.Chassis_SpeedLocation, ptrData, 4);
}
void CHASSIS_STATES_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, CHASSIS_STATES,
                  CanInfo.Chassis_SpeedLocation[[0]],
                  CanInfo.Chassis_SpeedLocation[1]);
}

void SUPERIOR_UP_RELATIVE_CMD_CanRx(uint32_t StdId, uint8_t *ptrData)
{
    if (StdId == SUPERIOR_UP_RELATIVE_CMD)
        memcpy(&CanInfo.up, ptrData, 8);
}
void SUPERIOR_UP_RELATIVE_CMD_CanTx()
{
    SentryCanSend(&CAN_INTERBOARD, SUPERIOR_UP_RELATIVE_CMD, CanInfo.DownCloudPitchYaw[0],
                  CanInfo.DownCloudPitchYaw[1]);
}
