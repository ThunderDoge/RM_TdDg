#ifndef	__APP_CLOUD_H_
#define	__APP_CLOUD_H_

#include "bsp_can.h"
#include "app_pid.h"
#include "app_quad_math.h"
#include "app_filter.h"

void Cloud_Stop(void);
void Cloud_PID_Init(void);
void Cloud_RunEncoder(uint16_t ecdr_pitch, uint16_t ecdr_yaw);
void Cloud_RunMPU(float pitch, float yaw);
void Cloud_RunEcdrMpuMix(uint16_t ecdr_pitch, uint16_t ecdr_yaw);

#endif	//__APP_CLOUD_H_
