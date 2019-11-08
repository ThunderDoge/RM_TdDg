#include "IMUtask.h"

TaskHandle_t IMU_Task_Handle = NULL;	/*姿态解算任务句柄*/
float my_pitch;

void IMU_Task(void* pvParameters)
{
	while(1)
	{
		IMUSO3Thread();         // 9250解算
		my_pitch = imu.Pitch;
		vTaskDelay(4);
	}
}