#include "SentryChassis.hpp"
//电机类型
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
//本体
SentryChassis Self(2, 0x201,
                   2, 0x206,
                   2, 0x203,
                   1, 0x201,
                   1, 0x202,
                   1, 0x203);

SentryChassis::SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id,
                             uint8_t up_yaw_can_num, uint16_t up_yaw_can_id,
                             uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
                             uint8_t up_feed_can_num, uint16_t up_feed_can_id,
                             uint8_t down_feed_can_num,
                             uint16_t down_feed_can_id, uint8_t up_fric_can_num,
                             uint16_t up_fric_can_id)
    : DriveSpeed(0.25, 0, 0, 1000, 16000, 100, 300),
      DriveLocation(0, 0, 0, 1000, 1000, 10, 200),
      DriveWheel(2, 0x203, &DJI_3508, &DriveSpeed, &DriveLocation){};

void SentryChassis::Handle()
{
    MotorSpeed = DriveWheel.RealSpeed;
    MotorSoftLocation = DriveWheel.SoftAngle;
    manager::CANSend();
}


