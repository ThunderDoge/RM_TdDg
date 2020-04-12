/**
 * @file        SentryChassis.hpp
 * @brief       �ڱ���������ʵ��
 * @details     
 *  �ڱ�������Ҫ����Ӳ���У�
 *  - ���������ֵ���������й��ʼ��оƬ�������ʵʱ�����빦��
 *  - ����̶̹���������
 *  - �ع�����Ҹ�һ��GY-53/VL531LX���ۼ�����ģ�顣��෶Χ2000mm��������10mm
 *  - �ع�����Ҹ�һ�����ɡ������Բ�����X.X mm/s���ٶ�ײ��������������������岻��ײ��
 *      - ������̨����̨�Լ����ƣ����̲����ơ�
 *      - ����
 *  ��Ҫʵ�����¹��ܣ�
 *  - �������д����������ܾ�ȷ���ƹ���λ��
 *  - ���������ֿ��ơ������ʿ���
 * 
 * @author   ThunderDoge
 * @date      2020-4-11
 * @version   v1.0
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: gb2312
 * ע����2019-12��֮ǰ���ڱ����̾��в�ͬ�Ľṹ��
 * ��ҩ��װ�ڵ��̣����������������ֱַ������²���������һĦ���ָ���������̨������ �Ƶ��衣
 * ��2019-1��֮ǰ�������������Ͽ����в������֣����󲿷��Ѿ�ע�͡�
 *  verison|    date|       author|         change|
 *  1.0          2020-4-11   ThunderDoge     ����˸���ϸ��ע��
 */
#ifndef __SENTRY_CHASSIS_HPP_
#define __SENTRY_CHASSIS_HPP_

// #include "app_AmmoFeed.hpp"  // 2020-1 ֮������Ҫ����
#include "app_imu.h"
#include "bsp_motor.hpp"
#include "bsp_current.h"
#include "bsp_adc_deal.h"
#include "bsp_encoder.hpp"
#include "bsp_gy53l1.h"
#include "app_math.h"
#include <cmath>
#include <cstring>
#include "app_mode.hpp"
#include "sentry_ctrl_def.hpp"

// ���ģ�����ӵĴ���
#define RANGING_LEFT_UART	huart3
#define RANGING_RIGHT_UART	huart4










extern float SpeedMax;

/**
 * @brief �����Ӿ����ö����
 */
enum PillarFlagEnum : int8_t
{
    PILLAR_NOT_DETECTED = 0x00,    ///< δ��⵽����
    PILLAR_LEFT = 0X01,         ///< ��⵽�������
	PILLAR_BOUNCE_LEFT=0x02,    ///< ײ���������
    PILLAR_RIGHT = 0X03,        ///< ��⵽�Ҳ�����
	PILLAR_BOUNCE_RIGHT=0x04,   ///< ײ���Ҳ�����
};

/**
 * @brief ��������ʵ����
 */
class SentryChassis
{
    friend class manager;   // ʹ�õ�����Է����ڱ��ڲ�����

public:
    SentryChassis(uint8_t drive_can_num, uint16_t drive_can_id);    ///<    ���캯��
//                  uint8_t down_yaw_can_num, uint16_t down_yaw_can_id,
//                  uint8_t up_feed_can_num, uint16_t up_feed_can_id,
//                  uint8_t down_feed_can_num, uint16_t down_feed_can_id,
//                  uint8_t up_fric_can_num, uint16_t up_fric_can_id);
    //------------------------ϵͳ����
    static SentryChassis* pointer;
    //-------------------------PID����
    pid pidDriveSpeed;      ///< �����������ٶȻ�
    pid pidDriveLocation;   ///< ����������λ�û�
    // pid FricSpeed;
    // pid FricLocation;
    // pid FeedUpSpeed;
    // pid FeedUpLocation;
    // pid FeedDownSpeed;
    // pid FeedDownLocation;
    pid pidDriveCurrent;    ///< ���������ֹ��ʿ��Ƶ�������
    pid pidPowerFeedback;   ///< ���������ֹ��ʿ��Ƶ�����������
    //-------------------------�������
    softmotor DriveWheel;
//    motor Fric;
//    AmmoFeed FeedUp;
//    AmmoFeed FeedDown;
	//-------------------------������GY-53/VL53L1X
	bsp_GY53L1_Object RangingLeft;
	bsp_GY53L1_Object RangingRight;

    //-------------------------����״̬����
    _chassis_mode Mode;
    _chassis_mode LastMode;
    //-------------------------����״̬�������ɹ���ѯ
    float MotorSpeed;           ///< ��������ٶȣ���λrpm
    float MotorSoftLocation;    ///< �����·�̣���λ�ǽǶ�
    float RealSpeed;            ///< �������ٶȣ���λmm/s���ɱ�����
    float RealPosition;         ///< �ɱ������뼤����ģ�� �����ںϹ��Ƶ������ľ��룬��λmm
	float Accel_Railward;	    ///<  �ع���ļ��ٶ�
	int16_t LazerRanging[2];
	float imuLeftBounceThreshold=5;		///<  �ж�Ϊײ�������������Ǽ��ٶ���ֵ
	enum PillarFlagEnum PillarFlag = PILLAR_NOT_DETECTED;
	//-------------------------���ʼ������

    float DrivePower;      ///< �����ֹ��ʣ���λW. ����ѯ
    float LimitPower = -1; ///<  �������ƹ���, <0 ��ʾ������
    //-------------------------�����йܺ���
    void Handle();                                         ///< ����ʱ���ݸ��º��߼����� ����
    //-------------------------��������
    void Safe_Set();                                       ///< ��ȫģʽ
    void MotorSpeed_Set(float speed_motor_rpm);            ///< �趨����ٶ�
    void MotorSoftLocation_Set(float location_motor_soft); ///< �趨�����·��
    void MotorSoftLocation_LimitSpeed_Set(float location_motor_soft, float speed_motor_rpm_limit);  ///< �趨�����·�̣���������
    // void Speed_Set(float speedBy_mm_s); //�趨��ʵ�ٶ�
    // void Location_Set(float locationBy_mm); //�趨��ʵλ�á�
private:
    //���ʼ����ñ���
    float TargetPowerInput = 0; 
    float PowerOutput = 0;
    uint32_t PwrUpdateTime = 0;
    LPF2 lpf;


    void CanSendHandle(); //�йܵ�CANSend�Ĳ�������
    float PowerFeedbackSystem(float TargetSpeedInput, float TargetCurInput,float PwrFeedbackInput);//��С��ʽ���ʱջ�

    // int8_t PillarHit_Check();
    // int8_t PillarHit_Handle();
};
extern SentryChassis ChassisEntity;     // ����ʵ�����

#endif // __SENTRY_CHASSIS_HPP_
