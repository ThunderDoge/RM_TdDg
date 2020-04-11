/**
 * 
 * @file SentryCloud.cpp
 * @brief    �ڱ���̨������Ƽ��� Sentry Cloud Motors Control
 * @details     Encoding - GB2312
 * @author   ThunderDoge
 * @date     2019/12/1
 * @version  v0.1-Develop
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
 */
#include "SentryCloud.hpp"

// ---------------------------------���߼���� ��������---------------------------
// !!!ʹ�����˴����궨�壬��С���Ķ�
// �궨��(1)�����庯�������� MotorObj �� isOffline�����˺���ָ�봫�� CheckDevice_Type 
#define DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(MotorObj)	\
uint8_t func_DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ_##MotorObj(void) \
{		\
	return CloudEntity.MotorObj.Is_Offline();	\
}

// �궨��(2)���������涨��ĺ����ĺ�����
#define FUNC_NAME(MotorObj) \
	func_DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ_##MotorObj

// (3):ʹ�ú궨��(1) ���庯��
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(PitchMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(YawMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(FricLeftMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(FricRightMotor)
DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ(Feed2nd)

//��������豸�ṹ�壬����ʹ����(3)�ж���ĺ�����ָ��
CheckDevice_Type UpCloudLeftFric_CheckDevice(
                (CheckDeviceID_Enum)         UpCloudLeftFricDevice,  // �豸ID
                                            100,                    // ��������ʱ��
                                            FUNC_NAME(FricLeftMotor) ); // ���߼�⺯�� ������
CheckDevice_Type UpCloudRightFric_CheckDevice(UpCloudRightFricDevice,100,FUNC_NAME(FricRightMotor));
CheckDevice_Type UpCloudYawMotor_CheckDevice(UpCloudYawMotorDevice,100,FUNC_NAME(YawMotor));
CheckDevice_Type UpCloudPitchMotor_CheckDevice(UpCloudPitchMotorDevice,100,FUNC_NAME(PitchMotor));
CheckDevice_Type UpCloudFeedMotor_CheckDevice(UpCloudFeedMotorDevice,100,FUNC_NAME(Feed2nd));

// ---------------------------------��̨ ��е�ǿ���&�����ǿ��� ���---------------------------
app_Mode ModeCloudCtrlMech(EnterModeCloudCtrlMech,RunModeCloudCtrlMech,NULL);
app_Mode ModeCloudCtrlGyro(EnterModeCloudCtrlGyro,RunModeCloudCtrlGyro,NULL);

// ---------------------------------��̨ ˫PITCH���
//app_Mode ModeDualPitch(EnterModeDualPitch,RunModeDualPitch,NULL);
//app_Mode ModeSinglePitch(EnterModeSinglePitch,RunModeSinglePitch,NULL);

/**
 * @brief �����е��
 * ����ͻ��
 */
void EnterModeCloudCtrlMech(void)
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //���� Ŀ��Ƕ�Ϊ��ǰ�Ƕȡ����Է�ֹģʽ�л�ʱ�Ƕ�ͻ�䡣
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.Mode = absolute_cloud; //��Ϊ���Խǿ���
}
/**
 * @brief ���л�е��ʱ�����ݳ���ά�� �߼���
 * �����Ǹ����е������
 */
void RunModeCloudCtrlMech(void)
{
    // ƽʱ���������ǻ������ݡ������л�е�ǵ�ʱ��ǿ�������Ǹ����е������
    app_imu_data.integral.Pitch = -CloudEntity.PitchMotor.RealAngle;//ע�⸺�š�
    app_imu_data.integral.Yaw = -CloudEntity.YawMotor.RealAngle;//ע�⸺�š�
}
/**
 * @brief 
 * 
 */
void EnterModeCloudCtrlGyro(void)
{
    CloudEntity.TargetPitch = CloudEntity.RealPitch; //���� Ŀ��Ƕ�Ϊ��ǰ�Ƕȡ����Է�ֹģʽ�л�ʱ�Ƕ�ͻ�䡣
    CloudEntity.TargetYaw = CloudEntity.RealYaw;
    CloudEntity.Mode = absolute_cloud; //��Ϊ���Խǿ���

}
void RunModeCloudCtrlGyro(void)
{

}


                                                                                                              
                                                                                                              
                                                                                                              
                                                                                                              
//    .@@@@@@@@@@]`                              =@@@^   /@@@@@@@@@O]`   =@@@^                      .@@@O        
//    .@@@@@@@@@@@@@`                            =@@@^   @@@@@@@@@@@@@^  =@@@^  =@@@.               .@@@@        
//    .@@@@.    =@@@^ =@@@^   =@@@^  @@@@@@@@@\  =@@@^   @@@@^    ,@@@@.]]]]]`.@@@@@@@@  ./@@@@@@@@^.@@@@@@@@@@\.
//    .@@@@.    =@@@^ =@@@^   =@@@^  @@@@O/@@@@\ =@@@^   @@@@^    .@@@@.@@@@@^.@@@@@@@@ ,@@@@@@@@@@^.@@@@@@@@@@@@
//    .@@@@.    =@@@^ =@@@^   =@@@^  ,/@@@@@@@@O =@@@^   @@@@@@@@@@@@@/  =@@@^  @@@@.   =@@@^       .@@@@   .@@@@
//    .@@@@.    =@@@^ =@@@^   =@@@^ @@@@@@@@@@@O =@@@^   @@@@@@@@@@@@/   =@@@^  @@@@.   =@@@^       .@@@@   .@@@@
//    .@@@@]]]]/@@@@^ =@@@\]]]@@@@^.@@@@]]]/@@@O =@@@\]` @@@@^           =@@@^  @@@@\]]`=@@@@`]]]]]`.@@@@   .@@@@
//    .@@@@@@@@@@@@`  .@@@@@@@@@@@^ =@@@@@@@@@@O .@@@@@^ @@@@^           =@@@^  ,@@@@@@@ \@@@@@@@@@^.@@@@   .@@@@
                                                                                                              
                                                                                                              
                                                                                                              

//------------------------˫PITCHģʽ���-----------------------------------------
float pid_param_backup[24];     // ��PITCH�������ݴ��ڴˡ��ڳ�ʼ��ʱ����
float dual_pitch_pid_param[24]=     //˫PITCH�����ڴ˵���
{-3, 0, -8, 2000, 30000, 500,
-15, -1, 0, 1800, 10000,120,
100, 0, 0, 2000, 10000,3000,
-10, 0, 0, 2000, 30000,500};
// ����һ�²����趨
//   PitchSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500), 
//   PitchPosition(-15, -1, 0, 1800, 10000, 10, 10, 120),//(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
//   PitchGyroPosition(200, 0, 0, 2000, 10000, 10, 10, 3000),
//   PitchGyroSpeed(-10, 0, 0, 2000, 30000, 10, 10, 500),

static void copy_pid_param_to_array(float* destination,pid* source)
{
	destination[0] = source->P;
	destination[1] = source->I;
	destination[2] = source->D;
	destination[3] = source->IMax;
	destination[4] = source->PIDMax;
	destination[5] = source->I_Limited;
}
static void copy_array_to_pid_param(pid* desti,float* src)
{
	desti->P = src[0];
	desti->I = src[1];
	desti->D = src[2];
	desti->IMax = src[3];
	desti->PIDMax = src[4];
	desti->I_Limited = src[5];
}

///���ݵ�PITCH������ pid_param_backup. BackupSingleMotorParam
static void copy_cloud_param_to_backup(softcloud* src)
{
    copy_pid_param_to_array(&pid_param_backup[0],src->PID_In);
	copy_pid_param_to_array(&pid_param_backup[6],src->PID_Out);
	copy_pid_param_to_array(&pid_param_backup[12],src->Gyro_PID_In);
    copy_pid_param_to_array(&pid_param_backup[18],src->Gyro_PID_Out);
}
///���������еĲ�������̨���PID
static void copy_array_to_cloud_param(softcloud* desti,float* src)
{
    copy_array_to_pid_param(desti->PID_In,&src[0]);
    copy_array_to_pid_param(desti->PID_Out,&src[6]);
    copy_array_to_pid_param(desti->Gyro_PID_In,&src[12]);
    copy_array_to_pid_param(desti->Gyro_PID_Out,&src[18]);
}
/**
 * @brief ����˫PITCHģʽ
 * 
 */
void SentryCloud::EnterModeDualPitch(void)
{
    copy_array_to_cloud_param(&PitchMotor,dual_pitch_pid_param);    //д��˫PITCH���� WriteDualMotorParam();

    PitchSecondMotor.cooperative = 1;       //�趨��PITCHΪ����ģʽ���������������PID���㡣
    // �������ֵ����ֵ��������ʱ����PitchMotor�ĵ������ֵ
}
/**
 * @brief �˳�˫PITCHģʽ���ָ���PITCHģʽ
 * 
 */
void SentryCloud::ExitModeDualPitch(void)
{
    copy_array_to_cloud_param(&PitchMotor,dual_pitch_pid_param);    //д�뵥PITCH���� WriteDualMotorParam();

    PitchSecondMotor.cooperative = 0;       //�رո�PITCHΪ����ģʽ���ָ�����PID���㡣
    // ���Ǵ˺�����ĳһPITCH����ʱ�Ż���á�
}
/**
 * @brief ����˫PITCHģʽ
 * �Ӷ�PITCH��������ֵ����ֵ��������ʱ����PitchMotor�ĵ������ֵ
 */
void SentryCloud::RunModeDualPitch()
{
    PitchSecondMotor.TargetCurrent = PitchMotor.TargetCurrent;  // ���Ƶ���ֵ
    PitchSecondMotor.InsertCurrent();   // д�����ֵ
}
/**
 * @brief PITCHģʽ���ơ���Handle()�е���
 * 
 */
void SentryCloud::PitchModeCtrl(void)
{
	last_pitch_ctrl_mode = pitch_ctrl_mode;
	
	// ����ģʽPITCH�Ĺ���
	if(PitchMotor.Is_Offline() == 0 && PitchSecondMotor.Is_Offline() ==0)
	{
		pitch_ctrl_mode = __cloud_dual_pitch;
	}
	else 
	{
		pitch_ctrl_mode = __cloud_main_pitch;
	}
	
	// ����ģʽ�л�������
	switch( (pitch_ctrl_mode*10)+(last_pitch_ctrl_mode) )
	{
	case __cloud_dual_pitch*10 + __cloud_dual_pitch:
		{			RunModeDualPitch();		}
		break;
		
	case __cloud_main_pitch*10 + __cloud_main_pitch:
		{/*DO NOTHING*/}
		break;
		
	case __cloud_dual_pitch*10 + __cloud_main_pitch:
	default:
		{			ExitModeDualPitch();		}
		break;
		
	case __cloud_main_pitch*10 + __cloud_dual_pitch:
		{			EnterModeDualPitch();		}
		break;
	}
}
//  `...*...............................................................................................................................
//  ^,`.,\`..`O[\/=/./O\[**************************************************************************************************[******[*****
//  \`/^=O*^^^O`]^==,`],@@@`,*******************************,[**************************************************************************
//  ,OO\/OoO@@\OoOO/[OoOO@@@@`\`*************************************************,]*****,`****************]*****************************
// .///=o`O/O@@@\\`OOo\\\^\@@@@/************************************=^***,**************************************************************
//  *]``*,,[,,@@@@`,o,*`*=\^\@@@\***********************************=^******************************************************************
// .***********,@@@@``]**[*`,*\@@@\*,***************************************************************************************************
// .*,,********,\,@@@O^******,**\@@@\*`*************************************************************************************************
// .****************@@@@*,******\*O@@@``************************************************************************************************
// .*****************,@@@@`*`****,``\@@@`***********************************************************************************************
// .*****************`*,@@@@********\,@@@@/`********************************************************************************************
// .********************`,@@@@`********,@@@@********************************************************************************************
// .**=********************,@@@@]\,****`*\@@@\**************************************************************************]***************
// .***********************`,,@@@@`****,*,*\@@@O*,**********************************************************************]***************
// .***************************/@@@\/]*****]*@@@@\`*************************************************************************************
// .***************************`\\@@@@`****^*`,\@@@\*\**********************************************************************************
// .**=****************************/@@@\`,,^***`,@@@@``/,*******************************************************************************
// .*******************************/`/@@@@/*******[@@@@`*`,****************************************************************]`***********
// .`**********************************=@@@\`\,****`\@@@@`*`*,\*************************************************************************
// .*************************************,@@@@/*****=*\@@@@`,***************************************************************************
// .***************************************\@@@@*\,*,`*`\@@@\,*=*]**********************************************************************
// .***************************************,*\@@@@*****`*,@@@@\*************************************************************************
// .*******************************************\@@@@*/,,***,@@@@\*`/*[,*****************************************************************
// .*******************************************\.\@@@@``***=*[@@@@\[`*^*****************************************************************
// .***********************************************\@@@@`//`,``\@@@@^,\,/*\*****************************`,,`,***************************
// .******,****************************************,`\@@@@``]O/``[@@@@O]O\,********`*******************,`   \*,*************************
// .**************************=**********************\`\@@@@@@^    =@@@@@@^***`*,***`=^********^*****. ,      `*,^,*********************
// .************************[[*****************/^******[@@@@@@  @^ =@@@@@@ .`*`******`*************``   ,.   @^ ************************
// .***************************************************=O@@@`  ..   @@@@^\@@@`[/*\,*`*[],,]****,\*,.  /]]]]@/@@].*`*****,***************
// .**************[*`*,**********************************.  ,@@@OOO@@/[`\@@@@@^    .``]*]*,***,**   =@^        @@..*********************
// .*****************************************************`    ,@@@@@@OO*..@@@@@@\         `**,`   ,@/         /O,@`.,*/****************`
// .*****************************************************       =@@@@@OOOO^=@@@@^     ]` . `    /@@`           O@^\\./,*****************
// .***********************************************,****.      ]/@@@@@@@@@]/@@@@.  . .. ..\@@@@@@@             ,OOO/@*******************
// .*************************************************,`` ....,     =\/=@@@@`         /`       @@@^              ,[``*,******************
// .************************************************/           /@@@@@@@@@@^  .     @@@.  ,,\  == ,^             ,]*********************
// .************************************************@@@@@]]\] @@. ,@@@@@@@@[     .\\@@/  =@@@^.=` @           ,^..**********************
// .*******************************************.          ,/@@@`   @\/@@@@@      .\OO@@@O..=`[\@ =@@@@@@@@@@@@@@\.[*=*******************
// .*******************************.******,.           ]@@@@@@@@@@@@@@@@@@@@^@@@@@@@@@@O^,*O^./@`           ..@@@@`^**]***]*************
// .***********************************`.          ]/\/O@@/    /@@@@@@@@@@@@`/@@@@@@@OOO^.oO/`            *...=@@@@^          ]]]^******
// .********************************.            .,]]]O@@`   ,@@@@@@@@@@@@@@@@@@OO@@@Oo^]@[            .*......\@@/]/o[[.........*******
// .*****************************`           =@``=oOOO@@    =@@@@@@@@@@@@@@@@@@@@@@@@OO`             *..........=@^...........   .******
// .***************************.              =`,]]OO@@@/[@ @@@@@@@@@@@@@@@@@@OOOOO@/             .*...............    .,]]/o[[`..******
// .****************************         [[[[...\oOOO@@@*/^=@@@@@@@@@@@@@@@@@*o.*.              *......      .]]/O[[[...    ,]`[[`******
// .***************************.         ....*      ,[[` ,@@@@@@@@@@@@@@@@@@^.,..                 ..]]/O[[....    .]]o/[`........*.*****
// .****************************         ...,oOOO@@@@OOO]],]]]                = .`           ,[`....     ,]]oO/o`*******...........*****
// .****************************           ...oOO@@@@/OO/.=@@@@@@@@@@@@@@@@@@@@]\`             .]]/O[[[[**o****************...**..******
// .****************************         ...`=@      .OO/@=@@@@@@@@@@@@@@@@@@@@@@^           .....*****=`**`****`,***,]]@@@/************
// .**********************************....,**..           =OO@@@@@@@@@@@@@@@@@@@@^           ....**`**\**\``*.]]@@@O/[`*,***************
// .**`*************************************..            .OO@@@@@@@@@@@@@@@@@@@@^           .*.**,***,]]@@@@@@*************************
// .****************************************.         . ...@@/@@@@@@/[[O@@O*\@@@@....,`      ..,]/@@@@@@@@@@OoO*************************
// .****************************************.       /^    .]`          /@@@OOoo\.    @@@@@@@@@@@@@@@@@@@@@@@\[\*************************
// .***************************************,        @^ .. .O\\/=..    .\. =@@@O^    ..@@@@@@@@@@@@@@@@@@@@@@\//*************************
// .*******************************************\]]]/@     `@\@@@@@   .OOO@@@@@@.     =/[[[@@@@@@@@@@@@@@[[`*****************************
// .*********************************************[O^     .`=O@@@@@@/]@@@@@@@@@\...,. =**..,****.**=@@@@O*,******************************
// .**`*****************************************`....[OOOO[\OO@@@@@@@@@\@@.//@O.,.=. O............/@@@/*,*******************************
// .**`****o*******^*\***************************[*@@@\[[`..        .,[\\` [O[.....==O.....    .,.@@@@^*,o`*****************************
// .*/**********************************************.[`.             ...`.. O]`.....=^....  .. ...@@OO\\oo******************************
// .**,***********************************************`.       .  ^     ./OOO[O@@\]]@@@@\]]`. ...,//************************************
// .^*********************=***,********************************`........... ......*oOOOOO[[[`*,***//`*****[**,*********o`**************,
// .^*****,********************************************************[[*,]]`*****]`***[[[***]**,[************^****************************
// .****************************\`****************************/*`*******************oo*************************************/************
// .****o**************************************************************************/oo*****//\]`*[***********************[[\]***********
// .**]**************************************************************************=^****/**,*********************************************
// .*,*****************oooo******************************************,[`***[[`*****************************************`****************
// ]]]]]]]]]]]]]]]]]]]]`]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
// @\\@/@//@O/@@=@/^O*OO.\@@@\`@/O@@@O=^@O`@@,\\@/O/=@/`O/\O@O/=@@=,=^//.=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

// ����ͺ���
Motor_t DJI_2006(8192, 36);
Motor_t DJI_6020(8192, 1);
Motor_t DJI_3508_Fric(8192, 1);

// ���ʵ�嶨�� >>>>>>>>>>>>>>>>��Ҫ<<<<<<<<<<<<<
SentryCloud CloudEntity(1, 0x206, 1, 0x205, 2, 0x207, 1, 0x202, 1, 0x203, 1, 0x204);


/// ��̨����ʵ���� ������ɾ������
SentryCloud::SentryCloud(uint8_t yaw_can_num, uint16_t yaw_can_id,
                         uint8_t pitch_can_num, uint16_t pitch_can_id,
						 uint8_t pitch2nd_can_num, uint16_t pitch2nd_can_id,
                         uint8_t fric_l_can_num, uint16_t fric_l_can_id,
                         uint8_t fric_r_can_num, uint16_t fric_r_can_id,
                         uint8_t feed_can_num, uint16_t feed_can_id)
        // ��ʼ������PID����
    : PitchSpeed(-6, 0, -8, 2000, 30000, 10, 10, 500), 
	  PitchPosition(-15, -1, 0, 1800, 10000, 10, 10, 120),//(-15, -3, -40, 1500, 10000, 10, 10, 80)	(-20, -8, 0, 1200, 10000, 10, 10, 80)
      PitchGyroPosition(200, 0, 0, 2000, 10000, 10, 10, 3000),
      PitchGyroSpeed(-10, 0, 0, 2000, 30000, 10, 10, 500),
	  Pitch2ndSpeed(6, 0, 8, 2000, 30000, 10, 10, 500),
	  Pitch2ndPosition(15, 1, 0, 1800, 10000, 10, 10, 120),
	  Pitch2ndGyroPosition(6, 0, 8, 2000, 30000, 10, 10, 500),
	  Pitch2ndGyroSpeed(10, 0, 0, 2000, 30000, 10, 10, 500),
      YawSpeed(20, 0, 0, 2000, 30000, 10, 10, 500),
      YawPosition(10, 1,-0.5, 200, 10000, 10, 2, 100),//10, 0, 0, 2000, 10000, 10, 10, 3000)
      YawGyroSpeed(15, 0, 0, 2000, 30000, 10, 10, 500),
      YawGyroPosition(0, 0, 0, 2000, 10000, 10, 10, 3000),
      FricLeftSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FricRightSpeed(1, 0, 0, 2000, 30000, 10, 10, 500),
      FeedSpeed(20, 0, 1, 1000, 7000),
      FeedPositon(0.5, 0.01, 0, 1000, 20000, 0, 200),

        //!!!!!!!!!!!>>>>>>>>>>>>>Ҫ����˫PITCH�����뵽 dual_pitch_pid_param

        // ��ʼ�����������
	  YawMotor(yaw_can_num, yaw_can_id, 4920, &DJI_6020, &YawSpeed, &YawPosition, &YawGyroSpeed, &YawGyroPosition, &RotatedImuAngleRate[2], &BaseImuAngleRate[2]),      // ��ע��YAW��λ�û�ֱ�Ӳ�ȡ���ǵ����ĳ���
      PitchMotor(pitch_can_num, pitch_can_id, 0, &DJI_6020, &PitchSpeed, &PitchPosition, &PitchGyroSpeed, &PitchGyroPosition, &RotatedImuAngleRate[1], &RotatedImuAngle[1]),
      PitchSecondMotor(pitch2nd_can_num, pitch2nd_can_id, 0, &DJI_6020, &Pitch2ndSpeed, &Pitch2ndPosition, &Pitch2ndGyroSpeed, &Pitch2ndGyroPosition),
	  FricLeftMotor(fric_l_can_num, fric_l_can_id, &DJI_3508_Fric, &FricLeftSpeed),
      FricRightMotor(fric_r_can_num, fric_r_can_id, &DJI_3508_Fric, &FricRightSpeed),
      Feed2nd(feed_can_num, feed_can_id, &DJI_2006, 7, -1, &FeedSpeed, &FeedPositon)
{
	Feed2nd.Enable_Block(4000,200,5);                       // ��ʼ����ת���
	PitchPosition.Custom_Diff = PitchMotor.Gyro_RealSpeed;  // �趨΢����ԴΪ������
	YawPosition.Custom_Diff = YawMotor.Gyro_RealSpeed;      // �趨΢����ԴΪ������
    PitchPosition.pid_run_CallBack = pidPitchCallBack;  // λ�û�PID���û��Զ���ص���������������ǰ��������
    PitchGyroPosition.pid_run_CallBack = pidPitchCallBack;  //λ�û�PID���û��Զ���ص���������������ǰ��������
	copy_cloud_param_to_backup(&PitchMotor);				// ������̨PITCH��������������顣�Դ�����ʱʹ�á�
};









void SentryCloud::Handle()
{
    //------------------------��ȫģʽ������Զ��ر�-------------------------
	if(Mode != save_cloud)
		LazerSwitchCmd(1);  
    else
        LazerSwitchCmd(0);  
	
    // ���������ݴ���
    ImuDataProcessHandle();
    //--------------------------��/˫PITCHģʽ�����߼�--------------------------
    PitchModeCtrl();    
    //--------------------------------��������߼�-----------------------------------
    ShootCtrl();

    // -------------------------------��ģʽ�߼�-------------------------------
    {
        switch (Mode)   // ���� Mode ѡ��ģʽ
        {

            case absolute_gyro_cloud:
            case relative_gyro_cloud:
                CurrentCloudMode = &ModeCloudCtrlGyro;
                break;

            case absolute_cloud:
            case relative_cloud:
            default:
                CurrentCloudMode = &ModeCloudCtrlMech;
                break;

        }

        if(LastCloudMode != CurrentCloudMode)   // ���ģʽ����
        {
            LastCloudMode->Exit();              // ʹ��ģʽ�л��ú���
            CurrentCloudMode->Enter();
        }
        CurrentCloudMode->Run();            // ����ִ��ģʽ���߼�

        LastCloudMode = CurrentCloudMode;   //  ���ģʽ����
    }

//CANSend�������߼�ͳһ����
//    manager::CANSend();


    
}
/**
 * @brief ���������ݴ���
 * 
 */
void SentryCloud::ImuDataProcessHandle()
{
	if(Mode != relative_gyro_cloud && Mode != absolute_gyro_cloud)  //���������ǿ���ģʽ��ʱ�������ǽǶ�ʼ�ո����е�ǽǶȣ�����Ҫ��ת�������ǽǶȣ�
	{
		app_imu_data.integral.Pitch = -CloudEntity.PitchMotor.RealAngle;//ע�⸺�š�
		app_imu_data.integral.Yaw = -CloudEntity.YawMotor.RealAngle;//ע�⸺�š�
	}
	//�����������ǽǶ���ת��ǹ�ڷ��������
	RotatedImuAngle[0] = -app_imu_data.integral.Roll;
    RotatedImuAngle[1] = -app_imu_data.integral.Pitch;
	RotatedImuAngle[2] = -app_imu_data.integral.Yaw;
	//�����������Ǽ��ٶ���ת��ǹ�ڷ��������
	RotatedImuAngleRate[0] = -app_imu_data.Angle_Rate[0];
    RotatedImuAngleRate[1] = -app_imu_data.Angle_Rate[1];
	RotatedImuAngleRate[2] = -app_imu_data.Angle_Rate[2];
    //�����������ǽ��ٶ���ת���������򣬼�Roll,Pitchˮƽ��Yaw��ǹ��Yaw������
    float Cp = cosf(RealPitch), Sp = sinf(RealPitch);

    BaseImuAngleRate[0] = Cp*RotatedImuAngleRate[0] + Sp*RotatedImuAngleRate[2];
    BaseImuAngleRate[1] = RotatedImuAngleRate[1];
    BaseImuAngleRate[2] = -Sp*RotatedImuAngleRate[0] + Cp*RotatedImuAngleRate[2];
    //������̨��Yaw,Pitch�Ƕ�
	RealYaw = YawMotor.RealAngle;   //���ǵ���ĽǶ�
	MechanicYaw = YawMotor.RealPosition*360.f/YawMotor.MotorType->max_mechanical_position;//���ݻ�е�Ǽ��������ʵ�Ƕ�
	RealPitch = - PitchMotor.RealAngle;	//ע�⸺��
}


///�趨��е�ǿ��ƽǶ�
void SentryCloud::SetAngleTo(float pitch, float yaw)
{
	Mode = absolute_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Angle_Set(-TargetPitch);	//ע�⸺��
    YawMotor.Angle_Set(TargetYaw);
}
///�趨�����ǿ��ƽǶ�
void SentryCloud::SetAngleTo_Gyro(float pitch, float yaw)
{
	Mode = absolute_gyro_cloud;
    TargetPitch = pitch;
    TargetYaw = yaw;
    PitchMotor.Gyro_Angle_Set(-TargetPitch);
    YawMotor.Gyro_Angle_Set(TargetYaw);
}


/**
 * @brief �������ڽǶȣ��л��Ƕȿ���λ�û�����Դ���߼���
 * 
 * @param     current_pitch     ���ڵĸ�����
 * @param     current_yaw       ���ڵĺ����
 * @return enum _cloud_ctrl_mode 
 */
static enum _cloud_ctrl_mode decide_mode_by_angle(float current_pitch, float current_yaw)
{
    float mechanic_pitch,mechanic_yaw;

    mechanic_pitch = app_math_fLimitPeriod(current_pitch,180.0f,-180.0f);
    mechanic_yaw = app_math_fLimitPeriod(current_yaw,180.0f,-180.0f);

    // if  // ת��Ϊ������ģʽ��������YAW�����ᶶ���ĽǶ�
    // ( 
    //     IS_IN_INTERVAL(mechanic_yaw,30,40)
    // )
    // {return absolute_gyro_cloud;}

    return absolute_cloud;
}


/**
 * @brief ͨ�õģ��ɵ�ģʽ�Ŀ��ƽǶȡ�����������Ӧѡ�������߼����ѽ����������
 * 
 * @param     pitch ����ĸ�����
 * @param     yaw   ����ĺ����
 * @param     mode  ѡ�еķ���ģʽ��ȡֵ�� @see enum _cloud_ctrl_mode
 */
void SentryCloud::SenAngleTo_Generic(float pitch, float yaw, enum _cloud_ctrl_mode mode)
{
    switch(mode)
	{
		case relative_auto_cloud:	// xx_auto ģʽ�»��Զ��л� ��е/������ ��ȡ��ѿ��Ʒ�ʽ
			pitch	+=	RealPitch;
			yaw		+=	RealYaw;
		case absolute_auto_cloud:
			{

                _cloud_ctrl_mode auto_decided_mode = decide_mode_by_angle(RealPitch, RealYaw);
                
                SenAngleTo_Generic(pitch,yaw,auto_decided_mode);

            }
			break;
		
		case save_cloud:
			Safe_Set();
			break;
			
		case absolute_cloud:
			SetAngleTo(pitch,yaw);
			break;
			
		case relative_cloud:
			SetAngleTo(pitch+RealPitch, yaw+RealYaw);
			break;
			
		case absolute_gyro_cloud:
			SetAngleTo_Gyro(pitch,yaw);
			break;
			
		case relative_gyro_cloud:
			SetAngleTo_Gyro(pitch+RealPitch, yaw+RealYaw);
			
		default:
			Safe_Set();
			break;
	}
}


/**
 * @brief �������
 * 
 * @param     bullet_speed  ���˵��١�Ŀǰ�˲������Ƶ���Ħ���ֵ����ת��
 * @param     fire_freq     ����Ƶ�ʡ�������ơ�
 * @param     Shoot_mode    ���ģʽ������������á�
 */
void SentryCloud::Shoot(float bullet_speed, uint8_t fire_freq, uint8_t Shoot_mode)
{
	Shoot_Speed = bullet_speed;     // �趨Ħ��������

	ShooterSwitchCmd(1);            // ����Ħ���ֺ�������

	if(fire_freq!=0)
    {
        Feed_Free_Once_Set(60000/fire_freq,1);
    }
    else
    {
        Feed_Safe_Set();
    }
    
}


///��ȫģʽ
void SentryCloud::Safe_Set()
{
	Mode = save_cloud;
    YawMotor.Safe_Set();
//    PitchMotor.Safe_Set();
	PitchMotor.Speed_Set(0);
    FricLeftMotor.Safe_Set();
    FricRightMotor.Safe_Set();
    Feed2nd.Safe_Set();
    manager::CANSend();
	LazerSwitchCmd(0);
}


//������������Ϊ������������İ�װ����Ҫfeed_is_permitted==1�������й������
 /**
  * @brief      ����װ��������������
  * @param     FreeSpeed �趨�������ٶ�
  */
void SentryCloud::Feed_Free_Fire_Set(int32_t FreeSpeed){
    if(feed_is_permitted)
    Feed2nd.Free_Fire_Set(FreeSpeed);
}
void SentryCloud::Feed_Burst_Set(uint8_t ShootCnt,int32_t	DiscreDelay,int16_t trig){
    if(feed_is_permitted)
    Feed2nd.Burst_Set(ShootCnt,DiscreDelay,trig);
}
void SentryCloud::Feed_Free_Once_Set(int32_t	DiscreDelay,int16_t trig){
    if(feed_is_permitted)
    Feed2nd.Free_Once_Set(DiscreDelay,trig);
}
void SentryCloud::Feed_Safe_Set(){
    Feed2nd.Safe_Set();
}


///����ƿ���
void SentryCloud::LazerSwitchCmd( int NewState )
{
	if(NewState == 0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	}
}



///�����ɿ��أ�Ħ���ֿ���
void SentryCloud::ShooterSwitchCmd(int NewState )
{
	if(NewState == 0)
	{
		feed_is_permitted=0;    //���������
        FricLeftMotor.Safe_Set();
        FricRightMotor.Safe_Set();
        CloudEntity.Feed2nd.Safe_Set();
	}
	else
	{
		feed_is_permitted=1;
        FricLeftMotor.Speed_Set(-Shoot_Speed);
        FricRightMotor.Speed_Set(Shoot_Speed);
	}
}


/**
 * @brief PITCH���лص�����������ǰ����
 * 
 */
void pidPitchCallBack(pid* self)
{
    self->PIDout+=CloudEntity.gravity_feedforward(CloudEntity.RealPitch);
}








                                                                                                                                                                                                                                                                                 
//                           *** **********]]]]************                ***     ****************                                     
//                           *** **    * **@@@@****     ***                *           ** *=@@@^***                                     
// * * *** ****        ****   ******* *****@@@@@@****************** * ** **** *************=@@@^****  *                                 
//  *  ********        **********=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^***                                     
//    */@@@^**************@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^*****        * *** ******             
//  * *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO@@@@@@@@@@@@@@@@@^************************************
//     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^*,*@@@@@@@@@@@@@@@@@@@@@****************
//     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@****************
//     @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@^**************************************
//     @@@@@@@@@@@@@@[\[[[[[[[[[[[[[[[[[[[\@@@@@@@@@@@@@@@@@@@[\/[[[[[[[[[[[[[[[[[[[[[[[/[[[********************************************
//     @@@@@@@@@@@@@@********************@@@@@@@@@@@@@@@@@@@@@**  *********             ************************************************
//     @@@@@@@@@@@@[`******************@@@@@@@@@@@@@@@@@@@@@@@*****                     ************************************************
// *  *@@@@@@@@@@@@******************=@@@@@@@@@@@***@@@@@@@@@@**************************************************************************
//    *@@@@@@@@@@******************=@@@@@@@@@*********@@@@@@@@@@************************************************************************
//  * *@@@@@@@@^*****************=@@@@@@@@@^**********@@@@@@@@@@,***********************************************************************
//    *\@@@@@^*****  **********=@@@@@@@@@@@^************@@@@@@@@@@**********************************************************************
//     **    ****    ************=@@@@@@@^*********** **@@@@@@@@************************************************************************
//        *          **********************  ************,@@`******      ***************************************************************

//------------------------------------����������-------------------------------------------
/**
 * @brief ��̨�ڲ� ��������߼�
 */
void SentryCloud::ShootCtrl()
{
	// if(REMAIN_HEAT<ONE_SHOT_HEAT)
    // {
    //     feed_is_permitted =0;
    // }


// --------------------------------�����ɿ���----------------------------
    if(feed_is_permitted != 1)   //��������ɣ������ ��ص���ȫ
	{
		Feed2nd.Safe_Set();
	}
    if(!fric_power_permitted != 1)   // �쳵Ħ�������
    {
        FricLeftMotor.Safe_Set();
		FricRightMotor.Safe_Set();
    }

    shoot_flag = Feed2nd.feed_mode; //д�����״̬Ϊ�������״̬

}

/**
 * @brief       ��ȡPID�����Ļص�������������ʵ��
 * 
 * @param     pid_id    PID���
 * @param     p         ��ȡ��д��P�ĵ�ַ
 * @param     i         ��ȡ��д��I�ĵ�ַ
 * @param     d         ��ȡ��д��D�ĵ�ַ
 * @return ״ֵ̬����������HAL_OK, �쳣����HAL_ERROR 
 */
HAL_StatusTypeDef CMD_READ_PID_Rx_GetPidCallback(uint8_t pid_id,float* p,float* i,float* d)
{
    switch (pid_id)
    {
    case __cloud_pitch_position_pid:
        *p = CloudEntity.PitchPosition.P;
		*i = CloudEntity.PitchPosition.I;
		*d = CloudEntity.PitchPosition.D;
        break;
    case __cloud_yaw_position_pid:
        *p = CloudEntity.YawPosition.P;
		*i = CloudEntity.YawPosition.I;
		*d = CloudEntity.YawPosition.D;
        break;
    case __cloud_pitch_speed_pid:
        *p = CloudEntity.PitchSpeed.P;
		*i = CloudEntity.PitchSpeed.I;
		*d = CloudEntity.PitchSpeed.D;
        break;
    case __cloud_yaw_speed_pid:
        *p = CloudEntity.YawSpeed.P;
		*i = CloudEntity.YawSpeed.I;
		*d = CloudEntity.YawSpeed.D;	
        break;
    default:
        break;
    }
	return HAL_OK;
}

/**
 * @brief ʵ�ִ������ �ص�����
 * 
 * @param     bullet_speed  �ӵ������ٶȡ�ʵ������̨Ħ���ֵ��
 * @param     fire_freq     �ӵ����Ƶ�ʣ���λ��RPM��ʵ�����ǰ���̨������ͣʱ���Ϊ 60*1000/fire_freq
 * @param     shoot_mode    �ӵ����ģʽ���������á�
 */
void CMD_SHOOT_ExecuteCallback(float bullet_speed, uint8_t fire_freq, uint8_t shoot_mode){
	CloudEntity.Shoot(bullet_speed,fire_freq,shoot_mode);
}







#undef DEF_CHECKDEVICE_IS_OFFLINE_FUNCION_MOTOR_OBJ
#undef FUNC_NAME





































































 

