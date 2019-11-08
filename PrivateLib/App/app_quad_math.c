/*****************************************************************************
*                                                                            *
*  @file     quad_math.c                                                     *
*  @brief    �����б�                                                        *
*  @details                                                                  *
*  @see      mpu9250.c��ע�����Ŷ��壬�ͳ�ʼ������ʹ�ã�                                                      *
*                                                                            *
*  @author   Nankel  Li                                                      *
*  @email    2512126660@qq.com                                               *                                             
*  @warning  1�����Ҫʹ�ý��ٶȣ�ע�����˲���У�����16λ����               *
             2��ʹ�ã���IMU_Init()��ʼ������IMUSO3Thread()ѭ��ִ��           *
						 3�����ʹ��stm32f1�����ڴ�С�ĵ�Ƭ�������ܻᱨ��ռ䲻�㣬��СZero_Sample_Num���ɣ�һ��1000�Ϳ�����*
*���������������������������ܵ���˵��Ҫʹ�ã������ʼ��IMU_Init()��MPU9250_Init(),����ִ��IMUSO3Thread(),��Ƭѡ��������ΪSPI1_nCS������������������������������������������������
�������������������������������⣬�Լ����ò���У�����������ף���Ȼ���û����Ʈ�Ļ���У������������ֻ�кô���Ȼ������������������������������������������������������������������
������������������������������ʵ9250��YAW��90%�Ŀ���������������̨�ϵģ����Ƕ����ø㣬Ҳ�ֻ���̬�ȡ��������̳�������̨��ֻ�õ��̣�˭��˭���ܣ�������������������������������������
*----------------------------------------------------------------------------*
*  @remark   1��ʹ�õ�ͨ�˲������һЩ����������Ȼ�˲�Ч����������󼸶ȹ���ƽ���������ٶȸ����ϣ��������õĲ���λ��*
             2���Ѿ����Թ�������ʹ�ô����ƽ��������YAW��û���õģ�ֻ��ָ��ŷ���*
						 3������ʹ��soft.Yaw,Ȼ��û�ж���̬У���Ļ�û�����ã���Ȼ������̬У���Ļ����ںϺ�Ľ���������1�����ҵı仯*
����У����0.5�����ң������˲²�������������ȱ��һ��ϵ����У�����������ϵ����Ҫ���ԣ����һ�仯�����¶ȹ�ϵͦ�󣬷������ǲ�����
����Ļ�ע�͵�USE_OFFSET��DYNAMIC_OFFSET����
    ʵ����ο�YAW����ֻ��YAW����У����
		         4��Offset_Coeffϵ��ѡȡ����������USE_OFFSET��Simple_Soft��תһȦ��softΪ360���Ҽ��ɡ�ֻ��soft�ǶȵĻ����Բ��ģ�ֻ��תһȦ����360���ѣ�����ʽ�Ƕȵ�������û��У���ı�Ҫ
						 5����Ԫ��У��PI����Ҳ���Ե�������ֹƵ��Ҳ���Ե���*
*----------------------------------------------------------------------------*
*  @version                                                                  *
*           <Date>      |<Version>| <Author> | <Description>               *
*----------------------------------------------------------------------------*
* 2019��1��24��08:53:30 |   1.0   |  Nankel  | DONE                        *
* 2019��1��30��10:28:12 |   1.1   |  Nankel  | ���ĳɲ�ʹ��USE_OFFSETʱ��ֻ�Խ��ٶȽ���У�����ں��������ݲ�У��*
* 2019��1��31��13:01:38 |   1.2   |  Nankel  | ����soft.Roll��pitch�������Կ����ڵ��̣�@remark�е�3����������,���ʵ����̨���ã�Ҳ����pitch��soft*
* 2019��1��31��13:12:19 |   1.3   |  Nankel  | ��Ϊ9250���Ƕ������ӣ���ȡ����ʼ��������⣬��֤9250��Ӱ���������*
* 2019��1��31��17:18:37 |   1.4   |  Nankel  | ���ӽ��ٶ�ϵ��У����ÿ�����Ӷ���һ����У��Խ׼���ں�Խ�죬�򵥽����ҲԽ׼*
* 2019��2��12��23:54:39 |   1.5   |  Nankel  | �޸����У��������⣬�޸Ĳ����ͽ�ֹƵ��*
* 2019��2��13��21:43:22 |   1.6   |  Nankel  | ���9250���ߣ�imu.ready��Ϊ0������;���Ļ�������жϣ�5s��ʼ�����ɹ���û��У��*
* 2019��4��4��09:27:02  |   1.7   |  Nankel  | ���9250���ߣ�imu.ready��Ϊ0�����⣨�����ж�������ԭ�ȵĲ��ԣ�
* 2019��4��5��16:17:18  |   2.0   |  Nankel  | 1�����Is_Yaw_Invalid���������soft_Yaw�����ã���ò���Ϊ1����������ô˲�����������
*                                              2�������Լ�ʱ�䣬��Ϊ��������ʱ����������5s(ԭ�ȵ�5s�ڳɹ�ֻҪ�ɹ�һ�ξ����¼�ʱ)������5s��Is_Yaw_InvalidΪ1
*                                              3�������Լ�������ʱ�����ƣ��������9250ʱ�ο���ԭʼ���ݣ����ú��ʵ�Zero_Threshold�������(���������ȫ�ֱ��������һ��)�У�����Dbug��ȡoriginal��
*                                              4����������unstable_num��������С��̬���У����ֵDynamic_Zero_Thre��4.0f->2.0f��
* 2019��4��12��09:11:50 |   2.1   |  Nankel  | 1�������˿������˲�ϵ����Ӧ�������ˣ����Ǹо�����Ҫ��
*                                              2����ֹƵ�ʿ��ܸ�����Щ�ߣ����ٶȵͣ�Ƶ�ʵ͵�ʱ����Щ��Ƶ�����������ݵ�����̨���˷�Ӧ�����ܵ���(��������һ���˵�û�����⣬Ӧ���Ǹ������)��
*                                              3�����µ���Dynamic_Zero_Thre��2.0f->4.0f��
* 2019��4��12��19:17:23 |   2.2   |  Nankel  | 1�����Flash�洢ÿ�ε�У��ֵ���ۺϼ�����ʷ���ֵ��������filter���棬�������´��ļ���
*                                              2������Is_Yaw_InvalidΪisThisTimeInvalid����Ϊflash������,���Ի���������Ϊ������Ư,�˲����������β���ʧ�ܣ�������̬У������ʱ��
*                                              3������ȫ�ֺ�����Ƿ�ʹ��Flash������F405����ȡ��Flash(����û�⣬Ӧ�ò��������)
* 2019��5��31��08:30:38 |   2.3   |  Nankel  | 1���Ż����ִ��룬�ı��������ͣ���Լ�ռ䣬��Ҫ���˲��������ͺͲ�����������
*                                              2��ɾ����Ӵ��������ݣ��滻Ϊ��ȡ9250����������(���Զ������������뱣֤EDA��ECL�������գ���Ȼ��·��˵�����ˣ���ʵ�⻹�Ǽ��ϲ��ܶ�����)
*                                              3�����ƶ�̬���£���ֹ50�����ڲſ�ʼ������㣨ԭ��������ɴ��������
*                                              4��ȡ���ɹ�������ʱ��ο�Flash�ڴ洢��ֵ
*                                              5���ſ�̬У����ֵ��5.0f
*                                              6��ȡ��ע��NonlinearSO3AHRSinit����������ʹ�ʼ�ںϵĸ���
* 2019��6��3��20:17:03  |   2.4   |  Nankel  | 1������У��ֵ�޷���Ҳ�������޷�������������������Ŀ����м�extern!!!!!!
*                                              2�����ӣ����㾲ֹ����50�����ڣ��ſ�ʼ�������
*                                              3�����ӣ��Աȶ�̬����ֵ����һ��ɸѡ���ݵĿɿ���
*                                              4�������ֶ�����У��ֵ��ԭ����ԭʼ����������������ɵģ�׷�������Ŀ��Ե�����ĳ��������У׼!!!!
* 2019��6��7��14:19:55  |   2.5   |  Nankel  | 1�����ֶ�У��ֵ�ƶ��������ִ�����У�����ٶ�
* 2019��6��8��21:55:35  |   2.6   |  Nankel  | 1�����������Ƿֱ���(2000dps->500dps),���ٶȼƷֱ���(8g->4g),�����Ұ�Angle_Rate����0.25�����Լ���ԭ���ģ�ʹ����������Ĳ���Ҫ�ٸ�PID
*                                              2���޸��˲��������ͣ�ԭ�����Ǵ���ģ�int16_t->float
*                                              3��ɾ���汾2.5����
* 2019��6��9��20:50:23  |   2.7   |  Nankel  | 1���޸�NonlinearSO3AHRSinit��BUG���ںϽ�������ʱ��������Ҫ�ȴ���ʼ��
* 2019��6��11��10:23:57 |   2.8   |  Nankel  | 1����ԭ��ԭ��������
*                                              2����С���ٶȼƵĵ�ͨ�˲�Ƶ�ʣ������������
*****************************************************************************/
//#include "bsp_config.h"

#include "app_quad_math.h"

float Limit(float data,float max,float min);
/** 
* @brief   �޷����� 
* @remarks min<data<max
*/
static float Limit(float data,float max,float min){
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}


/* �����Ժ����궨�� */
#define micros() 1000*HAL_GetTick()  //��ʱ����λus,��1000000����
#define ABS(x)   ((x)>0?(x):-(x))
//#define USE_MAG         // һ�򿪾ͻ��񵴼��Σ����˲���ͷ�ĸо�һ������ʱδ�����Ӧ�����ڲ�I2C�����˲���Ƶ��
//#define USE_OFFSET      // ��ʼ��ʹ�����У��   ֻҪ�ж���̬У���������Ǳ����ǲ�̫Ư�ģ�����ò��ÿ�Ч����   ���������������ע�͵��Ļ����������Ч����֮ǰ�ģ�����soft����Ӱ�죡��������������������������
#define DYNAMIC_OFFSET    // ʹ�ö�̬У����㣬�������ʹ�ã�imu������Ҫ�������Ӱ��    
//#define MAG_OFFSET      // ������У������ҪΧ�ƿռ���3������ת
//#define USE_KALMAN      // �������˲��͵�ͨ�˲�ֻ�ܴ���һ����ͬʱ�����Ļ�Ĭ��Ϊ��ͨ�˲�
#define USE_LPF           // ʹ�õ�ͨ�˲��� 
#define Simple_Soft       // ʹ�ü򵥽��������YAW������ʹ�ã���Ʈ������YAW�仯�ο�������һֱ���ţ�Ҳ����ʹ��     ���������������ע�͵��Ļ����������ڼ򵥽���soft,ǿ�ҽ��������Զ��ע�͵�������������
/* �����궨�� */
#define Sample_Frequency 1000     //����Ƶ��
#define Gyro_Cut_Off_Frequency 80    //��ֹƵ�ʣ�����Ҫ����������������  !!!!!!!!!!!!!!�����аѽ�ֹƵ�ʸĵ��ر�С�ģ�10���ң�������ʵ����������������ɳ������ʸĴ󡣹���Ļ����˸�Ƶ������������
#define Acce_Cut_Off_Frequency 30
#define SELF_TEST_T  5           //�Լ�ʱ�䣬��λ��
#define g 9.80665f                           //< �������ٶ�
#define TORADIAN   0.0174533f                //< ת��Ϊ�����ƣ���Ԫ������̬������Ҫʹ�� ��/180
#define TOANGLE    57.2957795f               //< ����������Ļ�����ת��Ϊ�Ƕ�
#define ACC_RESOLUTION  (8.0f*g/32768.0f)    //< ���ٶȼƷֱ��� m/s^2/LSb
#define GYRO_RESOLUTION (2000.0f/32768.0f)   //< �����Ƿֱ���   dps/LSb  
/* �������ȫ�ֱ��� */
float so3_comp_params_Kp = 2.0f ;            //< ��Ԫ��У��PI����
float so3_comp_params_Ki = 0.1f; 
uint8_t Zero_Threshold[3] = {100,100,100};      //< �������У�����ж������Ƿ�Ϊ��ֹ����------------------------> 100�㹻���ˣ���ÿ�һ��ԭʼ���ݣ���������������ʵ�е��<--------------------������ģ������Լ�Ҫ��
float  Dynamic_Zero_Thre = 4.0f;                //< ��̬У���������ֵ
float Offset_Coeff[3] = {1.0f,1.0f,1.0f};       //< �����̽���У�������ٶ�У��ϵ��           �������������ڴ˱�׼�����̣���תһ��ֱ�ǣ���ʾ90�ȵ�Ч������������������������������
float manualOffsetGyro[3] = {0,0,0};            //< �ֶ����У��ֵ�������������+-1֮������⣬��Ϊԭʼ������int16_t���ͣ���������ܽ���й��ɵ�Ư��,׷�������Ŀ�������
/* ���������ȫ�ֱ��� */
int16_t Flash_Val[4];                        //< [0]��¼Flashд��Ĵ�����[1-3]Ϊ���������ֵ
uint8_t Flash_Limit_t = 10;                  //< ���ֵ����Ĵ�������ʵ��׼ȷ������ҪĿ�Ļ����������µ����ֵռ�ȸ�һ�㣩
float startOffsetVal[3];
/* �ṹ�� */
MPU_HMC imu;
kalman_filter AccFilter[3];
kalman_filter GyroFilter[3];
LPF2 Acc_LPF[3];
LPF2 Gyro_LPF[3];
#ifdef USE_MAG
#define MAG_RESOLUTION  0.1464129f           //< �����Ʒֱ���   mG/LSb 
LPF2 Mag_LPF[3];
kalman_filter MagFilter[3];
#endif

/** 
* @brief   ��ƽ�����ĵ���
* @remarks ʹ�þ����Carmack�㷨��Ч�ʸߣ���������µ�����
*/
static float invSqrt(float number){
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5f;

    x = number * 0.5f;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

/** 
* @brief   ���ֵ����
* @remarks �������У����������ֵ,ֻ�������ǽ���
*/
uint8_t IMU_Init(void){
	static uint16_t unstable_num;
//#ifdef  STM32F405xx	
//	for(uint8_t i=0;i<4;i++)  // ��ȡflash�洢��ֵ
//	  Flash_Val[i] = (uint16_t)Flash_ReadOut(i);
//	if(Flash_Val[0] < 0 || Flash_Val[0] >10000)  Flash_Val[0] = 0;  //��һ���ճ����ֵ������Ҫ��
//	else if(Flash_Val[0] >= Flash_Limit_t)  Flash_Val[0] = Flash_Limit_t;    //���ƴ洢����Ϊ20�Σ�̫�ൣ�����µ����ֵռ��̫С
//#endif	
#ifdef USE_LPF	
	for(uint8_t i=0;i<3;i++){
		LPF2pSetCutoffFreq(&Acc_LPF[i],Sample_Frequency,Acce_Cut_Off_Frequency);
		LPF2pSetCutoffFreq(&Gyro_LPF[i],Sample_Frequency,Gyro_Cut_Off_Frequency);
	#ifdef USE_MAG		
		LPF2pSetCutoffFreq(&Mag_LPF[i],Sample_Frequency,Cut_Off_Frequency);
	#endif		
	}
#endif
	static uint8_t flag = 1;  ///<������־λ
	static float tick;        ///<���ڳ�ʱ����
	while(flag){
		tick = micros();
		for(uint16_t num=0;num<Zero_Sample_Num;num++){   /*������*/
			imu.original.Gyro[0] = SPI_MPU_Read(MPU6500_GYRO_XOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_XOUT_L);
			imu.original.Gyro[1] = SPI_MPU_Read(MPU6500_GYRO_YOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_YOUT_L);
			imu.original.Gyro[2] = SPI_MPU_Read(MPU6500_GYRO_ZOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_ZOUT_L);		
//			while(ABS(imu.original.Gyro[0])>Zero_Threshold[0] || ABS(imu.original.Gyro[1])>Zero_Threshold[1] || ABS(imu.original.Gyro[2])>Zero_Threshold[2])  {
//				imu.original.Gyro[0] = SPI_MPU_Read(MPU6500_GYRO_XOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_XOUT_L);
//				imu.original.Gyro[1] = SPI_MPU_Read(MPU6500_GYRO_YOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_YOUT_L);
//				imu.original.Gyro[2] = SPI_MPU_Read(MPU6500_GYRO_ZOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_ZOUT_L);
//				unstable_num++;
	      if((micros() - tick)/1000000.0f > SELF_TEST_T) {
					for(uint8_t j=0;j<3;j++){
					  imu.offset.Gyro[j] = Flash_Val[j+1]/300.0f;
					  imu.isThisTimeInvalid[j] = 1;
						imu.offset.Cnt[j] = 0;      // �������
					}
					return 0;   //5s�ڳ�ʼ�����ɹ���ʹ��Flash�ڴ����ʷ���ֵ
				}
//			}
			for(uint8_t k=0;k<3;k++){
				imu.offset.Data[k][imu.offset.Cnt[k]] = imu.original.Gyro[k]; //<������ֵ
				imu.offset.Sum[k] += imu.offset.Data[k][imu.offset.Cnt[k]];   //<��������
				imu.offset.Cnt[k]++;                                          //<��������
			}	
		}	
		if (unstable_num > 300){   /*����������Ч*/  
			unstable_num = 0;
			for(uint8_t i=0;i<3;i++){
				imu.offset.Sum[i] = 0;
				imu.offset.Cnt[i] = 0;
			}					 
		}
		else{
			for(uint8_t i=0;i<3;i++){  /*����������Ч*/
				startOffsetVal[i] = imu.offset.Gyro[i] = imu.offset.Sum[i]/Zero_Sample_Num;
				Flash_Val[i+1] = (int16_t)imu.offset.Gyro[i]*300;  //����flsh��ֵ����300��Ŀ��������ֵ��׼һ�㣬�Ͼ���floatת����int16_t��������			
				imu.offset.Cnt[i] = 0;
			}
			Flash_Val[0]++;
#ifdef  STM32F405xx			
			Flash_WriteIn(0,Flash_Val,4);
#endif			
			flag = 0;
		}
	} 
  return 1;	
}

/** 
* @brief   ��ȡԭʼ���ݺ͵�λ����
* @remarks 
*/
#ifdef USE_MAG
int16_t Mag_max[2];  //ƽ��У׼����������ֻУ��x,��y
int16_t Mag_min[2];
#endif
static void MPU_Read_Raw(void)
{
	static uint8_t dynamicFlag[3];
#ifdef USE_MAG	
	static uint8_t akm_data[6];	
#endif	
	static uint8_t mpu_data_buf[14];
	
	/* ��ȡ���ٶȼ�&������ */
	SPI_MPU_Read_Regs(MPU6500_ACCEL_XOUT_H,mpu_data_buf,14);      
	imu.original.Accel[0] = (mpu_data_buf[0]<<8 | mpu_data_buf[1]); 
	imu.original.Accel[1] = (mpu_data_buf[2]<<8 | mpu_data_buf[3]);
	imu.original.Accel[2] = (mpu_data_buf[4]<<8 | mpu_data_buf[5]);
	imu.original.MPU_Temp = (mpu_data_buf[6]<<8 | mpu_data_buf[7]);
	imu.unitized.MPU_Temp = imu.original.MPU_Temp/333.87f + 21;
	imu.original.Gyro[0] = (mpu_data_buf[8]<<8  | mpu_data_buf[9]);  
	imu.original.Gyro[1] = (mpu_data_buf[10]<<8 | mpu_data_buf[11]);
	imu.original.Gyro[2] = (mpu_data_buf[12]<<8 | mpu_data_buf[13]);
#ifdef USE_MAG	
	/* ��ȡ������ */
	Mag_Read(akm_data);
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;	����У׼�Ĺ�ʽ
	for(uint8_t i=0;i<3;i++)
		imu.original.Mag[i] = (akm_data[i*2+1]<<8 | akm_data[i*2]);	
#ifdef MAG_OFFSET  
	for(uint8_t i=0;i<2;i++){  //ˮƽУ��������
		Mag_max[i] = imu.original.Mag[i]>Mag_max[i]?imu.original.Mag[i]:Mag_max[i];
		Mag_min[i] = imu.original.Mag[i]<Mag_min[i]?imu.original.Mag[i]:Mag_min[i];
		imu.offset.Mag[i] = (float)(Mag_max[i] + Mag_min[i])/2;
	}
#endif	
#endif	
	for(uint8_t i=0;i<3;i++){	
		/* ���п������˲� */
#ifdef USE_KALMAN		
		imu.kalman.Accel[i] = Kalman(&AccFilter[i],(float)imu.original.Accel[i]);
    imu.kalman.Gyro[i] = Kalman(&GyroFilter[i],(float)(imu.original.Gyro[i]));	
    		/* ȡ���ٶ� */
		imu.Angle_Rate[i] = (float)(imu.kalman.Gyro[i]  - imu.offset.Gyro[i])*Offset_Coeff[i];  //dps		
		/* ��λ�� */
#ifdef USE_OFFSET
		imu.unitized.Gyro[i] = imu.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;              //rad/s 		
#else
		imu.unitized.Gyro[i] = imu.kalman.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif		
		imu.unitized.Accel[i] = imu.kalman.Accel[i]*ACC_RESOLUTION;     //m/s^2  

#ifdef USE_MAG		
		imu.kalman.Mag[i] = Kalman(&MagFilter[i],imu.original.Mag[i]);		
		imu.unitized.Mag[i] = (float)imu.kalman.Mag[i] * ((AK8963_ASA[i]-128)/256.0f+1.0f); //uT	
#endif		
#endif

#ifdef USE_LPF
		imu.LPF.Accel[i] = LPF2pApply(&Acc_LPF[i],(float)imu.original.Accel[i]);
    imu.LPF.Gyro[i] = LPF2pApply(&Gyro_LPF[i],(float)imu.original.Gyro[i]);
		/* ȡ���ٶ� */
    imu.Angle_Rate[i] = (float)(imu.LPF.Gyro[i] - imu.offset.Gyro[i] + manualOffsetGyro[i])*Offset_Coeff[i];  //16λ���̣�ֻ��ԭʼ���ݽ��в������˲�����,��Ҫ����9250��		
    /* ��λ�� */		
#ifdef 	USE_OFFSET
    imu.unitized.Gyro[i] = imu.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#else
    imu.unitized.Gyro[i] = (float)imu.LPF.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif	
		imu.unitized.Accel[i] = (float)imu.LPF.Accel[i]*ACC_RESOLUTION;     //m/s^2  
#ifdef USE_MAG	
		imu.LPF.Mag[i] = LPF2pApply(&Mag_LPF[i],imu.original.Mag[i]);
		imu.unitized.Mag[i] = (float)imu.LPF.Mag[i] * ((AK8963_ASA[i]-128)/256.0f+1.0f); //uT
#endif		
#endif	
#ifdef DYNAMIC_OFFSET		
		/*��ֹʱ���������*/
		if (ABS(imu.Angle_Rate[i]) < Dynamic_Zero_Thre && ABS(imu.offset.Gyro[i] - imu.original.lastGyro[i]) < 3){
			dynamicFlag[i]++;
			if(dynamicFlag[i] >= 200)  dynamicFlag[i] = 200; //��λ
	  }
		else
			dynamicFlag[i] = 0;
    if(dynamicFlag[i] >= 50){  //����50�����ڿ�ʼ�������
			imu.offset.Sum[i] -= imu.offset.Data[i][imu.offset.Cnt[i]];    //< ���������
			imu.offset.Data[i][imu.offset.Cnt[i]] = imu.original.Gyro[i];  //< ��������
			imu.offset.Sum[i] += imu.offset.Data[i][imu.offset.Cnt[i]];    //< ����������
			if(imu.isThisTimeInvalid[i] == 0) /* ����ʱ�����ɹ� */
			  imu.offset.Gyro[i] = Limit(imu.offset.Sum[i] / Zero_Sample_Num,startOffsetVal[i]+0.5f,startOffsetVal[i]-0.5f);    //< �������
			imu.offset.Cnt[i]++;
			if(imu.offset.Cnt[i] == Zero_Sample_Num){
				imu.offset.Cnt[i] = 0;
				imu.isThisTimeInvalid[i] = 0;  /* ������ʱû�г�ʼ���ɹ����ȴ���̬�����ɹ������ö�̬У��ֵ */
			}
      imu.Angle_Rate[i] = 0;	
		}			
#endif 
    imu.original.lastGyro[i] = imu.original.Gyro[i];			
	}
}

/***************************��Ԫ�����㲿��*********************************/
/* ���������ȫ�ֱ��� */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f,q3 = 0.0f;  /** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f,dq3 = 0.0f;  /** quaternion of sensor frame relative to auxiliary frame */
static float q0q0, q0q1, q0q2, q0q3,q1q1, q1q2, q1q3,q2q2, q2q3,q3q3;
static float gyro_bias[3];  
static uint8_t bFilterInit;
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx,
                                 float my, float mz)        //��ʵ���������ûʲô�õģ�������������ü��ٶȼƺʹ��������ƫ���ǣ��������Ԫ��������һ�㲻�ô����ƣ�����ע�ͽ��궨����
{
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;	
	
	initialRoll = 3.1415f + atan2(-ay, -az);
	initialPitch = - atan2(ax, -az);

	cosRoll = cosf(initialRoll);
	sinRoll = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);
	if(mx == 0 || mz == 0){
		mx=1;  
    mz=1;
	}
	magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
	magY = my * cosRoll - mz * sinRoll;

	initialHdg = atan2f(-magY, magX);

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);

	q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}
/** 
* @brief   Mahony�㷨
* @remarks 
*/
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax,
                                   float ay, float az, float mx, float my, float mz, float twoKp, float twoKi,
                                   float dt)
{
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float recipNorm;
    // Make filter converge to initial solution faster
    if (bFilterInit == 0){
			NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz);
			bFilterInit = 1;
    }
    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
    {
			float hx, hy, hz, bx, bz;
			float halfwx, halfwy, halfwz;
			
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
			// Reference direction of Earth's magnetic field
			hx = 2.0f*(mx*(0.5f-q2q2-q3q3)+my*(q1q2-q0q3)+mz*(q1q3+q0q2));
			hy = 2.0f*(mx*(q1q2+q0q3)+my*(0.5f-q1q1-q3q3)+mz*(q2q3 - q0q1));
			hz = 2.0f*mx*(q1q3-q0q2)+2.0f*my*(q2q3+q0q1)+2.0f*mz*(0.5f-q1q1-q2q2);
			bx = sqrt(hx*hx+hy*hy);
			bz = hz;
			// Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (my * halfwz - mz * halfwy);
			halfey += (mz * halfwx - mx * halfwz);
			halfez += (mx * halfwy - my * halfwx);
    }
	
    //����һ��������  ���ٶȵ�ģ����G��Զʱ�� 0.75*G < normAcc < 1.25*G
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
			float halfvx, halfvy, halfvz;
			
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			if((1/(0.75f*9.8f))>recipNorm>(1/(1.25f*9.8f))){
				ax *= recipNorm;
				ay *= recipNorm;
				az *= recipNorm;
				// Estimated direction of gravity and magnetic field
				halfvx = q1q3 - q0q2;
				halfvy = q0q1 + q2q3;
				halfvz = q0q0 - 0.5f + q3q3;
				// Error is sum of cross product between estimated direction and measured direction of field vectors
				halfex += ay * halfvz - az * halfvy;
				halfey += az * halfvx - ax * halfvz;
				halfez += ax * halfvy - ay * halfvx;
			}
    }
    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f){
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f){
            gyro_bias[0] += twoKi * halfex * dt;    // integral error scaled by Ki
            gyro_bias[1] += twoKi * halfey * dt;
            gyro_bias[2] += twoKi * halfez * dt;
            // apply integral feedback
            gx += gyro_bias[0];
            gy += gyro_bias[1];
            gz += gyro_bias[2];
        }
        else{
            gyro_bias[0] = 0.0f;    // prevent integral windup
            gyro_bias[1] = 0.0f;
            gyro_bias[2] = 0.0f;
        }
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
    //! q_k = q_{k-1} + dt*\dot{q}
    //! \dot{q} = 0.5*q \otimes P(\omega)
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    q0 += dt * dq0;
    q1 += dt * dq1;
    q2 += dt * dq2;
    q3 += dt * dq3;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

#ifndef Simple_Soft
/** 
* @brief    SoftYaw
* @remarks  û�б߽��Yawֵ
* @par ��־
*/
static void Soft_Yaw(float Yaw)
{
	static uint8_t startflag;
	static float LastYaw;
	if(startflag == 0)
	{
		imu.soft.StartYaw = Yaw;
		startflag = 1;
	}
	if((Yaw - LastYaw) > 180) 
		imu.soft.YawCircle--;
	else if((Yaw - LastYaw) < -180)
		imu.soft.YawCircle++;
	imu.soft.Yaw = imu.soft.YawCircle*360 + Yaw - imu.soft.StartYaw;
	LastYaw = Yaw;
}
#endif
/** 
* @brief   ��̬����
* @remarks 
*/
uint32_t tPrev,tNow; 
void IMUSO3Thread(void)
{   
    float euler[3] = {0,0,0};            //rad  
    float Rot_matrix[9] = {1.0f,  0.0f,  0.0f, 0.0f,  1.0f,  0.0f, 0.0f,  0.0f,  1.0f };       /**< init: identity matrix */
    /* �������ν���ʱ���� */
    tNow = micros();
    float dt = (tPrev > 0) ? (tNow - tPrev) / 1000000.0f : 0;
    tPrev = tNow;
    if(dt == 0)  return;
    /* ��ȡ����(�Ѿ��˲���У������λ��) */
    MPU_Read_Raw();
    /* ��Ԫ����̬�ں� */  
    #ifdef USE_MAG		
    NonlinearSO3AHRSupdate(imu.unitized.Gyro[0],imu.unitized.Gyro[1],imu.unitized.Gyro[2],
													 imu.unitized.Accel[0],imu.unitized.Accel[1],imu.unitized.Accel[2],
													 imu.unitized.Mag[0],imu.unitized.Mag[1],imu.unitized.Mag[2],so3_comp_params_Kp,so3_comp_params_Ki,dt);
    #else
    NonlinearSO3AHRSupdate(imu.unitized.Gyro[0],imu.unitized.Gyro[1],imu.unitized.Gyro[2],
													 imu.unitized.Accel[0],imu.unitized.Accel[1],imu.unitized.Accel[2],
													 0,0,0,so3_comp_params_Kp,so3_comp_params_Ki,dt);		
		#endif

    /* ת���ɷ������Ҿ��� */
    // Convert q->R, This R converts inertial frame to body frame.
    Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
    Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3); // 12
    Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2); // 13
//    Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3); // 21
//    Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
    Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1); // 23
//    Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2); // 31
//    Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1); // 32
    Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33 
    /* ת����ŷ���� */
    euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);    //! Roll
    euler[1] = -asinf(Rot_matrix[2]);                   //! Pitch
    euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);    //��Yaw
    /* �ó���̬�� */
    imu.Roll = euler[0] * TOANGLE;
    imu.Pitch = euler[1] * TOANGLE;	
    imu.Yaw = -euler[2] * TOANGLE;
#ifdef Simple_Soft  /* �򵥻��ֽ��� */
		imu.soft.Roll += imu.Angle_Rate[0]*GYRO_RESOLUTION*dt;
		imu.soft.Pitch += imu.Angle_Rate[1]*GYRO_RESOLUTION*dt;
    imu.soft.Yaw += imu.Angle_Rate[2]*GYRO_RESOLUTION*dt;
#else	
		Soft_Yaw(imu.Yaw);	
#endif
    if(ABS(imu.original.Gyro[0]) <= 1 && ABS(imu.original.Gyro[1]) <= 1 && ABS(imu.original.Gyro[2]) <= 1 &&
			 ABS(imu.original.Accel[0] <= 1) && ABS(imu.original.Accel[1] <= 1) && ABS(imu.original.Accel[2] <= 1))
			imu.ready = 0;
		else  imu.ready = 1;
}
