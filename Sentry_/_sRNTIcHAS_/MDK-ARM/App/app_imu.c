/** 
* @brief        ʹ��icm20602��Ϊapp_imu_data
* @details  
* @author      Asn
* @date     2019��11��17��
* @version  
* @par Copyright (c):  RM2020���
*  
* @par ��־
* 2019��11��17��  1.0  Asn  1.��ֲ�Ͻ�mpu9250�㷨��icm20602��(��ʵ�����ֲ�ˣ���û�ϴ���v��)
*														2.�޸���mahony�㷨�е�PI����������������Ӧicm20602оƬ
*/  
#include "app_imu.h"
#include "app_math.h"
#include "bsp_spi.h"
 
#define USE_LPF           //ʹ�õ�ͨ�˲�
#define MAG_OFFSET
#define USE_OFFSET				//ʹ�����У��
#define DYNAMIC_OFFSET    //ʹ�ö�̬У����㣬�������ʹ�ã�imu������Ҫ�������Ӱ��
// ������ı���,�������ʹ��static 
#ifdef USE_MAG
float so3_comp_params_Kp = 2.0f ;            //< ��Ԫ��У��PI����
float so3_comp_params_Ki = 0.05f; 
float so3_comp_params_mKp = 1.6f; 
#else
float so3_comp_params_Kp = 3.0f ;            //< ��Ԫ��У��PI����
float so3_comp_params_Ki = 0.05f; 
#endif
uint16_t Zero_Threshold[3] = {100,100,100};  //< �������У�����ж������Ƿ�Ϊ��ֹ����------------------------> 100�㹻���ˣ���ÿ�һ��ԭʼ���ݣ���������������ʵ�е��<--------------------������ģ������Լ�Ҫ��
float  Dynamic_Zero_Thre = 4.0f;             //< ��̬У���������ֵ
float Offset_Coeff[3] = {1.0f,1.0f,1.0f};    //< �����̽���У�������ٶ�У��ϵ��           �������������ڴ˱�׼�����̣���תһ��ֱ�ǣ���ʾ90�ȵ�Ч������������������������������
float manualOffsetGyro[3] = {0,0,0};         //< �ֶ����У��ֵ�������������+-1֮������⣬��Ϊԭʼ������int16_t���ͣ���������ܽ���й��ɵ�Ư��,׷�������Ŀ�������

static int16_t Flash_Val[4];                        //< [0]��¼Flashд��Ĵ�����[1-3]Ϊ���������ֵ
static float app_imu_OffsetVal[3];
/* �����Ժ����궨�� */
#define MICROS() 1000*HAL_GetTick()  //��ʱ����λus,��1000000����


/* �����궨�� */
#define SAMPLE_FREQUENCY 1000     //����Ƶ��
#define GYRO_CUT_OFF_FREQUENCY 40    //��ֹƵ�ʣ�����Ҫ����������������  !!!!!!!!!!!!!!�����аѽ�ֹƵ�ʸĵ��ر�С�ģ�10���ң�������ʵ����������������ɳ������ʸĴ󡣹���Ļ����˸�Ƶ������������
#define ACCE_CUT_OFF_FREQUENCY 5
#define MAG_CUT_OFF_FREQUENCY  20
#define SELF_TEST_T  5           //�Լ�ʱ�䣬��λ��
#define G 9.80665f                           //< �������ٶ�
#define TORADIAN   0.0174533f                //< ת��Ϊ�����ƣ���Ԫ������̬������Ҫʹ�� ��/180
#define TOANGLE    57.2957795f               //< ����������Ļ�����ת��Ϊ�Ƕ�
#define ACC_RESOLUTION  (2.0f*G/32768.0f)    //< ���ٶȼƷֱ��� m/s^2/LSb
#define GYRO_RESOLUTION (2000.0f/32768.0f)   //< �����Ƿֱ���   dps/LSb 
#define MAG_RESOLUTION  0.3f*0.001f          //< �����Ʒֱ���   uT/LSb
/* �ṹ�� */
MPU_DEF app_imu_data;

LPF2 Acc_LPF[3];
LPF2 Gyro_LPF[3];
#ifdef USE_MAG
LPF2 Mag_LPF[3];
#endif

/** 
* @brief   ���ֵ����
* @remarks �������У����������ֵ,ֻ�������ǽ���
*/
uint8_t app_imu_Init(void){
	static uint16_t unstable_num;
	app_imu_data.reset = 1;
#ifdef USE_LPF	
	for(uint8_t i=0;i<3;i++){
		app_math_Lpf2set(&Acc_LPF[i],SAMPLE_FREQUENCY,ACCE_CUT_OFF_FREQUENCY);
		app_math_Lpf2set(&Gyro_LPF[i],SAMPLE_FREQUENCY,GYRO_CUT_OFF_FREQUENCY);
	#ifdef USE_MAG		
		app_math_Lpf2set(&Mag_LPF[i],SAMPLE_FREQUENCY,MAG_CUT_OFF_FREQUENCY);
	#endif		
	}
#endif
	static uint8_t flag = 1;  ///<������־λ
	static float tick;        ///<���ڳ�ʱ����
	while(flag){
		tick = MICROS();
		for(uint16_t num=0;num<ZERO_SAMPLE_NUM;num++){   /*������*/
			app_imu_data.original.Gyro[0] = bsp_spi_ReadReg(MPUREG_GYRO_XOUT_H)<<8|bsp_spi_ReadReg(MPUREG_GYRO_XOUT_L);
			app_imu_data.original.Gyro[1] = bsp_spi_ReadReg(MPUREG_GYRO_YOUT_H)<<8|bsp_spi_ReadReg(MPUREG_GYRO_YOUT_L);
			app_imu_data.original.Gyro[2] = bsp_spi_ReadReg(MPUREG_GYRO_ZOUT_H)<<8|bsp_spi_ReadReg(MPUREG_GYRO_ZOUT_L);		
			while(APP_MATH_ABS(app_imu_data.original.Gyro[0])>Zero_Threshold[0] || APP_MATH_ABS(app_imu_data.original.Gyro[1])>Zero_Threshold[1] || APP_MATH_ABS(app_imu_data.original.Gyro[2])>Zero_Threshold[2])  {
				app_imu_data.original.Gyro[0] = bsp_spi_ReadReg(MPUREG_GYRO_XOUT_H)<<8|bsp_spi_ReadReg(MPUREG_GYRO_XOUT_L);
				app_imu_data.original.Gyro[1] = bsp_spi_ReadReg(MPUREG_GYRO_YOUT_H)<<8|bsp_spi_ReadReg(MPUREG_GYRO_YOUT_L);
				app_imu_data.original.Gyro[2] = bsp_spi_ReadReg(MPUREG_GYRO_ZOUT_H)<<8|bsp_spi_ReadReg(MPUREG_GYRO_ZOUT_L);
				unstable_num++;
	      if((MICROS() - tick)/1000000.0f > SELF_TEST_T) {
					for(uint8_t j=0;j<3;j++){
					  app_imu_data.offset.Gyro[j] = Flash_Val[j+1]/300.0f;
					  app_imu_data.isThisTimeInvalid[j] = 1;
						app_imu_data.offset.Cnt[j] = 0;      // �������
					}
					return 0;   //5s�ڳ�ʼ�����ɹ���ʹ��Flash�ڴ����ʷ���ֵ
				}
			}
			for(uint8_t k=0;k<3;k++){
				app_imu_data.offset.Data[k][app_imu_data.offset.Cnt[k]] = app_imu_data.original.Gyro[k]; //<������ֵ
				app_imu_data.offset.Sum[k] += app_imu_data.offset.Data[k][app_imu_data.offset.Cnt[k]];   //<��������
				app_imu_data.offset.Cnt[k]++;                                          //<��������
			}	
		}	
		if (unstable_num > 300){   /*����������Ч*/  
			unstable_num = 0;
			for(uint8_t i=0;i<3;i++){
				app_imu_data.offset.Sum[i] = 0;
				app_imu_data.offset.Cnt[i] = 0;
			}					 
		}
		else{
			for(uint8_t i=0;i<3;i++){  /*����������Ч*/
				app_imu_OffsetVal[i] = app_imu_data.offset.Gyro[i] = app_imu_data.offset.Sum[i]/ZERO_SAMPLE_NUM;
				Flash_Val[i+1] = (int16_t)app_imu_data.offset.Gyro[i]*300;  //����flash��ֵ����300��Ŀ��������ֵ��׼һ�㣬�Ͼ���floatת����int16_t��������			
				app_imu_data.offset.Cnt[i] = 0;
			}
			Flash_Val[0]++;		
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
	bsp_spi_ReadRegs(MPUREG_ACCEL_XOUT_H,mpu_data_buf,14);      
	app_imu_data.original.Accel[0] = (mpu_data_buf[0]<<8 | mpu_data_buf[1]); 
	app_imu_data.original.Accel[1] = (mpu_data_buf[2]<<8 | mpu_data_buf[3]);
	app_imu_data.original.Accel[2] = (mpu_data_buf[4]<<8 | mpu_data_buf[5]);
	app_imu_data.original.MPU_Temp = (mpu_data_buf[6]<<8 | mpu_data_buf[7]);
	app_imu_data.unitized.MPU_Temp = app_imu_data.original.MPU_Temp/333.87f + 21;
	app_imu_data.original.Gyro[0] = (mpu_data_buf[8]<<8  | mpu_data_buf[9]);  
	app_imu_data.original.Gyro[1] = (mpu_data_buf[10]<<8 | mpu_data_buf[11]);
	app_imu_data.original.Gyro[2] = (mpu_data_buf[12]<<8 | mpu_data_buf[13]);
#ifdef USE_MAG	
	/* ��ȡ������ */
	bsp_spi_MagReads(AK8975_HXL_REG,akm_data,6);
	bsp_spi_MagTrig();
	bsp_spi_MagReads(AK8975_ASAX_REG,bsp_spi_MagAsa,3);
	bsp_spi_MagTrig();
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;	����У׼�Ĺ�ʽ
	for(uint8_t i=0;i<3;i++)
		app_imu_data.original.Mag[i] = (akm_data[i*2+1]<<8 | akm_data[i*2]);	
#ifdef MAG_OFFSET  
	for(uint8_t i=0;i<2;i++){  //ˮƽУ��������
		Mag_max[i] = app_imu_data.original.Mag[i]>Mag_max[i]?app_imu_data.original.Mag[i]:Mag_max[i];
		Mag_min[i] = app_imu_data.original.Mag[i]<Mag_min[i]?app_imu_data.original.Mag[i]:Mag_min[i];
		app_imu_data.offset.Mag[i] = (float)(Mag_max[i] + Mag_min[i])/2;
	}
#endif
#endif	
	for(uint8_t i=0;i<3;i++){	
		/* ���п������˲� */
#ifdef USE_KALMAN		
		app_imu_data.kalman.Accel[i] = Kalman(&AccFilter[i],(float)app_imu_data.original.Accel[i]);
    app_imu_data.kalman.Gyro[i] = Kalman(&GyroFilter[i],(float)(app_imu_data.original.Gyro[i]));	
    		/* ȡ���ٶ� */
		app_imu_data.Angle_Rate[i] = (float)(app_imu_data.kalman.Gyro[i]  - app_imu_data.offset.Gyro[i])*Offset_Coeff[i];  //dps		
		/* ��λ�� */
#ifdef USE_OFFSET
		app_imu_data.unitized.Gyro[i] = app_imu_data.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;              //rad/s 		
#else
		app_imu_data.unitized.Gyro[i] = app_imu_data.kalman.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif		
		app_imu_data.unitized.Accel[i] = app_imu_data.kalman.Accel[i]*ACC_RESOLUTION;     //m/s^2  

#ifdef USE_MAG		
		app_imu_data.kalman.Mag[i] = Kalman(&MagFilter[i],app_imu_data.original.Mag[i]);		
		app_imu_data.unitized.Mag[i] = (float)app_imu_data.kalman.Mag[i] * ((bsp_spi_MagAsa[i]-128)/256.0f+1.0f); //uT	
#endif		
#endif

#ifdef USE_LPF
		app_imu_data.LPF.Accel[i] = app_math_Lpf2apply(&Acc_LPF[i],(float)app_imu_data.original.Accel[i]);
    app_imu_data.LPF.Gyro[i] = app_math_Lpf2apply(&Gyro_LPF[i],(float)app_imu_data.original.Gyro[i]);
		/* ȡ���ٶ� */
    app_imu_data.Angle_Rate[i] = (float)(app_imu_data.LPF.Gyro[i] - app_imu_data.offset.Gyro[i] + manualOffsetGyro[i])*Offset_Coeff[i];  //16λ���̣�ֻ��ԭʼ���ݽ��в������˲�����,��Ҫ����9250��		
    /* ��λ�� */		
#ifdef 	USE_OFFSET  
    app_imu_data.unitized.Gyro[i] = app_imu_data.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#else
    app_imu_data.unitized.Gyro[i] = (float)app_imu_data.LPF.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif	
		app_imu_data.unitized.Accel[i] = (float)app_imu_data.LPF.Accel[i]*ACC_RESOLUTION;     //m/s^2  
#ifdef USE_MAG	
		app_imu_data.LPF.Mag[i] = app_math_Lpf2apply(&Mag_LPF[i],app_imu_data.original.Mag[i]);
		if(bsp_spi_MagAsa[0]<=254&&bsp_spi_MagAsa[1]<=254&&bsp_spi_MagAsa[2]<=254)
		{
			app_imu_data.unitized.Mag[i] = (float)app_imu_data.LPF.Mag[i] * ((bsp_spi_MagAsa[i]-128)/256.0f+1.0f); //uT
		}
#endif		
#endif	
#ifdef DYNAMIC_OFFSET		
		/*��ֹʱ���������*/
		if (APP_MATH_ABS(app_imu_data.Angle_Rate[i]) < Dynamic_Zero_Thre && APP_MATH_ABS(app_imu_data.offset.Gyro[i] - app_imu_data.original.lastGyro[i]) < 3){
			dynamicFlag[i]++;
			if(dynamicFlag[i] >= 200)  dynamicFlag[i] = 200; //��λ
	  }
		else
			dynamicFlag[i] = 0;
    if(dynamicFlag[i] >= 50){  //����50�����ڿ�ʼ�������
			app_imu_data.offset.Sum[i] -= app_imu_data.offset.Data[i][app_imu_data.offset.Cnt[i]];    //< ���������
			app_imu_data.offset.Data[i][app_imu_data.offset.Cnt[i]] = app_imu_data.original.Gyro[i];  //< ��������
			app_imu_data.offset.Sum[i] += app_imu_data.offset.Data[i][app_imu_data.offset.Cnt[i]];    //< ����������
			if(app_imu_data.isThisTimeInvalid[i] == 0) /* ����ʱ�����ɹ� */
			  app_imu_data.offset.Gyro[i] = app_math_Limit(app_imu_data.offset.Sum[i] / ZERO_SAMPLE_NUM,app_imu_OffsetVal[i]+0.5f,app_imu_OffsetVal[i]-0.5f);    //< �������
			app_imu_data.offset.Cnt[i]++;
			if(app_imu_data.offset.Cnt[i] == ZERO_SAMPLE_NUM){
				app_imu_data.offset.Cnt[i] = 0;
				app_imu_data.isThisTimeInvalid[i] = 0;  /* ������ʱû�г�ʼ���ɹ����ȴ���̬�����ɹ������ö�̬У��ֵ */
			}
      app_imu_data.Angle_Rate[i] = 0;	
		}			
#endif 
    app_imu_data.original.lastGyro[i] = app_imu_data.original.Gyro[i];			
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
float kp_use = 0.0f,ki_use = 0.0f,mkp_use = 0.0f;
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax,
                                   float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float twomKp,
                                   float dt)
{
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float recipNorm;
		static uint8_t reset_cnt = 0;
    // Make filter converge to initial solution faster
    if (bFilterInit == 0){
			NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz);
			bFilterInit = 1;
    }
    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
    {
			float hx, hy, hz, bx, bz;
			float halfwx, halfwy, halfwz;
			
			recipNorm = app_math_Invsqrt(mx * mx + my * my + mz * mz);
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
			halfex += mkp_use*(my * halfwz - mz * halfwy);
			halfey += mkp_use*(mz * halfwx - mx * halfwz);
			halfez += mkp_use*(mx * halfwy - my * halfwx);
    }
	
    //����һ��������  ���ٶȵ�ģ����G��Զʱ�� 0.75*G < normAcc < 1.25*G
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
			float halfvx, halfvy, halfvz;
			
			recipNorm = app_math_Invsqrt(ax * ax + ay * ay + az * az);
			//if((1/(0.75f*9.8f))>recipNorm>(1/(1.25f*9.8f))){
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
			//}
    }
    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f){
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f){
            gyro_bias[0] += ki_use * halfex * dt;    // integral error scaled by Ki
            gyro_bias[1] += ki_use * halfey * dt;
            gyro_bias[2] += ki_use * halfez * dt;
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
        gx += kp_use * halfex;
        gy += kp_use * halfey;
        gz += kp_use * halfez;
    }
#ifdef USE_MAG
		if(app_imu_data.reset == 1)
		{
			kp_use = 50.0f;
			ki_use = 0.0f;
			mkp_use = 5.0f;
			if((APP_MATH_ABS(halfex)+APP_MATH_ABS(halfey)+APP_MATH_ABS(halfez)<0.30f))
			{
				reset_cnt ++;
				if(reset_cnt>20)
				{
					reset_cnt = 0;
					app_imu_data.reset = 0;
				}
			}
			else
			{
				reset_cnt = 0;
			}
		}
		else
		{
			kp_use = twoKp;
			ki_use = twoKi;
			mkp_use = twomKp;
		}
#else
		if(app_imu_data.reset == 1)
		{
			kp_use = 20.0f;
			ki_use = 0.0f;
			if(APP_MATH_ABS(halfex)+APP_MATH_ABS(halfey)+APP_MATH_ABS(halfez)<0.01f)
			{
				app_imu_data.reset = 0;
			}
		}
		else
		{
			kp_use = twoKp;
			ki_use = twoKi;
		}
#endif
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
    recipNorm = app_math_Invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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

/** 
* @brief    SoftYaw
* @remarks  û�б߽��Yawֵ
* @par ��־
*/
static float Soft_Angle(float angle,uint8_t whichAngle)
{
	static int16_t angleCircle[3];	
	static float LastAngle[3];
	if((angle - LastAngle[whichAngle]) > 180) 
		angleCircle[whichAngle]--;
	else if((angle - LastAngle[whichAngle]) < -180)
		angleCircle[whichAngle]++;
	LastAngle[whichAngle] = angle;
	return  angleCircle[whichAngle]*360 + angle;
}
/** 
* @brief   ��̬����
* @remarks 
*/
uint32_t tPrev,tNow; 
void app_imu_So3thread(void)
{   
    float euler[3] = {0,0,0};            //rad  
    float Rot_matrix[9] = {1.0f,  0.0f,  0.0f, 0.0f,  1.0f,  0.0f, 0.0f,  0.0f,  1.0f };       /**< init: identity matrix */
    /* �������ν���ʱ���� */
    tNow = MICROS();
    float dt = (tPrev > 0) ? (tNow - tPrev) / 1000000.0f : 0;
    tPrev = tNow;
//    if(dt == 0)  return;    // ��һ����0Ҳû��ϵ�������ǻ���
    /* ��ȡ����(�Ѿ��˲���У������λ��) */
    MPU_Read_Raw();
    /* ��Ԫ����̬�ں� */  
    #ifdef USE_MAG		
    NonlinearSO3AHRSupdate(app_imu_data.unitized.Gyro[0],app_imu_data.unitized.Gyro[1],app_imu_data.unitized.Gyro[2],
													 app_imu_data.unitized.Accel[0],app_imu_data.unitized.Accel[1],app_imu_data.unitized.Accel[2],
													 app_imu_data.unitized.Mag[0],app_imu_data.unitized.Mag[1],app_imu_data.unitized.Mag[2],so3_comp_params_Kp,so3_comp_params_Ki,so3_comp_params_mKp,dt);
    #else
    NonlinearSO3AHRSupdate(app_imu_data.unitized.Gyro[0],app_imu_data.unitized.Gyro[1],app_imu_data.unitized.Gyro[2],
													 app_imu_data.unitized.Accel[0],app_imu_data.unitized.Accel[1],app_imu_data.unitized.Accel[2],
													 0,0,0,so3_comp_params_Kp,so3_comp_params_Ki,0,dt);		
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
    app_imu_data.Roll = euler[0] * TOANGLE;            // ���Խ�
    app_imu_data.Pitch = euler[1] * TOANGLE;	
    app_imu_data.Yaw = -euler[2] * TOANGLE;
    app_imu_data.soft.Roll = Soft_Angle(app_imu_data.Roll,0);   // ����·�̽�
    app_imu_data.soft.Pitch = Soft_Angle(app_imu_data.Pitch,1);
    app_imu_data.soft.Yaw = Soft_Angle(app_imu_data.Yaw,2);
		app_imu_data.integral.Roll += app_imu_data.Angle_Rate[0]*GYRO_RESOLUTION*dt;  // ��Ի��ֽ�
		app_imu_data.integral.Pitch += app_imu_data.Angle_Rate[1]*GYRO_RESOLUTION*dt;
		app_imu_data.integral.Yaw += app_imu_data.Angle_Rate[2]*GYRO_RESOLUTION*dt;
    if(APP_MATH_ABS(app_imu_data.original.Gyro[0]) <= 1 && APP_MATH_ABS(app_imu_data.original.Gyro[1]) <= 1 && APP_MATH_ABS(app_imu_data.original.Gyro[2]) <= 1 &&
			 APP_MATH_ABS(app_imu_data.original.Accel[0] <= 1) && APP_MATH_ABS(app_imu_data.original.Accel[1] <= 1) && APP_MATH_ABS(app_imu_data.original.Accel[2] <= 1))
			app_imu_data.ready = 0;
		else  app_imu_data.ready = 1;
}
