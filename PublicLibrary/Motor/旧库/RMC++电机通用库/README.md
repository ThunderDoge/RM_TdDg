# ReadMe简述

## 更新记录:
### 2019年3月19日16:05:43 version 6.12  
移除电机中自带的电流环，同时在底盘类型中提供了电流环控制，并提供了功率控制系统，使用时需要在CarDrv_Config.hpp中添加功率限制LIMIT_P宏定义， 不需要限制功率的话不提供电流环即可.同时添加了 @ref manager::UserProcess() 中间处理函数，允许用户在pid运行后发送前添加自定义处理逻辑，同时还允许用户将已定义的电机作为一个非独立电机通过参数 @ref manager::cooperative 。设定该参数后 @ref manager::CANSend() 将不会自动运行该电机的pid，需要用户在@ref manager::UserProcess() 自行处理pid.
### 2018年10月31日20:11:19 version 5.0 
	将Speed_Set()等函数分离成```Speed_Run()```与```Speed_Set()```,其中Run后缀的函数为PID计算函数，交由内部处理函数CANSend()负责，无需管理，而Speed_Set()为设定目标值。分离后PID的执行周期由CANSend决定，不再与Set类型相关，统一了频率
	整合了旧版本中弃用的update函数
### 2019年1月17日20:37:00 Version 6.0
	将motor类电机向上再分出一类manager管理类，该类不属于任何实体电机，但是是一切大疆CAN电机的父系。优化softmotor结构，新增了云台电机类型与6020电机类型。新增了专用配置文件CarDrv_config.h以用于不同工程下的不同配置结构
### 早期版本记录
	*		V2.6  测试版
	*		V3.0  将softmotor电机堵转类完成封装 可以在任意一个softmotor内通过Enbale_Block()来开启堵转检测 \n
	*		V4.0  将电机与CAN合为一体 不需要再考虑ID和发送的组合的事情了 全部交由库解决 \n
	*		V4.1  取消了堵转类的模板 此时堵转类仅能用于softmotor了 考虑到普通motor堵转检测少 减少复杂度 \n
	*		V4.2  PID类新增Custom_Diff参数 用于代替差分值 motor类带外环时外环微分误差默认为motor的RealSpeed \n
	*		V4.3  添加了car库的稳定性 阻止电机在未被声明的情况下更新数据 \n
	*		V5.0  大型重构 将所有电机使用统一线程管理，控制参数可以在外部更改 \n
	*		V5.1  清理旧库残留函数 重新管理电机发送规则 \n
	*		V5.2  清理chassis的部分没有使用的函数，并修改了motor成员为public，新定义cloud用作云台电机，下一版预定修改cloud \n
	*		V5.3  底盘加入是否通过采样板控制选项,通过宏定义#define CHASSIS_POWER_CONTROL设置
	*		V5.4  完善cloud类，可选纯电机反馈数据/纯陀螺仪反馈/陀螺仪+电机反馈，适用于pitch和yaw轴电机


## 简述
RM一系列操作的控制方案
OnePointFive内部方案

## 当前功能
1. 自带CAN自选一键初始化，默认配置为FIFO0，中断方式接收
2. PID控制对象，除基本PID外还有区间积分，时间区间微分等扩展功能，使用```pid_run()```执行一次PID计算
3. 普通电机类```motor```,初始化电机的PID参数后，可以使用```Speed_Set()```进行速度环控制或```Angle_Set()```进行位置环控制
4. 累计路程电机类```softmotor```,初始化电机PID参数后，可以使用```Angle_Set()```进行电机长路程控制，圈数控制.
5. 电机堵转检测类```block_type```,初始化后可以检测电机的堵转情况并有标志位
6. 底盘类型```chassis_t,Run()```以进行底盘速度控制，```RunByPosition()```以进行底盘位置控制

## 使用方式
1.按照C++工程配置Keil
2.加入本驱动包的文件
3.使用```motor::CANSelect(CAN_HandleTypeDef* canhandle1,CAN_HandleTypeDef* canhandle2)```来决定使用的CAN，如使用两个CAN则第二个参数传入NULL即可
4.在板级支持文件上初始化电机，如一个3508带路程环和速度环pid，接在CAN1，CANID为0x206的初始化:
```C
static pid Lift_left_pid_out(0.5,0.006,0,3000,10000);
static pid Lift_left_pid_in(10,0.2,0.1,3000,10000);
softmotor Lift_left(1,0x206,&Motor,&Lift_left_pid_in,&Lift_left_pid_out);
```
5.在接收中断中使用接收总处理函数```motor::CANUpdate(CAN_HandleTypeDef* _hcan,CAN_RxHeaderTypeDef* RxHead,uint8_t* Data)```将接收到的数据包传入库内，更新电机数据
6.在一个确定的周期线程或中断执行```motor::CANSend(void)```以发送所有的电机数据
7.可以在其他任意地方使用```Speed_Set()```等执行函数了