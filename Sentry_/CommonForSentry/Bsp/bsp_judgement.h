#ifndef _JUDGEMENT_H_
#define _JUDGEMENT_H_

#include "stm32f4xx.h"
#include "usart.h"

#define BSP_JUDGEMENT_UART huart3

typedef __packed struct
{
	uint8_t SOF;					//帧起始字节，固定值为0xA5
	uint16_t DataLength;	//数据段DATA长度
	uint8_t Seq;					//包序号
	uint8_t CRC8;					//帧头CRC校验
}frame_header_t;				//帧头

/************************************************cmd_id_t***********************************************/
typedef enum
{
	ext_game_status_id=0x0001,						//3 比赛状态数据，1Hz 周期发送
	ext_game_result_id = 0x0002, 					//1 比赛结果数据，比赛结束后发送
	ext_game_robot_HP_id = 0x0003, 					//32 比赛机器人血量数据，1Hz 周期发送
	ext_dart_status_id = 0x0004,					//3 飞镖发射状态，飞镖发射时发送	
	ext_event_data_id = 0x0101, 					//4 场地事件数据，1Hz 周期发送
	ext_supply_projectile_action_id = 0x0102, 		//4 场地补给站动作标识数据，动作改变后发送
	ext_referee_warning_id = 0x0104,				//2 裁判警告数据，警告发生后发送
	ext_dart_remaining_time_id = 0x0105,			//1 飞镖发射口倒计时，1Hz 周期发送
	ext_game_robot_status_id = 0x0201,				//18 机器人状态数据，10Hz 周期发送
	ext_power_heat_data_id = 0x0202,				//16 实时功率热量数据，50Hz 周期发送
	ext_game_robot_pos_id = 0x0203,					//16 机器人位置数据，10Hz 发送
	ext_buff_id = 0x0204,							//1 机器人增益数据，1Hz 周期发送
	ext_aerial_robot_energy_id = 0x0205,			//3 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
	ext_robot_hurt_id = 0x0206,						//1 伤害状态数据，伤害发生后发送
	ext_shoot_data_id = 0x0207,						//6 实时射击数据，子弹发射后发送
	ext_bullet_remaining_id = 0x0208,				//2 子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人
	ext_rfid_status_id = 0x0209,					//4 机器人 RFID 状态，1Hz 周期发送
	ext_student_interactive_header_data_id = 0x0301,			//n 机器人间交互数据，发送方触发发送
}cmd_id_t;//命令码 ID 
/************************************************robot_index***********************************************/
enum robot_index
{
	Hero = 1,		//英雄
	Engineer,		//工程
	Standard1,		//步兵1
	Standard2,		//步兵2
	Standard3,		//步兵3
	Aerial, 		//飞机
	Sentry, 		//哨兵
//	Dart			//飞镖
};//机器人代号

/************************************************data_t***********************************************/
//1 比赛状态 命令码0x0001 1Hz
typedef __packed struct 
{   
	uint8_t game_type : 4;				//比赛类型 1：RM 对抗赛；2：单项赛；3：RM ICRA 
	uint8_t game_progress : 4;		//4-7bit：当前比赛阶段 0：未开始比赛；1：准备阶段；2：自检阶段；3：5s 倒计时；4：对战中；5：比赛结算中 
	uint16_t stage_remain_time; 	//当前阶段剩余时间，单位 s 
}ext_game_status_t; 

//2 比赛结果数据：0x0002。发送频率：比赛结束后发送 
typedef __packed struct 
{    
	uint8_t winner; //0 平局 1 红方胜利 2 蓝方胜利 
}ext_game_result_t; 

//3 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人
typedef __packed struct
{
	uint16_t red_1_robot_HP;	//红 1 英雄机器人血量，未上场以及罚下血量为 0
	uint16_t red_2_robot_HP;	//红 2 工程机器人血量
	uint16_t red_3_robot_HP;	//红 3 步兵机器人血量
	uint16_t red_4_robot_HP;	//红 4 步兵机器人血量
	uint16_t red_5_robot_HP;	//红 5 步兵机器人血量
	uint16_t red_7_robot_HP;	//红 7 哨兵机器人血量
	uint16_t red_outpost_HP;	//红方前哨站血量
	uint16_t red_base_HP;		//红方基地血量
	uint16_t blue_1_robot_HP;	//蓝 1 英雄机器人血量
	uint16_t blue_2_robot_HP;	//蓝 2 工程机器人血量
	uint16_t blue_3_robot_HP;	//蓝 3 步兵机器人血量
	uint16_t blue_4_robot_HP;	//蓝 4 步兵机器人血量
	uint16_t blue_5_robot_HP;	//蓝 5 步兵机器人血量
	uint16_t blue_7_robot_HP;	//蓝 7 哨兵机器人血量
	uint16_t blue_outpost_HP;	//蓝方前哨站血量
	uint16_t blue_base_HP;		//蓝方基地血量
}ext_game_robot_HP_t;

//4 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人
typedef __packed struct
{
	uint8_t dart_belong;			//1:红方飞镖 2：蓝方飞镖
	uint16_t stage_remaining_time;	//发射时的剩余比赛时间，单位 s
}ext_dart_status_t;

//5 场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人 
//bit 0-1：己方停机坪占领状态  
//				0为无机器人占领； 
//				1为空中机器人已占领但未停桨； 
//				2为空中机器人已占领并停桨 
//bit 2-3：己方能量机关状态
//				bit2为小能量机关激活状态，1为已激活
//				bit3为大能量机关激活状态，1为已激活
//bit4：己方基地虚拟护盾状态
//				1为基地有虚拟护盾血量
//				0为基地无虚拟护盾血量
//其余保留 
typedef __packed struct 
{ 
	uint32_t event_type; 
}ext_event_data_t; 

//6 补给站动作标识：0x0102。发送频率：动作改变后发送，发送范围：己方机器人
typedef __packed struct 
{
	uint8_t supply_projectile_id;   //补给站口 ID：1：1 号补给口；2 号补给口 
	uint8_t supply_robot_id;    		//机器人 ID：0 为当前无机器人补弹，1 为红方英雄预约，以此类推其他机器人 ID 号预约
	uint8_t supply_projectile_step; //子弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落 
	uint8_t supply_projectile_num;  //补弹数量 分别有50,100,150,200
}ext_supply_projectile_action_t;

//7 裁判警告信息：0x0104。发送频率：警告发生后发送，发送范围：己方机器人
typedef __packed struct
{
	uint8_t level;			//警告等级
	uint8_t foul_robot_id;	//犯规机器人ID：1 级以及 5 级警告时，机器人 ID 为 0；二三四级警告时，机器人 ID 为犯规机器人 ID
}ext_referee_warning_t;

//8 飞镖发射口倒计时：0x0105。发送频率：1Hz 周期发送，发送范围：己方机器人
typedef __packed struct
{
	uint8_t dart_remaining_time;	//15s倒计时
}ext_dart_remaining_time_t;

//9 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人
//robot_id		1：红方英雄机器人；
//				2：红方工程机器人；
//				3/4/5：红方步兵机器人；
//				6：红方空中机器人；
//				7：红方哨兵机器人；
//				8：红方飞镖机器人；
//				9：红方雷达站；
//				101：蓝方英雄机器人；
//				102：蓝方工程机器人；
//				103/104/105：蓝方步兵机器人；
//				106：蓝方空中机器人；
//				107：蓝方哨兵机器人；
//				108：蓝方飞镖机器人；
//				109：蓝方雷达站
///////////////////////////////////////////
//robot_level	1：一级；2：二级；3：三级。
///////////////////////////////////////////
typedef __packed struct
{
	uint8_t robot_id;			
	uint8_t robot_level;
	uint16_t remain_HP;						//机器人剩余血量
	uint16_t max_HP;						//机器人上限血量
	uint16_t shooter_heat0_cooling_rate;	//机器人 17mm 枪口每秒冷却值
	uint16_t shooter_heat0_cooling_limit;	//机器人 17mm 枪口热量上限
	uint16_t shooter_heat1_cooling_rate;	//机器人 42mm 枪口每秒冷却值
	uint16_t shooter_heat1_cooling_limit;	//机器人 42mm 枪口热量上限
	uint8_t shooter_heat0_speed_limit;		//机器人 17mm 枪口上限速度 单位 m/s
	uint8_t shooter_heat1_speed_limit;		//机器人 42mm 枪口上限速度 单位 m/s
	uint8_t max_chassis_power;				//机器人最大底盘功率， 单位 w
	uint8_t mains_power_gimbal_output : 1;	//gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
	uint8_t mains_power_chassis_output : 1;	//chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
	uint8_t mains_power_shooter_output : 1;	//shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
}ext_game_robot_status_t;
//10 实时功率热量数据：0x0202。发送频率：50Hz 
typedef __packed struct
{
	uint16_t chassis_volt;			//底盘输出电压 单位 毫伏
	uint16_t chassis_current;		//底盘输出电流 单位 毫安
	float chassis_power;			//底盘输出功率 单位 W 瓦
	uint16_t chassis_power_buffer;	//底盘功率缓冲 单位 J 焦耳，备注：飞坡根据规则增加至 250J
	uint16_t shooter_heat0;			//17mm 枪口热量
	uint16_t shooter_heat1;			//42mm 枪口热量
	uint16_t mobile_shooter_heat2;	//机动 17 mm 枪口热量
}ext_power_heat_data_t;

//11 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct
{   
	float x;   //位置 x 坐标，单位 m 
	float y;   //位置 y 坐标，单位 m 
	float z;   //位置 z 坐标，单位 m 
	float yaw; //位置枪口，单位度 
}ext_game_robot_pos_t;

//12 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人 
//bit 0：机器人血量补血状态 
//bit 1：枪口热量冷却加速 
//bit 2：机器人防御加成 
//bit 3：机器人攻击加成 
//其他 bit 保留
typedef __packed struct 
{   
	uint8_t power_rune_buff; 
}ext_buff_t;

//13 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct 
{   
	uint16_t energy_point;   	//积累的能量点 
	uint8_t attack_time; 		//可攻击时间 单位 s。30s 递减至 0 
}ext_aerial_robot_energy_t; 

//14 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人
typedef __packed struct 
{
	uint8_t armor_id : 4;  //当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。 
	uint8_t hurt_type : 4; //0x0 装甲伤害扣血；0x1 模块掉线扣血；0x2 超射速扣血；0x3 超枪口热量扣血；0x4 超底盘功率扣血；0x5 装甲撞击扣血
}ext_robot_hurt_t; 

//15 实时射击信息：0x0207。发送频率：射击后发送 
typedef __packed struct 
{
	uint8_t bullet_type; //子弹类型: 1：17mm 弹丸 2：42mm 弹丸 
	uint8_t bullet_freq; //子弹射频 单位 Hz 
	float bullet_speed;  //子弹射速 单位 m/s 
}ext_shoot_data_t; 

//16 子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人
typedef __packed struct
{
	uint16_t bullet_remaining_num;	//子弹剩余发射数目
}ext_bullet_remaining_t;

//17 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人
//bit 0：基地增益点 RFID 状态；
//bit 1：高地增益点 RFID 状态；
//bit 2：能量机关激活点 RFID 状态；
//bit 3：飞坡增益点 RFID 状态；
//bit 4：前哨岗增益点 RFID 状态；
//bit 5：资源岛增益点 RFID 状态；
//bit 6：补血点增益点 RFID 状态；
//bit 7：工程机器人补血卡 RFID 状态；
//bit 8-25：保留
//bit 26-31：人工智能挑战赛 F1-F6 RFID 状态；
//RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不
//能获取对应的增益效果。
typedef __packed struct
{
	uint32_t rfid_status;
}ext_rfid_status_t;

/**14*******************************************机器人间交互*******************************************/
//交互数据接收信息：0x0301。
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

//客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。 发送频率：上限 10Hz 
typedef __packed struct 
{ 
	float data1;
	float data2;
	float data3;
	uint8_t masks; //bit 0-5：分别控制客户端自定义数据显示面板上的六个指示灯，值为1时显示绿色，值为0是显示红色。 Bit 6-7：保留 
}client_custom_data_t;

//交互数据 机器人间通信：0x0301。内容 ID:0x0200~0x02FF  发送频率：上限 10Hz 
typedef __packed struct 
{
	uint8_t data[113];//自定义长度 不超过113
}robot_interactive_data_t;

//客户端删除图形操作
typedef __packed struct
{
	uint8_t operate_tpye;	//0：空操作；1：删除图层；2：删除所有
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

//图形数据
//operate_tpye	bit 0-2：图形操作：
//				
//				Bit 3-5：图形类型：
//				
//				Bit 6-9：图层数，0~9
//				Bit 10-13：
//				Bit 14-22：起始角度，单位：°，范围[0,360]；
//				Bit 23-31：终止角度，单位：°，范围[0,360]
typedef __packed struct
{
	uint8_t graphic_name[3];	//图形名，在删除，修改等操作中，作为客户端的索引
	uint32_t operate_tpye:3;	//图形操作：0：空操作；1：增加；2：修改；3：删除；
	uint32_t graphic_tpye:3;	//图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；5：浮点数；6：整型数；7：字符；
	uint32_t layer:4;			//图层数，0~9
	uint32_t color:4;			//颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	uint32_t start_angle:9;		//起始角度，单位：°，范围[0,360]；
	uint32_t end_angle:9;		//终止角度，单位：°，范围[0,360]；
	uint32_t width:10;			//线宽；
	uint32_t start_x:11;		//起点 x 坐标；
	uint32_t start_y:11;		//起点 y 坐标
	uint32_t radius:10;			//字体大小或者半径；
	uint32_t end_x:11;			//终点 x 坐标；
	uint32_t end_y:11;			//终点 y 坐标。
}graphic_data_struct_t;

//Examples
//客户端绘制一个图形
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

//客户端绘制二个图形
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];		//字符数据
}ext_client_custom_character_t;
/////////////////////////////////////////////////////
enum GraphicOperate
{
	Null,		//空操作
	Add,		//增加
	Rev,		//修改
	Del,		//删除
	DelLayer,	//删除图层
	DelAll,		//删除所有
};
enum GraphicType
{
	Line = 0,	//直线
	Rctangle,	//矩形
	Circle,		//正圆
	Ellipse,	//椭圆
	Arc,		//圆弧
	Data_float,	//浮点数
	Data_int,	//整型数
	Text		//字符
};
enum GraphicColor
{
	Default,	//红蓝主色
	Yellow,		
	Green,
	Orange,
	PurpleRed,
	Pink,
	Cyan,
	Black,
	White,
};

/*********************************************package*******************************************/
typedef __packed struct
{
	frame_header_t Header_Tx; //5bytes HEAD
	uint16_t CmdID_Tx;				//2bytes CMD
	ext_student_interactive_header_data_t student_interactive_header_data;//6bytes ID
	client_custom_data_t client_custom_data;//13bytes 客户端自定义数据
	uint16_t CRC16_Tx;				//2bytes 整包校验
}client_custom_dataPack_t;	//28bytes客户端自定义数据包

//typedef __packed struct
//{
//	frame_header_t Header_Tx; //5bytes HEAD
//	uint16_t CmdID_Tx;				//2bytes CMD
//	ext_student_interactive_header_data_t student_interactive_header_data;//6bytes ID
//	supply_projectile_booking_t supply_projectile_booking;//预约子弹
//	uint16_t CRC16_Tx;				//2bytes
//}robot_booking_dataPack_t;	//bytes预约子弹数据包

typedef __packed struct
{
	frame_header_t Header_Tx; //5bytes HEAD
	uint16_t CmdID_Tx;				//2bytes CMD
	ext_student_interactive_header_data_t student_interactive_header_data;//6bytes ID
	robot_interactive_data_t robot_interactive_data;//自定义3bytes
	uint16_t CRC16_Tx;				//2bytes
}robot_interactive_dataPack_t;//bytes机器人间通信数据包

typedef __packed struct
{
	frame_header_t Header_Tx; //5bytes HEAD
	uint16_t CmdID_Tx;				//2bytes CMD
	ext_student_interactive_header_data_t student_interactive_header_data;//6bytes ID
	graphic_data_struct_t client_graphic_draw;//13bytes 客户端自定义数据
	uint16_t CRC16_Tx;				//2bytes 整包校验
}client_graphic_dataPack_t;	//28bytes客户端自定义数据包

//接收数据包
extern ext_game_status_t game_status;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP;
extern ext_dart_status_t dart_status;
extern ext_event_data_t event_data;
extern ext_supply_projectile_action_t supply_projectile_action;
extern ext_referee_warning_t referee_warning;
extern ext_dart_remaining_time_t dart_remaining_time;
extern ext_game_robot_status_t game_robot_status;
extern ext_power_heat_data_t power_heat_data;
extern ext_game_robot_pos_t game_robot_pos;
extern ext_buff_t buff_musk;
extern ext_aerial_robot_energy_t aerial_robot_energy;
extern ext_robot_hurt_t robot_hurt;
extern ext_shoot_data_t shoot_data;
extern ext_bullet_remaining_t bullet_remaining;
extern ext_rfid_status_t rfid_status;
extern robot_interactive_dataPack_t robot_interactive_dataPack_r;//接收机间交互数据包

//发送数据包
extern client_custom_dataPack_t client_custom_dataPack;//客户端自定义数据包
extern robot_interactive_dataPack_t robot_interactive_dataPack_s;//机间交互数据发送数据包
extern client_graphic_dataPack_t client_graphic_dataPack;//客户端图形自定义数据包


//发送函数
void bsp_bsp_judgement_SendCustomData(float data1,float data2,float data3,uint8_t singleLight);
void bsp_judgement_SendInteractiveData(uint8_t receiver_index, uint16_t data_id);
void bsp_judgement_SendGraphicData(void);

//图形设置函数
void Client_Line(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end,
	uint8_t color, uint8_t line_with);
void Client_Rctangle(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t x_length, uint16_t y_length, 
	uint8_t color, uint8_t line_with);
void Client_Circle(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t radius,
	uint8_t color, uint8_t line_with);
void Client_Ellipse(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t x_length, uint16_t y_length,
	uint8_t color, uint8_t line_with);
void Client_Arc(char *name, uint8_t operation, uint8_t layer, uint16_t x_start, uint16_t y_start, uint16_t half_axis_length,
	int16_t start_angle, int16_t end_angle, uint8_t color, uint8_t line_with);

//更新时间戳
extern uint32_t JudgeMsgRxTick;
void bsp_judgement_Init(void);
void bsp_judgement_It(void);//串口中断函数，记得丢到对应的串口中断里面

//unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
//unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
//void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

//uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
//uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
//void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

//void uart_reset_idle_rx_callback(UART_HandleTypeDef *huart);
//void uart_reset_uartIT(UART_HandleTypeDef *huart); 

//void JudgeMentWinUart_OnLineCheck(void);

#endif
