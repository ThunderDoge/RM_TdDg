# RM2020电控 Board Support Package

> Evan-GH的改动日志
>
>   * V7.0 	修改pid::run计算函数，现在T是真正的积分时间变量了
>   * V7.1 	修改云台类的陀螺仪角度设置串级计算函数，把err作为浮点数传入，不再乘上50了，不改这里之前写的参数是需要重新调整的
>     	*		V7.2 	修改云台类的机械角角度设置串级计算函数，修改err为浮点数，改动目标值的传入，适应之前调整的参数
>     	*		V7.2a 对pid::run计算函数做了一次兼容性更新，适应没有做积分分离的参数
>     	*		V7.2b 使用机械角模式调节Yaw轴的时候发现了无法克服8192到0的突变，根据RM2019步兵上的代码对softcloud::Angle_Set，softcloud::Position_Run
>     						两个函数进行了一次修改，已经正常
>     	*		V7.3	修改底盘类，整合标志位
>     	*		V7.3a	根据RM2019步兵代码加入缓冲能量的一些参数，后续将继续完善
>     	*		V7.4	根据代码规范修改了一下文件名，由于内部函数和类成员较多，不对内部进行修改，避免出现隐形Bug，代码规范终究还是对C++库下手了
>     						Car_Driver.cpp -> bsp_motor.cpp			Car_Driver.hpp -> bsp_motor.hpp		universe.cpp -> bsp_universe.cpp
>     						CarDrv_config.hpp -> bsp_car_config.hpp  注意只是修改了文件名，内部函数并没有做命名修改，还是通用的，使用方法不变！
>     	*		V7.4a	修改了文件包含关系，现在电机库处于Bsp层，不再调用app_math里面的函数，文件分层更加清晰
>     	*		V7.5	修改了电机离线检测的时间阈值10ms到50ms
>     	*		V7.5a	根据ToolMan ThunderDoge的需求对C++电机库进行了修改，修改之前git一下保平安
>     						按照GM6020的手册，6020的最大ID可以达到0x20B，所以此次修改更新了电机列表等相关数据类型的最大长度，现在能支持到0x20B了
>     						简单测试后感觉功能是正常的，现在can_code的表示方法为百位表示can几，后面的尾数表示ID-1
>     	*		V7.5b 根据ToolMan ThunderDoge的需求对C++电机库进行了修改，给softmotor类加入新成员 SoftAngle，RealAngle现在和motor类计算方法一致，范围
>     						0~360，SoftAngle范围为负无穷到正无穷
>     	*		V7.5c 根据ToolMan ThunderDoge的需求对C++电机库进行了修改，修改了motor类电流成员的名字LastCurrentEncoder -> LastRealCurrent
>     						CurrentEncoder->RealCurrent
>     	*		V7.5d 根据ToolMan ThunderDoge的需求对C++电机库进行了修改，加入原始位置信息成员OriginalPosition，这个量没有经过CLOUD_STD校正
>     						把云台类的软件限位调整到父类cloud里面来，并在cloud类里面添加了限位用的保护型成员max，min 这两个值只能通过Limit方法进行限制
>     	*		V7.6	在pid类中明确了积分时间和微分时间，不传入参数的时候都默认为1.现在这两个变量都可以进行调节了,积分，微分两个环节有了自己独立的时间变量