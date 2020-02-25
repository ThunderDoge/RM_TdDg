/**
 * @file app_mode.hpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief Mode类 用于哨兵逻辑部分
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __APP_MODE_HPP_
#define __APP_MODE_HPP_
#include "stm32f4xx.h"

typedef void (*vivoFuncPtr)(void); ///指向void 函数名(void)的指针。Pointer to void input void output function
/**
 * @brief Mode状态枚举值
 * 
 */
enum mode_status_enum:uint8_t{
    MODE_EXITED = 0,
    MODE_ENTERED = 1,
    MODE_RUNNING = 2,
};
/**
  * @brief  废案重启 Mode类定义
  */
class Mode
{
public:
    Mode(vivoFuncPtr entry, vivoFuncPtr runner, vivoFuncPtr exit) : ///< 构造函数Constructor，参数为三个函数的指针，分别对应Enter,Run,Exit。函数体需要用户自行定义。
    status(MODE_EXITED), EnterCallback(entry), RunningCallback(runner), ExitCallback(exit){};

    void Enter();   ///<进入函数
    void Run();     ///<运行时函数
    void Exit();    ///<退出函数
private:
    uint8_t status; ///<Mode的状态，调用模式的方法会改变此数字。取值见 mode_status_enum
    vivoFuncPtr EnterCallback;
    vivoFuncPtr RunningCallback;
    vivoFuncPtr ExitCallback;
};

#endif // __APP_MODE_HPP_
