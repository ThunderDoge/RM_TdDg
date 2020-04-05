/**
 * @file app_mode.cpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief 
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "app_mode.hpp"

void app_Mode::Enter()
{
    if (EnterCallback != nullptr || EnterCallback != NULL)
    {
        status = MODE_ENTERED;
        EnterCallback(); //Call callback function
    }
    else
        return;
}
void app_Mode::Run()
{
    if (RunningCallback != nullptr || RunningCallback != NULL)
    {
        status = MODE_RUNNING;
        RunningCallback(); //Call callback function
    }
    else
        while (1)
        {;} //ERROR: Try to call NULL Function Pointer
}
void app_Mode::Exit()
{
    if (ExitCallback != nullptr || ExitCallback != NULL)
    {
        status = MODE_EXITED;
        ExitCallback(); //Call callback function
    }
    else
        return;    
}
