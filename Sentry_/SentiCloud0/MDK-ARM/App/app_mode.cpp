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

void Mode::Enter()
{
    if (EnterCallback != nullptr || EnterCallback != NULL)
        EnterCallback(); //Call callback function
    else
        return;
}
void Mode::Run()
{
    if (RunningCallback != nullptr || RunningCallback != NULL)
        RunningCallback(); //Call callback function
    else
        while (1)
        {;} //ERROR: Try to call NULL Function Pointer
}
void Mode::Exit()
{
    if (ExitCallback != nullptr || ExitCallback != NULL)
        ExitCallback(); //Call callback function
    else
        return;    
}
