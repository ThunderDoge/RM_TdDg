/**
 * @file app_mode.hpp
 * @author ThunderDoge (thunderdoge@qq.com)
 * @brief Mode�� �����ڱ��߼�����
 * @version 0.1
 * @date 2020-02-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "stm32f4xx.h"

typedef void (*vivoFuncPtr)(void); ///ָ��void ������(void)��ָ�롣Pointer to void input void output function
/**
 * @brief Mode״̬ö��ֵ
 * 
 */
enum mode_status_enum:uin8_t{
    MODE_OFF = 0,
    MODE_ON = 1,
}
/**
  * @brief  �ϰ����� Mode�ඨ��
  */
class Mode
{
public:
    Mode(vivoFuncPtr entry, vivoFuncPtr runner, vivoFuncPtr exit) : ///< ���캯��Constructor������Ϊ����������ָ�룬�ֱ��ӦEnter,Run,Exit����������Ҫ�û����ж��塣
    status(MODE_OFF), EnterCallback(entry), RunningCallback(runner), ExitCallback(exit){};

    void Enter();   ///<���뺯��
    void Run();     ///<����ʱ����
    void Exit();    ///<�˳�����
private:
    uint8_t status; ///<Mode��״̬������ģʽ�ķ�����ı�����֡�ȡֵ�� mode_status_enum
    vivoFuncPtr EnterCallback;
    vivoFuncPtr RunningCallback;
    vivoFuncPtr ExitCallback;
};
