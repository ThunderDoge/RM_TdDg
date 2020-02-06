/** 
* @file     universe.cpp
* @brief    ����Ƕ��ʽC++���̵Ĳ����ļ�
* @details  ��Ƕ��ʽ����ʹ��C++ʱ����һЩ������Ҫ������ʹC++���õ������ڹ��� \n
*           ���ļ������Խϴ�����������Ϊʹ��keilΪ���뻷����ʹ��FreeRTOS
* @author   WMD
* @date     2018��10��26��22:06:00
* @version  0.1
* @par Copyright (c):  
*       WMD 
* @par ��־
*/  
#include <rt_sys.h>
#include "CarDrv_config.hpp"
#include "freertos.h"
#include <time.h>
using namespace std;
#ifdef USE_OWM_SYS_DEF
extern "C" {
#pragma import(__use_no_semihosting_swi)  
#pragma import(_main_redirection)  
const char __stdin_name[150]; //�������ھ����� 
const char __stdout_name[150];  
const char __stderr_name[150];  
typedef int FILEHANDLE;  

//��д��׼�⺯������ʱprintf��fopen��fclose���ļ�������������ʱ�ͻ���������д��������Щ��д����ֻ�Ǽ����򵥵����ӣ���û����д���е��ļ���������  
void _sys_exit(int status)  
{  
    while(1);  
}  
FILEHANDLE _sys_open(const char *name, int openmode)  
{  
    return 0;  
}  

int _sys_close(FILEHANDLE fh)  
{  
    return 0;  
}  

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)  
{  
    return 0;  
}  

int _sys_read(FILEHANDLE fh, unsigned char*buf, unsigned len, int mode)  
{  
    return 0;  
}  

int _sys_istty(FILEHANDLE fh)  
{  
    return 0;  
}  

int _sys_seek(FILEHANDLE fh, long pos)  
{  
    return 0;  
}  

int _sys_ensure(FILEHANDLE fh)  
{  
    return 0;  
}  

long _sys_flen(FILEHANDLE fh)  
{  
    return 0;  
}  

int _sys_tmpnam(char *name, int fileno, unsigned maxlength)  
{  
    return 0;  
}  

void _ttywrch(int ch)  
{  
}
time_t time(time_t *t)  
{  
    return 0;  
}  
int remove(const char *filename)  
{  
    return 0;  
}  

char *_sys_command_string(char *cmd, int len)  
{  
    return 0;  
}  

clock_t clock(void)  
{  
    return 0;  
}  
///�ض����һ������Ҫ������������Ҫ�����ⲿ���Ƕ���
__attribute__((weak)) int fputc(int ch, FILE* f)
{
    return ch;
}
}//C��C++�ķֽ���
#endif

/** 
* @brief  ����new�����ʹ�����ʹ�ò���ϵͳ���ڴ���ȹ���
* @warning pvPortMalloc()������Ҫ����ϵͳ֧��
* @par ��־ 
*
*/
void *operator new (size_t size)
{
  void* p=pvPortMalloc(size);
	if(p==NULL)while(1);
	return p;
}
/** 
* @brief  ����new�����ʹ�����ʹ�ò���ϵͳ���ڴ���ȹ���
* @warning pvPortMalloc()������Ҫ����ϵͳ֧��
* @par ��־ 
*
*/
void*
operator new[](size_t size)
{
  return operator new(size);
}
/** 
* @brief  ����delete�����ʹ�����ʹ�ò���ϵͳ���ڴ���ȹ���
* @warning vPortFree()������Ҫ����ϵͳ֧��
* @par ��־ 
*
*/
void operator delete(void* p)
{
	vPortFree(p);
}
/** 
* @brief  ����delete�����ʹ�����ʹ�ò���ϵͳ���ڴ���ȹ���
* @warning vPortFree()������Ҫ����ϵͳ֧��
* @par ��־ 
*
*/
void
operator delete[](void* pointer)
{
  operator delete(pointer);
}
