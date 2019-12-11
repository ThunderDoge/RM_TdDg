#ifndef __SENTRY_CHASSIS_LOGIC_H_
#define __SENTRY_CHASSIS_LOGIC_H_
#include "SentryChassis.hpp"
#include "SentryCanCommu.hpp"

enum GlobalModeName
{
    MODE_SAFE=1,
    MODE_MANUAL_SHOOTING_TEST,
    MODE_VIISON_SHOOTING_TEST,
    MODE_KEYBOARD_TEST,
    MODE_MANUAL_CHASSIS_MOVE,
    MODE_AUTONOMOUS,
};
enum CommandSourceName
{
	CMDSRC_DBUS,
	CMDSRC_CAN,
	CMDSRC_SELF,
};

GlobalModeName GetGlobalMode();
extern CommandSourceName CommandSource;
extern GlobalModeName RecvCMD;

void ModeSelect(void);
void VisionControl();   //�Ӿ�����

#endif // __SENTRY_CHASSIS_LOGIC_H_
