#ifndef __CHUANPID_H__
#define __CHUANPID_H__

#include "stm32f10x.h"

void CONTROL1(float rol_now, float pit_now, float yaw_now, u16 throttle, float rol_tar, float pit_tar, s16 yaw_gyro_tar,s16* ACC,s16* GYRO);

#endif
