#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
void motor_speed(uint8_t side,float duty);
void xunjiopen();
void read_sensors();
int get_gray_error(void);
void Set_Angle(uint8_t side, float duty);
#endif
