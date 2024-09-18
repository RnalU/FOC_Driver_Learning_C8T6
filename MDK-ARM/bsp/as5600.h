#ifndef _AS5600_H_
#define _AS5600_H_

#include "main.h"
#include "stdio.h"
#include "math.h"
#include "tim.h"

#define PI					3.14159265358979f
#define cpr (float)(2.0f*PI)
#define rad_to_grad(x) ((x / PI) * 180.0)
#define Slave_Addr                0x36<<1//�豸�ӵ�ַ
#define Write_Bit                 0	   //д���	
#define Read_Bit                  1    //�����
#define Angle_Hight_Register_Addr 0x0C //�Ĵ�����λ��ַ
#define Angle_Low_Register_Addr   0x0D //�Ĵ�����λ��ַ


extern I2C_HandleTypeDef hi2c1;
unsigned char read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
unsigned char write_reg(unsigned char reg, unsigned char value);
float i2c_AS5600_get_angle_with_track(void);
float i2c_AS5600_get_angle(void);
float as5600_get_velocity(void);
float lowpassfilter(float input_value);

#endif