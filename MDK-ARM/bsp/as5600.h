#ifndef _AS5600_H_
#define _AS5600_H_

#include "main.h"
#include "stdio.h"
#include "math.h"
#include "tim.h"

#define PI					3.14159265358979f
#define cpr (float)(2.0f*PI)
#define rad_to_grad(x) ((x / PI) * 180.0)
#define Slave_Addr                0x36<<1//设备从地址
#define Write_Bit                 0	   //写标记	
#define Read_Bit                  1    //读标记
#define Angle_Hight_Register_Addr 0x0C //寄存器高位地址
#define Angle_Low_Register_Addr   0x0D //寄存器低位地址


extern I2C_HandleTypeDef hi2c1;
unsigned char read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
unsigned char write_reg(unsigned char reg, unsigned char value);
float i2c_AS5600_get_angle_with_track(void);
float i2c_AS5600_get_angle(void);
float as5600_get_velocity(void);
float lowpassfilter(float input_value);

#endif