#include "as5600.h"

extern __IO uint32_t uwTick;  // 弃用
extern uint32_t us_counter_period_tim2;

float angle_prev = 0; 
int full_rotations = 0; // 旋转圈数
float angle_d;  //  不带圈数角度
float angle_d_with_track;  // 带圈数角度

// 计算速度
uint32_t now_us = 0;
uint32_t pre_us = 0;
float angle_d_pre = 0;
int full_rotations_pre = 0;
float velocity = 0;

// 低通滤波
uint32_t time_stamp = 0;
uint32_t time_stamp_pre = 0;
float Tf = 0.001;
float y_prev = 0;
float dt = 0;

unsigned char write_reg(unsigned char reg, unsigned char value)
{
	return HAL_I2C_Mem_Write(&hi2c1, Slave_Addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
}

unsigned char write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	return HAL_I2C_Mem_Write(&hi2c1, Slave_Addr, reg, I2C_MEMADD_SIZE_8BIT, value, len, 50);
}

unsigned char read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	return HAL_I2C_Mem_Read(&hi2c1, Slave_Addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}



float i2c_AS5600_get_angle(void)
{   
	int16_t in_angle;
    uint8_t temp[2]={0,0};
	read_reg( Angle_Hight_Register_Addr, temp, 2);
    in_angle = ((int16_t)temp[0] <<8) | (temp[1]);
	
	angle_d_pre = angle_d;
	
    angle_d = (float)in_angle * cpr / 4096;
	
	// 更新时间
	pre_us = now_us;
	now_us = TIM2->CNT + us_counter_period_tim2;
	
	
	return angle_d;
	//angle_d为弧度制，范围在0-6.28	
}

float i2c_AS5600_get_angle_with_track(void)
{
    float val = angle_d;
    float d_angle = val - angle_prev;
	
	// 更新上次的圈数
	full_rotations_pre = full_rotations;
	
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(fabs(d_angle) > (0.8f*2.0f*PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
	
	angle_d_with_track = full_rotations * (2.0f*PI) + angle_prev;
	return angle_d_with_track;
//    return (float)full_rotations * 6.28318530718f + angle_prev;
}

float as5600_get_velocity(void)
{
	float Ts = (now_us - pre_us) * 1e-6;
	if (Ts < 0)
	{
		Ts = 1e-3;
		us_counter_period_tim2 = 0;
	}
	
	velocity = ((float)(full_rotations - full_rotations_pre) * 2.f*PI + (angle_d - angle_d_pre) ) / Ts;
	return velocity;
}

float lowpassfilter(float input_value)
{
	time_stamp = TIM2->CNT + us_counter_period_tim2;
	dt = (time_stamp - time_stamp_pre) * 1e-6;
	
	if (dt < 0.f)
	{
		dt = 1e-3f;
		us_counter_period_tim2 = 0;
	}
	else if (dt > 0.3f)
	{
		y_prev = input_value;
		time_stamp_pre = time_stamp;
		return input_value;
	}
	float alpha = Tf / ( Tf + dt );
	float y = alpha * y_prev + (1.0f - alpha) * input_value;
	y_prev = y;
	time_stamp_pre = time_stamp;
	return y;
}
