#include "motor.h"
#include "tim.h"
#include "pid.h"

#define MAX_SPEED_UP  3

extern float Motor1Speed ;
extern float Motor2Speed ;
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern float ATmo1;
extern float ATmo2;
float motorSpeedUpCut = 0.5;

void Motor_Set(int Motor1,int Motor2)
{
	Motor1 =-Motor1;
	Motor2 =-Motor2;
	
	if(Motor1 <0) BIN1_SET;
	else  BIN1_RESET;
	
	if(Motor2 <0) AIN1_SET;
	else AIN1_RESET;
	  
	if(Motor1 <0)
	{
		if(Motor1 <-99) Motor1 =-99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (100+Motor1));
	}
	else 
	{
		if(Motor1 >99) Motor1 = 99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,Motor1);
	}

	if(Motor2<0)
	{
		if(Motor2 <-99) Motor2=-99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (100+Motor2));
	}
	else
	{
		if(Motor2 >99) Motor2 =99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor2);
	}


}


//控制电机转向tPid pidMotor2Speed;
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed)
{
	//设置pid目标转速
	pidMotor1Speed.target_val=Motor1SetSpeed;
	pidMotor2Speed.target_val=Motor2SetSpeed;
	//pid计算控制电机转速
	Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
}
//向前加速函数
void motorPidSpeedUp(void)
{
if(ATmo1<= MAX_SPEED_UP&&ATmo2>=0) 
	{ATmo1 +=0.5;
	}
		if(ATmo2<= MAX_SPEED_UP&&ATmo2>=0) 
	{ATmo2 +=0.5;
	}
		if(ATmo1<0) 
	{ATmo1 -=0.5;
	}
		if(ATmo2<0) 
	{ATmo2 -=0.5;
	}
	motorPidSetSpeed(ATmo1,ATmo2);
}
//向前减速函数
void motorPidSpeedCut(void)
{
	
	if(ATmo1 >=0.5)ATmo1-=0.5;
	if(ATmo2 >=0.5)ATmo2-=0.5;
	if(ATmo1 <=-0.5)ATmo1+=0.5;
	if(ATmo2 <=-0.5)ATmo2+=0.5;
	motorPidSetSpeed(ATmo1,ATmo2);
}






