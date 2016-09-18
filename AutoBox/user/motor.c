#include "motor.h"
#include <string.h>

void Motor_init(struct motor *pMotor)
{
	P1M0 |= 0x03; 
	P1M1 &= ~0x03;
	//电机状态初始化
	Motor_set_status(pMotor,MOTOR_STOP);
	memset(pMotor, 0, sizeof(struct motor));
	//电机PWM控制器初始化
	Motor_set_pwm(pMotor,0xff);	
	pMotor->AdChNum = 2;
	adc_init();
}

void Motor_set_pwm(struct motor *pMotor,u8 pwm)
{
	pMotor->pwm = pwm;	
} 

void Motor_set_status(struct motor *pMotor,s8 status)
{
	switch(status)
	{
		case MOTOR_FORWORD:
			motor_pinN = 0;
			motor_pinP = 1;
			pMotor->status = MOTOR_FORWORD;
			break;
		case MOTOR_REVERSAL:
			motor_pinP = 0;
			motor_pinN = 1;	
			pMotor->status = MOTOR_REVERSAL;		
			break;
		case MOTOR_STOP:
			motor_pinP = 0;
			motor_pinN = 0;
			pMotor->status = MOTOR_STOP;
			break;
	}
}
u8 	Motor_get_current(struct motor *pMotor)
{
	pMotor->current = GetADCResult();
	return pMotor->current;	
}
