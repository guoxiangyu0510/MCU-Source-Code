#ifndef _MOTOR_
#define _MOTOR_
	#include "typeDef.h"
	#include <STC15.h>
	#include "adc.h"
	#define MOTOR_FORWORD 1
	#define MOTOR_REVERSAL -1
	#define MOTOR_STOP 0
	sbit motor_pinN = P1^0;
	sbit motor_pinP	= P1^1;
	struct motor
	{
		s8 status; 	//��ǰ����״̬	
		u8 pwm;		//��ǰPWMֵ				
		u8 current;	//��ǰ����ֵ	
		u8 AdChNum;	//adc��ͨ��
	};
	extern void Motor_init(struct motor *pMotor);
	extern void Motor_set_status(struct motor *pMotor,s8 status);
	extern void Motor_set_pwm(struct motor *pMotor,u8 pwm);
	extern u8 	Motor_get_current(struct motor *pMotor);
#endif