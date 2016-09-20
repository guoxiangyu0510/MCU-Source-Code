/*
	待完成功能
	1.产品出厂初始设定：产品关机键连续按5次启动出厂设置
						即按顺序学习小盒的关闭和打开遥控命令
						（此功能有可能会无意中混乱按键，为防止此情况发生，
						  加入recover Pin有硬件操作完成）
	2.出现阻转故障后的处理
	3.添加看门狗
	4.增加AD采样滤波器
	5.考虑添加复位按键
	6.软启动功能
*/

#include <STC15.h>
#include <string.h>
#include <intrins.h>
#include "timer0.h"
#include "typeDef.h"
#include "motor.h"
#include "math.h"
#include "eeprom.h"

#define FOSC    12000000L
#define T100Hz  (FOSC / 12 / 100)
#define CCP_S0 0x10                 //P_SW1.4
#define CCP_S1 0x20                 //P_SW1.5

#define IR_HEADER_TIME_Y 9000   	//前导码有38K调制波时间 9ms
#define IR_HEADER_TIME_N 4500   	//前导码无38K调制波时间 4.5ms
#define IR_DATA_TIME_Y 	 560		//数据码有38K调制波固定时间	0.56ms
#define IR_LOGIC_1_CYCLE 2250		//逻辑1时数据码周期	    2.25ms
#define IR_LOGIC_0_CYCLE 1120		//逻辑0时数据码周期		1.12ms
#define this_recv_usr_code IR_RecvBuf[0]
#define this_recv_cmd_code IR_RecvBuf[2]
#define IR_IAP_INFO_ADDR 0x0400

#define SIG_STOP 0
#define SIG_BEGIN 1

//*********************************
//About Box status
// x	 x	 x	  x	 x	 x	 	xx
//活动 完成	错误  0  0   0   1:打开 2:关闭
#define STAT_BOX_OPENING	0x81
#define STAT_BOX_OPENED		0x41
#define STAT_BOX_CLOSING	0x82
#define STAT_BOX_CLOSED		0x42
#define STAT_BOX_OPEN_ERROR	0x21
#define STAT_BOX_CLOSE_ERROR 0x22
#define STAT_BOX_STOP_IN_OP	0x01
#define STAT_BOX_STOP_IN_CL	0x02

#define BOX_IS_ACTIVE()	((Box_status&0x80) != 0)
#define BOX_IS_FINISH()	((Box_status&0x40) != 0)

#define ASSERT(RESULT,FLAG)   											\
			if(RESULT == -1 ) 											\
			{				  											\
				debug_uart_print_string_int(FLAG);	   					\
				while(IR_wait_status(1, SIG_STOP, 6000, 6000)!=-1);	 	\
				IE0=0;EX0 = 1;		 									\
				return;													\
			}

#define _DEBUG_

sbit IR_Pin = P3^2;

u8 IR_RecvBuf[4],IR_RecvFlag,set_times,set_index;

u8 IR_usr_code,IR_cmd_code;

u8 Box_status;

#define AD_HIST_SIZE 8
u8 xdata AD_hist_data[AD_HIST_SIZE];
u8 xdata AD_sample_pos;
u8 xdata AD_data_max,AD_data_min;
u16 xdata AD_data_sum;
u8 xdata AD_this_data;
u8 xdata AD_filter_out;

struct motor myMotor,*pMyMotor;

struct IR_info_t
{	
	u8 user_code;	//用户码		
	u8 set_code;	//设置键码	
	u8 open_code;	//打开键码
	u8 close_code;	//关闭键码
	u8 enable;		//IR信息有效标记
}IR_info;

u8 xdata cnt, sys_clk_100HZ_flag;
u16 xdata value;



#if defined (_DEBUG_)
#define debug_uart_print_string(str) uart_print_string(str)
#define debug_uart_print_value(val)  uart_print_value(val)
#define debug_uart_print_hex(val)	 uart_print_hex(val)
#define debug_uart_print_string_int(str) uart_print_string_int(str)
void usart_init(void)
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xC7;			//设定定时初值
	T2H = 0xFE;			//设定定时初值
	AUXR |= 0x10;		//启动定时器2	
}
char putchar (char c)  
{
	SBUF = c;
  	while (!TI);
  	TI = 0;
  	return c;
}
void uart_print_string_int(char *str)
{
	while(*str !='\0')
	{
		putchar(*str);
		str++;
	}
}
void uart_print_string(char *str)
{
	char ch;

	ch = *str;
	while(ch != '\0')
	{		
		if(ch == '\\')
		{
			switch(*(str+1))
			{
				case 'n': ch='\n'; str ++; break;
				case 'r': ch='\r'; str ++; break;
				case 't': ch='\t'; str ++; break;
			}	
		}
		putchar(ch);
		str ++;
		ch = *str;
	}
}
void uart_print_value(int value)
{
	int base;
	
	base = 10000;
	if(value < 0)
	{
		value = abs(value);
		putchar('-');
	}
	while(base != 0)
	{		
		putchar('0'+value/base);
		value %= base;
		base/=10;
	}
}
char hexTab[]="01234567890ABCDEF";
void uart_print_hex(u8 value)
{
	putchar(hexTab[value>>4]);
	putchar(hexTab[value&0x0f]);		
}
#else
	#define debug_uart_print_string(str) 
	#define debug_uart_print_value(val) 
	#define debug_uart_print_string_int(str)	
#endif

/*=============================*/
/*	 看门狗初始化（打开状态）
/*=============================*/
#define feed_dog() (WDT_CONTR|=0x10)
#define watchDog_disable() (WDT_CONTR&=~0x20)
#define watchDog_enable()  (WDT_CONTR|=0x20)
void watchDog_init(void)
{
	//看门狗定时器溢出时间计算公式: (12 * 32768 * PS) / FOSC (秒)
	//12M : 1.0485s
	WDT_CONTR = 0x04;			 
	watchDog_enable();	
}
/*=============================*/
/*		  软件延时
/*=============================*/
void delayMs(u8 time)
{
	u16 timeUs;

	timeUs = (time>=65)? 65000:(time*1000);
	timer0_clear();
	timer0_start();
	while(timer0_read() < timeUs);
	timer0_stop();
}
void delayS(unsigned char time)
{
	u16 temp;

	feed_dog();	 //喂狗
	temp = time*20;
	for(;temp>0;temp--)
	{
		delayMs(50);
		feed_dog();	 //喂狗
	}
}

/*=============================*/
/*		  系统时钟初始化
/*=============================*/
void system_clk_init(void)
{
 	ACC = P_SW1;
    ACC &= ~(CCP_S0 | CCP_S1);      //CCP_S0=0 CCP_S1=0
    P_SW1 = ACC;                    //(P1.2/ECI, P1.1/CCP0, P1.0/CCP1, P3.7/CCP2)
    
//  ACC = P_SW1;
//  ACC &= ~(CCP_S0 | CCP_S1);      //CCP_S0=1 CCP_S1=0
//  ACC |= CCP_S0;                  //(P3.4/ECI_2, P3.5/CCP0_2, P3.6/CCP1_2, P3.7/CCP2_2)
//  P_SW1 = ACC;  
//  
//  ACC = P_SW1;
//  ACC &= ~(CCP_S0 | CCP_S1);      //CCP_S0=0 CCP_S1=1
//  ACC |= CCP_S1;                  //(P2.4/ECI_3, P2.5/CCP0_3, P2.6/CCP1_3, P2.7/CCP2_3)
//  P_SW1 = ACC;  

    CCON = 0;                       //初始化PCA控制寄存器
                                    //PCA定时器停止
                                    //清除CF标志
                                    //清除模块中断标志
    CL = 0;                         //复位PCA寄存器
    CH = 0;
    CMOD = 0x00;                    //设置PCA时钟源
                                    //禁止PCA定时器溢出中断
    value = T100Hz;
    CCAP0L = value;
    CCAP0H = value >> 8;            //初始化PCA模块0
    value += T100Hz;
    CCAPM0 = 0x49;                  //PCA模块0为16位定时器模式

    CR = 1;                         //PCA定时器开始工作
	sys_clk_100HZ_flag = 0;
    cnt = 0;
}

/*=============================*/
/*	  红外信息恢复出厂状态
/*=============================*/
s8 IR_recovery_factory_mode(struct IR_info_t *info)
{
	if(info->enable != 0xff)
	{
		IapEraseSector(IR_IAP_INFO_ADDR);
		info->user_code	= 0xff;
		info->set_code = 0xff;
		info->open_code = 0xff;
		info->close_code = 0xff;
		info->enable = 0xff;
	}
	debug_uart_print_string_int("IR is recovery factory mode\r\n");

	return 0;
}

/*=============================*/
/*	设置红外信息（保存到eeprom）
/*=============================*/
s8 IR_remote_info_set(struct IR_info_t *info)
{
	IapEraseSector(IR_IAP_INFO_ADDR);
	IapProgramByte(IR_IAP_INFO_ADDR,0x55);
	IapProgramByte(IR_IAP_INFO_ADDR+1,info->user_code);
	IapProgramByte(IR_IAP_INFO_ADDR+2,info->set_code);
	IapProgramByte(IR_IAP_INFO_ADDR+3,info->open_code);
	IapProgramByte(IR_IAP_INFO_ADDR+4,info->close_code);
	info->enable = 0x00;
	debug_uart_print_string_int("IR remote info set ok\r\n");

	return 0;		
}

/*=============================*/
/*	  从eeprom中读取IR信息
/*=============================*/
s8 IR_info_read_from_eeprom(struct IR_info_t *info)
{
	if(IapReadByte(IR_IAP_INFO_ADDR) == 0x55)
	{
		info->user_code = IapReadByte(IR_IAP_INFO_ADDR+1);
		info->set_code  = IapReadByte(IR_IAP_INFO_ADDR+2);
		info->open_code  = IapReadByte(IR_IAP_INFO_ADDR+3);
		info->close_code  = IapReadByte(IR_IAP_INFO_ADDR+4);
		info->enable = 0x00;
		debug_uart_print_string_int("IR remote info read ok\r\n");

		return 0;
	} 
	info->enable = 0xFF;
	debug_uart_print_string_int("IR remote info is null\r\n");

	debug_uart_print_string_int("Please press shutdown code:\r\n");
	return -1;
}

/*=============================*/
/*		  红外控制初始化
/*=============================*/
void IR_init(void)
{
	P32 = 1;
	//下降沿触发
	IT0 = 1;
	memset(IR_RecvBuf, 0, sizeof(IR_RecvBuf));
	IR_RecvFlag = 0;
	set_times = 0;
	set_index = 0;
	IR_info_read_from_eeprom(&IR_info);
	EX0 = 1;	
}

/*=============================*/
/*		  限位开关初始化
/*=============================*/
void limit_switch_init(void)
{
	P33 = P36 = 1;
	INT_CLKO |= 0x10;
	IT1 = 1;
	EX1 = 1;
}

/*=============================*/
/*		平均值滤波器初始化
/*=============================*/
void filter_init(void)
{
	memset(AD_hist_data,0,sizeof(AD_hist_data));
	AD_sample_pos = 0;
	AD_data_max = 0;
	AD_data_min = 0;
	AD_data_sum = 0;
	AD_this_data = 0;
	AD_filter_out = 0;
}

/*=============================*/
/*	  MCU进入掉电模式（睡眠）
/*=============================*/
void MCU_sleep_mode(void)
{	
	//进入掉电模式之前确保外部中断为开启状态
	IE0 = 0;
	EX0 = 1;
	EA = 1;
	//MCU进入掉电模式
	PCON = 0x02;            
	//掉电模式被唤醒后,首先执行此语句,然后再进入中断服务程序
	_nop_();                
    _nop_();
	_nop_();
	_nop_();
}

/*=============================*/
/*		  打开盒子
/*=============================*/
void box_open_begin(void)
{
	if(Box_status != STAT_BOX_OPENED && Box_status != STAT_BOX_OPENING)// && Box_status !=STAT_BOX_OPEN_ERROR)
	{
		Motor_set_status(pMyMotor,MOTOR_FORWORD);
		Box_status = STAT_BOX_OPENING;		
		debug_uart_print_string_int("Box is openning ...\r\n");			
	}
	else
	{
		debug_uart_print_string_int("Box_open_finish\r\n");
	}		
}
void box_open_finish(void)
{
	if(Box_status != STAT_BOX_OPENED)
	{
		Motor_set_status(pMyMotor,MOTOR_STOP);
		Box_status = STAT_BOX_OPENED;
	}	
	debug_uart_print_string_int("Box_open_finish\r\n");		
}

void box_close_begin(void)
{
	if(Box_status != STAT_BOX_CLOSED && Box_status != STAT_BOX_CLOSING)// && Box_status !=STAT_BOX_CLOSE_ERROR)
	{
		Motor_set_status(pMyMotor,MOTOR_REVERSAL);	
		Box_status = STAT_BOX_CLOSING;
		debug_uart_print_string_int("Box is closing ...\r\n");		
	} 
	else
	{
		debug_uart_print_string_int("Box close finish\r\n");	
	}	
}
void box_close_finish(void)
{
	if(Box_status != STAT_BOX_CLOSED)
	{
		Motor_set_status(pMyMotor,MOTOR_STOP); 
		Box_status = STAT_BOX_CLOSED;
	}	
	debug_uart_print_string_int("Box close finish\r\n");
}

void box_stop_in_activity()
{
	if(BOX_IS_ACTIVE())
	{
		Motor_set_status(pMyMotor,MOTOR_STOP);
		if( !BOX_IS_FINISH() )
		{
			if(Box_status == STAT_BOX_OPENING)
			{
				Box_status = STAT_BOX_STOP_IN_OP;
			}
			else
			{
				Box_status = STAT_BOX_STOP_IN_CL;	
			}	
		}
		debug_uart_print_string_int("Box stop in activity\r\n");
	}	
	else
	{
		debug_uart_print_string_int("Box always is stoped!\r\n");
	}
}
/*=============================*/
/*		  盒子初始状态设置
/*=============================*/
void box_status_init(void)
{
	pMyMotor = &myMotor;
	Motor_init(pMyMotor);
	Box_status = STAT_BOX_STOP_IN_OP;
	box_stop_in_activity();	
}

/*=============================*/
/*		  板级初始化
/*=============================*/
void board_init(void)
{
#ifdef _DEBUG_
	usart_init();
	debug_uart_print_string("============================\r\n");
	debug_uart_print_string(">  Mini remote control box  \r\n");
	debug_uart_print_string(">  Version : 1.0            \r\n");
	debug_uart_print_string(">  Author  : GXY            \r\n");
	debug_uart_print_string("============================\r\n");	
#endif
	box_status_init();
	timer0_init(); 
	limit_switch_init();
	IR_init();
	filter_init();
	system_clk_init();
	watchDog_init();
	//开启总中断			
	EA = 1;		
}

void main(void)
{
	board_init();

	while(1)
	{
		//电机为活动状态
		if(BOX_IS_ACTIVE())
		{
			//平均值滤波器
			if(sys_clk_100HZ_flag)
			{				
				AD_this_data = Motor_get_current(pMyMotor);	
				//去掉最旧的历史数据
				AD_data_sum -= AD_hist_data[AD_sample_pos];
				//添加最新的历史数据
				AD_hist_data[AD_sample_pos] =  AD_this_data;
				AD_data_sum += AD_this_data;
				//计算平均值		
				AD_filter_out = AD_data_sum >> 3;
				//改变数据指针（指向下一个最旧数据）
				AD_sample_pos++;
				if(AD_sample_pos >= sizeof(AD_hist_data))
				{
					AD_sample_pos = 0; 	 	
				}
				//读取到电机的电流值，如果过大就自动停止
				if(AD_filter_out >= 12)
				{
					if(Box_status == STAT_BOX_CLOSING)
					{
						box_close_finish();
					}
					else if(Box_status == STAT_BOX_OPENING)
					{
						box_open_finish();	
					}
					else
					{
						Motor_set_status(pMyMotor,MOTOR_STOP);
						debug_uart_print_string("Undefined status!\r\n");
					}			
					feed_dog();
					debug_uart_print_string("Motor is stoped to Motor current too much[");
					debug_uart_print_hex(pMyMotor->current);
					debug_uart_print_string("]\r\n");
				}
				sys_clk_100HZ_flag = 0;
			}
		}
		//电机为非激活状态
		else
		{	
			//没有待处理的数据,进入掉电模式
			if(IR_RecvFlag == 0)
	 		{
				debug_uart_print_string("Box is sleepping...\r\n");	
				MCU_sleep_mode();
				debug_uart_print_string("Box was waked up\r\n");  
			}				
		}
		//就在这也喂一次狗吧
		feed_dog();	
		//红外数据处理   
		if(IR_RecvFlag)
		{
			//打印用户码和命令码
			debug_uart_print_string("\r\nUser:[");
			debug_uart_print_hex(this_recv_usr_code);
			debug_uart_print_string("]  Cmd:[");
			debug_uart_print_hex(this_recv_cmd_code);
			debug_uart_print_string("]\r\n"); 
		   	//红外信息有效
			if(IR_info.enable != 0xff)
			{
				//用户码正确
				if(this_recv_usr_code == IR_info.user_code)
				{
					//打开和关闭命令码相同
					if(IR_info.open_code==IR_info.close_code)
					{
						if(this_recv_cmd_code == IR_info.open_code)
						{
							if((Box_status & 0x03) == 0x01)
							{
								goto close_Lable;
							}
							else if((Box_status & 0x03) == 0x02)
							{
								goto open_Lable;
							}
						}
					}
					//打开盒子命令
					if(this_recv_cmd_code == IR_info.open_code)
					{
						open_Lable:
						//立即由正在关闭状态转变为打开延时1S，防止电流过大
						if(Box_status == STAT_BOX_CLOSING)
						{
							box_stop_in_activity();
							delayS(1);
						}
						filter_init();
						box_open_begin();
						set_times = 0;
					}
					//关闭盒子命令
					else if(this_recv_cmd_code ==IR_info.close_code)
					{
						close_Lable:
						//立即由正在打开状态转变为关闭延时1S，防止电流过大
						if(Box_status == STAT_BOX_OPENING)
						{
							box_stop_in_activity();
							delayS(1);				
						}
						filter_init();
						box_close_begin();
						set_times = 0;
					}
					//设置红外遥控信息（出厂设置）
					else if(this_recv_cmd_code == IR_info.set_code)
					{	
						box_stop_in_activity();
						/*set_times++;
						//连续按下关机键5次，恢复出厂设置
						if(set_times>=5)
						{
							set_times = 0;								
							IR_recovery_factory_mode(&IR_info);
							debug_uart_print_string("Please press set code:\r\n");
						}	 */
					}
				#ifdef _DEBUG_
					//获取当前电流值命令（调试用）
					else if(this_recv_cmd_code == 22)
					{
						debug_uart_print_string("current: [");
						debug_uart_print_hex(Motor_get_current(pMyMotor));
						debug_uart_print_string("]\r\n");
					}	
				#endif
					/*else
					{	
						Motor_set_status(pMyMotor,MOTOR_STOP);
					}  */
				}
			}
			//红外遥控信息为空
			else
			{
				switch(set_index)
				{
					//第一次设置关机键
					case 0:
						IR_info.set_code = this_recv_cmd_code;
						IR_info.user_code = this_recv_usr_code;
						debug_uart_print_string("Please press open code:\r\n");
						break;
					//第二次设置打开盒子按键
					case 1:
						IR_info.open_code = this_recv_cmd_code;
						debug_uart_print_string("Please press close code:\r\n");
						break;
					//第三次设置关闭盒子按键
					case 2:
						IR_info.close_code = this_recv_cmd_code;
						break;
				}
				//红外信息全部接收到后再一次性写入到EEPROM中保存				
				set_index ++;
				if(set_index>=3)
				{
					set_index = 0;
					IR_remote_info_set(&IR_info);
					debug_uart_print_string("IR info set ok\r\n");
				}		
			}		 
			IR_RecvFlag = 0;
			IE0 = 0;
			EX0 = 1;
		}
		feed_dog();
	}
}

u16 IR_wait_status(bit logicVal,u8 opt, u16 stdTime,u16 allowError)
{
	u16 time;

	timer0_clear();
	time = 0;
	timer0_start();
	while(time <= stdTime+allowError)
	{
		time = timer0_read();
		if(IR_Pin == logicVal && opt == SIG_BEGIN )
		{
			break;		
		}
		else if(IR_Pin != logicVal && opt == SIG_STOP)
		{
			break;
		}				
	}
	timer0_stop();
	if(time > stdTime + allowError || time < stdTime - allowError)
	{
		return -1;
	}
	return time;
}
//红外接收中断服务程序
void IR_rceived_int() interrupt 0
{
	u16 result;
	u8 num,bit_n;

	if(IR_RecvFlag != 0)
	{
		return;
	}
	//暂时关闭中断，防止数据接收出错
	EX0 = 0;

	//先喂一次狗狗
	feed_dog();

	//接受红外的引导码
	result = IR_wait_status(0, SIG_STOP, IR_HEADER_TIME_Y, 1000);
	ASSERT(result,"IR head error 1\r\n");	
	result = IR_wait_status(1, SIG_STOP, IR_HEADER_TIME_N, IR_HEADER_TIME_N);
	ASSERT(result,"IR head error 2\r\n");

	for(num=0; num<sizeof(IR_RecvBuf); num++)
	{
		for(bit_n=0; bit_n<8; bit_n++)
		{
			result = IR_wait_status(0, SIG_STOP, IR_DATA_TIME_Y, 200);
			ASSERT(result,"IR data error 1\r\n");
			result = IR_wait_status(1, SIG_STOP, 1170, 810);
			//result = IR_wait_status(0, SIG_BEGIN, (IR_LOGIC_1_CYCLE + IR_LOGIC_1_CYCLE)>>1, (IR_LOGIC_1_CYCLE>>1)+200);
			ASSERT(result,"IR data error 2\r\n");
			IR_RecvBuf[num] >>= 1;
			if(result > IR_DATA_TIME_Y-200 && result < IR_DATA_TIME_Y+200)
			{
				
			}
			else
			{
				IR_RecvBuf[num]	|= 0x80;
			}
		}
	}
	//喂狗
	feed_dog();

	//确认IR信号线被释放
	while(IR_wait_status(1, SIG_STOP, 50000, 50000)!=-1);

	//校验数据
	if(IR_RecvBuf[0] == ~IR_RecvBuf[1] && IR_RecvBuf[2] == ~IR_RecvBuf[3])
	{
		//接收成功，关闭中断等待处理完成
		IR_RecvFlag = 1;
		EX0 = 0;	
	}		
	else
	{
		//接收出错，恢复中断准备下次接收
		debug_uart_print_string_int("code check error!\r\n");
		EX0 = 1;		
	} 	
	IE0 = 0; 	
}
//小盒闭合的限位开关0中断服务程序(主要)
void limit_switch0() interrupt 2
{
	if(Box_status == STAT_BOX_CLOSING)
	{
		box_close_finish();
	}
	IR_recovery_factory_mode(&IR_info);
	//debug_uart_print_string_int("limit switch [0] is trigered!\r\n");
	IE1 = 0;
}
//小盒打开的限位开关1中断服务程序
void limit_switch1() interrupt 10
{
	if(Box_status == STAT_BOX_OPENING)
	{
		box_open_finish();
	}
	debug_uart_print_string_int("limit switch [1] is trigered!\r\n");
}
//系统时钟中断服务
void PCA_isr() interrupt 7 using 1
{
    CCF0 = 0;                       //清中断标志
    CCAP0L = value;
    CCAP0H = value >> 8;            //更新比较值
    value += T100Hz;
	sys_clk_100HZ_flag = 1;
}






/*void box_open_error(void)
{
	Motor_set_status(pMyMotor,MOTOR_STOP);
	Box_status = STAT_BOX_OPEN_ERROR;	
	debug_uart_print_string_int("box_open_error\r\n");
}
void box_close_error(void)
{
	Motor_set_status(pMyMotor,MOTOR_STOP);
	Box_status = STAT_BOX_CLOSE_ERROR;
	debug_uart_print_string_int("box_close_error\r\n");
}		   */