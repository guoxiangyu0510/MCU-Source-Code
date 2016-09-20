/*
	����ɹ���
	1.��Ʒ������ʼ�趨����Ʒ�ػ���������5��������������
						����˳��ѧϰС�еĹرպʹ�ң������
						���˹����п��ܻ������л��Ұ�����Ϊ��ֹ�����������
						  ����recover Pin��Ӳ��������ɣ�
	2.������ת���Ϻ�Ĵ���
	3.��ӿ��Ź�
	4.����AD�����˲���
	5.������Ӹ�λ����
	6.����������

	��������
	1.
	2.
	3.
	4.
	5.
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

#define IR_HEADER_TIME_Y 9000   	//ǰ������38K���Ʋ�ʱ�� 9ms
#define IR_HEADER_TIME_N 4500   	//ǰ������38K���Ʋ�ʱ�� 4.5ms
#define IR_DATA_TIME_Y 	 560		//��������38K���Ʋ��̶�ʱ��	0.56ms
#define IR_LOGIC_1_CYCLE 2250		//�߼�1ʱ����������	    2.25ms
#define IR_LOGIC_0_CYCLE 1120		//�߼�0ʱ����������		1.12ms
#define this_recv_usr_code IR_RecvBuf[0]
#define this_recv_cmd_code IR_RecvBuf[2]
#define IR_IAP_INFO_ADDR 0x0400

#define SIG_STOP 0
#define SIG_BEGIN 1

//*********************************
//About Box status
// x	 x	 x	  x	 x	 x	 	xx
//� ���	����  0  0   0   1:�� 2:�ر�
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
	u8 user_code;	//�û���		
	u8 set_code;	//���ü���	
	u8 open_code;	//�򿪼���
	u8 close_code;	//�رռ���
	u8 enable;		//IR��Ϣ��Ч���
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
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x01;		//����1ѡ��ʱ��2Ϊ�����ʷ�����
	AUXR |= 0x04;		//��ʱ��2ʱ��ΪFosc,��1T
	T2L = 0xC7;			//�趨��ʱ��ֵ
	T2H = 0xFE;			//�趨��ʱ��ֵ
	AUXR |= 0x10;		//������ʱ��2	
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
/*	 ���Ź���ʼ������״̬��
/*=============================*/
#define feed_dog() (WDT_CONTR|=0x10)
#define watchDog_disable() (WDT_CONTR&=~0x20)
#define watchDog_enable()  (WDT_CONTR|=0x20)
void watchDog_init(void)
{
	//���Ź���ʱ�����ʱ����㹫ʽ: (12 * 32768 * PS) / FOSC (��)
	//12M : 1.0485s
	WDT_CONTR = 0x04;			 
	watchDog_enable();	
}
/*=============================*/
/*		  �����ʱ
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

	feed_dog();	 //ι��
	temp = time*20;
	for(;temp>0;temp--)
	{
		delayMs(50);
		feed_dog();	 //ι��
	}
}

/*=============================*/
/*		  ϵͳʱ�ӳ�ʼ��
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

    CCON = 0;                       //��ʼ��PCA���ƼĴ���
                                    //PCA��ʱ��ֹͣ
                                    //���CF��־
                                    //���ģ���жϱ�־
    CL = 0;                         //��λPCA�Ĵ���
    CH = 0;
    CMOD = 0x00;                    //����PCAʱ��Դ
                                    //��ֹPCA��ʱ������ж�
    value = T100Hz;
    CCAP0L = value;
    CCAP0H = value >> 8;            //��ʼ��PCAģ��0
    value += T100Hz;
    CCAPM0 = 0x49;                  //PCAģ��0Ϊ16λ��ʱ��ģʽ

    CR = 1;                         //PCA��ʱ����ʼ����
	sys_clk_100HZ_flag = 0;
    cnt = 0;
}

/*=============================*/
/*	  ������Ϣ�ָ�����״̬
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
/*	���ú�����Ϣ�����浽eeprom��
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
/*	  ��eeprom�ж�ȡIR��Ϣ
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
/*		  ������Ƴ�ʼ��
/*=============================*/
void IR_init(void)
{
	P32 = 1;
	//�½��ش���
	IT0 = 1;
	memset(IR_RecvBuf, 0, sizeof(IR_RecvBuf));
	IR_RecvFlag = 0;
	set_times = 0;
	set_index = 0;
	IR_info_read_from_eeprom(&IR_info);
	EX0 = 1;	
}

/*=============================*/
/*		  ��λ���س�ʼ��
/*=============================*/
void limit_switch_init(void)
{
	P33 = P36 = 1;
	INT_CLKO |= 0x10;
	IT1 = 1;
	EX1 = 1;
}

/*=============================*/
/*		ƽ��ֵ�˲�����ʼ��
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
/*	  MCU�������ģʽ��˯�ߣ�
/*=============================*/
void MCU_sleep_mode(void)
{	
	//�������ģʽ֮ǰȷ���ⲿ�ж�Ϊ����״̬
	IE0 = 0;
	EX0 = 1;
	EA = 1;
	//MCU�������ģʽ
	PCON = 0x02;            
	//����ģʽ�����Ѻ�,����ִ�д����,Ȼ���ٽ����жϷ������
	_nop_();                
    _nop_();
	_nop_();
	_nop_();
}

/*=============================*/
/*		  �򿪺���
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
/*		  ���ӳ�ʼ״̬����
/*=============================*/
void box_status_init(void)
{
	pMyMotor = &myMotor;
	Motor_init(pMyMotor);
	Box_status = STAT_BOX_STOP_IN_OP;
	box_stop_in_activity();	
}

/*=============================*/
/*		  �弶��ʼ��
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
	//�������ж�			
	EA = 1;		
}

void main(void)
{
	board_init();

	while(1)
	{
		//���Ϊ�״̬
		if(BOX_IS_ACTIVE())
		{
			//ƽ��ֵ�˲���
			if(sys_clk_100HZ_flag)
			{				
				AD_this_data = Motor_get_current(pMyMotor);	
				//ȥ����ɵ���ʷ����
				AD_data_sum -= AD_hist_data[AD_sample_pos];
				//������µ���ʷ����
				AD_hist_data[AD_sample_pos] =  AD_this_data;
				AD_data_sum += AD_this_data;
				//����ƽ��ֵ		
				AD_filter_out = AD_data_sum >> 3;
				//�ı�����ָ�루ָ����һ��������ݣ�
				AD_sample_pos++;
				if(AD_sample_pos >= sizeof(AD_hist_data))
				{
					AD_sample_pos = 0; 	 	
				}
				//��ȡ������ĵ���ֵ�����������Զ�ֹͣ
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
		//���Ϊ�Ǽ���״̬
		else
		{	
			//û�д����������,�������ģʽ
			if(IR_RecvFlag == 0)
	 		{
				debug_uart_print_string("Box is sleepping...\r\n");	
				MCU_sleep_mode();
				debug_uart_print_string("Box was waked up\r\n");  
			}				
		}
		//������Ҳιһ�ι���
		feed_dog();	
		//�������ݴ���   
		if(IR_RecvFlag)
		{
			//��ӡ�û����������
			debug_uart_print_string("\r\nUser:[");
			debug_uart_print_hex(this_recv_usr_code);
			debug_uart_print_string("]  Cmd:[");
			debug_uart_print_hex(this_recv_cmd_code);
			debug_uart_print_string("]\r\n"); 
		   	//������Ϣ��Ч
			if(IR_info.enable != 0xff)
			{
				//�û�����ȷ
				if(this_recv_usr_code == IR_info.user_code)
				{
					//�򿪺͹ر���������ͬ
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
					//�򿪺�������
					if(this_recv_cmd_code == IR_info.open_code)
					{
						open_Lable:
						//���������ڹر�״̬ת��Ϊ����ʱ1S����ֹ��������
						if(Box_status == STAT_BOX_CLOSING)
						{
							box_stop_in_activity();
							delayS(1);
						}
						filter_init();
						box_open_begin();
						set_times = 0;
					}
					//�رպ�������
					else if(this_recv_cmd_code ==IR_info.close_code)
					{
						close_Lable:
						//���������ڴ�״̬ת��Ϊ�ر���ʱ1S����ֹ��������
						if(Box_status == STAT_BOX_OPENING)
						{
							box_stop_in_activity();
							delayS(1);				
						}
						filter_init();
						box_close_begin();
						set_times = 0;
					}
					//���ú���ң����Ϣ���������ã�
					else if(this_recv_cmd_code == IR_info.set_code)
					{	
						box_stop_in_activity();
						/*set_times++;
						//�������¹ػ���5�Σ��ָ���������
						if(set_times>=5)
						{
							set_times = 0;								
							IR_recovery_factory_mode(&IR_info);
							debug_uart_print_string("Please press set code:\r\n");
						}	 */
					}
				#ifdef _DEBUG_
					//��ȡ��ǰ����ֵ��������ã�
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
			//����ң����ϢΪ��
			else
			{
				switch(set_index)
				{
					//��һ�����ùػ���
					case 0:
						IR_info.set_code = this_recv_cmd_code;
						IR_info.user_code = this_recv_usr_code;
						debug_uart_print_string("Please press open code:\r\n");
						break;
					//�ڶ������ô򿪺��Ӱ���
					case 1:
						IR_info.open_code = this_recv_cmd_code;
						debug_uart_print_string("Please press close code:\r\n");
						break;
					//���������ùرպ��Ӱ���
					case 2:
						IR_info.close_code = this_recv_cmd_code;
						break;
				}
				//������Ϣȫ�����յ�����һ����д�뵽EEPROM�б���				
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
//��������жϷ������
void IR_rceived_int() interrupt 0
{
	u16 result;
	u8 num,bit_n;

	if(IR_RecvFlag != 0)
	{
		return;
	}
	//��ʱ�ر��жϣ���ֹ���ݽ��ճ���
	EX0 = 0;

	//��ιһ�ι���
	feed_dog();

	//���ܺ����������
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
	//ι��
	feed_dog();

	//ȷ��IR�ź��߱��ͷ�
	while(IR_wait_status(1, SIG_STOP, 50000, 50000)!=-1);

	//У������
	if(IR_RecvBuf[0] == ~IR_RecvBuf[1] && IR_RecvBuf[2] == ~IR_RecvBuf[3])
	{
		//���ճɹ����ر��жϵȴ��������
		IR_RecvFlag = 1;
		EX0 = 0;	
	}		
	else
	{
		//���ճ����ָ��ж�׼���´ν���
		debug_uart_print_string_int("code check error!\r\n");
		EX0 = 1;		
	} 	
	IE0 = 0; 	
}
//С�бպϵ���λ����0�жϷ������(��Ҫ)
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
//С�д򿪵���λ����1�жϷ������
void limit_switch1() interrupt 10
{
	if(Box_status == STAT_BOX_OPENING)
	{
		box_open_finish();
	}
	debug_uart_print_string_int("limit switch [1] is trigered!\r\n");
}
//ϵͳʱ���жϷ���
void PCA_isr() interrupt 7 using 1
{
    CCF0 = 0;                       //���жϱ�־
    CCAP0L = value;
    CCAP0H = value >> 8;            //���±Ƚ�ֵ
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