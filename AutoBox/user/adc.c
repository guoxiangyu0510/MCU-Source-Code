																					 /*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC15F4K60S4 系列 AD转换查询方式举例----------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966-------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.GXWMCU.com --------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序        */
/* 如果要在文章中应用此代码,请在文章中注明使用了STC的资料及程序        */
/*---------------------------------------------------------------------*/

//本示例在Keil开发环境下请选择Intel的8058芯片型号进行编译
//若无特别说明,工作频率一般为11.0592MHz


#include "adc.h"

#define FOSC    11059200L
#define BAUD    9600

#define ADC_POWER   0x80            //ADC电源控制位
#define ADC_FLAG    0x10            //ADC完成标志
#define ADC_START   0x08            //ADC起始控制位
#define ADC_SPEEDLL 0x00            //540个时钟
#define ADC_SPEEDL  0x20            //360个时钟
#define ADC_SPEEDH  0x40            //180个时钟
#define ADC_SPEEDHH 0x60            //90个时钟

void InitADC();
void Delay(u16 n);
u8 GetADCResult(void);

/*----------------------------
读取ADC结果
----------------------------*/
u8 GetADCResult(void)
{
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL | 0x02 | ADC_START;
    _nop_();                        //等待4个NOP
    _nop_();
    _nop_();
    _nop_();
    while (!(ADC_CONTR & ADC_FLAG));//等待ADC转换完成
    ADC_CONTR &= ~ADC_FLAG;         //Close ADC

    return ADC_RES;                 //返回ADC结果
}

/*----------------------------
初始化ADC
----------------------------*/
void adc_init(void)
{
    P1ASF = 0x04;                   //设置P1^2口为AD口
    ADC_RES = 0;                    //清除结果寄存器
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL;
    Delay(2);                       //ADC上电并延时
}


/*----------------------------
软件延时
----------------------------*/
void Delay(u16 n)
{
    u16 x;

    while (n--)
    {
        x = 5000;
        while (x--);
    }
}

