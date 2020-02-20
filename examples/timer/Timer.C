
/********************************** (C) COPYRIGHT *******************************
* File Name          : Timer.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH554 Time 初始化、定时器、计数器赋值、T2捕捉功能开启函数等
                       定时器中断函数          		 		   
*******************************************************************************/
#include ".\Public\CH554.H"                                                  
#include ".\Public\Debug.H"
#include "Timer.H" 
#include "stdio.h"

#pragma  NOAREGS

/*******************************************************************************
* Function Name  : mTimer_x_ModInit(UINT8 x ,UINT8 mode)
* Description    : CH554定时计数器x模式设置
* Input          : UINT8 mode,Timer模式选择
                   0：模式0，13位定时器，TLn的高3位无效
                   1：模式1，16位定时器
                   2：模式2，8位自动重装定时器
                   3：模式3，两个8位定时器  Timer0
                   3：模式3，Timer1停止									 
* Output         : None
* Return         : 成功  SUCCESS
                   失败  FAIL
*******************************************************************************/
UINT8 mTimer_x_ModInit(UINT8 x ,UINT8 mode)
{
    if(x == 0){TMOD = TMOD & 0xf0 | mode;}
    else if(x == 1){TMOD = TMOD & 0x0f | (mode<<4);}
    else if(x == 2){RCLK = 0;TCLK = 0;CP_RL2 = 0;}                               //16位自动重载定时器
    else return FAIL;
    return SUCCESS;
}

/*******************************************************************************
* Function Name  : mTimer_x_SetData(UINT8 x,UINT16 dat)
* Description    : CH554Timer0 TH0和TL0赋值
* Input          : UINT16 dat;定时器赋值
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer_x_SetData(UINT8 x,UINT16 dat)
{
    UINT16 tmp;
    tmp = 65536 - dat;	
		if(x == 0){TL0 = tmp & 0xff;TH0 = (tmp>>8) & 0xff;}
		else if(x == 1){TL1 = tmp & 0xff;TH1 = (tmp>>8) & 0xff;}
		else if(x == 2){
      RCAP2L = TL2 = tmp & 0xff;                                               //16位自动重载定时器
      RCAP2H = TH2 = (tmp>>8) & 0xff;
    }                                                 
}

/*******************************************************************************
* Function Name  : CAP2Init(UINT8 mode)
* Description    : CH554定时计数器2 T2EX引脚捕捉功能初始化
                   UINT8 mode,边沿捕捉模式选择
                   0:T2ex从下降沿到下一个下降沿
                   1:T2ex任意边沿之间
                   3:T2ex从上升沿到下一个上升沿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP2Init(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;	
    C_T2  = 0;
    EXEN2 = 1; 
    CP_RL2 = 1;                                                                //启动T2ex的捕捉功能
    T2MOD |= mode << 2;                                                        //边沿捕捉模式选择
}

/*******************************************************************************
* Function Name  : CAP1Init(UINT8 mode)
* Description    : CH554定时计数器2 T2引脚捕捉功能初始化T2
                   UINT8 mode,边沿捕捉模式选择
                   0:T2ex从下降沿到下一个下降沿
                   1:T2ex任意边沿之间
                   3:T2ex从上升沿到下一个上升沿
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP1Init(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 1;
    C_T2 = 0;
    T2MOD = T2MOD & ~T2OE | (mode << 2) | bT2_CAP1_EN;                         //使能T2引脚捕捉功能,边沿捕捉模式选择
}

#ifdef T0_INT
/*******************************************************************************
* Function Name  : mTimer0Interrupt()
* Description    : CH554定时计数器0定时计数器中断处理函数
*******************************************************************************/
void	mTimer0Interrupt( void ) interrupt INT_NO_TMR0 using 1                //timer0中断服务程序,使用寄存器组1
{                                                                           //方式3时，TH0使用Timer1的中断资源
    SCK = ~SCK;
		printf("t0 interrupt\n");
//     mTimer_x_SetData(0,0x0000);                                          //非自动重载方式需重新给TH0和TL0赋值      
}
#endif

#ifdef T1_INT
/*******************************************************************************
* Function Name  : mTimer1Interrupt()
* Description    : CH554定时计数器0定时计数器中断处理函数
*******************************************************************************/
void	mTimer1Interrupt( void ) interrupt INT_NO_TMR1 using 2                //timer1中断服务程序,使用寄存器组2
{                                                                           //方式3时，Timer1停止
    SCK = ~SCK;
//     mTimer_x_SetData(1,0x0000);                                          //非自动重载方式需重新给TH1和TL1赋值      
}
#endif

#ifdef T2_INT
/*******************************************************************************
* Function Name  : mTimer2Interrupt()
* Description    : CH554定时计数器0定时计数器中断处理函数
*******************************************************************************/
void	mTimer2Interrupt( void ) interrupt INT_NO_TMR2 using 3                //timer2中断服务程序,使用寄存器组3
{                                                                             
    mTimer2RunCTL( 0 );                                                     //关定时器
#ifdef  T2_CAP   
    if(EXF2)                                                                //T2ex电平变化中断中断标志
    {
        SCK = ~SCK;                                                         //P17电平指示监控
        Cap[FLAG++] = RCAP2;                                                //T2EX
        printf("RCAP2 %04x  \n",RCAP2);                                  			
        EXF2 = 0;                                                           //清空T2ex捕捉中断标志		
    }
    if(CAP1F)                                                               //T2电平捕捉中断标志
    {
        Cap[FLAG++] = T2CAP1;                                               //T2;	  	
        printf("T2CAP1 %04x  \n",T2CAP1);				
        CAP1F = 0;                                                          //清空T2捕捉中断标志
    }
#endif	
    if(TF2)
    {
        TF2 = 0;                                                             //清空定时器2溢出中断	                                                      
        SCK = ~SCK;                                                          //P17电平指示监控
    }
    mTimer2RunCTL( 1 );                                                      //开定时器   
}
#endif
