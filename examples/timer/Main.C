
/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.1
* Date               : 2017/09/05
* Description        : CH554 Time 初始化、定时器、计数器赋值，T2捕捉功能等
                       定时器中断处理 
*******************************************************************************/
#include ".\Public\CH554.H"                                                  
#include ".\Public\Debug.H"
#include "Timer.H"
#include "stdio.h"

#pragma  NOAREGS

UINT8 FLAG;
UINT16 Cap[8] = {0};

main( ) 
{
    CfgFsys( );                                                                //CH554时钟选择配置   
    mDelaymS(5);                                                               //修改主频，建议稍加延时等待主频稳定 	
    mInitSTDIO( );                                                             //串口初始化
    printf("start ...\n"); 


#ifdef T0_INT
    printf("T0 Test ...\n"); 
    mTimer0Clk12DivFsys();	                                                   //T0定时器时钟设置
    mTimer_x_ModInit(0,2);                                                     //T0 定时器模式设置
    mTimer_x_SetData(0,0x5555);	                                               //T0定时器赋值
    mTimer0RunCTL(1);                                                          //T0定时器启动	
    ET0 = 1;                                                                   //T0定时器中断开启		
    EA = 1;
    while(1);
#endif	

#ifdef T1_INT
    printf("T1 Test ...\n"); 
    mTimer1Clk12DivFsys();	                                                   //T1定时器时钟设置	
    mTimer_x_ModInit(1,2);                                                     //T1 定时器模式设置
    mTimer_x_SetData(1,0xEEEE);	                                               //T1定时器赋值
    mTimer1RunCTL(1);                                                          //T1定时器启动		
    ET1 = 1;                                                                   //T1定时器中断开启		
    EA = 1;
    while(1); 
#endif	


#ifdef T2_INT
    printf("T2 Test ...\n"); 
    mTimer2ClkFsys();	                                                         //T2定时器时钟设置
    mTimer_x_ModInit(2,0);                                                     //T2 定时器模式设置
    mTimer_x_SetData(2,0xAAAA);	                                               //T2定时器赋值
    mTimer2RunCTL(1);                                                          //T2定时器启动			
    ET2 = 1;                                                                   //T2定时器中断开启		
    EA = 1;
    while(1); 
#endif


#ifdef T2_CAP
    printf("T2_CAP Test ...\n"); 
    mTimer2ClkFsys();                                                         //T2定时器时钟设置
    mTimer_x_SetData(2,0);                                                    //T2 定时器模式设置捕捉模式		
    CAP2Init(1);	                                                            //T2 CAP2设置，任意沿捕捉
    CAP1Init(1);	                                                            //T2 CAP2设置，任意沿捕捉
    mTimer2RunCTL(1);                                                         //T2定时器启动			
    ET2 = 1;                                                                  //T2定时器中断开启	
    EA = 1;
    while(1); 		
#endif
		
    while(1);
}