

#define  T0_INT   1                                                        //T中断开启
//#define  T1_INT   1
// #define  T2_INT   1
// #define  T2_CAP   1

extern UINT8 FLAG; 
extern UINT16 Cap[8];

//CH554 Timer0时钟选择   
//bTMR_CLK同时影响Timer0&1&2,使用时要注意 (除定时使用标准时钟)            
#define mTimer0Clk12DivFsys( ) (T2MOD &= ~bT0_CLK)                          //定时器,时钟=Fsys/12 T0标准时钟
#define mTimer0ClkFsys( )      (T2MOD |= bTMR_CLK | bT0_CLK)                //定时器,时钟=Fsys
#define mTimer0Clk4DivFsys( )  (T2MOD &= ~bTMR_CLK;T2MOD |=  bT0_CLK)       //定时器,时钟=Fsys/4
#define mTimer0CountClk( )     (TMOD |= bT0_CT)                             //计数器,T0引脚的下降沿有效

//CH554 Timer0 开始(SS=1)/结束(SS=0)
#define mTimer0RunCTL( SS )    (TR0 = SS ? START : STOP)


#define mTimer1Clk12DivFsys( ) (T2MOD &= ~bT1_CLK)                          //定时器,时钟=Fsys/12  T1标准时钟
#define mTimer1ClkFsys( )      (T2MOD |= bTMR_CLK | bT1_CLK)                //定时器,时钟=Fsys
#define mTimer1Clk4DivFsys( )  (T2MOD &= ~bTMR_CLK;T2MOD |=  bT1_CLK)       //定时器,时钟=Fsys/4
#define mTimer1CountClk( )     (TMOD |= bT1_CT)                             //计数器,T0引脚的下降沿有效

//CH554 Timer1 开始(SS=1)/结束(SS=0)
#define mTimer1RunCTL( SS )    (TR1 = SS ? START : STOP)


#define mTimer2Clk12DivFsys( ) {T2MOD &= ~ bT2_CLK;C_T2 = 0;}      //定时器,时钟=Fsys/12 T2标准时钟
#define mTimer2ClkFsys( )      {T2MOD |= (bTMR_CLK | bT2_CLK);C_T2=0;}         //定时器,时钟=Fsys
#define mTimer2Clk4DivFsys( )  {T2MOD &= ~bTMR_CLK;T2MOD |=  bT2_CLK;C_T2 = 0;}//定时器,时钟=Fsys/4
#define mTimer2CountClk( )     {C_T2 = 1;}                                     //计数器,T2引脚的下降沿有效

//CH554 Timer2 开始(SS=1)/结束(SS=0)
#define mTimer2RunCTL( SS )    {TR2 = SS ? START : STOP;}
#define mTimer2OutCTL( )       (T2MOD |= T2OE)                               //T2输出  频率TF2/2   
#define CAP1Alter( )           (PIN_FUNC |= bT2_PIN_X;)                      //CAP1由P10 映射到P14
#define CAP2Alter( )           (PIN_FUNC |= bT2EX_PIN_X;)                    //CAP2由P11 映射RST

/*******************************************************************************
* Function Name  : mTimer_x_ModInit(UINT8 x ,UINT8 mode)
* Description    : CH554定时计数器x模式设置
* Input          : UINT8 mode,Timer模式选择
                   0：模式0，13位定时器，TLn的高3位无效
                   1：模式1，16位定时器
                   2：模式2，8位自动重装定时器
                   3：模式3，两个8位定时器  Timer0
                   3：模式3，Timer1停止		
                   UINT8 x 定时器  0 1 2
* Output         : None
* Return         : 成功  SUCCESS
                   失败  FAIL
*******************************************************************************/
UINT8 mTimer_x_ModInit(UINT8 x ,UINT8 mode);

/*******************************************************************************
* Function Name  : mTimer_x_SetData(UINT8 x,UINT16 dat)
* Description    : CH554Timer 
* Input          : UINT16 dat;定时器赋值
                   UINT8 x 定时器  0 1 2
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer_x_SetData(UINT8 x,UINT16 dat);

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
void CAP2Init(UINT8 mode);

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
void CAP1Init(UINT8 mode);