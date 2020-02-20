
/********************************** (C) COPYRIGHT *******************************
* File Name          :Composite_Dev.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH559模拟USB复合设备，键鼠，支持类命令 
*******************************************************************************/

#include "./Public/CH554.H"                                                      
#include "./Public/DEBUG.H"
#include <string.h>
#include <stdio.h>

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define CapsLockLED 0x02
#define BUFMAX 16

UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //端点1 IN缓冲区,必须是偶地址
UINT8   SetupReq,SetupLen,Ready,Count,FLAG,UsbConfig,LED_VALID;
PUINT8  pDescr;                                                                //USB配置标志
USB_SETUP_REQ   SetupReqBuf;                                                   //暂存Setup包
bit Ep2InKey;
#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma  NOAREGS
/*设备描述符*/
UINT8C DevDesc[18] = {0x12,0x01,0x10,0x01,0x00,0x00,0x00,0x08,
                      0x3d,0x41,0x07,0x21,0x00,0x00,0x00,0x00,
                      0x00,0x01
                     };
UINT8C CfgDesc[59] =
{
    0x09,0x02,0x22,0x00,0x01,0x01,0x00,0xA0,0x32,             //配置描述符
    0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00,             //接口描述符,键盘
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00,             //HID类描述符
    0x07,0x05,0x81,0x03,0x08,0x00,0x0a,                       //端点描述符
};
/*字符串描述符*/
/*HID类报表描述符*/
UINT8C KeyRepDesc[62] =
{
    0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
    0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
    0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
    0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
    0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
    0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
    0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
    0x00,0x29,0x91,0x81,0x00,0xC0
};

/*键盘数据*/
UINT8 HIDKey[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
UINT8 DataBuffer[BUFMAX];
UINT8 DataLen =0;
UINT8 RecvPoint =0;
UINT8 SendPoint =0;
/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	  IE_USB = 0;
	  USB_CTRL = 0x00;                                                           // 先设定USB设备模式
    UEP0_DMA = Ep0Buffer;                                                      //端点0数据传输地址
    UEP1_DMA = Ep1Buffer;                                                      //端点1数据传输地址
    UEP4_1_MOD = ~(bUEP4_RX_EN | bUEP4_TX_EN |bUEP1_RX_EN | bUEP1_BUF_MOD) | bUEP4_TX_EN;//端点1单64字节收发缓冲区,端点0收发
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT事务返回ACK，IN事务返回NAK
    UEP1_CTRL = bUEP_T_TOG | UEP_T_RES_NAK;
		
	  USB_DEV_AD = 0x00;
	  UDEV_CTRL = bUD_PD_DIS;                                                    // 禁止DP/DM下拉电阻
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
	  UDEV_CTRL |= bUD_PORT_EN;                                                  // 允许USB端口
	  USB_INT_FG = 0xFF;                                                         // 清中断标志
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	  IE_USB = 1;
}
/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey));                              //加载上传数据
    UEP1_T_LEN = sizeof(HIDKey);                                             //上传数据长度
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void    DeviceInterrupt( void ) interrupt INT_NO_USB using 1                      //USB中断服务程序,使用寄存器组1
{
    UINT8 len;
    if(UIF_TRANSFER)                                                            //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 中断端点上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# 中断端点上传
            UEP1_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            FLAG = 1;                                                           /*传输完成标志*/
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP事务
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // 限制总长度
                }
                len = 0;                                                        // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;								
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID类命令 */
                {
									switch( SetupReq ) 
									{
										case 0x01://GetReport
												 break;
										case 0x02://GetIdle
												 break;	
										case 0x03://GetProtocol
												 break;				
										case 0x09://SetReport										
												 break;
										case 0x0A://SetIdle
												 break;	
										case 0x0B://SetProtocol
												 break;
										default:
												 len = 0xFF;  								 					            /*命令不支持*/					
												 break;
								  }	
                }
                else
                {//标准请求
                    switch(SetupReq)                                        //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //设备描述符
                            pDescr = DevDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //配置描述符
                            pDescr = CfgDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                                          //报表描述符
                            if(UsbSetupBuf->wIndexL == 0)                   //接口0报表描述符
                            {
                                pDescr = KeyRepDesc;                        //数据准备上传
                                len = sizeof(KeyRepDesc);
                                Ready = 1;                                  //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
                                IE_UART1 = 1;//开启串口中断															
															
                            }
                            else
                            {
                                len = 0xff;                                 //本程序只有2个接口，这句话正常不可能执行
                            }
                            break;
                        default:
                            len = 0xff;                                     //不支持的命令或者出错
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //限制总长度
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen;                  //本次传输长度
                        memcpy(Ep0Buffer,pDescr,len);                        //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                            // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF;                                                // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* 操作失败 */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* 设置端点 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                               //操作失败
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //操作失败
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //操作失败
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff;                                           //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //包长度错误
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len <= 8)                                                //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                          //本次传输长度
                memcpy( Ep0Buffer, pDescr, len );                            //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if((SetupReq == 0x09)&& (len == 1))
            {
              LED_VALID = Ep0Buffer[0];							
            }
            else if((SetupReq == 0x09) && (len == 8)){//SetReport						 
            }							
            UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA0,返回应答ACK
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //写0清空中断
    }
    if(UIF_BUS_RST)                                                       //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //清中断标志
    }
    if (UIF_SUSPEND)                                                     //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
    }
    else {                                                               //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                               //清中断标志
    }
}

/*******************************************************************************
* Function Name  : QueryUART1Interrupt(void)
* Description    : UART1中断服务程序
*******************************************************************************/
void    QueryUART1Interrupt( void ) interrupt INT_NO_UART1 using 2             //UART1中断服务程序,使用寄存器组1
{
    if(U1RI)
    {
        U1RI = 0;
        DataBuffer[RecvPoint++] = SBUF1;
        DataLen++;
        if(RecvPoint>=BUFMAX)RecvPoint=0;			
    }
    U1TI = 0;
}

/*******************************************************************************
* Function Name  : KeyCodeCorrespond()
* Description    : 键码比对表，由数值对应键盘值
* Input          : UINT8 keyCode UINT8 loc
* Output         : None
* Return         : None
*******************************************************************************/
void KeyCodeCorrespond(UINT8 keyCode,UINT8 loc)
{
  if((keyCode>='a')&&(keyCode<='z')){                                       //键值a-z
    keyCode -= 0x5D; 
    HIDKey[loc] = keyCode;  
    if(LED_VALID&CapsLockLED){	                                            //大写有效
      HIDKey[0]	|= 0x02;	                                                  //shift+	
    }
		return;
  }
  else if((keyCode>='A')&&(keyCode<='Z')){			
    keyCode -= 0x3D; 
    HIDKey[loc] = keyCode;  
    if((LED_VALID&CapsLockLED) == 0){	                                        //大写无效	
      HIDKey[0]	|= 0x02;	                                                  //shift+		
    }
		return;    
  }
	else if((keyCode>='1')&&(keyCode<='9')){
		keyCode -= 0x13;                                                        //字母区数字键
		HIDKey[loc] = keyCode; 
		return;		
	}
	else if(keyCode=='0'){
		HIDKey[loc] = 0x27;   
		return;		
	}		
  else if(keyCode <= 0x2f){		
		if(keyCode == 0x20){
			HIDKey[loc] = 0x2C;                                                     //空格        
		}		
		else if(keyCode == 0x21){//'!'
			HIDKey[loc] = 0x1E; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x22){//'"'
			HIDKey[loc] = 0x34; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x23){//'#'
			HIDKey[loc] = 0x20; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x24){//'$'
			HIDKey[loc] = 0x21; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}	
		else if(keyCode == 0x25){//'%'
			HIDKey[loc] = 0x22; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x26){//'&'
			HIDKey[loc] = 0x24; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x27){//'%'
			HIDKey[loc] = 0x34; 	
		}
		else if(keyCode == 0x28){/*(*/
			HIDKey[loc] = 0x26; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x29){/*)*/
			HIDKey[loc] = 0x27; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x2a){/***/
			HIDKey[loc] = 0x25; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}
		else if(keyCode == 0x2b){/*+*/
			HIDKey[loc] = 0x2e; 
			HIDKey[0]	|= 0x02;	                                                  //shift+		
		}	
		else if(keyCode == 0x2c){/*,*/
			HIDKey[loc] = 0x36; 	
		}	
		else if(keyCode == 0x2d){/*-*/
			HIDKey[loc] = 0x2d; 	
		}	
		else if(keyCode == 0x2e){/*,*/
			HIDKey[loc] = 0x37; 	
		}	
		else if(keyCode == 0x2f){/*/*/
			HIDKey[loc] = 0x38; 		
		}	
  }
  else if(keyCode <= 0x3f){	
		if(keyCode == 0x3a){/*:*/
			HIDKey[loc] = 0x33; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x3b){/*;*/
			HIDKey[loc] = 0x33; 			
		}	
		else if(keyCode == 0x3c){/*<*/
			HIDKey[loc] = 0x36; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x3d){/*=*/
			HIDKey[loc] = 0x2e; 				
		}	
		else if(keyCode == 0x3e){/*>*/
			HIDKey[loc] = 0x37; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x3f){/*?*/
			HIDKey[loc] = 0x38; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
  }
  else if(keyCode == 0x40){/*@*/
    HIDKey[loc] = 0x1f; 	
    HIDKey[0]	|= 0x02;	                                                  //shift+			
  }
  else if((keyCode >= 0x5b)&&(keyCode <= 0x60)){	
		if(keyCode == 0x5b){/*[*/
			HIDKey[loc] = 0x2f; 			
		}	
		else if(keyCode == 0x5c){/*\*/
			HIDKey[loc] = 0x31; 			
		}
		else if(keyCode == 0x5d){/*:*/
			HIDKey[loc] = 0x30; 		
		}	
		else if(keyCode == 0x5e){/*:*/
			HIDKey[loc] = 0x23; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x5f){/*:*/
			HIDKey[loc] = 0x2d; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x60){/*`*/
			HIDKey[loc] = 0x35; 			
		}	
	}
  else if((keyCode >= 0x7b)&&(keyCode <= 0x7f)){
		if(keyCode == 0x7b){/*{*/
			HIDKey[loc] = 0x2f; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x7c){/*|*/
			HIDKey[loc] = 0x31; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x7d){/*}*/
			HIDKey[loc] = 0x30; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x7e){/*~*/
			HIDKey[loc] = 0x35; 	
			HIDKey[0]	|= 0x02;	                                                  //shift+			
		}
		else if(keyCode == 0x7f){/*{*/
			HIDKey[loc] = 0x4c; 			
		}
	}
  else if((keyCode >= 0x80)&&(keyCode <= 0x87)){/*left ctl*/	
    keyCode &= 0x0f; 
    HIDKey[0]	|= (1<<keyCode);	                                          //			
  }
  else if((keyCode >= 0xd7)&&(keyCode <= 0xda)){/**/
    keyCode -= 0x88; 		
    HIDKey[loc]	= keyCode;	                                         	
  }
  else if((keyCode >= 0xb0)&&(keyCode <= 0xb3)){/**/
    keyCode -= 0x88; 		
    HIDKey[loc]	= keyCode;	
  }
  else if((keyCode >= 0xd1)&&(keyCode <= 0xd5)){/**/
    keyCode -= 0x88; 		
    HIDKey[loc]	= keyCode;	                                         	
  }
  else if((keyCode >= 0xC1)&&(keyCode <= 0xCD)){/**/
    keyCode -= 0x88; 		
    HIDKey[loc]	= keyCode;	                                         	
  }
}

void HIDValueHandle()
{
      FLAG = 0;		
      KeyCodeCorrespond(DataBuffer[SendPoint++],2);	
      if(SendPoint>=BUFMAX)SendPoint=0;
      DataLen--;
      if(DataLen&&(DataBuffer[SendPoint]!=DataBuffer[SendPoint-1]))
      {
				KeyCodeCorrespond(DataBuffer[SendPoint++],3);	
				if(SendPoint>=BUFMAX)SendPoint=0;
				DataLen--;				
      }		
      if(DataLen&&(DataBuffer[SendPoint]!=DataBuffer[SendPoint-1])&&(DataBuffer[SendPoint-1]!=DataBuffer[SendPoint-2]))
      {
				KeyCodeCorrespond(DataBuffer[SendPoint++],4);	
				if(SendPoint>=BUFMAX)SendPoint=0;
				DataLen--;				
      }		
      if(DataLen&&(DataBuffer[SendPoint]!=DataBuffer[SendPoint-1])&&(DataBuffer[SendPoint-1]!=DataBuffer[SendPoint-2])&&(DataBuffer[SendPoint-2]!=DataBuffer[SendPoint-3]))
      {
				KeyCodeCorrespond(DataBuffer[SendPoint++],5);	
				if(SendPoint>=BUFMAX)SendPoint=0;
				DataLen--;				
      }				
      Enp1IntIn();
      memset(&HIDKey[0],0,8);//按键结束		
      while(FLAG == 0)
      {;}   /*等待上一包传输完成*/
      FLAG = 0;						
      Enp1IntIn();		
      while(FLAG == 0)
      {;}   /*等待上一包传输完成*/				
}

main()
{
    CfgFsys( );                                                           //CH559时钟选择配置
    mDelaymS(5);                                                          //修改主频等待内部晶振稳定,必加	
    mInitSTDIO( );                                                        //串口0初始化	
    USBDeviceInit();                                                      //USB设备模式初始化
    UART1Setup();
    EA = 1;                                                               //允许单片机中断
    IP_EX = bIP_UART1;//串口优先等级高
    UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    FLAG = 0;
    Ready = 0;
    while(1)
    {
        if(Ready&&DataLen)//恒 = 0xBAE3 = 47843d
        {
            HIDValueHandle();//恒
        }
    }
}
