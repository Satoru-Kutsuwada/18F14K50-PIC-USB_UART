/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/

/** INCLUDES *******************************************************/
#include    <xc.h>
#include    <stdio.h>
//#include    <types.h>
//#include    <wait.h>

#include "system.h"

//#include "app_device_cdc_basic.h"
//#include "app_led_usb_status.h"

#include "usb.h"
#include "usb_device.h"
#include "usb_device_cdc.h"
#include "GenericTypeDefs.h"
 /**

#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"

**/

// コンフィギュレーションの設定
#pragma config CPUDIV = NOCLKDIV// システムクロックの分周はしない
#pragma config USBDIV = OFF		// 分周なしでUSBクロックを直接供給(ロースピード時)
#pragma config FOSC   = HS		// システムクロックは外部で高い振動子を使用する
#pragma config PLLEN  = ON		// 動作クロックを４倍で動作させる
#pragma config PCLKEN = ON		// プライマリシステムクロックを有効にする
#pragma config FCMEN  = OFF		// 外部オシレータに障害が発生した場合に内部オシレータに切替えない
#pragma config IESO   = OFF		// 外部・内部ｸﾛｯｸの切替えでの起動はしない
#pragma config PWRTEN = OFF		// 電源ONから後65.6msにﾌﾟﾛｸﾞﾗﾑを開始させない
#pragma config BOREN  = OFF		// 電源電圧降下常時監視機能はＯＦＦ
#pragma config BORV   = 30		// 監視電圧は(3.0V)に設定
#pragma config WDTEN  = OFF		// ウォッチドッグタイマは使用しない
#pragma config WDTPS  = 32768	// ウォッチドッグタイマ有効時のタイマ値の設定
#pragma config MCLRE  = OFF		// MCLRピンは無効に設定(RA3入力ピンを有効にする)
#pragma config HFOFST = OFF		// オシレータが安定してからシステムクロックを供給する
#pragma config STVREN = ON		// スタックのアンダー・フルのオーバが発生した時にリセットを行う
#pragma config LVP    = OFF		// 低電圧ICSPプログラミングを使用しない(RC3のピンが利用可能になる)
#pragma config XINST  = OFF		// 拡張命令は使用しない

// 変数の定義
#pragma udata					// 変数の格納先を指定(アドレスは指定せず)

//uint8_t     USB_In_Buffer[64] ;       // USBの送信用バッファ
uint8_t     USB_Out_Buffer[64] ;      // USBの受信用バッファ
char     USB_Send_buf[64] ;
int         USB_Send_ptr = -1;


char CharConv[16]={
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'a', 'b', 'c', 'd', 'e', 'f'
};

// 関数プロトタイプの宣言
void ProcessUSB(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void skUSB_ini(void);
void UsbPutString01(char *str, uint8_t flg);
void Wait(uint16_t num);
uint8_t skStrlen(char *str, uint8_t flg);

void UsbPutString03(char *str, uint16_t data, uint8_t flg, uint8_t keta);
void uint2string(char *buf, uint16_t data, uint8_t flg);
void putsUSBUSART(char *data);
void CDCInitEP(void);
void UsbPutStringAll(void);
void PutStringLF(void);
void PutString(char *string);
char GetChar(void);
void PutString03(char *string,uint16_t data, uint16_t flg, uint16_t keta);
void PutString01(char *string,uint16_t flg);

#define _XTAL_FREQ 4000000
/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/


/*******************************************************************************
*  メインの処理                                                                *
*******************************************************************************/
void main(void)
{
     BYTE numBytesRead ;
     BYTE i ;
     char RxData;
     
    // bit7 IDLEN=0 : 0 = SLEEP ?????????????????????
    // bit6-4 IRCF<2:0>: ???????????????
    //      111 = 16 MHz
    //     *110 = 8 MHz
    //      101 = 4 MHz
    //      100 = 2 MHz
    //      011 = 1 MHz(3)
    //      010 = 500 kHz
    //      001 = 250 kHz
    //      000 = 31 kHz(2)
    // bit3 OSTS=1: 1 = ?????CONFIG1 ?????FOSC<2:0> ????????????????
    // bit2 HFIOFS=0: 0 = HFINTOSC ????????????
    // bit1-0 SCS<1:0>: ?????????????
    //      1x = ??????? ????
    // SYNC = 0?BRGH = 1?BRG16 = 0 FOSC = 8.000 MHz
     
     OSCCON = 0b00000000 ;     // 外部クロックとする(12MHz x 4倍 = 48MHz)
                                // OSCCON = 0b01101010;
     
     ANSEL  = 0b00000000 ;     // *ANS3-7 アナログは使用しない、デジタルI/Oに割当
     ANSELH = 0b00000000 ;     // *ANS8-11アナログは使用しない、デジタルI/Oに割当


     
     TRISC  = 0b00000000 ;     // RC0-RC7全て出力に設定 
     PORTA  = 0b00000000 ;     // 出力ピンの初期化(全てLOWにする)
     PORTB  = 0b00000000 ;     // 出力ピンの初期化(全てLOWにする)
     PORTC  = 0b00000000 ;     // 出力ピンの初期化(全てLOWにする)
     
    //============================================================
    // UART設定 モジュールの動作は、以下の3 つのレジスタで制御する
    // • TXSTA ( 送信状態/ 制御) レジスタ
    // • RCSTA ( 受信状態/ 制御) レジスタ
    // • BAUDCTL (baud レート制御) レジスタ
    // RX/DT およびTX/CK ピンに対応するTRIS 制御ビットは「1」にセットしてください。
    // EUSART の制御により、ピン設定は必要に応じて入力から出力に自動的に変更されます。 
    //============================================================
    TRISA  = 0b00000000 ;     // 1で入力 0で出力 RA4-RA5全て出力に設定(RA3は入力専用)
    //TRISA = 0b00100000; // RA5: Input
    
    TRISB  = 0b00000000 ;     // RB4-RB7全て出力に設定 
    //TRISB = 0b00000010; // RX: Input
    
    
    // TXSTA: TXEN(bit5) = 1, SYNC(bit4) = 0, BRGH(bit2)=1
    TXSTA = 0b00100100;
    
    // RCSTA: SPEN(bit7)=1), CREN(bit4)=1)
    RCSTA = 0b10010000;
    
    // BAUDCON
    // BRG16(bit3)=0 
    BAUDCON = 0b00000000;
    
    // SYNC = 0, BRGH = 1, BRG16 = 0 FOSC = 48.000 MHz
    // 19200bps 
    
    SPBRG = 155;
    
    
    skUSB_ini(); 
    PutString01("*******************",1);
    PutString01("*** UART START ****",1);
    PutString01("*******************",1);         

     while(1) {
         
         
        USBDeviceTasks() ;  // USBイベントの確認

        numBytesRead = getsUSBUSART(USB_Out_Buffer,64) ;  // 受信データが有れば取り出す
        if(numBytesRead != 0) {
            // 受信データり
            PutString03("USB-GetChaNumr = ", numBytesRead, 1, 16);
            for (i=0 ; i<numBytesRead ; i++) {           // 受信した個数分繰り返す
                UsbPutString03("GetChar = ", USB_Out_Buffer[i], 1, 16);
                PutString03("USB-GetChar = ", USB_Out_Buffer[i], 1, 16);
            }
        }
        
        //UsbPutStringAll();
        
        
        
        RxData = GetChar();
        
        if(RxData > 0){
            PutString03("UART-Rxdata = ", RxData, 1, 16);
        }
                
    }
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
char GetChar(void)
{
    char rxdt;
    
    if (PIR1bits.RCIF){      // ????????
       rxdt = RCREG;
    }
    else{
        rxdt = 0;
    }
    
    return rxdt;
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void PutString01(char *string,uint16_t flg)
{
    PutString(string);
    if(flg == 1)
        PutStringLF();
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void PutString03(char *string,uint16_t data, uint16_t flg, uint16_t keta)
{
    char buf[20];
    
    PutString(string);
    uint2string(buf, data, keta);
    PutString(buf);
    if(flg == 1)
        PutStringLF();
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void PutString(char *string)

{
    char dummy;
    
    while( *string != (char)NULL ){
        while(TXIF==0){
            
        } 
        
        // ???????????    *1)
        TXREG = *string ;          // ????   
        dummy = TXREG;
        string ++;
    }
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void PutStringLF(void)
{
    char dummy;
        while(TXIF==0){
            
        }     // ???????????    *1)
        TXREG = 0x0d ;          // ????   
        dummy = TXREG;
        
        while(TXIF==0) {
            
        }     // ???????????    *1)
        
        TXREG = 0x0a ;          // ????  
        dummy = TXREG;
}
/*******************************************************************************
*  ProcessUSB()                                                                *
*    ＵＳＢで送受信するメインの処理                                            *
*******************************************************************************/

/****
void ProcessUSB(void)
{
     BYTE numBytesRead ;
     BYTE i ;

     // ＵＳＢが通信可能になるまで待つ
     if ( (USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1) ) return ;
     
    
     // データを受信したら"受信データ＋１"で送り返す処理
     if (USBUSARTIsTxTrfReady()) {      // ホストにデータを送信する準備ができているかどうかをチェックする
          numBytesRead = getsUSBUSART(USB_Out_Buffer,64) ;  // 受信データが有れば取り出す
          if(numBytesRead != 0) {
               for (i=0 ; i<numBytesRead ; i++) {           // 受信した個数分繰り返す
                    switch(USB_Out_Buffer[i]) {
                         case 0x0A:
                         case 0x0D:
                              USB_In_Buffer[i] = USB_Out_Buffer[i] ;       // 改行コードはそのまま移す
                              break ;
                         default:
                              USB_In_Buffer[i] = USB_Out_Buffer[i] + 2 ;   // 他のコードは＋１して移す
                              break ;
                    }
               }
               // ＵＳＢホストへデータを送信する
               putUSBUSART(USB_In_Buffer,numBytesRead) ;
          }
     }

    // デバイスからホストへのトランザクションを処理します
    CDCTxService() ;   //CDCIniEP() 関数は既に実行されている必要があります/デバイスは CONFIGURED_STATE になっている必要があります。
}
***/

//==============================================================================
// Waite time = num x 10ms
//==============================================================================
void Wait(uint16_t num)
{
     int i ;

     for (i=0 ; i < num ; i++) {
          __delay_ms(10) ;
     }
}
//==============================================================================
//
//==============================================================================

void    skUSB_ini(void)
{
    BYTE numBytesRead ;
    
    USBDeviceInit() ;         // ＵＳＢの初期化を行う(フルスピードで内部プルアップ有り)
    CDCInitEP();
    
    
    /*****
    Wait(10);
    
    USBDeviceTasks() ;  // USBイベントの確認
    Wait(10);
    
    while ( (USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1) ){
        USBDeviceTasks() ;
        Wait(1);
    }

    if (!USBUSARTIsTxTrfReady()) {      // ホストにデータを送信する準備ができているかどうかをチェックする
        USBDeviceTasks() ;
        Wait(1);
        CDCTxService() ;
    }

    numBytesRead = getsUSBUSART(USB_Out_Buffer,64) ;  // 受信データが有れば取り出す
    UsbPutString01("*********1**********",1);
    Wait(1);
    
    while ( (USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1) ){
        USBDeviceTasks() ;
        Wait(1);
    }
    if (!USBUSARTIsTxTrfReady()) {      // ホストにデータを送信する準備ができているかどうかをチェックする
        USBDeviceTasks() ;
        Wait(1);
        CDCTxService() ;
    }
    numBytesRead = getsUSBUSART(USB_Out_Buffer,64) ;  // 受信データが有れば取り出す
    UsbPutString01("*** UART START ****",1);
    Wait(1);
    
    while ( (USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1) ){
        USBDeviceTasks() ;
        Wait(1);
    }
    if (!USBUSARTIsTxTrfReady()) {      // ホストにデータを送信する準備ができているかどうかをチェックする
        USBDeviceTasks() ;
        Wait(1);
        CDCTxService() ;
    }
    numBytesRead = getsUSBUSART(USB_Out_Buffer,64) ;  // 受信データが有れば取り出す
    UsbPutString01("*********2**********",1);
     
     *****/

}
//==============================================================================
//
//==============================================================================
uint8_t  skStrlen(char *str, uint8_t flg)
{
    uint8_t rtn;
    uint8_t i;
    
    rtn = 0;
    while( *str != (char)NULL )
    {
        str ++;
        rtn ++;
    }
    
    if( flg == 1){
        *str = 0x0d;
        str ++;
        rtn ++;
        *str = 0x0a;
        str ++;
        rtn ++;
        *str = 0x00;
    }
    
    return rtn;
    
}

//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void UsbPutString01(char *str, uint8_t flg)
{
    uint8_t  num,i;
    char *string;
    
    string = str;
    
    num = skStrlen( string , flg );    
    // putUSBUSART( str, num);
    
    if(USBUSARTIsTxTrfReady()){
        putsUSBUSART(str);
        CDCTxService() ;
    }
    else{
        if(USB_Send_ptr == -1){
            USB_Send_ptr = 0;
        }
        for(i=0;i<num;i++){
            USB_Send_buf[USB_Send_ptr]=*string;
            USB_Send_ptr++;
            string++;
            if(USB_Send_ptr >255 )
                break;
        }
    }
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void UsbPutString03(char *str, uint16_t data, uint8_t flg, uint8_t keta)
{
    uint8_t  i;
    uint8_t  num;
    char string[30];
    
    i=0;
    while( *str != (char)NULL )
    {
        string[i]=*str;
        str ++;
        i ++;
    }

    uint2string( &string[i], data, keta);
    num = skStrlen( string , flg );   
    
            
        putsUSBUSART(string);
        CDCTxService() ;
        /*************  
    if(cdc_trf_state == CDC_TX_READY)
    {
        putsUSBUSART(string);
        CDCTxService() ;
    }
    
    
    else{
        if(USB_Send_ptr == -1){
            USB_Send_ptr = 0;
        }
        
        i=0;
        do{
            USB_Send_buf[USB_Send_ptr] = string[i];
            USB_Send_ptr++;
            i++;
            if(USB_Send_ptr >64 )
                break;
        }while(string[i]);
        USB_Send_buf[USB_Send_ptr] = 0;
        
    }
    **************/
         
}
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void UsbPutStringAll(void)
{
    if (USBUSARTIsTxTrfReady()) { 
        if(USB_Send_ptr != -1){
            putsUSBUSART(USB_Send_buf);
            USB_Send_ptr = -1;
            CDCTxService() ;
        }
    }    
}



//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
void uint2string(char *buf, uint16_t data, uint8_t flg)
{
    uint16_t dt;
    uint16_t i;
    uint16_t sw;
    
    sw = 0;
    
    dt = data;
    if( flg == 10){
        for(i=10000; i>0; i/=10){
            dt = dt/i;
            if(sw==0){
                if(dt != 0){
                    *buf = CharConv[dt];
                    sw=1;
                }
            }
            else{
                    *buf = CharConv[dt];            
            }
            dt = data - dt*i;
            buf ++;
        }
    }
    else{
        *buf = '0';
        buf ++;
        *buf = 'x';
        buf ++;

        dt = (data >> 12)& 0x0f;
        *buf = CharConv[dt];
        buf ++;
        
        dt = (data >> 8)& 0x0f;
        *buf = CharConv[dt];
        buf ++;
        
        dt = (data >> 4)& 0x0f;
        *buf = CharConv[dt];
        buf ++;
        
        dt = data & 0x0f;
        *buf = CharConv[dt];
        buf ++;
                
    }
    
    *buf =(char) NULL;
    
}






/***
MAIN_RETURN main(void)
{
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();
    
    while(1)
    {
        SYSTEM_Tasks();

        #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif



    }
    

}//end main
**/
/*******************************************************************************
 End of File
*/
