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

#include "system.h"

#include "app_device_cdc_basic.h"
#include "app_led_usb_status.h"

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

char USB_In_Buffer[64] ;       // USBの送信用バッファ
char USB_Out_Buffer[64] ;      // USBの受信用バッファ

// 関数プロトタイプの宣言
void ProcessUSB(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);


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

     OSCCON = 0b00000000 ;     // 外部クロックとする(12MHz x 4倍 = 48MHz)
     ANSEL  = 0b00000000 ;     // ANS3-7 アナログは使用しない、デジタルI/Oに割当
     ANSELH = 0b00000000 ;     // ANS8-11アナログは使用しない、デジタルI/Oに割当
     TRISA  = 0b00000000 ;     // 1で入力 0で出力 RA4-RA5全て出力に設定(RA3は入力専用)
     TRISB  = 0b00000000 ;     // RB4-RB7全て出力に設定 
     TRISC  = 0b00000000 ;     // RC0-RC7全て出力に設定 
     PORTA  = 0b00000000 ;     // 出力ピンの初期化(全てLOWにする)
     PORTB  = 0b00000000 ;     // 出力ピンの初期化(全てLOWにする)
     PORTC  = 0b00000000 ;     // 出力ピンの初期化(全てLOWにする)

     USBDeviceInit() ;         // ＵＳＢの初期化を行う(フルスピードで内部プルアップ有り)

     while(1) {
          // 割込みで処理する場合
          #if defined(USB_INTERRUPT)
               if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) {
                    USBDeviceAttach() ;
               }
          #endif
          // USBホストからSETUPパケットを受けてポーリングで処理する場合
          #if defined(USB_POLLING)
               USBDeviceTasks() ;
          #endif
          // ＵＳＢのメイン処理
          ProcessUSB() ;
    }
}
/*******************************************************************************
*  ProcessUSB()                                                                *
*    ＵＳＢで送受信するメインの処理                                            *
*******************************************************************************/
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
                              USB_In_Buffer[i] = USB_Out_Buffer[i] + 1 ;   // 他のコードは＋１して移す
                              break ;
                    }
               }
               // ＵＳＢホストへデータを送信する
               putUSBUSART(USB_In_Buffer,numBytesRead) ;
          }
     }

    // デバイスからホストへのトランザクションを処理します
    CDCTxService() ;
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
