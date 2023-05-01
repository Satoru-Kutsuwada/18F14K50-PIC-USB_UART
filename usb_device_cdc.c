// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 Microchip Technology Inc. (www.microchip.com)

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
//DOM-IGNORE-END

/********************************************************************
 Change History:
  Rev    Description
  ----   -----------
  2.3    Deprecated the mUSBUSARTIsTxTrfReady() macro.  It is
         replaced by the USBUSARTIsTxTrfReady() function.

  2.6    Minor definition changes

  2.6a   No Changes

  2.7    Fixed error in the part support list of the variables section
         where the address of the CDC variables are defined.  The
         PIC18F2553 was incorrectly named PIC18F2453 and the PIC18F4558
         was incorrectly named PIC18F4458.

         http://www.microchip.com/forums/fb.aspx?m=487397

  2.8    Minor change to CDCInitEP() to enhance ruggedness in
         multi0-threaded usage scenarios.

  2.9b   Updated to implement optional support for DTS reporting.

********************************************************************/

/** I N C L U D E S **********************************************************/
#include "system.h"
#include "usb.h"
#include "usb_device_cdc.h"

#ifdef USB_USE_CDC

#ifndef FIXED_ADDRESS_MEMORY
    #define IN_DATA_BUFFER_ADDRESS_TAG
    #define OUT_DATA_BUFFER_ADDRESS_TAG
    #define CONTROL_BUFFER_ADDRESS_TAG
    #define DRIVER_DATA_ADDRESS_TAG
#endif

#if !defined(IN_DATA_BUFFER_ADDRESS_TAG) || !defined(OUT_DATA_BUFFER_ADDRESS_TAG) || !defined(CONTROL_BUFFER_ADDRESS_TAG) || !defined(DRIVER_DATA_ADDRESS_TAG)
    #error "One of the fixed memory address definitions is not defined.  Please define the required address tags for the required buffers."
#endif

/** V A R I A B L E S ********************************************************/
volatile unsigned char cdc_data_tx[CDC_DATA_IN_EP_SIZE] IN_DATA_BUFFER_ADDRESS_TAG;
volatile unsigned char cdc_data_rx[CDC_DATA_OUT_EP_SIZE] OUT_DATA_BUFFER_ADDRESS_TAG;

typedef union
{
    LINE_CODING lineCoding;
    CDC_NOTICE cdcNotice;
} CONTROL_BUFFER;

//static CONTROL_BUFFER controlBuffer CONTROL_BUFFER_ADDRESS_TAG;

LINE_CODING line_coding;    // Buffer to store line coding information
CDC_NOTICE cdc_notice;

#if defined(USB_CDC_SUPPORT_DSR_REPORTING)
    SERIAL_STATE_NOTIFICATION SerialStatePacket DRIVER_DATA_ADDRESS_TAG;
#endif

uint8_t cdc_rx_len;            // total rx length
uint8_t cdc_trf_state;         // States are defined cdc.h
POINTER pCDCSrc;            // Dedicated source pointer
POINTER pCDCDst;            // Dedicated destination pointer
uint8_t cdc_tx_len;            // total tx length
uint8_t cdc_mem_type;          // _ROM, _RAM

USB_HANDLE CDCDataOutHandle;
USB_HANDLE CDCDataInHandle;


CONTROL_SIGNAL_BITMAP control_signal_bitmap;
uint32_t BaudRateGen;			// BRG value calculated from baud rate

#if defined(USB_CDC_SUPPORT_DSR_REPORTING)
    BM_SERIAL_STATE SerialStateBitmap;
    BM_SERIAL_STATE OldSerialStateBitmap;
    USB_HANDLE CDCNotificationInHandle;
#endif

/**************************************************************************
  SEND_ENCAPSULATED_COMMAND and GET_ENCAPSULATED_RESPONSE are required
  requests according to the CDC specification.
  However, it is not really being used here, therefore a dummy buffer is
  used for conformance.
 **************************************************************************/
#define dummy_length    0x08
uint8_t dummy_encapsulated_cmd_response[dummy_length];

#if defined(USB_CDC_SET_LINE_CODING_HANDLER)
CTRL_TRF_RETURN USB_CDC_SET_LINE_CODING_HANDLER(CTRL_TRF_PARAMS);
#endif

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void USBCDCSetLineCoding(void);

/** D E C L A R A T I O N S **************************************************/
//#pragma code

/** C L A S S  S P E C I F I C  R E Q ****************************************/
/******************************************************************************
 	Function:
 		void USBCheckCDCRequest(void)

 	Description:
 		This routine checks the most recently received SETUP data packet to
 		see if the request is specific to the CDC class.  If the request was
 		a CDC specific request, this function will take care of handling the
 		request and responding appropriately.

 	PreCondition:
 		This function should only be called after a control transfer SETUP
 		packet has arrived from the host.

	Parameters:
		None

	Return Values:
		None

	Remarks:
		This function does not change status or do anything if the SETUP packet
		did not contain a CDC class specific request.
  *****************************************************************************/
void USBCheckCDCRequest(void)
{
    /*
     * If request recipient is not an interface then return
     */
    if(SetupPkt.Recipient != USB_SETUP_RECIPIENT_INTERFACE_BITFIELD) return;

    /*
     * If request type is not class-specific then return
     */
    if(SetupPkt.RequestType != USB_SETUP_TYPE_CLASS_BITFIELD) return;

    /*
     * Interface ID must match interface numbers associated with
     * CDC class, else return
     */
    if((SetupPkt.bIntfID != CDC_COMM_INTF_ID)&&
       (SetupPkt.bIntfID != CDC_DATA_INTF_ID)) return;

    switch(SetupPkt.bRequest)
    {
        //****** These commands are required ******//
        case SEND_ENCAPSULATED_COMMAND:
         //send the packet
            inPipes[0].pSrc.bRam = (uint8_t*)&dummy_encapsulated_cmd_response;
            inPipes[0].wCount.Val = dummy_length;
            inPipes[0].info.bits.ctrl_trf_mem = USB_EP0_RAM;
            inPipes[0].info.bits.busy = 1;
            break;
        case GET_ENCAPSULATED_RESPONSE:
            // Populate dummy_encapsulated_cmd_response first.
            inPipes[0].pSrc.bRam = (uint8_t*)&dummy_encapsulated_cmd_response;
            inPipes[0].info.bits.busy = 1;
            break;
        //****** End of required commands ******//

        #if defined(USB_CDC_SUPPORT_ABSTRACT_CONTROL_MANAGEMENT_CAPABILITIES_D1)
        case SET_LINE_CODING:
            outPipes[0].wCount.Val = SetupPkt.wLength;
            outPipes[0].pDst.bRam = (uint8_t*)LINE_CODING_TARGET;
            outPipes[0].pFunc = LINE_CODING_PFUNC;
            outPipes[0].info.bits.busy = 1;
            break;

        case GET_LINE_CODING:
            USBEP0SendRAMPtr(
                (uint8_t*)&line_coding,
                LINE_CODING_LENGTH,
                USB_EP0_INCLUDE_ZERO);
            break;

        case SET_CONTROL_LINE_STATE:
            control_signal_bitmap._byte = (uint8_t)SetupPkt.wValue;
            //------------------------------------------------------------------
            //One way to control the RTS pin is to allow the USB host to decide the value
            //that should be output on the RTS pin.  Although RTS and CTS pin functions
            //are technically intended for UART hardware based flow control, some legacy
            //UART devices use the RTS pin like a "general purpose" output pin
            //from the PC host.  In this usage model, the RTS pin is not related
            //to flow control for RX/TX.
            //In this scenario, the USB host would want to be able to control the RTS
            //pin, and the below line of code should be uncommented.
            //However, if the intention is to implement true RTS/CTS flow control
            //for the RX/TX pair, then this application firmware should override
            //the USB host's setting for RTS, and instead generate a real RTS signal,
            //based on the amount of remaining buffer space available for the
            //actual hardware UART of this microcontroller.  In this case, the
            //below code should be left commented out, but instead RTS should be
            //controlled in the application firmware responsible for operating the
            //hardware UART of this microcontroller.
            //---------
            //CONFIGURE_RTS(control_signal_bitmap.CARRIER_CONTROL);
            //------------------------------------------------------------------

            #if defined(USB_CDC_SUPPORT_DTR_SIGNALING)
                if(control_signal_bitmap.DTE_PRESENT == 1)
                {
                    UART_DTR = USB_CDC_DTR_ACTIVE_LEVEL;
                }
                else
                {
                    UART_DTR = (USB_CDC_DTR_ACTIVE_LEVEL ^ 1);
                }
            #endif
            inPipes[0].info.bits.busy = 1;
            break;
        #endif

        #if defined(USB_CDC_SUPPORT_ABSTRACT_CONTROL_MANAGEMENT_CAPABILITIES_D2)
        case SEND_BREAK:                        // Optional
            inPipes[0].info.bits.busy = 1;
			if (SetupPkt.wValue == 0xFFFF)  //0xFFFF means send break indefinitely until a new SEND_BREAK command is received
			{
				UART_Tx = 0;       // Prepare to drive TX low (for break signaling)
				UART_TRISTx = 0;   // Make sure TX pin configured as an output
				UART_ENABLE = 0;   // Turn off USART (to relinquish TX pin control)
			}
			else if (SetupPkt.wValue == 0x0000) //0x0000 means stop sending indefinite break
			{
    			UART_ENABLE = 1;   // turn on USART
				UART_TRISTx = 1;   // Make TX pin an input
			}
			else
			{
                //Send break signaling on the pin for (SetupPkt.wValue) milliseconds
                UART_SEND_BREAK();
			}
            break;
        #endif
        default:
            break;
    }//end switch(SetupPkt.bRequest)

}//end USBCheckCDCRequest

/** U S E R  A P I ***********************************************************/

/**************************************************************************
■Function:
        void CDCInitEP(void)

■Summary:
この関数は、CDC 関数ドライバーを初期化します。 この関数は、SET_CONFIGURATION
 コマンドの後に呼び出す必要があります (例: USBCBInitEP() 関数のコンテキスト内)。
■Description:
この関数は、CDC 関数ドライバーを初期化します。 この関数は、デフォルトのライン コーディング 
(ボー レート、ビット パリティ、データ ビット数、およびフォーマット) を設定します。 
この関数はまた、エンドポイントを有効にし、ホストからの最初の転送に備えます。
この関数は、SET_CONFIGURATION コマンドの後に呼び出す必要があります。
これは、USBCBInitEP() 関数からこの関数を呼び出すことによって最も簡単に実行できます。

■Typical Usage:
    <code>
        void USBCBInitEP(void)
        {
            CDCInitEP();
        }
    </code>
■Conditions:
    None
■Remarks:
    None
  **************************************************************************/
void CDCInitEP(void)
{
    //Abstract line coding information
    line_coding.dwDTERate   = 19200;      // baud rate
    line_coding.bCharFormat = 0x00;             // 1 stop bit
    line_coding.bParityType = 0x00;             // None
    line_coding.bDataBits = 0x08;               // 5,6,7,8, or 16

    cdc_rx_len = 0;

    /*
     * Do not have to init Cnt of IN pipes here.
     * Reason:  Number of BYTEs to send to the host
     *          varies from one transaction to
     *          another. Cnt should equal the exact
     *          number of BYTEs to transmit for
     *          a given IN transaction.
     *          This number of BYTEs will only
     *          be known right before the data is
     *          sent.
     */
    USBEnableEndpoint(CDC_COMM_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(CDC_DATA_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    CDCDataOutHandle = USBRxOnePacket(CDC_DATA_EP,(uint8_t*)&cdc_data_rx,sizeof(cdc_data_rx));
    CDCDataInHandle = NULL;

    #if defined(USB_CDC_SUPPORT_DSR_REPORTING)
      	CDCNotificationInHandle = NULL;
        mInitDTSPin();  //Configure DTS as a digital input
      	SerialStateBitmap.byte = 0x00;
      	OldSerialStateBitmap.byte = !SerialStateBitmap.byte;    //To force firmware to send an initial serial state packet to the host.
        //Prepare a SerialState notification element packet (contains info like DSR state)
        SerialStatePacket.bmRequestType = 0xA1; //Always 0xA1 for this type of packet.
        SerialStatePacket.bNotification = SERIAL_STATE;
        SerialStatePacket.wValue = 0x0000;  //Always 0x0000 for this type of packet
        SerialStatePacket.wIndex = CDC_COMM_INTF_ID;  //Interface number
        SerialStatePacket.SerialState.byte = 0x00;
        SerialStatePacket.Reserved = 0x00;
        SerialStatePacket.wLength = 0x02;   //Always 2 bytes for this type of packet
        CDCNotificationHandler();
  	#endif

  	#if defined(USB_CDC_SUPPORT_DTR_SIGNALING)
  	    mInitDTRPin();
  	#endif

  	#if defined(USB_CDC_SUPPORT_HARDWARE_FLOW_CONTROL)
  	    mInitRTSPin();
  	    mInitCTSPin();
  	#endif

    cdc_trf_state = CDC_TX_READY;
}//end CDCInitEP


/**************************************************************************
■Function: void CDCNotificationHandler(void)

■Summary: 
DSR ステータスの変化をチェックし、USB ホストに報告します。

■Description: 
DSR ピンの状態の変化をチェックし、変化を USB ホストに報告します。

■Conditions: 
CDCInitEP() は、CDCNotificationHandler() を初めて呼び出す前に、以前に呼び出されている必要があります。

■Remarks:
この関数は、USB_CDC_SUPPORT_DSR_REPORTING オプションが有効になっている場合にのみ実装され、
必要になります。 この関数が有効になっている場合は、定期的に呼び出して DSR ピンをサンプリングし、
情報を USB ホストにフィードする必要があります。 これは、 CDCNotificationHandler() を
単独で呼び出すか、必要に応じて内部で CDCNotificationHandler() も呼び出す CDCTxService() を呼び出すことで実行できます。
**************************************************************************/
#if defined(USB_CDC_SUPPORT_DSR_REPORTING)
void CDCNotificationHandler(void)
{
    //Check the DTS I/O pin and if a state change is detected, notify the
    //USB host by sending a serial state notification element packet.
    if(UART_DTS == USB_CDC_DSR_ACTIVE_LEVEL) //UART_DTS must be defined to be an I/O pin in the hardware profile to use the DTS feature (ex: "PORTXbits.RXY")
    {
        SerialStateBitmap.bits.DSR = 1;
    }
    else
    {
        SerialStateBitmap.bits.DSR = 0;
    }

    //If the state has changed, and the endpoint is available, send a packet to
    //notify the hUSB host of the change.
    if((SerialStateBitmap.byte != OldSerialStateBitmap.byte) && (!USBHandleBusy(CDCNotificationInHandle)))
    {
        //Copy the updated value into the USB packet buffer to send.
        SerialStatePacket.SerialState.byte = SerialStateBitmap.byte;
        //We don't need to write to the other bytes in the SerialStatePacket USB
        //buffer, since they don't change and will always be the same as our
        //initialized value.

        //Send the packet over USB to the host.
        CDCNotificationInHandle = USBTransferOnePacket(CDC_COMM_EP, IN_TO_HOST, (uint8_t*)&SerialStatePacket, sizeof(SERIAL_STATE_NOTIFICATION));

        //Save the old value, so we can detect changes later.
        OldSerialStateBitmap.byte = SerialStateBitmap.byte;
    }
}//void CDCNotificationHandler(void)
#else
    #define CDCNotificationHandler() {}
#endif


/**********************************************************************************
  Function:
    bool USBCDCEventHandler(USB_EVENT event, void *pdata, uint16_t size)

  Summary:
    Handles events from the USB stack, which may have an effect on the CDC
    endpoint(s).

  Description:
    Handles events from the USB stack.  This function should be called when
    there is a USB event that needs to be processed by the CDC driver.

  Conditions:
    Value of input argument 'len' should be smaller than the maximum
    endpoint size responsible for receiving bulk data from USB host for CDC
    class. Input argument 'buffer' should point to a buffer area that is
    bigger or equal to the size specified by 'len'.
  Input:
    event - the type of event that occurred
    pdata - pointer to the data that caused the event
    size - the size of the data that is pointed to by pdata

  **********************************************************************************/
bool USBCDCEventHandler(USB_EVENT event, void *pdata, uint16_t size)
{
    switch( (uint16_t)event )
    {
        case EVENT_TRANSFER_TERMINATED:
            if(pdata == CDCDataOutHandle)
            {
                CDCDataOutHandle = USBRxOnePacket(CDC_DATA_EP,(uint8_t*)&cdc_data_rx,sizeof(cdc_data_rx));
            }
            if(pdata == CDCDataInHandle)
            {
                //flush all of the data in the CDC buffer
                cdc_trf_state = CDC_TX_READY;
                cdc_tx_len = 0;
            }
            break;
        default:
            return false;
    }
    return true;
}

/**********************************************************************************
  Function:
        uint8_t getsUSBUSART(char *buffer, uint8_t len)

  Summary:
    getsUSBUSART copies a string of BYTEs received through USB CDC Bulk OUT
    endpoint to a user's specified location. It is a non-blocking function.
    It does not wait for data if there is no data available. Instead it
    returns '0' to notify the caller that there is no data available.

  Description:
    getsUSBUSART copies a string of BYTEs received through USB CDC Bulk OUT
    endpoint to a user's specified location. It is a non-blocking function.
    It does not wait for data if there is no data available. Instead it
    returns '0' to notify the caller that there is no data available.

    Typical Usage:
    <code>
        uint8_t numBytes;
        uint8_t buffer[64]

        numBytes = getsUSBUSART(buffer,sizeof(buffer)); //until the buffer is free.
        if(numBytes \> 0)
        {
            //we received numBytes bytes of data and they are copied into
            //  the "buffer" variable.  We can do something with the data
            //  here.
        }
    </code>
  Conditions:
    Value of input argument 'len' should be smaller than the maximum
    endpoint size responsible for receiving bulk data from USB host for CDC
    class. Input argument 'buffer' should point to a buffer area that is
    bigger or equal to the size specified by 'len'.
  Input:
    buffer -  Pointer to where received BYTEs are to be stored
    len -     The number of BYTEs expected.

  **********************************************************************************/


uint8_t getsUSBUSART(uint8_t *buffer, uint8_t len)
{
    cdc_rx_len = 0;

    if(!USBHandleBusy(CDCDataOutHandle))
    {
        /*
         * Adjust the expected number of BYTEs to equal
         * the actual number of BYTEs received.
         */
        if(len > USBHandleGetLength(CDCDataOutHandle))
            len = USBHandleGetLength(CDCDataOutHandle);

        /*
         * Copy data from dual-ram buffer to user's buffer
        */
        for(cdc_rx_len = 0; cdc_rx_len < len; cdc_rx_len++)
            buffer[cdc_rx_len] = cdc_data_rx[cdc_rx_len];

        /*
         * Prepare dual-ram buffer for next OUT transaction
         */

        CDCDataOutHandle = USBRxOnePacket(CDC_DATA_EP,(uint8_t*)&cdc_data_rx,sizeof(cdc_data_rx));

    }//end if

    return cdc_rx_len;

}//end getsUSBUSART


/******************************************************************************
■Function:
	void putUSBUSART(char *data, uint8_t length)

■Summary:
putUSBUSART は、データの配列を USB に書き込みます。 このバージョンを使用すると、0x00 を転送できます (これは通常、文字列転送関数の NULL 文字です)。

■Description:

putUSBUSART は、データの配列を USB に書き込みます。 このバージョンを使用すると、0x00 を転送できます (これは通常、文字列転送関数の NULL 文字です)。
 
■Typical Usage:
    <code>
        if(USBUSARTIsTxTrfReady())
        {
            char data[] = {0x00, 0x01, 0x02, 0x03, 0x04};
            putUSBUSART(data,5);
        }
    </code>

デバイスからホストへの転送メカニズム (put) は、ホストからデバイスへの転送メカニズム (get) よりも柔軟です。 
バルク IN エンドポイントの最大サイズを超えるデータの文字列を処理できます。 ステート マシンは、 
\long 複数の USB トランザクションにわたるデータの文字列。 
データのブロックをホストに送信し続けるために、CDCTxService() を定期的に呼び出す必要があります。
 

■Conditions:

USBUSARTIsTxTrfReady() は true を返す必要があります。 これは、最後の転送が完了し、新しいデータ ブロックを受信する準備ができていることを示します。 'data' が指す文字列は、255 バイト以下でなければなりません。
 
■Input:
 char *data - ホストに転送されるデータの RAM 配列へのポインタ uint8_t length - 転送されるバイト数 (255 未満である必要があります)。
 
 *****************************************************************************/

/********
void putUSBUSART(uint8_t *data, uint8_t  length)
{
    /*
     * この関数を呼び出す前に、cdc_trf_state が CDC_TX_READY 状態であることを
     * 確認する必要があります。 安全上の予防措置として、この関数は状態をもう一度
     * チェックして、保留中のトランザクションを上書きしないようにします。
     *
     * 現在、ユーザーにエラーを報告せずにルーチンを終了するだけです。
     *
     * 結論: ユーザーは、この関数を呼び出す前に USBUSARTIsTxTrfReady()==1 
     * であることを確認する必要があります!
     * 
     * Example:
     * if(USBUSARTIsTxTrfReady())
     *     putUSBUSART(pData, Length);
     *
     * IMPORTANT: Never use the following blocking while loop to wait:
     * 重要: 次のブロッキング while ループを使用して待機しないでください。
     * 
     * while(!USBUSARTIsTxTrfReady())
     *     putUSBUSART(pData, Length);
     *
     * ファームウェア フレームワーク全体が協調マルチタスクに基づいて記述されており、
     * ブロッキング コードは受け入れられません。
     * 代わりにステート マシンを使用してください。
     
    
    USBMaskInterrupts();
    if(cdc_trf_state == CDC_TX_READY)
    {
        mUSBUSARTTxRam((uint8_t*)data, length);     // See cdc.h
    }
    USBUnmaskInterrupts();
}//end putUSBUSART

 * *****/


/******************************************************************************
■Function:
	void putsUSBUSART(char *data)

■Summary:
putsUSBUSART は、ヌル文字を含むデータの文字列を USB に書き込みます。 このバージョンの「puts」を使用して、RAM バッファからデータを転送します。

■Description:
putsUSBUSART は、ヌル文字を含むデータの文字列を USB に書き込みます。 このバージョンの「puts」を使用して、RAM バッファからデータを転送します。

■Typical Usage:
    <code>
        if(USBUSARTIsTxTrfReady())
        {
            char data[] = "Hello World";
            putsUSBUSART(data);
        }
    </code>

デバイスからホストへの転送メカニズム (put) は、ホストからデバイスへの転送メカニズム (get) よりも柔軟です。
バルク IN エンドポイントの最大サイズを超えるデータの文字列を処理できます。 
ステート マシンは、複数の USB トランザクションでデータの \long 文字列を転送するために使用されます。
データのブロックをホストに送信し続けるために、CDCTxService() を定期的に呼び出す必要があります。

■Conditions:
USBUSARTIsTxTrfReady() は true を返す必要があります。 これは、最後の転送が完了し、
新しいデータ ブロックを受信する準備ができていることを示します。 'data' が指す文字列は、255 バイト以下でなければなりません。

■Input:
char *data - null\ で終わる定数データの文字列。 ヌル文字が見つからない場合、255 バイトのデータがホストに転送されます。

 *****************************************************************************/

void putsUSBUSART(char *data)
{
    uint8_t len;
    char *pData;

    /*
     * この関数を呼び出す前に、cdc_trf_state が CDC_TX_READY 状態であることを確認する必要があります。
     *  安全上の予防措置として、この関数は状態をもう一度チェックして、保留中のトランザクションを上書きしないようにします。
     * 
     *  現在、ユーザーにエラーを報告せずにルーチンを終了するだけです。
     * 
     * ■Bottom line: 
     * ユーザーは、この関数を呼び出す前に、USBUSARTIsTxTrfReady()==1 であることを確認する必要があります。
     * 例：
     * if(USBUSARTIsTxTrfReady())
     *     putsUSBUSART(pData, 長さ);
     * ■IMPORTANT:
     * 次のブロッキング while ループを使用して待機しないでください。
     * ファームウェア フレームワーク全体が協調マルチタスクに基づいて記述されており、
     * ブロッキング コードは受け入れられません。 代わりにステート マシンを使用してください。
     */
    USBMaskInterrupts();
    if(cdc_trf_state != CDC_TX_READY)
    {
        USBUnmaskInterrupts();
        return;
    }

    /*
     * while ループは、NULL文字を含めて送信する BYTE の数をカウントします。
     */
    len = 0;
    pData = data;
    do
    {
        len++;
        if(len == 255) break;       // Break loop once max len is reached.
    }while(*pData++);

    /*
     * 2 番目の情報 (送信するデータの長さ) の準備が整いました。 
     * mUSBUSARTTxRam を呼び出して、転送をセットアップします。 
     * 実際の転送プロセスは CDCTxService() によって処理され、メイン プログラム ループごとに 1 回呼び出される必要があります。
     */
    mUSBUSARTTxRam((uint8_t*)data, len);     // See cdc.h
    USBUnmaskInterrupts();
}//end putsUSBUSART

/**************************************************************************
  Function:
        void putrsUSBUSART(const const char *data)

  Summary:
    putrsUSBUSART writes a string of data to the USB including the null
    character. Use this version, 'putrs', to transfer data literals and
    data located in program memory.

  Description:
    putrsUSBUSART writes a string of data to the USB including the null
    character. Use this version, 'putrs', to transfer data literals and
    data located in program memory.

    Typical Usage:
    <code>
        if(USBUSARTIsTxTrfReady())
        {
            putrsUSBUSART("Hello World");
        }
    </code>

    The transfer mechanism for device-to-host(put) is more flexible than
    host-to-device(get). It can handle a string of data larger than the
    maximum size of bulk IN endpoint. A state machine is used to transfer a
    \long string of data over multiple USB transactions. CDCTxService()
    must be called periodically to keep sending blocks of data to the host.

  Conditions:
    USBUSARTIsTxTrfReady() must return true. This indicates that the last
    transfer is complete and is ready to receive a new block of data. The
    string of characters pointed to by 'data' must equal to or smaller than
    255 BYTEs.

  Input:
    const const char *data -  null\-terminated string of constant data. If a
                            null character is not found, 255 uint8_ts of data
                            will be transferred to the host.

  **************************************************************************/

/**********************
void putrsUSBUSART(const char *data)
{
    uint8_t len;
    const char *pData;

    /*
     * User should have checked that cdc_trf_state is in CDC_TX_READY state
     * before calling this function.
     * As a safety precaution, this function checks the state one more time
     * to make sure it does not override any pending transactions.
     *
     * Currently it just quits the routine without reporting any errors back
     * to the user.
     *
     * Bottom line: User MUST make sure that USBUSARTIsTxTrfReady()
     *             before calling this function!
     * Example:
     * if(USBUSARTIsTxTrfReady())
     *     putsUSBUSART(pData);
     *
     * IMPORTANT: Never use the following blocking while loop to wait:
     * while(cdc_trf_state != CDC_TX_READY)
     *     putsUSBUSART(pData);
     *
     * The whole firmware framework is written based on cooperative
     * multi-tasking and a blocking code is not acceptable.
     * Use a state machine instead.
     
    USBMaskInterrupts();
    if(cdc_trf_state != CDC_TX_READY)
    {
        USBUnmaskInterrupts();
        return;
    }

    /*
     * While loop counts the number of BYTEs to send including the
     * null character.
     
    len = 0;
    pData = data;
    do
    {
        len++;
        if(len == 255) break;       // Break loop once max len is reached.
    }while(*pData++);

    /*
     * Second piece of information (length of data to send) is ready.
     * Call mUSBUSARTTxRom to setup the transfer.
     * The actual transfer process will be handled by CDCTxService(),
     * which should be called once per Main Program loop.
     

    mUSBUSARTTxRom((const uint8_t*)data,len); // See cdc.h
    USBUnmaskInterrupts();

}//end putrsUSBUSART
*****/


/************************************************************************
■Function:
        void CDCTxService(void)

■Summary:

CDCTxService は、デバイスからホストへのトランザクションを処理します。 こ
の関数は、デバイスが構成された状態に達した後、メイン プログラム ループごとに 1 回呼び出す必要があります。

■Description:

CDCTxService は、デバイスからホストへのトランザクションを処理します。 
この関数は、デバイスが構成された状態に達した後 (CDCIniEP() 関数が既に実行された後)、
メイン プログラム ループごとに 1 回呼び出す必要があります。 この関数は、CDC シリアル データに
関連付けられた複数のトランザクションに相当する IN USB データをホストに送信する内部
ソフトウェア ステート マシンを進めるために必要です。 CDCTxService() を定期的に呼び出さないと、
データが CDC シリアル データ インターフェイス経由で USB ホストに送信されません。

    Typical Usage:
    <code>
    void main(void)
    {
        USBDeviceInit();
        while(1)
        {
            USBDeviceTasks();
            if((USBGetDeviceState() \< CONFIGURED_STATE) ||
               (USBIsDeviceSuspended() == true))
            {
                //Either the device is not configured or we are suspended
                //  so we don't want to do execute any application code
                continue;   //go back to the top of the while loop
            }
            else
            {
                //Keep trying to send data to the PC as required
                CDCTxService();

                //Run application code.
                UserApplication();
            }
        }
    }
    </code>

■Conditions:
    CDCIniEP() 関数は既に実行されている必要があります/デバイスは CONFIGURED_STATE になっている必要があります。
  ************************************************************************/

void CDCTxService(void)
{
    uint8_t byte_to_send;
    uint8_t i;

    USBMaskInterrupts();

    CDCNotificationHandler();

    if(USBHandleBusy(CDCDataInHandle))
    {
        USBUnmaskInterrupts();
        return;
    }

    /*
     * Completing stage is necessary while [ mCDCUSartTxIsBusy()==1 ].
     * By having this stage, user can always check cdc_trf_state,
     * and not having to call mCDCUsartTxIsBusy() directly.
     * [ mCDCUSartTxIsBusy()==1 ] の間にステージを完了する必要があります。 
     * この段階を持つことにより、ユーザーは常に cdc_trf_state をチェックでき、
     * mCDCUsartTxIsBusy() を直接呼び出す必要はありません。
     */
    if(cdc_trf_state == CDC_TX_COMPLETING)
        cdc_trf_state = CDC_TX_READY;

    /*
     * If CDC_TX_READY state, nothing to do, just return.
     * CDC_TX_READY 状態の場合は、何もせずそのままリターンします。
     */
    if(cdc_trf_state == CDC_TX_READY)
    {
        USBUnmaskInterrupts();
        return;
    }

    /*
     * If CDC_TX_BUSY_ZLP state, send zero length packet
     * CDC_TX_BUSY_ZLP 状態の場合、長さゼロのパケットを送信
     */
    if(cdc_trf_state == CDC_TX_BUSY_ZLP)
    {
        CDCDataInHandle = USBTxOnePacket(CDC_DATA_EP,NULL,0);
        //CDC_DATA_BD_IN.CNT = 0;
        cdc_trf_state = CDC_TX_COMPLETING;
    }
    else if(cdc_trf_state == CDC_TX_BUSY)
    {
        /*
         * First, have to figure out how many byte of data to send.
         * まず、送信するデータのバイト数を把握する必要があります。
         */
    	if(cdc_tx_len > sizeof(cdc_data_tx))
    	    byte_to_send = sizeof(cdc_data_tx);
    	else
    	    byte_to_send = cdc_tx_len;

        /*
         * Subtract the number of bytes just about to be sent from the total.
         * 送信しようとしているバイト数を合計から引きます。
         */
    	cdc_tx_len = cdc_tx_len - byte_to_send;

        pCDCDst.bRam = (uint8_t*)&cdc_data_tx; // Set destination pointer 宛先ポインタを設定

        i = byte_to_send;
        if(cdc_mem_type == USB_EP0_ROM)  // Determine type of memory source メモリ ソースの種類を決定する
        {
            while(i)
            {
                *pCDCDst.bRam = *pCDCSrc.bRom;
                pCDCDst.bRam++;
                pCDCSrc.bRom++;
                i--;
            }//end while(byte_to_send)
        }
        else
        {
            while(i)
            {
                *pCDCDst.bRam = *pCDCSrc.bRam;
                pCDCDst.bRam++;
                pCDCSrc.bRam++;
                i--;
            }
        }

        /*
         * Lastly, determine if a zero length packet state is necessary.
         * See explanation in USB Specification 2.0: Section 5.8.3
         * 最後に、長さゼロのパケット状態が必要かどうかを判断します。 
         * USB 仕様 2.0 の説明を参照してください: セクション 5.8.3
         */
        if(cdc_tx_len == 0)
        {
            if(byte_to_send == CDC_DATA_IN_EP_SIZE)
                cdc_trf_state = CDC_TX_BUSY_ZLP;
            else
                cdc_trf_state = CDC_TX_COMPLETING;
        }//end if(cdc_tx_len...)
        CDCDataInHandle = USBTxOnePacket(CDC_DATA_EP,(uint8_t*)&cdc_data_tx,byte_to_send);

    }//end if(cdc_tx_sate == CDC_TX_BUSY)

    USBUnmaskInterrupts();
}//end CDCTxService

#endif //USB_USE_CDC

/** EOF cdc.c ****************************************************************/
