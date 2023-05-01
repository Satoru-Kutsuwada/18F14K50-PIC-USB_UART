/*******************************************************************************

 代表的な送信シーケンスは、以下のように進行します。
1. ユーザがSSPCON2 レジスタのSEN ビットをセットしてスタート条件を生成する。
2. SSPIF がセットされる。MSSP モジュールは、必要なスタート時間が経過するまで待機してから、他の動作を実行する。
3. ユーザが、送信するスレーブアドレスをSSPBUF に読み込む。
4. アドレスの8ビットが全て送信されるまでSDAピンからシフト出力する。
5. MSSP モジュールがスレーブデバイスからのACK ビットをシフト入力し、SSPCON2 レジスタのACKSTAT ビットにその値を書き込む。
6. MSSP モジュールが、9 番目のクロックサイクルの終端でSSPIFビットをセットして割り込みを生成する。
7. ユーザがSSPBUF に8 ビットのデータを読み込む。
8. データの8ビットが全て送信されるまでSDAピンからシフト出力する。
9. MSSP モジュールがスレーブデバイスからのACK ビットをシフト入力し、SSPCON2 レジスタのACKSTAT ビットにその値を書き込む。
10. MSSP モジュールが、9 番目のクロックサイクルの終端でSSPIFビットをセットして割り込みを生成する。
11. ユーザがSSPCON2 レジスタのPEN ビットをセットしてストップ条件を生成する。
12. ストップ条件が完了すると割り込みが生成される。
 
 *******************************************************************************/

#include    <xc.h>
#include    <stdio.h>
#include    "i2c_main.h"


//-------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------

uint8_t last_status; // status of last I2C transmission
uint16_t timeout_start_ms = 0;
uint16_t io_timeout = 0;


uint8_t address;
bool    did_timeout;
uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t measurement_timing_budget_us;

bool io_2v8 = TRUE;

uint8_t vl53l0x_address;

//==============================================================================
//  関数
//==============================================================================
void I2C_Master_Wait(void);
void I2C_Master_Start(void);
void I2C_Master_Stop(void);
void I2C_Master_Write(uint8_t d);
void I2C_Master_RepeatedStart(void);

uint8_t I2C_Master_Read(uint8_t acknNak);

// uint8_t I2C_M_MasterSendStart(uint8_t slaveAddress, uint8_t R_nW);
// uint8_t I2C_M_MasterWriteByte(uint8_t theByte) ;
// uint8_t I2C_M_MasterSendStop(void);
// uint8_t I2C_M_MasterReadByte(uint8_t acknNak) ;
uint16_t vl53l0x_readRangeSingleMillimeters(void);
extern void PutString01(char *string,uint8_t flg);
extern void PutString03(char *string,uint16_t data, uint8_t flg, uint8_t keta);
uint8_t vl53l0x_readReg(uint8_t reg);
void vl53l0x_writeReg16Bit(uint8_t reg, uint16_t value);
void vl53l0x_writeReg(uint8_t reg, uint8_t value);
void Wait(uint16_t num);
uint16_t vl53l0x_readRangeContinuousMillimeters(void);
void vl53l0x_writeReg(uint8_t reg, uint8_t value);
uint16_t vl53l0x_readReg16Bit(uint8_t reg);
uint8_t vl53l0x_timeoutOccurred(void);
uint16_t vl53l0x_encodeTimeout(uint16_t timeout_mclks);
uint32_t vl53l0x_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint16_t vcsel_period_pclks);
uint8_t vl53l0x_init(uint8_t io_2v8);
uint8_t vl53l0x_getVcselPulsePeriod(vcselPeriodType type);
uint32_t vl53l0x_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint16_t vcsel_period_pclks);
uint8_t vl53l0x_getSpadInfo(uint8_t * count, uint8_t * type_is_aperture);
uint16_t vl53l0x_decodeTimeout(uint16_t reg_val);
void vl53l0x_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
uint8_t vl53l0x_performSingleRefCalibration(uint8_t vhv_init_byte);

// define I2C_M_WRITE_XFER_MODE    (0x00u) /* Write */
// #define I2C_M_READ_XFER_MODE     (0x01u) /* Read */
#define I2C_M_ACK_DATA           (0x01u) /* Send ACK */
#define I2C_M_NAK_DATA           (0x00u) /* Send NAK */

//==============================================================================
//
//==============================================================================
uint8_t I2C_init(void)
{
    uint8_t rtn;
    
    PutString01("I2C Init",1);
  
    rtn = vl53l0x_init(io_2v8);
    
    return rtn;
    
}
//==============================================================================
//
//==============================================================================
void I2C_main(void)
{
    uint16_t dt;
    
    dt = vl53l0x_readRangeSingleMillimeters();
    PutString03("Sens = ",dt , 0, 10);
    PutString01(" mm", 1);

}

//==============================================================================
// 単発距離測定を実行し、vl53l0x_PerformSingleRangingMeasurement() に基づいて読み取り値をミリメートル単位で返します。
//==============================================================================

uint16_t vl53l0x_readRangeSingleMillimeters(void)
{
    uint16_t dt;
    
    vl53l0x_writeReg(0x80, 0x01);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x00, 0x00);
    vl53l0x_writeReg(0x91, stop_variable);
    vl53l0x_writeReg(0x00, 0x01);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x80, 0x00);
    vl53l0x_writeReg(SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"

    // #define startTimeout() (timeout_start_ms = millis())
    // startTimeout();     
    //  timeout_start_ms = millis();

    dt = 100;
    while (vl53l0x_readReg(SYSRANGE_START) & 0x01)
    {
        // #define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)
        
        if(dt == 0){
            did_timeout = TRUE;
            return 65535;
        }
        else{
            dt --;
            Wait(5);
        }
        
    }

    return vl53l0x_readRangeContinuousMillimeters();
}
//==============================================================================
// 連続モードがアクティブな場合、距離の読み取り値をミリメートル単位で返します 
// (readRangeSingleMillimeters() は、単発距離測定の開始後にもこの関数を呼び出します)。
//==============================================================================

uint16_t vl53l0x_readRangeContinuousMillimeters(void)
{

    uint16_t range ;
    uint16_t dt;

    // #define startTimeout() (timeout_start_ms = millis())
    // startTimeout();     
    // timeout_start_ms = millis();

    dt = 100;
    while ((vl53l0x_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        // #define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)
        
        if(dt == 0){
            did_timeout = TRUE;
            return 65535;
        }
        else{
            dt --;
            Wait(5);
        }
    }

    //-------------------------------------------------------------------------------------
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    //-------------------------------------------------------------------------------------

    range = vl53l0x_readReg16Bit(RESULT_RANGE_STATUS + 10);
    vl53l0x_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (vl53l0x_timeoutOccurred()) { 
        range = 0x1FFE;
    }

    return range;

}
//-------------------------------------------------------------------------------------
// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
//
// timeoutOccurred() への最後の呼び出し以降、読み取り関数の 1 つでタイムアウトが発生しましたか?
//-------------------------------------------------------------------------------------
uint8_t vl53l0x_timeoutOccurred(void)
{

    uint8_t tmp = did_timeout;

    did_timeout = FALSE;
    return tmp;

}

//-------------------------------------------------------------------------------------
// Read a 16-bit register
//-------------------------------------------------------------------------------------

uint16_t vl53l0x_readReg16Bit(uint8_t reg)

{

    uint16_t value = 0;
    uint8_t status;

    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    I2C_Master_Write(reg);
    I2C_Master_Stop();
    
    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    value = I2C_Master_Read(I2C_M_ACK_DATA);
    value = value << 8;
    value |= I2C_Master_Read(I2C_M_ACK_DATA) & 0xFF;
    I2C_Master_Stop();

    /**************************************************************************
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_WRITE_XFER_MODE);
    Wait(1);   
    status = I2C_M_MasterWriteByte(reg);
    Wait(1);
    status = I2C_M_MasterSendStop();
    Wait(1);   

     * 
     * 
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_READ_XFER_MODE);
    Wait(1);   

    value = I2C_M_MasterReadByte(I2C_M_ACK_DATA);
    Wait(1);

    value = value << 8;

    value |= I2C_M_MasterReadByte(I2C_M_NAK_DATA) & 0xFF;
    Wait(1);

    status = I2C_M_MasterSendStop();

    ***************************************************************************/


    return value;

}
//==============================================================================
// 
//==============================================================================
void I2C_Master_Wait(void)
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)); //Transmit is in progress
}
//==============================================================================
//  関数
//==============================================================================
void I2C_Master_Start(void)
{
    I2C_Master_Wait();    
    SSPCON2bits.SEN = 1;             //Initiate start condition
}

//==============================================================================
//  関数
//==============================================================================
void I2C_Master_Stop(void)
{
    I2C_Master_Wait();
    SSPCON2bits.PEN = 1;           //Initiate stop condition
}

//==============================================================================
//  関数
//==============================================================================
void I2C_Master_Write(uint8_t d)
{
    I2C_Master_Wait();
    SSPBUF = d;         //Write data to SSPBUF
}

//==============================================================================
//  関数
//==============================================================================
uint8_t I2C_Master_Read(uint8_t acknNak)
{
    uint8_t rtn;

    I2C_Master_RepeatedStart();
    
    I2C_Master_Wait();
    rtn = SSPBUF;                 //Read data from SSP1BUF
    
    I2C_Master_Wait();
    SSPCON2bits.ACKDT = acknNak;         //Acknowledge bit    1:NACK  0:ACK
    SSPCON2bits.ACKEN = 1;         //Acknowledge sequence
    
    return rtn;
}


//==============================================================================
//  関数
//==============================================================================
void I2C_Master_RepeatedStart(void)
{
  I2C_Master_Wait();
  SSPCON2bits.RSEN = 1;          //Initiate repeated start condition
}



//==============================================================================
// Read an 8-bit register
//==============================================================================

uint8_t vl53l0x_readReg(uint8_t reg)

{
    
    uint8_t value = 0;
    uint8_t status;

    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    I2C_Master_Write(reg);
    I2C_Master_Stop();
    
    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    value = I2C_Master_Read(I2C_M_ACK_DATA);
    I2C_Master_Stop();

    /**************************************************************************
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_WRITE_XFER_MODE);
    Wait(1);   
    
    status = I2C_M_MasterWriteByte(reg);
    Wait(1);   
    
    status = I2C_M_MasterSendStop();
    Wait(1);   
    
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_READ_XFER_MODE);
    Wait(1);   
    
    value = I2C_M_MasterReadByte(I2C_M_ACK_DATA);
    Wait(1);   
    
    status = I2C_M_MasterSendStop();
    **************************************************************************/
    
    
    return value;

}
//-------------------------------------------------------------------------------------
// Write an 8-bit register
//-------------------------------------------------------------------------------------

void vl53l0x_writeReg(uint8_t reg, uint8_t value)
{

    uint8_t status;

    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    I2C_Master_Write(reg);
    I2C_Master_Write(value);
    I2C_Master_Stop();
    

    /**************************************************************************
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_WRITE_XFER_MODE);
    Wait(1);

    status = I2C_M_MasterWriteByte(reg);
    Wait(1);

    status = I2C_M_MasterWriteByte(value);
    Wait(1);

    status = I2C_M_MasterSendStop();
    **************************************************************************/
}    
//-------------------------------------------------------------------------------------
// Write a 16-bit register
//-------------------------------------------------------------------------------------

void vl53l0x_writeReg16Bit(uint8_t reg, uint16_t value)

{

    uint8_t status;

    
    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    I2C_Master_Write(reg);
    I2C_Master_Write((value>>8) & 0xff);
    I2C_Master_Write(value & 0xff);
    I2C_Master_Stop();
    

    /**************************************************************************
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_WRITE_XFER_MODE);
    Wait(1);

    status = I2C_M_MasterWriteByte(reg);
    Wait(1);

    status = I2C_M_MasterWriteByte((value >> 8) & 0xFF);
    Wait(1);

    status = I2C_M_MasterWriteByte( value       & 0xFF);
    Wait(1);

    status = I2C_M_MasterSendStop();
    **************************************************************************/
}
//-------------------------------------------------------------------------------------
//返信信号レート制限チェック値を MCPS (メガカウント/秒) 単位で設定します。 
//「これは、ターゲットから反射され、デバイスによって検出された信号の振幅を表します」;
//この制限を設定すると、センサーが有効な読み取り値を報告するために必要な最小測定値が決まると考えられます。
//下限を設定すると、センサーの潜在的な範囲が広がりますが、意図したターゲット以外の
//オブジェクトからの不要な反射のために、不正確な読み取り値が得られる可能性も高くなるようです.
//ST API およびこのライブラリによって初期化されるように、デフォルトは 0.25 MCPS です。
//-------------------------------------------------------------------------------------
uint8_t vl53l0x_setSignalRateLimit(float limit_Mcps)

{

    if (limit_Mcps < 0.0 || limit_Mcps > 511.99) 
    { 
        return FALSE; 
    }

    //-------------------------------------------------------------------------------------
    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    //-------------------------------------------------------------------------------------
    vl53l0x_writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,(uint16_t)( limit_Mcps * (1 << 7)));

    return TRUE;

}
//-------------------------------------------------------------------------------------
// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
//-------------------------------------------------------------------------------------

void vl53l0x_readMulti(uint8_t reg, uint8_t *dst, uint8_t count)

{
    uint8_t i;
    uint8_t status;

    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    I2C_Master_Write(reg);
    I2C_Master_Stop();
    
    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    
    for (i = 0; i < count; i++ )
    {
        if ( i < (count-1) )
        {   
            dst[i] = I2C_Master_Read(I2C_M_ACK_DATA);
        } 
        else 
        {
            dst[i] = I2C_Master_Read(I2C_M_NAK_DATA);
        }
    }
    
    I2C_Master_Stop();

    /**************************************************************************
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_WRITE_XFER_MODE);
    Wait(1);

    status = I2C_M_MasterWriteByte(reg);
    Wait(1);

    status = I2C_M_MasterSendStop();
    Wait(1);

    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_READ_XFER_MODE);
    Wait(1);

    for (i = 0; i < count; i++ )
    {
        if ( i < (count-1) )
        {   
            dst[i] = I2C_M_MasterReadByte(I2C_M_ACK_DATA);
        } 
        else 
        {
            dst[i] = I2C_M_MasterReadByte(I2C_M_NAK_DATA);
        }
        Wait(1);
    }

    status = I2C_M_MasterSendStop();  
     *************************************************************************/
}
//-------------------------------------------------------------------------------------
// Get the VCSEL pulse period in PCLKs for the given period type.
// based on vl53l0x_get_vcsel_pulse_period()
//-------------------------------------------------------------------------------------

uint8_t vl53l0x_getVcselPulsePeriod(vcselPeriodType type)
{
    
    uint8_t temp;

    if (type == VcselPeriodPreRange)
    {
        temp = vl53l0x_readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD);
        // #define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
        temp = ( uint8_t)(((temp) + 1) << 1);
        
        //return decodeVcselPeriod(temp);
        return temp;
    }
    else if (type == VcselPeriodFinalRange)
    {
        temp = vl53l0x_readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD);
        // #define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
        
        temp =  ( uint8_t)(((temp) + 1) << 1);
        
        //return decodeVcselPeriod(temp);
        return temp;
    }
    else { 
        return 255; 
    }

}

//-------------------------------------------------------------------------------------
// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on vl53l0x_calc_timeout_us()
//-------------------------------------------------------------------------------------

uint32_t vl53l0x_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint16_t vcsel_period_pclks)
{

  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks); // #define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}
//-------------------------------------------------------------------------------------
// Decode sequence step timeout in MCLKs from register value
// based on vl53l0x_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
//-------------------------------------------------------------------------------------
uint16_t vl53l0x_decodeTimeout(uint16_t reg_val)
{

  // format: "(LSByte * 2^MSByte) + 1"

  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;

}
//-------------------------------------------------------------------------------------
// get_sequence_step_timeout() に基づいてシーケンスステップのタイムアウトを取得しますが、
// 要求されたタイムアウトだけでなくすべてのタイムアウトを取得し、中間値も保存します
//-------------------------------------------------------------------------------------

void vl53l0x_getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{

    timeouts->pre_range_vcsel_period_pclks = vl53l0x_getVcselPulsePeriod(VcselPeriodPreRange);
    timeouts->msrc_dss_tcc_mclks = vl53l0x_readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = vl53l0x_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
    timeouts->pre_range_mclks = vl53l0x_decodeTimeout(vl53l0x_readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    

    
    timeouts->pre_range_us =vl53l0x_timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    
    
    timeouts->final_range_vcsel_period_pclks = vl53l0x_getVcselPulsePeriod(VcselPeriodFinalRange);
    timeouts->final_range_mclks = vl53l0x_decodeTimeout(vl53l0x_readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =vl53l0x_timeoutMclksToMicroseconds(timeouts->final_range_mclks,timeouts->final_range_vcsel_period_pclks);

}

//-------------------------------------------------------------------------------------
//vl53l0x_GetSequenceStepEnables() に基づいてシーケンス ステップの有効化を取得します。
//-------------------------------------------------------------------------------------

void vl53l0x_getSequenceStepEnables(SequenceStepEnables * enables)
{

  uint8_t sequence_config;
  
  sequence_config = vl53l0x_readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;

}
//-------------------------------------------------------------------------------------
// vl53l0x_get_measurement_timing_budget_micro_seconds() に基づいてマイクロ秒単位で測定タイミング バジェットを取得します。
//-------------------------------------------------------------------------------------

uint32_t vl53l0x_getMeasurementTimingBudget(void)
{

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    uint16_t const StartOverhead     = 1910; // これは set_ の値とは異なることに注意してください
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;
    
    //-------------------------------------------------------------------------------------
    // 開始および終了のオーバーヘッド時間は常に存在します
    //-------------------------------------------------------------------------------------

    uint32_t budget_us = StartOverhead + EndOverhead;
    vl53l0x_getSequenceStepEnables(&enables);
    vl53l0x_getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }

    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse

    return budget_us;

}

//-------------------------------------------------------------------------------------
// 測定タイミング バジェットをマイクロ秒単位で設定します。これは、1 回の測定に許容される時間です。
// ST API とこのライブラリは、測距シーケンスのサブステップ間でタイミング バジェットを分割します。
// タイミング バジェットを長くすると、より正確な測定が可能になります。 バジェットを N 倍に増やすと、
// 範囲測定の標準偏差が sqrt(N) 分の 1 に減少します。 デフォルトは約 33 ミリ秒です。 最小値は 20 ミリ秒です。 
// vl53l0x_set_measurement_timing_budget_micro_seconds()に基づく
//-------------------------------------------------------------------------------------

uint8_t vl53l0x_setMeasurementTimingBudget(uint32_t budget_us)

{
    //struct SequenceStepEnables enables;
    //struct SequenceStepTimeouts timeouts;
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;
    uint32_t const MinTimingBudget = 20000;
    uint32_t final_range_timeout_us ;
    uint32_t used_budget_us ;
    uint16_t final_range_timeout_mclks ;


    if (budget_us < MinTimingBudget) 
    { 
        return FALSE; 
    }

    used_budget_us = StartOverhead + EndOverhead;
    vl53l0x_getSequenceStepEnables(&enables);
    vl53l0x_getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {

        used_budget_us += FinalRangeOverhead;

        //-------------------------------------------------------------------------------------
        // 最終的な範囲のタイムアウトは、タイミング バジェットと、シーケンス内の他のすべての
        // タイムアウトの合計によって決定されることに注意してください。 最終範囲タイムアウトの
        // 余地がない場合は、エラーが設定されます。 そうしないと、残り時間が最終範囲に適用されます。
        //-------------------------------------------------------------------------------------

        if (used_budget_us > budget_us)
        {

            //-------------------------------------------------------------------------------------
            // 要求されたタイムアウトが大きすぎます
            //-------------------------------------------------------------------------------------

            return FALSE;

        }

        final_range_timeout_us = budget_us - used_budget_us;

        //-------------------------------------------------------------------------------------
        // set_sequence_step_timeout() begin (SequenceStepId == vl53l0x_SEQUENCESTEP_FINAL_RANGE) 
        // 最終レンジ タイムアウトの場合、プレレンジ タイムアウトを追加する必要があります。
        // これを行うには、最終レンジ タイムアウトとプレレンジ タイムアウトの両方をマクロ周期 
        // MClks で表す必要があります。これは、vcsel 周期が異なるためです。
        //-------------------------------------------------------------------------------------

        final_range_timeout_mclks = (uint16_t)vl53l0x_timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
        


                

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        vl53l0x_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        vl53l0x_encodeTimeout(final_range_timeout_mclks));

        //-------------------------------------------------------------------------------------
        // set_sequence_step_timeout() end
        //-------------------------------------------------------------------------------------

        measurement_timing_budget_us = budget_us; // store for internal reuse

    }

    return TRUE;

}
//-------------------------------------------------------------------------------------
// vl53l0x_encode_timeout() に基づいて、タイムアウトからのシーケンス ステップ タイムアウト レジスタ値を MCLK でエンコードします。
//-------------------------------------------------------------------------------------

uint16_t vl53l0x_encodeTimeout(uint16_t timeout_mclks)
{

    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {

        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else 
    { 
        return 0; 
    }

}
//-------------------------------------------------------------------------------------
// vl53l0x_calc_timeout_mclks() に基づいて、シーケンス ステップのタイムアウトをマイクロ秒から MCLK に変換します。
//-------------------------------------------------------------------------------------

uint32_t vl53l0x_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint16_t vcsel_period_pclks)
{

  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);  // #define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);

}

//==============================================================================
// ■Function 
//      uint8_t vl53l0x_init(uint8_t io_2v8)
//
// vl53l0x_DataInit()、vl53l0x_StaticInit()、および vl53l0x_PerformRefCalibration() 
// に基づくシーケンスを使用してセンサーを初期化します。
// この関数は、リファレンス SPAD キャリブレーション (vl53l0x_PerformRefSpadManagement()) を実行しません。
// これは、API ユーザー マニュアルに、ベア モジュール上で ST によって実行されると記載されているためです。 
// カバーガラスを追加しない限り、それで十分に機能するようです。
// io_2v8 (オプション) が TRUE の場合、または指定されていない場合、センサーは 2V8 モード用に構成されています。
//-------------------------------------------------------------------------------------

uint8_t vl53l0x_init(uint8_t io_2v8)
{

    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    uint8_t ref_spad_map[6];
    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;
    uint8_t i;
    uint8_t dt;

    
    // #define ADDRESS_DEFAULT 0x29
    vl53l0x_address = ADDRESS_DEFAULT;  

    io_timeout = 0; // no timeout

    did_timeout = 0;

    //-------------------------------------------------------------------------------------
    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    //-------------------------------------------------------------------------------------
    PutString03("I2c init01:io_2v8= ",io_2v8,1,16);
    if (io_2v8 )
    {
        PutString01("I2C init01-1",1);
        dt = vl53l0x_readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
        PutString01("I2C init01-2",1);
        vl53l0x_writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, (dt | 0x01));
        //vl53l0x_readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
        
    }

    //-------------------------------------------------------------------------------------
    // "Set I2C standard mode"
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(0x88, 0x00);
    vl53l0x_writeReg(0x80, 0x01);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x00, 0x00);

    PutString01("I2C init01-3",1);
    stop_variable = vl53l0x_readReg(0x91);
    PutString01("I2C init01-4",1);
    
    vl53l0x_writeReg(0x00, 0x01);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x80, 0x00);

    PutString01("I2C init02",1);
    //-------------------------------------------------------------------------------------
    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(MSRC_CONFIG_CONTROL, vl53l0x_readReg(MSRC_CONFIG_CONTROL) | 0x12);

    //-------------------------------------------------------------------------------------
    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    //-------------------------------------------------------------------------------------

    vl53l0x_setSignalRateLimit(0.25);

    vl53l0x_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    if (!vl53l0x_getSpadInfo(&spad_count, &spad_type_is_aperture)) 
    { 
        PutString01("I2C FALSE00",1);
        return FALSE; 
    }
    PutString01("I2C init03",1);

    //-------------------------------------------------------------------------------------
    // SPADマップ(RefGoodSpadMap)はAPIでvl53l0x_get_info_from_device()で読み込んでいますが、
    // 同じデータはGLOBAL_CONFIG_SPAD_ENABLES_REF_0～_6の方が読みやすいようなのでそちらから読み込んでください
    //-------------------------------------------------------------------------------------

    vl53l0x_readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    //-------------------------------------------------------------------------------------
    // -- vl53l0x_set_reference_spads() begin (assume NVM values are valid)
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    vl53l0x_writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    for (i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            //-------------------------------------------------------------------------------------
            // このビットは、有効にする必要がある最初のビットよりも低いか、または
            // (reference_spad_count) ビットが既に有効になっているため、このビットをゼロにします
            //-------------------------------------------------------------------------------------
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    vl53l0x_writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    PutString01("I2C init04",1);

    //-------------------------------------------------------------------------------------
    // -- vl53l0x_set_reference_spads() end
    // -- vl53l0x_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x00, 0x00);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x09, 0x00);
    vl53l0x_writeReg(0x10, 0x00);
    vl53l0x_writeReg(0x11, 0x00);
    vl53l0x_writeReg(0x24, 0x01);
    vl53l0x_writeReg(0x25, 0xFF);
    vl53l0x_writeReg(0x75, 0x00);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x4E, 0x2C);
    vl53l0x_writeReg(0x48, 0x00);
    vl53l0x_writeReg(0x30, 0x20);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x30, 0x09);
    vl53l0x_writeReg(0x54, 0x00);
    vl53l0x_writeReg(0x31, 0x04);
    vl53l0x_writeReg(0x32, 0x03);
    vl53l0x_writeReg(0x40, 0x83);
    vl53l0x_writeReg(0x46, 0x25);
    vl53l0x_writeReg(0x60, 0x00);
    vl53l0x_writeReg(0x27, 0x00);
    vl53l0x_writeReg(0x50, 0x06);
    vl53l0x_writeReg(0x51, 0x00);
    vl53l0x_writeReg(0x52, 0x96);
    vl53l0x_writeReg(0x56, 0x08);
    vl53l0x_writeReg(0x57, 0x30);
    vl53l0x_writeReg(0x61, 0x00);
    vl53l0x_writeReg(0x62, 0x00);
    vl53l0x_writeReg(0x64, 0x00);
    vl53l0x_writeReg(0x65, 0x00);
    vl53l0x_writeReg(0x66, 0xA0);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x22, 0x32);
    vl53l0x_writeReg(0x47, 0x14);
    vl53l0x_writeReg(0x49, 0xFF);
    vl53l0x_writeReg(0x4A, 0x00);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x7A, 0x0A);
    vl53l0x_writeReg(0x7B, 0x00);
    vl53l0x_writeReg(0x78, 0x21);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x23, 0x34);
    vl53l0x_writeReg(0x42, 0x00);
    vl53l0x_writeReg(0x44, 0xFF);
    vl53l0x_writeReg(0x45, 0x26);
    vl53l0x_writeReg(0x46, 0x05);
    vl53l0x_writeReg(0x40, 0x40);
    vl53l0x_writeReg(0x0E, 0x06);
    vl53l0x_writeReg(0x20, 0x1A);
    vl53l0x_writeReg(0x43, 0x40);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x34, 0x03);
    vl53l0x_writeReg(0x35, 0x44);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x31, 0x04);
    vl53l0x_writeReg(0x4B, 0x09);
    vl53l0x_writeReg(0x4C, 0x05);
    vl53l0x_writeReg(0x4D, 0x04);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x44, 0x00);
    vl53l0x_writeReg(0x45, 0x20);
    vl53l0x_writeReg(0x47, 0x08);
    vl53l0x_writeReg(0x48, 0x28);
    vl53l0x_writeReg(0x67, 0x00);
    vl53l0x_writeReg(0x70, 0x04);
    vl53l0x_writeReg(0x71, 0x01);
    vl53l0x_writeReg(0x72, 0xFE);
    vl53l0x_writeReg(0x76, 0x00);
    vl53l0x_writeReg(0x77, 0x00);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x0D, 0x01);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x80, 0x01);
    vl53l0x_writeReg(0x01, 0xF8);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x8E, 0x01);
    vl53l0x_writeReg(0x00, 0x01);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x80, 0x00);
    
    //-------------------------------------------------------------------------------------
    // -- vl53l0x_load_tuning_settings() end
    // "Set interrupt config to new sample ready"
    // -- vl53l0x_SetGpioConfig() begin
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    vl53l0x_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, vl53l0x_readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    vl53l0x_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    //-------------------------------------------------------------------------------------
    // -- vl53l0x_SetGpioConfig() end
    //-------------------------------------------------------------------------------------

    measurement_timing_budget_us = vl53l0x_getMeasurementTimingBudget();

    //-------------------------------------------------------------------------------------
    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- vl53l0x_SetSequenceStepEnable() begin
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    //-------------------------------------------------------------------------------------
    // -- vl53l0x_SetSequenceStepEnable() end
    // "Recalculate timing budget"
    //-------------------------------------------------------------------------------------

    vl53l0x_setMeasurementTimingBudget(measurement_timing_budget_us);

    //-------------------------------------------------------------------------------------
    // vl53l0x_StaticInit() end
    // vl53l0x_PerformRefCalibration() begin (vl53l0x_perform_ref_calibration())
    // -- vl53l0x_perform_vhv_calibration() begin
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);

    if (!vl53l0x_performSingleRefCalibration(0x40)) 
    { 
        PutString01("I2C FALSE01",1);
        return FALSE; 
    }
    PutString01("I2C init05",1);

    //-------------------------------------------------------------------------------------
    // -- vl53l0x_perform_vhv_calibration() end
    // -- vl53l0x_perform_phase_calibration() begin
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);

    if (!vl53l0x_performSingleRefCalibration(0x00)) 
    { 
        PutString01("I2C FALSE02",1);
        return FALSE; 
    }
    PutString01("I2C init06",1);

    //-------------------------------------------------------------------------------------
    // -- vl53l0x_perform_phase_calibration() end
    // "restore the previous Sequence Config"
    //-------------------------------------------------------------------------------------

    vl53l0x_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    //-------------------------------------------------------------------------------------
    // vl53l0x_PerformRefCalibration() end
    //-------------------------------------------------------------------------------------

    // setTimeout(500);
    PutString01("I2C init07",1);

    return TRUE;

}
//-------------------------------------------------------------------------------------
// Private Methods /////////////////////////////////////////////////////////////
// Get reference SPAD (single photon avalanche diode) count and type
// based on vl53l0x_get_info_from_device(),
// but only gets reference SPAD count and type
//
// vl53l0x_get_info_from_device() に基づいて参照 SPAD (単一光子アバランシェ ダイオード) のカウントとタイプを取得しますが、参照 SPAD のカウントとタイプのみを取得します
//-------------------------------------------------------------------------------------

uint8_t vl53l0x_getSpadInfo(uint8_t * count, uint8_t * type_is_aperture)
{

    uint8_t tmp;
    uint16_t dt;

    vl53l0x_writeReg(0x80, 0x01);
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x00, 0x00);
    vl53l0x_writeReg(0xFF, 0x06);
    vl53l0x_writeReg(0x83, vl53l0x_readReg(0x83) | 0x04);
    vl53l0x_writeReg(0xFF, 0x07);
    vl53l0x_writeReg(0x81, 0x01);
    vl53l0x_writeReg(0x80, 0x01);
    vl53l0x_writeReg(0x94, 0x6b);
    vl53l0x_writeReg(0x83, 0x00);

    // #define startTimeout() (timeout_start_ms = millis())
    // startTimeout();     
    // timeout_start_ms = millis();

    dt = 100;
    while (vl53l0x_readReg(0x83) == 0x00)
    {
        // #define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)
        if(dt == 0 ){
            return FALSE; 
        }
        else{
            dt --;
            Wait(5);
        }
    }

    vl53l0x_writeReg(0x83, 0x01);
    tmp = vl53l0x_readReg(0x92);
    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    vl53l0x_writeReg(0x81, 0x00);
    vl53l0x_writeReg(0xFF, 0x06);
    vl53l0x_writeReg(0x83, vl53l0x_readReg( 0x83  & ~0x04));
    vl53l0x_writeReg(0xFF, 0x01);
    vl53l0x_writeReg(0x00, 0x01);
    vl53l0x_writeReg(0xFF, 0x00);
    vl53l0x_writeReg(0x80, 0x00);

    return TRUE;

}
//-------------------------------------------------------------------------------------
// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
//-------------------------------------------------------------------------------------

void vl53l0x_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)

{

    uint8_t status;
    uint8_t i;

    
    I2C_Master_Start();
    I2C_Master_Write(vl53l0x_address);
    I2C_Master_Write(reg);
    for (i = 0; i < count; i++) 
    {
          I2C_Master_Write(*(src+i));
    }
    I2C_Master_Stop();
    

    /***************************************************************************
    status = I2C_M_MasterSendStart(vl53l0x_address, I2C_M_WRITE_XFER_MODE);
    Wait(1);

    status = I2C_M_MasterWriteByte(reg);
    Wait(1);

    for (i = 0; i < count; i++) 
    {

        status = I2C_M_MasterWriteByte(*(src+i));
        Wait(1);

    }

    status = I2C_M_MasterSendStop();
    ***************************************************************************/
}
//-------------------------------------------------------------------------------------
// based on vl53l0x_perform_single_ref_calibration()
//-------------------------------------------------------------------------------------

uint8_t vl53l0x_performSingleRefCalibration(uint8_t vhv_init_byte)
{
    uint16_t dt;
    
    vl53l0x_writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // vl53l0x_REG_SYSRANGE_MODE_START_STOP

    // #define startTimeout() (timeout_start_ms = millis())
    // startTimeout();     
    // timeout_start_ms = millis();

    dt = 100;
    while ((vl53l0x_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        // #define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)
        if(dt == 0 ){
            return FALSE; 
        }
        else{
            dt --;
            Wait(5);
        }
    }

    vl53l0x_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    vl53l0x_writeReg(SYSRANGE_START, 0x00);

    return TRUE;

}