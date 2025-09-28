// Updated to use Raspberry Pi Pico SDK instead of Arduino
#include "MC3479.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#define MC34X9_CFG_MODE_DEFAULT                 MC34X9_MODE_STANDBY
#define MC34X9_CFG_SAMPLE_RATE_DEFAULT    MC34X9_SR_DEFAULT_1000Hz
#define MC34X9_CFG_RANGE_DEFAULT                MC34X9_RANGE_8G

uint8_t CfgRange, CfgFifo;
bool M_bSpi;
uint8_t M_chip_select;
short x, y, z;

// SPI and I2C instances
#define I2C_PORT i2c0
// --- Function Prototypes ---

uint8_t readRegister8(MC3479_t *sens, uint8_t reg) {
  uint8_t value;

  i2c_write_blocking(sens->inst, sens->addr, &reg, 1, true);
  i2c_read_blocking(sens->inst, sens->addr, &value, 1, false);
  return value;
}

void writeRegister8(MC3479_t *sens, uint8_t reg, uint8_t value) {
  uint8_t data[] = {reg, value}; 
  i2c_write_blocking(sens->inst, sens->addr, data, 2, false);
  return;
}

// Initialize the MC34X9 sensor and set as the default configuration
bool start(MC3479_t *sens){
  // Init Reset
  reset(sens);
  SetMode(sens, MC34X9_MODE_STANDBY);

  /* Check I2C connection */
  uint8_t id = readRegister8(sens, MC34X9_REG_PROD);
  if (id != MC34X9_CHIP_ID)
  {
    /* No MC34X9 detected ... return false */
    printf("No MC34X9 detected!");
    printf("Chip ID: %x", id);
    return false;
  }

  // Range: 8g
  SetRangeCtrl(sens, MC34X9_CFG_RANGE_DEFAULT);
  // Sampling Rate: 50Hz by default
  SetSampleRate(sens, MC34X9_CFG_SAMPLE_RATE_DEFAULT);
  // Mode: Active
  SetMode(sens, MC34X9_MODE_CWAKE);

  sleep_ms(50);

  return true;
}

void wake(MC3479_t *sens)
{
  //Set mode as wake
  SetMode(sens, MC34X9_MODE_CWAKE);
}

void stop(MC3479_t *sens)
{
  //Set mode as Sleep
  SetMode(sens, MC34X9_MODE_STANDBY);
}

//Initial reset
void reset(MC3479_t *sens)
{
  // Stand by mode
  writeRegister8(sens, MC34X9_REG_MODE, MC34X9_MODE_STANDBY);

  sleep_ms(10);

  // power-on-reset
  writeRegister8(sens, 0x1c, 0x40);

  sleep_ms(50);

  // Disable interrupt
  writeRegister8(sens, 0x06, 0x00);
  sleep_ms(10);
  // 1.00x Aanalog Gain
  writeRegister8(sens, 0x2B, 0x00);
  sleep_ms(10);
  // DCM disable
  writeRegister8(sens, 0x15, 0x00);

  sleep_ms(50);
}

//Set the operation mode
void SetMode(MC3479_t *sens,  MC34X9_mode_t mode)
{
  uint8_t value;

  value = readRegister8(sens, MC34X9_REG_MODE);
  value &= 0b11110000;
  value |= mode;

  writeRegister8(sens, MC34X9_REG_MODE, value);
}

//Set the range control
void SetRangeCtrl(MC3479_t *sens, MC34X9_range_t range)
{
  uint8_t value;
  CfgRange = range;
  SetMode(sens, MC34X9_MODE_STANDBY);
  value = readRegister8(sens, MC34X9_REG_RANGE_C);
  value &= 0b00000111;
  value |= (range << 4) & 0x70;
  writeRegister8(sens, MC34X9_REG_RANGE_C, value);
}

//Set the sampling rate
void SetSampleRate(MC3479_t *sens, MC34X9_sr_t sample_rate)
{
  uint8_t value;
  SetMode(sens, MC34X9_MODE_STANDBY);
  value = readRegister8(sens, MC34X9_REG_SR);
  value &= 0b00000000;
  value |= sample_rate;
  writeRegister8(sens, MC34X9_REG_SR, value);
}

//Set FIFO feature
void SetFIFOCtrl(MC3479_t *sens, MC34X9_fifo_ctl_t fifo_ctl, MC34X9_fifo_mode_t fifo_mode, uint8_t fifo_thr){
  if (fifo_thr > 31)  //maximum threshold
    fifo_thr = 31;

  SetMode(sens, MC34X9_MODE_STANDBY);

  CfgFifo = (MC34X9_COMB_INT_ENABLE << 3) | ((fifo_ctl << 5) | (fifo_mode << 6)) ;

  writeRegister8(sens, MC34X9_REG_FIFO_CTRL, CfgFifo);

  uint8_t CfgFifoThr = fifo_thr;
  writeRegister8(sens, MC34X9_REG_FIFO_TH, CfgFifoThr);
}

void SetGerneralINTCtrl(MC3479_t *sens) {
  // Gerneral Interrupt setup
  uint8_t CfgGPIOINT = (((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 2) // int1
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 3)// int1
                        | ((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 6)// int2
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 7));// int2

  writeRegister8(sens, MC34X9_REG_GPIO_CTRL, CfgGPIOINT);
}

//Set interrupt control register
void SetINTCtrl(MC3479_t *sens, bool tilt_int_ctrl, bool flip_int_ctl, bool anym_int_ctl, bool shake_int_ctl, bool tilt_35_int_ctl){

  SetMode(sens, MC34X9_MODE_STANDBY);

  uint8_t CfgINT = (((tilt_int_ctrl & 0x01) << 0)
                    | ((flip_int_ctl & 0x01) << 1)
                    | ((anym_int_ctl & 0x01) << 2)
                    | ((shake_int_ctl & 0x01) << 3)
                    | ((tilt_35_int_ctl & 0x01) << 4)
                    | ((MC34X9_AUTO_CLR_ENABLE & 0x01) << 6));
  writeRegister8(sens, MC34X9_REG_INTR_CTRL, CfgINT);

  SetGerneralINTCtrl(sens);
}

//Set FIFO interrupt control register
void SetFIFOINTCtrl(MC3479_t *sens,  bool fifo_empty_int_ctl, bool fifo_full_int_ctl, bool fifo_thr_int_ctl){
  
  SetMode(sens, MC34X9_MODE_STANDBY);

  CfgFifo = CfgFifo
            | (((fifo_empty_int_ctl & 0x01) << 0)
              | ((fifo_full_int_ctl & 0x01) << 1)
              | ((fifo_thr_int_ctl & 0x01) << 2));

  writeRegister8(sens, MC34X9_REG_FIFO_CTRL, CfgFifo);

  SetGerneralINTCtrl(sens);
}

//Interrupt handler (clear interrupt flag)
void INTHandler(MC3479_t *sens, MC34X9_interrupt_event_t *ptINT_Event){
  
  uint8_t value;

  value = readRegister8(sens, MC34X9_REG_INTR_STAT);

  ptINT_Event->bTILT           = ((value >> 0) & 0x01);
  ptINT_Event->bFLIP           = ((value >> 1) & 0x01);
  ptINT_Event->bANYM           = ((value >> 2) & 0x01);
  ptINT_Event->bSHAKE          = ((value >> 3) & 0x01);
  ptINT_Event->bTILT_35        = ((value >> 4) & 0x01);

  value &= 0x40;
  writeRegister8(sens, MC34X9_REG_INTR_STAT, value);
}

//FIFO Interrupt handler (clear interrupt flag)
void FIFOINTHandler(MC3479_t *sens, MC34X9_fifo_interrupt_event_t *ptFIFO_INT_Event)
{
  uint8_t value;

  value = readRegister8(sens, MC34X9_REG_FIFO_INTR);

  ptFIFO_INT_Event->bFIFO_EMPTY           = ((value >> 0) & 0x01);
  ptFIFO_INT_Event->bFIFO_FULL            = ((value >> 1) & 0x01);
  ptFIFO_INT_Event->bFIFO_THRESH          = ((value >> 2) & 0x01);
}

//Get the range control
MC34X9_range_t GetRangeCtrl(MC3479_t *sens){
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister8(sens, MC34X9_REG_RANGE_C);
  printf("In GetRangeCtrl(): %x", value);
  value &= 0x70;
  return (MC34X9_range_t) (value >> 4);
}

//Get the output sampling rate
MC34X9_sr_t GetSampleRate(MC3479_t *sens){
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister8(sens, MC34X9_REG_SR);
  printf("In GetCWakeSampleRate(): %x", value);
  value &= 0b00011111;
  return (MC34X9_sr_t) (value);
}

//Is FIFO empty
bool IsFIFOEmpty(MC3479_t *sens){
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister8(sens, MC34X9_REG_FIFO_STAT);
  value &= 0x01;
  //Serial.println("FIFO_Status");
  //Serial.println(value, HEX);

  if (value ^ 0x01)
    return false;	//Not empty
  else {
    return true;  //Is empty
  }
}

//Read the raw counts and SI units measurement data
void readRawAccel(MC3479_t *sens){
  //{2g, 4g, 8g, 16g, 12g}
  float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
  // 16bit
  float faResolution = 32768.0f;

  uint8_t rawData[6];
  // Read the six raw data registers into data array
  read_regs(sens, MC34X9_REG_XOUT_LSB, rawData, 6);
  x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
  y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
  z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

  sens->XAxis = (short) (x);
  sens->YAxis = (short) (y);
  sens->ZAxis = (short) (z);
  sens->XAxis_g = (float) (x) / faResolution * faRange[CfgRange];
  sens->YAxis_g = (float) (y) / faResolution * faRange[CfgRange];
  sens->ZAxis_g = (float) (z) / faResolution * faRange[CfgRange];

  return;
}

// ***BUS***

/** I2C/SPI read function */
uint8_t read_regs(MC3479_t *sens, uint8_t reg, uint8_t *value, uint8_t size)
{
  i2c_write_blocking(sens->inst, sens->addr, &reg, 1, true);
  i2c_read_blocking(sens->inst, sens->addr, value, size, false);
  return 0;
}

/** I2C/SPI write function */
uint8_t mcube_write_regs(MC3479_t *sens, uint8_t reg, uint8_t *value, uint8_t size)
{
  uint8_t buffer[size + 1];
  buffer[0] = reg;
  memcpy(&buffer[1], value, size);
  i2c_write_blocking(sens->inst, sens->addr, buffer, size + 1, false);
  return 0;
}

// ***MC34X9 dirver motion part*** 
//TODO Keep going here
void M_DRV_MC34X6_SetTFThreshold(MC3479_t *sens, uint16_t threshold) {
  uint8_t _bFTThr[2] = {0};

  _bFTThr[0] = (threshold & 0x00ff);
  _bFTThr[1] = ((threshold & 0x7f00) >> 8 );


  // set threshold
  writeRegister8(sens, MC34X9_REG_TF_THRESH_LSB, _bFTThr[0]);
  writeRegister8(sens, MC34X9_REG_TF_THRESH_MSB, _bFTThr[1]);
}

void M_DRV_MC34X6_SetTFDebounce(MC3479_t *sens, uint8_t debounce) {
  // set debounce
  writeRegister8(sens, MC34X9_REG_TF_DB, debounce);
}

void M_DRV_MC34X6_SetANYMThreshold(MC3479_t *sens, uint16_t threshold) {
  uint8_t _bANYMThr[2] = {0};

  _bANYMThr[0] = (threshold & 0x00ff);
  _bANYMThr[1] = ((threshold & 0x7f00) >> 8 );

  // set threshold
  writeRegister8(sens, MC34X9_REG_AM_THRESH_LSB, _bANYMThr[0]);
  writeRegister8(sens, MC34X9_REG_AM_THRESH_MSB, _bANYMThr[1]);
}

void M_DRV_MC34X6_SetANYMDebounce(MC3479_t *sens, uint8_t debounce) {
  writeRegister8(sens, MC34X9_REG_AM_DB, debounce);
}

void M_DRV_MC34X6_SetShakeThreshold(MC3479_t *sens, uint16_t threshold) {
  uint8_t _bSHKThr[2] = {0};

  _bSHKThr[0] = (threshold & 0x00ff);
  _bSHKThr[1] = ((threshold & 0xff00) >> 8 );

  // set threshold
  writeRegister8(sens, MC34X9_REG_SHK_THRESH_LSB, _bSHKThr[0]);
  writeRegister8(sens, MC34X9_REG_SHK_THRESH_MSB, _bSHKThr[1]);
}

void M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(MC3479_t *sens, uint16_t threshold, uint8_t shakeCount) {

  uint8_t _bSHKP2PDuration[2] = {0};

  _bSHKP2PDuration[0] = (threshold & 0x00ff);
  _bSHKP2PDuration[1] = ((threshold & 0x0f00) >> 8);
  _bSHKP2PDuration[1] |= ((shakeCount & 0x7) << 4);

  // set peak to peak duration and count
  writeRegister8(sens, MC34X9_REG_PK_P2P_DUR_THRESH_LSB, _bSHKP2PDuration[0]);
  writeRegister8(sens, MC34X9_REG_PK_P2P_DUR_THRESH_MSB, _bSHKP2PDuration[1]);
}

void M_DRV_MC34X6_SetTILT35Threshold(MC3479_t *sens, uint16_t threshold) {
  M_DRV_MC34X6_SetTFThreshold(sens, threshold);
}

void M_DRV_MC34X6_SetTILT35Timer(MC3479_t *sens, uint8_t timer) {
  uint8_t value;

  value = readRegister8(sens, MC34X9_REG_TIMER_CTRL);
  value &= 0b11111000;
  value |= MC34X9_TILT35_2p0;

  writeRegister8(sens, MC34X9_REG_TIMER_CTRL, timer);
}

// Tilt & Flip
void _M_DRV_MC34X6_SetTilt_Flip(MC3479_t *sens) {
  // set threshold
  M_DRV_MC34X6_SetTFThreshold(sens, s_bCfgFTThr);
  // set debounce
  M_DRV_MC34X6_SetTFDebounce(sens, s_bCfgFTDebounce);
  return;
}

// AnyMotion
void _M_DRV_MC34X6_SetAnym(MC3479_t *sens) {
  // set threshold
  M_DRV_MC34X6_SetANYMThreshold(sens, s_bCfgANYMThr);

  // set debounce
  M_DRV_MC34X6_SetANYMDebounce(sens, s_bCfgANYMDebounce);
  return;
}

// Shake
void _M_DRV_MC34X6_SetShake(MC3479_t *sens) {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym(sens);

  // Config shake
  // set threshold
  M_DRV_MC34X6_SetShakeThreshold(sens, s_bCfgShakeThr);

  // set peak to peak duration and count
  M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(sens, s_bCfgShakeP2PDuration, s_bCfgShakeCount);
  return;
}

// Tilt 35
void _M_DRV_MC34X6_SetTilt35(MC3479_t *sens) {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym(sens);

  // Config Tilt35
  // set threshold
  M_DRV_MC34X6_SetTILT35Threshold(sens, s_bCfgTILT35Thr);

  //set timer
  M_DRV_MC34X6_SetTILT35Timer(sens, MC34X9_TILT35_2p0);
  return;
}