
#include <Arduino.h>
#include "Torque_Sensor.h"
#include <SPI.h>

void ads1292r::Torque_sensor_initial()
{
  delay(2000);
  
  // initalize the  data ready and chip select pins:
  pinMode(ADS1292_CS_PIN, OUTPUT);    //52
  pinMode(ADS1292_DRDY_PIN, INPUT_PULLUP);  //A10
  pinMode(ADS1292_START_PIN, OUTPUT);  //24
  pinMode(ADS1292_PWDN_PIN, OUTPUT);  //24
  //pinMode(11, INPUT);
  //pinMode(12, INPUT);
  //pinMode(13, INPUT);
  ads1292_Init();  //initalize ADS1292 slave
}

void ads1292r::Torque_sensor_read()
{
  if ((digitalRead(ADS1292_DRDY_PIN)) == LOW)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
  {
    SPI_RX_Buff_Ptr = ads1292_Read_Data(); // Read the data,point the data to a pointer
    //Serial.println("inside");
    for (int i = 0; i < 9; i++)
    {
      SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
    }
    ads1292dataReceived = true;
  }
  if (ads1292dataReceived == true)      // process the data
  {
    //double a=0;
    chi = 0;
    for (int ii = 3; ii < 9; ii += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
    {
      uecgtemp=0;
      secgtemp=0;
      uecgtemp = (unsigned long) (((unsigned long)SPI_RX_Buff[ii + 0] << 16) | ( (unsigned long) SPI_RX_Buff[ii + 1] << 8) |  (unsigned long) SPI_RX_Buff[ii + 2]);
      uecgtemp = (unsigned long) (uecgtemp << 8);
      secgtemp = (signed long) (uecgtemp);
      secgtemp = secgtemp>>8;
      s32DaqVals[chi++] = secgtemp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
    }
    torque[0] = (s32DaqVals[0] - torque_offset[0]) * torque_scale[0];
    torque[1] = (s32DaqVals[1] - torque_offset[1]) * torque_scale[1];
//    Serial.print(s32DaqVals[1]);
//    Serial.print("  ");
//    Serial.print(torque_offset[1]);
//    Serial.print("  ");
//    Serial.print(torque_scale[1]);
//    Serial.print("  ");
//    Serial.println(torque[1]);
       
//    index=index+1;
//    if(index%1000==0)
//    {
//    Serial.print("  ");
//    Serial.println(index);
//    }
    
  }
  ads1292dataReceived = false;
  SPI_RX_Buff_Count = 0;
}

void ads1292r::Torque_sensor_gain(double scale1,double scale2)
{
   torque_scale[0]=scale1;
   torque_scale[1]=scale2; 
}

void  ads1292r::Torque_sensor_offset_calibration()
{
  int offseti = 0;
  long torque_offset_array[2][200];
  long torque_calibrate_time_start = 0;
  long torque_calibrate_time_end = 0;
  long torque_calibrate_time_pass = 2000;
  torque_calibrate_time_start = millis();
  torque_calibrate_time_end = torque_calibrate_time_start;
  offseti = 0;
  while ((torque_calibrate_time_end - torque_calibrate_time_start) < torque_calibrate_time_pass)
  {
    torque_calibrate_time_end = millis();
    if ((digitalRead(ADS1292_DRDY_PIN)) == LOW)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
    {
      SPI_RX_Buff_Ptr = ads1292_Read_Data(); // Read the data,point the data to a pointer
      for (int i = 0; i < 9; i++)
      {
        SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
      }
      ads1292dataReceived = true;
    }
    if (ads1292dataReceived == true)      // process the data
    {
      chi = 0;
      for (int i = 3; i < 9; i += 3)         // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
      {

        uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff[i + 0] << 16) | ( (unsigned long) SPI_RX_Buff[i + 1] << 8) |  (unsigned long) SPI_RX_Buff[i + 2]);
        uecgtemp = (unsigned long) (uecgtemp << 8);
        secgtemp = (signed long) (uecgtemp);
        secgtemp = (signed long) (secgtemp >> 8);
        s32DaqVals[chi++] = secgtemp;  //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
      }
      torque_offset_array[0][offseti] = s32DaqVals[0];
      torque_offset_array[1][offseti] = s32DaqVals[1];
      offseti++;
      offseti = offseti % 100;
    }
    ads1292dataReceived = false;
    SPI_RX_Buff_Count = 0;
  }
  for (int c = 0; c < 100; c++)
  {
    torque_offset[0] = torque_offset[0] + torque_offset_array[0][c];
    torque_offset[1] = torque_offset[1] + torque_offset_array[1][c];
  }
  torque_offset[0] = torque_offset[0] / 100;
  torque_offset[1] = torque_offset[1] / 100;
}
char* ads1292r::ads1292_Read_Data()
{

  static char SPI_Dummy_Buff[12];
  SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS1292_CS_PIN, LOW);
  for (int i = 0; i < 9; ++i)
  {
    SPI_Dummy_Buff[i] = SPI1.transfer(CONFIG_SPI_MASTER_DUMMY);

  }
  digitalWrite(ADS1292_CS_PIN, HIGH);
  SPI1.endTransaction();
  return SPI_Dummy_Buff;
}

void ads1292r::ads1292_Init()
{
  // start the SPI library:
  SPI1.begin();
  SPI1.setBitOrder(MSBFIRST);
  //CPOL = 0, CPHA = 1
  SPI1.setDataMode(SPI_MODE1);
  // Selecting 1Mhz clock for SPI
  SPI1.setClockDivider(SPI_CLOCK_DIV16);

  ads1292_Reset();
  delay(100);
  ads1292_Disable_Start();
  ads1292_Enable_Start();

  ads1292_Hard_Stop();
  ads1292_Start_Data_Conv_Command();
  ads1292_Soft_Stop();
  delay(50);
  ads1292_Stop_Read_Data_Continuous();					// SDATAC command
  delay(300);

  ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x00); 		//Set sampling rate to 250 SPS
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CONFIG2, 0b10100000);	//Lead-off comp off, test signal disabled
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFF, 0b00010000);		//Lead-off defaults
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH1SET, 0b01100000);	//Ch 1 enabled, gain 12, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH2SET, 0b01100000);	//Ch 2 enabled, gain 12, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RLDSENS, 0b00101100);	//0b00100000RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFFSENS, 0x00);		//LOFF settings: all disabled
  delay(10);
  //Skip register 8, LOFF Settings default
  ads1292_Reg_Write(ADS1292_REG_RESP1, 0b00000010);		//00000010 011110010Respiration: MOD/DEMOD turned only, phase 0
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RESP2, 0b00000011);		//00000011 Respiration: Calib OFF, respiration freq defaults
  delay(10);
  ads1292_Start_Read_Data_Continuous();
  delay(10);
  ads1292_Enable_Start();
}

void ads1292r::ads1292_Reset()
{
  digitalWrite(ADS1292_PWDN_PIN, HIGH);
  delay(100);					// Wait 100 mSec
  digitalWrite(ADS1292_PWDN_PIN, LOW);
  delay(100);
  digitalWrite(ADS1292_PWDN_PIN, HIGH);
  delay(100);
}

void ads1292r::ads1292_Disable_Start()
{
  digitalWrite(ADS1292_START_PIN, LOW);
  delay(20);
}

void ads1292r::ads1292_Enable_Start()
{
  digitalWrite(ADS1292_START_PIN, HIGH);
  delay(20);
}

void ads1292r::ads1292_Hard_Stop (void)
{
  digitalWrite(ADS1292_START_PIN, LOW);
  delay(100);
}


void ads1292r::ads1292_Start_Data_Conv_Command (void)
{
  ads1292_SPI_Command_Data(START);					// Send 0x08 to the ADS1x9x
}

void ads1292r::ads1292_Soft_Stop (void)
{
  ads1292_SPI_Command_Data(STOP);                   // Send 0x0A to the ADS1x9x
}

void ads1292r::ads1292_Start_Read_Data_Continuous (void)
{
  ads1292_SPI_Command_Data(RDATAC);					// Send 0x10 to the ADS1x9x
}

void ads1292r::ads1292_Stop_Read_Data_Continuous (void)
{
  ads1292_SPI_Command_Data(SDATAC);					// Send 0x11 to the ADS1x9x
}

void ads1292r::ads1292_SPI_Command_Data(unsigned char data_in)
{
  //byte data[1];
  //SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  //data[0] = data_in;
  digitalWrite(ADS1292_CS_PIN, LOW);
  SPI1.transfer(data_in);
  delay(2);
  digitalWrite(ADS1292_CS_PIN, HIGH);
  SPI1.endTransaction();
}

//Sends a write command to SCP1000
void ads1292r::ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
  switch (READ_WRITE_ADDRESS)
  {
    case 1:
      DATA = DATA & 0x87;
      break;
    case 2:
      DATA = DATA & 0xFB;
      DATA |= 0x80;
      break;
    case 3:
      DATA = DATA & 0xFD;
      DATA |= 0x10;
      break;
    case 7:
      DATA = DATA & 0x3F;
      break;
    case 8:
      DATA = DATA & 0x5F;
      break;
    case 9:
      DATA |= 0x02;
      break;
    case 10:
      DATA = DATA & 0x87;
      DATA |= 0x01;
      break;
    case 11:
      DATA = DATA & 0x0F;
      break;
    default:
      break;
  }

  // now combine the register address and the command into one byte:
  byte dataToSend = READ_WRITE_ADDRESS | WREG;
  //SPI1.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  // take the chip select low to select the device:
  digitalWrite(ADS1292_CS_PIN, LOW);
  delay(2);
  SPI1.transfer(dataToSend); //Send register location
  SPI1.transfer(0x00);		//number of register to wr
  SPI1.transfer(DATA);		//Send value to record into register
  delay(2);
  digitalWrite(ADS1292_CS_PIN, HIGH); // take the chip select high to de-select:
  SPI1.endTransaction();
}
