//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//
/////////////////////////////////////////////////////////////////////////////////////////


#ifndef TORQUE_SENSOR_h
#define TORQUE_SENSOR_h
#include <Arduino.h>
#include <SPI.h>


#define CONFIG_SPI_MASTER_DUMMY   0xFF

// Register Read Commands
#define  RREG    0x20;		//Read n nnnn registers starting at address r rrrr
//first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG    0x40;		//Write n nnnn registers starting at address r rrrr
//first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define START		0x08		//Start/restart (synchronize) conversions
#define STOP		0x0A		//Stop conversion
#define RDATAC      0x10		//Enable Read Data Continuous mode. 

//This mode is the default mode at power-up.
#define SDATAC		0x11		//Stop Read Data Continuously mode
#define RDATA		0x12		//Read data by command; supports multiple read back.

//Pin declartion the other you need are controlled by the SPI library
const int ADS1292_CS_PIN = 0;//35;//39;
const int ADS1292_DRDY_PIN = 33;//36;//36;
const int ADS1292_START_PIN = 36;//37;//37;
const int ADS1292_PWDN_PIN = 37;//38;//38;

//register address
#define ADS1292_REG_ID			0x00
#define ADS1292_REG_CONFIG1		0x01
#define ADS1292_REG_CONFIG2		0x02
#define ADS1292_REG_LOFF		0x03
#define ADS1292_REG_CH1SET		0x04
#define ADS1292_REG_CH2SET		0x05
#define ADS1292_REG_RLDSENS		0x06
#define ADS1292_REG_LOFFSENS    0x07
#define ADS1292_REG_LOFFSTAT    0x08
#define ADS1292_REG_RESP1	    0x09
#define ADS1292_REG_RESP2	    0x0A

class ads1292r
{
  private:
    double torque_offset[2] = {0};
    double torque_scale[2] = {0.0003446};
    volatile signed long s32DaqVals[8];
    volatile byte SPI_RX_Buff[12] ;
    volatile int SPI_RX_Buff_Count = 0;
    volatile char *SPI_RX_Buff_Ptr;
    volatile bool ads1292dataReceived = false;
    unsigned long uecgtemp = 0;
    long secgtemp = 0;
    int chi = 0;
  public:
    int index=0;
    //volatile uint8_t SPI_Dummy_Buff[10];

    double torque[2] = {0};
    static void ads1292_Init(void);
    static void ads1292_Reset(void);
    static void ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA);
    static void ads1292_Reg_Read (unsigned char READ_WRITE_ADDRESS);
    static void ads1292_SPI_Command_Data(unsigned char data_in);
    static void ads1292_Disable_Start(void);
    static void ads1292_Enable_Start(void);
    static void ads1292_Hard_Stop (void);
    static void ads1292_Start_Data_Conv_Command (void);
    static void ads1292_Soft_Stop (void);
    static void ads1292_Start_Read_Data_Continuous (void);
    static void ads1292_Stop_Read_Data_Continuous (void);
    static char* ads1292_Read_Data(void);
    void Torque_sensor_initial(void);
    void Torque_sensor_read(void);
    void Torque_sensor_offset_calibration(void);
    void Torque_sensor_gain(double scale1,double scale2);
};

#endif
