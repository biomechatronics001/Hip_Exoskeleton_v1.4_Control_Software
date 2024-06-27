// #include <variant.h>
#include <Arduino.h>

#define PROFILE_SELECTOR (0)

// SERIAL_WL for wireless serial
// Yellow 3.3V    White RX    Red TX    Black GROUND
#define SERIAL_WL (Serial2)
#define INIT_TIME (4) // unit : second

typedef float CAL_TYPE;

class IMU
{
public:
  void INIT();
  void INIT_MEAN();
  void READ();
  void GetData();
  void Print();

  // Body parameters
  float m_t = 19.6;    
  float l_t = 0.441;      
  float l_t1 = 0.245;
  float m_b = 52.2;
  float l_b = 0.574; 
  float l_b1 = 0.287;
  float g = 9.81; // gravity acceleration

  CAL_TYPE init_TKx=0;
  CAL_TYPE init_TKy=0;
  CAL_TYPE init_TKz=0;
  CAL_TYPE init_LTx=0;
  CAL_TYPE init_LTy=0;
  CAL_TYPE init_LTz=0;
  CAL_TYPE init_RTx=0;
  CAL_TYPE init_RTy=0;
  CAL_TYPE init_RTz=0;
  CAL_TYPE init_LSx=0;
  CAL_TYPE init_LSy=0;
  CAL_TYPE init_LSz=0;
  CAL_TYPE init_RSx=0;
  CAL_TYPE init_RSy=0;
  CAL_TYPE init_RSz=0;
  CAL_TYPE init_LFx=0;
  CAL_TYPE init_LFy=0;
  CAL_TYPE init_LFz=0;
  CAL_TYPE init_RFx=0;
  CAL_TYPE init_RFy=0;
  CAL_TYPE init_RFz=0;

  CAL_TYPE TKx=0;
  CAL_TYPE LTx=0;
  CAL_TYPE RTx=0;
  CAL_TYPE LSx=0;
  CAL_TYPE RSx=0;
  CAL_TYPE LFx=0;
  CAL_TYPE RFx=0;
  CAL_TYPE TKy=0;
  CAL_TYPE LTy=0;
  CAL_TYPE RTy=0;
  CAL_TYPE LSy=0;
  CAL_TYPE RSy=0;
  CAL_TYPE LFy=0;
  CAL_TYPE RFy=0;
  CAL_TYPE TKz=0;
  CAL_TYPE LTz=0;
  CAL_TYPE RTz=0;
  CAL_TYPE LSz=0;
  CAL_TYPE RSz=0;
  CAL_TYPE LFz=0;
  CAL_TYPE RFz=0;

  CAL_TYPE TKAVx=0;
  CAL_TYPE LTAVx=0;
  CAL_TYPE RTAVx=0;
  CAL_TYPE LSAVx=0;
  CAL_TYPE RSAVx=0;
  CAL_TYPE LFAVx=0;
  CAL_TYPE RFAVx=0;
  CAL_TYPE TKAVy=0;
  CAL_TYPE LTAVy=0;
  CAL_TYPE RTAVy=0;
  CAL_TYPE LSAVy=0;
  CAL_TYPE RSAVy=0;
  CAL_TYPE LFAVy=0;
  CAL_TYPE RFAVy=0;
  CAL_TYPE TKAVz=0;
  CAL_TYPE LTAVz=0;
  CAL_TYPE RTAVz=0;
  CAL_TYPE LSAVz=0;
  CAL_TYPE RSAVz=0;
  CAL_TYPE LFAVz=0;
  CAL_TYPE RFAVz=0;

  
  // CAL_TYPE DOTC_walking2[2];
  // 
  // CAL_TYPE DOTC_descending[2];
  
  
  // CAL_TYPE y_delay_walking2[100]={0};
  
  // CAL_TYPE y_delay_descending_left[100]={0};
  // CAL_TYPE y_delay_descending_right[100]={0};
  
  
  // CAL_TYPE RLKx_delay_walking2[100]={0};
  // CAL_TYPE RLKx_delay_ascending[100]={0};
  // CAL_TYPE RLKx_delay_descending[100]={0};
   
  // int delayindex_walking=0;
  // int delayindex_walking2=0;
  // 
  // int delayindex_descending=0;
   
  // int currentpoint_walking=0;
  // int currentpoint_walking2=0;
  // 
  // int currentpoint_descending=0;
  
  // int doi_walking=0;
  // int doi_walking2=0;
  // 
  // int doi_descending=0;

  // double y_filtered_walking=0;
  // double y_filtered_walking2=0;
  
  // double y_filtered_descending_right=0;
  // double y_filtered_descending_left=0;
  CAL_TYPE DOFC_ascending[2];
  CAL_TYPE DOFC_descending[2];
  
  double y_filtered_ascending_right=0;
  double y_filtered_ascending_left=0;
  CAL_TYPE y_delay_ascending_left[100]={0};
  CAL_TYPE y_delay_ascending_right[100]={0};
  int currentpoint_ascending=0;
  int delayindex_ascending=0;
  int doi_ascending=0;
  CAL_TYPE DOTC_ascending[2];
  double knee_average_angle;

  // void DelayedOutputTorqueControl();
  // void Walking_algorithm();
  // void Walking_algorithm_modified();
  void Stair_Ascending();
  // void Stair_descending_algorithm();
  
  CAL_TYPE LKx=0; 
  CAL_TYPE RKx=0;
  CAL_TYPE RLKx=0;
  CAL_TYPE LKx_filtered_last=0;
  CAL_TYPE LKx_filtered=0;
  CAL_TYPE RKx_filtered_last=0;
  CAL_TYPE RKx_filtered=0;
  CAL_TYPE RLKx_filtered=0;
  CAL_TYPE DOTC[2];
  CAL_TYPE SquatTorque;
  CAL_TYPE STSTorque; // sit-to-stand
  CAL_TYPE y_delay[100]={0};
  CAL_TYPE RLKx_delay[100]={0};
  CAL_TYPE y_raw=0;
  CAL_TYPE y_filtered=0;
  CAL_TYPE y_filtered_last=0;
  
  // CAL_TYPE DOTC_walking[2];
  // CAL_TYPE y_delay_walking[100]={0};
  // CAL_TYPE RLKx_delay_walking[100]={0};
   
  int delayindex=0;
  int delaypoint=50;
  int currentpoint=0;
  int doi=0;
  double Gain_E=0;
  double Gain_F=0;
  double STS_Gain = 1;
  double test=0;
  double test1=0;
  double test2=0;
  double alpha = 0; // for IMU-based sit-to-stand
  double beta = 0; // for IMU-based sit-to-stand
  double angleThreshold = 0; // for IMU-based sit-to-stand
  double angleRelativeThreshold = 0; // for IMU-based sit-to-stand
  double theta = 0; // for IMU-based sit-to-stand

  

  void DelayOutputTorqueCommand();
  void STSTorqueCommand();
  void SquatTorqueCommand();
  
private:

  int count1 = 0;
  uint8_t st = 0;
  uint8_t Datain[203];
  int read_count = 0;
  uint8_t ch;
  void Packet_Decode(uint8_t c);

};
