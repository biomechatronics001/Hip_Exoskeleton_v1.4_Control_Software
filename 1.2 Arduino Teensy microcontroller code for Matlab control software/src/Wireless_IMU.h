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
  
  double y_filtered_ascending_right=0;
  double y_filtered_ascending_left=0;
  CAL_TYPE y_delay_ascending_left[100]={0};
  CAL_TYPE y_delay_ascending_right[100]={0};
  int currentpoint_ascending=0;
  int delayindex_ascending=0;
  int doi_ascending=0;
  CAL_TYPE DOTC_ascending[2];
  

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
  CAL_TYPE STSTorque2; // sit-to-stand2 Nm
  CAL_TYPE STSTorque3; // sit-to-stand2 Nm
  CAL_TYPE y_delay[100]={0};
  CAL_TYPE RLKx_delay[100]={0};
  CAL_TYPE y_raw=0;
  CAL_TYPE y_filtered=0;
  CAL_TYPE y_filtered_last=0;
  // for Samsung controller using hip angle as input
  CAL_TYPE RLTx = 0; // difference between left and right thigh angle
  CAL_TYPE LTx_filtered_last = 0;
  CAL_TYPE LTx_filtered = 0;
  CAL_TYPE RTx_filtered_last = 0;
  CAL_TYPE RTx_filtered = 0;
  CAL_TYPE RLTx_filtered = 0;
  CAL_TYPE RLTx_delay[100] = {0}; // difference between left and right thigh with delay
  
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
  double knee_average_angle; // for IMU-based sit-to-stand
  // for sit-to-stand v2
  double hipXHistory[11] = {0.0};
  double thighXHistory[21] = {0.0}; // XX second long buffer
  double kneeXHistory[11] = {0.0};
  int isTriggered = 0;
  int isSitting = 0;
  int dataCounter = 1;
  double thigh_average_angle;
  double knee_average_speed;
  double STSSlopeThreshold = 0.15;
  // for STSTorqueCommand3()
  int STSStandThreshold = 5; //deg
  double STSStandPreviousMinAngle = 5.0; // deg, used to update threshold angle if necessary
  int STSSitThreshold = 75; // deg
  double STSSitPreviousMaxAngle = 75.0; // deg, used to update threshold angle if necessary
  double STSStandSlackAngle = 7.5; // deg
  double STSSitSlackAngle = 7.5; // deg
  double STS_Gain_Stand = 1.0;
  double STS_Gain_Sit = 1.0;
  double thighXSlopeStandThreshold = 0.03; // rad/s, above this threshold is considered standing up
  double thighXSlopeSitThreshold = -0.03; // rad/s, below this threshold is considered sitting down
  double isFirstSTS = 1;
  double slopeLTx = 0.0;
  double numeratorLTx = 0.0;
  double denominator = 0.0;
  int STS_in_stand_region = 2;
  int STS_in_sit_region = 2;
  int STS_is_sitting_down = 0;
  int STS_is_standing_up = 0;
  int STS_state = 0; // 0 = just started; 1 = stand; 2 = sitting down; 3 = sit; 4 = standing up;
  double temp = 0.0;
  double temp2 = 0.0;
  double STSStandUpMagnitude = 0.0;
  double STSSitDownMagnitude = 0.0;
  int STSStandMagnitudeUpdated = 0;
  int STSSitMagnitudeUpdated = 0;

  double Sit2StandProfileList[101] = {0,0.01982,0.03956,0.05922,0.07881,0.0983,0.1177,0.137,0.1562,0.1753,0.1943,0.2132,0.232,0.2506,0.2691,0.2875,0.3058,0.3239,0.3419,0.3597,0.3774,0.3949,0.4123,0.4295,0.4465,0.4633,0.48,0.4965,0.5128,0.529,0.5449,0.5606,0.5762,0.5915,0.6066,0.6215,0.6362,0.6506,0.6649,0.6789,0.6926,0.7061,0.7194,0.7325,0.7452,0.7577,0.77,0.782,0.7937,0.8051,0.8163,0.8272,0.8378,0.8481,0.8581,0.8678,0.8772,0.8863,0.895,0.9035,0.9116,0.9194,0.9269,0.9341,0.9409,0.9473,0.9535,0.9592,0.9646,0.9697,0.9744,0.9787,0.9826,0.9862,0.9894,0.9922,0.9946,0.9966,0.9982,0.9994,1,1.001,0.9997,0.9971,0.9919,0.9836,0.9713,0.9545,0.9324,0.9043,0.8694,0.8273,0.777,0.7179,0.6494,0.5706,0.4811,0.3799,0.2665,0.1401,0};
  int thetaIndex = 0;
  

  
  void AssignLimbSegmentKinemtaics();
  void DelayOutputTorqueCommand();
  void STSTorqueCommand();
  void STSTorqueCommand3();
  void SquatTorqueCommand();
  void SamsungWalkingController();
  
  
private:

  int count1 = 0;
  uint8_t st = 0;
  uint8_t Datain[203];
  int read_count = 0;
  uint8_t ch;
  void Packet_Decode(uint8_t c);

};
