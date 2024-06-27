#include "Wireless_IMU.h"
union u_tag
{
  byte b[4];
  float fval;
} u;
float IMUdata[52] = {0};

void IMU::Packet_Decode(uint8_t c)
{
  switch (st)
  {
  case 0: // Read 1st Byte
    if (c == 0x3a)
    {
      st = 1;
      Datain[read_count] = c;
      read_count += 1;
    }
    break;
  case 1: // Read 2nd Byte
    if (c == 0xc4)
    {
      st = 2;
      Datain[read_count] = c;
      read_count += 1;
    }
    else
    {
      st = 0;
      read_count = 0;
    }
    break;
  case 2:
    Datain[read_count] = c;
    read_count += 1;
    if (read_count >= 203)
    {
      st = 0;
      GetData();
      AssignLimbSegmentKinemtaics();
      SamsungWalkingController();
      // STSTorqueCommand();
      // STSTorqueCommand3();
      // Stair_Ascending();
      // SquatTorqueCommand();
      read_count = 0;
    }
    break;
  default:
    st = 0;
    break;
  }
}

void IMU::INIT()
{
  SERIAL_WL.begin(230400);
}

void IMU::READ()
{
  if (SERIAL_WL.available())
  {
    ch = SERIAL_WL.read();
    Packet_Decode(ch);
  }
}

void IMU::INIT_MEAN()
{
  unsigned long timing = micros();
  while (micros() - timing < INIT_TIME * 1000000)
  {
    unsigned long n = 0;
    READ();
    GetData();

    init_TKx += (IMUdata[4] - init_TKx) / (n + 1);
    init_TKy += (IMUdata[5] - init_TKy) / (n + 1);
    init_TKz += (IMUdata[6] - init_TKz) / (n + 1);

    init_LTx += (IMUdata[11] - init_LTx) / (n + 1);
    init_LTy += (IMUdata[12] - init_LTy) / (n + 1);
    init_LTz += (IMUdata[13] - init_LTz) / (n + 1);

    init_RTx += (IMUdata[18] - init_RTx) / (n + 1);
    init_RTy += (IMUdata[19] - init_RTy) / (n + 1);
    init_RTz += (IMUdata[20] - init_RTz) / (n + 1);

    init_LSx += (IMUdata[25] - init_LSx) / (n + 1);
    init_LSy += (IMUdata[26] - init_LSy) / (n + 1);
    init_LSz += (IMUdata[27] - init_LSz) / (n + 1);

    init_RSx += (IMUdata[32] - init_RSx) / (n + 1);
    init_RSy += (IMUdata[33] - init_RSy) / (n + 1);
    init_RSz += (IMUdata[34] - init_RSz) / (n + 1);

    init_LFx += (IMUdata[39] - init_LFx) / (n + 1);
    init_LFy += (IMUdata[40] - init_LFy) / (n + 1);
    init_LFz += (IMUdata[41] - init_LFz) / (n + 1);

    init_RFx += (IMUdata[46] - init_RFx) / (n + 1);
    init_RFy += (IMUdata[47] - init_RFy) / (n + 1);
    init_RFz += (IMUdata[48] - init_RFz) / (n + 1);

    n += 1;
  }
  // Serial.println("11111");
  // Serial.print(init_LTx);
  // Serial.print(" ");
  // Serial.println(init_RTx);
}

void IMU::GetData()
{
  for (int i = 3; i < 198; i = i + 4)
  {
    u.b[0] = Datain[i];
    u.b[1] = Datain[i + 1];
    u.b[2] = Datain[i + 2];
    u.b[3] = Datain[i + 3];
    IMUdata[(i - 3) / 4] = u.fval;
  }
}

void IMU::AssignLimbSegmentKinemtaics()
{
  GetData();
  TKx = IMUdata[4] - init_TKx; // Trunk angle
  TKy = IMUdata[5] - init_TKy;
  TKz = IMUdata[6] - init_TKz;
  TKAVx = IMUdata[1];
  TKAVy = IMUdata[2];
  TKAVz = IMUdata[3];

  LTx = IMUdata[11] - init_LTx; // Left Thigh
  LTy = IMUdata[12] - init_LTy;
  LTz = IMUdata[13] - init_LTz;
  LTAVx = IMUdata[8];
  LTAVy = IMUdata[9];
  LTAVz = IMUdata[10];

  RTx = IMUdata[18] - init_RTx; // Right Thigh
  RTy = IMUdata[19] - init_RTy;
  RTz = IMUdata[20] - init_RTz;
  RTAVx = IMUdata[15];
  RTAVy = IMUdata[16];
  RTAVz = IMUdata[17];

  LSx = IMUdata[25] - init_LSx; // Left Shank
  LSy = IMUdata[26] - init_LSy;
  LSz = IMUdata[27] - init_LSz;
  LSAVx = IMUdata[22];
  LSAVy = IMUdata[23];
  LSAVz = IMUdata[24];

  RSx = IMUdata[32] - init_RSx; // Right Shank
  RSy = IMUdata[33] - init_RSy;
  RSz = IMUdata[34] - init_RSz;
  RSAVx = IMUdata[29];
  RSAVy = IMUdata[30];
  RSAVz = IMUdata[31];

  LFx = IMUdata[39] - init_LFx; // Left Foot
  LFy = IMUdata[40] - init_LFy;
  LFz = IMUdata[41] - init_LFz;
  LFAVx = IMUdata[36];
  LFAVy = IMUdata[37];
  LFAVz = IMUdata[38];

  RFx = IMUdata[46] - init_RFx; // Right Foot
  RFy = IMUdata[47] - init_RFy;
  RFz = IMUdata[48] - init_RFz;
  RFAVx = IMUdata[43];
  RFAVy = IMUdata[44];
  RFAVz = IMUdata[45];

  LKx = LSx - LTx;  // Left Knee
  RKx = RSx - RTx;  // Right Knee
}

void IMU::SamsungWalkingController()
{
  // GetData();
  // AssignLimbSegmentKinemtaics();
  // TKx = IMUdata[4] - init_TKx; // Trunk angle
  // TKy = IMUdata[5] - init_TKy;
  // TKz = IMUdata[6] - init_TKz;
  // TKAVx = IMUdata[1];
  // TKAVy = IMUdata[2];
  // TKAVz = IMUdata[3];

  // LTx = IMUdata[11] - init_LTx; // Left Thigh
  // LTy = IMUdata[12] - init_LTy;
  // LTz = IMUdata[13] - init_LTz;
  // LTAVx = IMUdata[8];
  // LTAVy = IMUdata[9];
  // LTAVz = IMUdata[10];

  // RTx = IMUdata[18] - init_RTx; // Right Thigh
  // RTy = IMUdata[19] - init_RTy;
  // RTz = IMUdata[20] - init_RTz;
  // RTAVx = IMUdata[15];
  // RTAVy = IMUdata[16];
  // RTAVz = IMUdata[17];

  // LSx = IMUdata[25] - init_LSx; // Left Shank
  // LSy = IMUdata[26] - init_LSy;
  // LSz = IMUdata[27] - init_LSz;
  // LSAVx = IMUdata[22];
  // LSAVy = IMUdata[23];
  // LSAVz = IMUdata[24];

  // RSx = IMUdata[32] - init_RSx; // Right Shank
  // RSy = IMUdata[33] - init_RSy;
  // RSz = IMUdata[34] - init_RSz;
  // RSAVx = IMUdata[29];
  // RSAVy = IMUdata[30];
  // RSAVz = IMUdata[31];

  // LFx = IMUdata[39] - init_LFx; // Left Foot
  // LFy = IMUdata[40] - init_LFy;
  // LFz = IMUdata[41] - init_LFz;
  // LFAVx = IMUdata[36];
  // LFAVy = IMUdata[37];
  // LFAVz = IMUdata[38];

  // RFx = IMUdata[46] - init_RFx; // Right Foot
  // RFy = IMUdata[47] - init_RFy;
  // RFz = IMUdata[48] - init_RFz;
  // RFAVx = IMUdata[43];
  // RFAVy = IMUdata[44];
  // RFAVz = IMUdata[45];

  // LTx_ble = imu.LTx * 100;
  // RTx_ble = imu.RTx * 100;

  // LKx = LSx - LTx;  // Left Knee
  // RKx = RSx - RTx;  // Right Knee

  // Using hip angle as input
  RLTx = RTx - LTx; // right-left thigh angle difference
  LTx_filtered_last = LTx_filtered;
  LTx_filtered = ((0.95 * LTx_filtered_last) + (0.05 * LTx));
  RTx_filtered_last = RTx_filtered;
  RTx_filtered = ((0.95 * RTx_filtered_last) + (0.05 * RTx));
  RLTx_filtered = RTx_filtered - LTx_filtered;
  RLTx_delay[doi] = RLTx_filtered;
  y_filtered = (sin(RTx_filtered * PI / 180) - sin(LTx_filtered * PI / 180));
  y_delay[doi] = y_filtered;
  
  currentpoint = doi;
  delayindex = doi - delaypoint;
  if (delayindex < 0)
  {
    delayindex = delayindex + 100;
  }
  else if (delayindex >= 100)
  {
    delayindex = delayindex - 100;
  }
  doi++;
  doi = doi % 100;
  double samsung_gain = 0.8;
  if ((RLTx_delay[delayindex] >= 0) && (RLTx_delay[delayindex] < 120))
  {
    DOTC[0] = - samsung_gain * Gain_E * y_delay[delayindex]; // left hip torque
    DOTC[1] = samsung_gain * Gain_F * y_delay[delayindex];  // right hip torque
  }
  else if ((RLTx_delay[delayindex] < 0) && (RLTx_delay[delayindex] > -120))
  {
    DOTC[1] = samsung_gain * Gain_E * y_delay[delayindex];  // right hip torque
    DOTC[0] = -samsung_gain * Gain_F * y_delay[delayindex]; // left hip torque
  }
  else
  {
    // DOTC[1]=0;
    // DOTC[0]=0;
  }
  // Serial.println(DOTC[0]);

  // Using knee angle as input
  // RLKx = RKx - LKx; // right-left knee angle difference
  // LKx_filtered_last = LKx_filtered;
  // LKx_filtered = ((0.95 * LKx_filtered_last) + (0.05 * LKx));
  // RKx_filtered_last = RKx_filtered;
  // RKx_filtered = ((0.95 * RKx_filtered_last) + (0.05 * RKx));
  // RLKx_filtered = RKx_filtered - LKx_filtered;
  // y_filtered = (sin(RKx_filtered * PI / 180) - sin(LKx_filtered * PI / 180));
  // y_delay[doi] = y_filtered;
  // RLKx_delay[doi] = RLKx_filtered;
  // currentpoint = doi;
  // delayindex = doi - delaypoint;
  // if (delayindex < 0)
  // {
  //   delayindex = delayindex + 100;
  // }
  // doi++;
  // doi = doi % 100;
  // //////**************Samsung walking algorithm************
  // //  test = -Gain_E*y_delay[delayindex];//left knee torque
  // //  test1 = -Gain_F*y_delay[delayindex];//left knee torque
  // //  test2 = RLKx_delay[delayindex];
  // if ((RLKx_delay[delayindex] >= 0) && (RLKx_delay[delayindex] < 120))
  // {
  //   DOTC[0] = -Gain_E * y_delay[delayindex]; // left knee torque
  //   DOTC[1] = Gain_F * y_delay[delayindex];  // right knee torque
  // }
  // else if ((RLKx_delay[delayindex] < 0) && (RLKx_delay[delayindex] > -120))
  // {
  //   DOTC[1] = Gain_E * y_delay[delayindex];  // right knee torque
  //   DOTC[0] = -Gain_F * y_delay[delayindex]; // left knee torque
  // }
  // else
  // {
  //   // DOTC[1]=0;
  //   // DOTC[0]=0;
  // }
}

// squat torque assistant
void IMU::SquatTorqueCommand()
{
  SquatTorque = -0.5 * (52.2 * (-9.8) * (0.287 * (-1) * sin(TKx * PI / 180) + 0.441 * sin((-1) * (RTx + LTx) / 2 * PI / 180)) + 19.6 * (-9.8) * 0.245 * sin((-1) * (RTx + LTx) / 2 * PI / 180));
}

// void IMU::STSTorqueCommand()
// {
//   double alpha = 3.69; // 3.69;
//   double beta = 2;     // 2;
//   knee_average_angle = (LKx + RKx) / 2;
//   double theta = 0;
//   angleThreshold = 100; // 80 + angleRelativeThreshold;
//   if (knee_average_angle < 0)
//   {
//     STSTorque = 0; //(unit Amp);
//   }
//   else if (knee_average_angle > angleThreshold)
//   {
//     STSTorque = 0; //(unit Amp);
//   }
//   else
//   {
//     theta = knee_average_angle / angleThreshold;
//     // larger alpha leads to steeper rising edge
//     // smaller beta leads to steeper dropping edge
//     if (LSAVx - LTAVx > 10)
//     {
//       STSTorque = 0;
//     }
//     else
//     {
//       STSTorque = pow(theta, (alpha - 1)) * pow((1 - theta), (beta - 1)) * 12.15 * 0.6 * STS_Gain; // the maximum value is 1Nm
//     }
//   }
// }

void IMU::STSTorqueCommand3()
{
  // GetData();
  // AssignLimbSegmentKinemtaics();
  // Serial.println(LTx);

  thigh_average_angle = (-LTx - RTx) / 2; // make it a positive value
  for (int ii = 0; ii <= 19; ii++)
  {
    thighXHistory[ii] = thighXHistory[ii + 1];
  }
  thighXHistory[20] = LTx;
  // slopeLTx = 0.0;

  // State transitions
  if ((STS_state == 0) & (thigh_average_angle < (STSStandThreshold + STSStandSlackAngle))) // just started the controller, default to standing position
  {
    STS_state = 1;
  }

  if ((STS_state == 4) & (thigh_average_angle < (STSStandThreshold + STSStandSlackAngle))) // from standing up to standing upright
  {
    STS_state = 1;
  }

  if ((STS_state == 1) & (thigh_average_angle >= (STSStandThreshold + STSStandSlackAngle)) & (thigh_average_angle < (STSSitThreshold - STSSitSlackAngle))) // from standing upright to sitting down
  {
    STS_state = 2;
  }

  if ((STS_state == 2) & (thigh_average_angle >= (STSSitThreshold - STSSitSlackAngle))) // from sitting down to sitting
  {
    STS_state = 3;
  }

  if ((STS_state == 3) & (thigh_average_angle >= (STSStandThreshold + STSStandSlackAngle)) & (thigh_average_angle < (STSSitThreshold - STSSitSlackAngle))) // from sitting to standing up
  {
    STS_state = 4;
  }

  // Apply control for each state
  if (STS_state == 0) // initial state, no torque
  {
    STSTorque3 = 0.0;
  }

  if (STS_state == 1) // standing
  {
    STSTorque3 = 0.0;
    STSStandPreviousMinAngle = (thigh_average_angle < STSStandPreviousMinAngle) ? thigh_average_angle : STSStandPreviousMinAngle;
    STSStandMagnitudeUpdated = 0;
  }

  if (STS_state == 2) // sitting down
  {
    // STSTorque3 = 0.0;
    // if ((STSStandMagnitudeUpdated == 0) & (STSStandPreviousMinAngle < STSStandThreshold))
    if ((STSStandMagnitudeUpdated == 0))
    {
      STSStandThreshold = (int)round(0.4 * STSStandThreshold + 0.6 * STSStandPreviousMinAngle);
      STSStandPreviousMinAngle = STSStandThreshold + STSStandSlackAngle;
      STSStandMagnitudeUpdated = 1;
    }
    theta = (thigh_average_angle - STSStandThreshold) / (STSSitThreshold - STSStandThreshold);
    thetaIndex = round(theta * 100);
    thetaIndex = (thetaIndex > 100) ? 100 : thetaIndex;
    thetaIndex = (thetaIndex < 0) ? 0 : thetaIndex;
    STSTorque3 = -Sit2StandProfileList[thetaIndex] * STSSitDownMagnitude;
  }

  if (STS_state == 3) // sitting
  {
    STSTorque3 = 0.0;
    STSSitPreviousMaxAngle = (thigh_average_angle > STSSitPreviousMaxAngle) ? thigh_average_angle : STSSitPreviousMaxAngle;
    STSSitMagnitudeUpdated = 0;
  }

  if (STS_state == 4) // standing up
  {
    // STSTorque3 = 0.0;
    // if ((STSSitMagnitudeUpdated == 0) & (STSSitThreshold < STSSitPreviousMaxAngle))
    if ((STSSitMagnitudeUpdated == 0))
    {
      STSSitThreshold = (int)round(0.4 * STSSitThreshold + 0.6 * STSSitPreviousMaxAngle);
      STSSitPreviousMaxAngle = STSSitThreshold - STSSitSlackAngle;
      STSSitMagnitudeUpdated = 1;
    }
    theta = (thigh_average_angle - STSStandThreshold) / (STSSitThreshold - STSStandThreshold);
    thetaIndex = round(theta * 100);
    thetaIndex = (thetaIndex > 100) ? 100 : thetaIndex;
    thetaIndex = (thetaIndex < 0) ? 0 : thetaIndex;
    STSTorque3 = -Sit2StandProfileList[thetaIndex] * STSStandUpMagnitude;
  }

  // Serial.print(thigh_average_angle);
  // Serial.print(" ");
  // Serial.print(STS_state);
  // // Serial.print(thigh_average_angle);
  // Serial.print(" ");
  // Serial.print(STSStandThreshold);
  // Serial.print(" ");
  // Serial.print(STSSitThreshold);
  // // Serial.print(" ");
  // // Serial.println(STSTorque3*10);
  // // Serial.print(STS_state);
  // Serial.print(" ");
  // Serial.println(STSTorque3);
  
  // if (thigh_average_angle < (STSStandThreshold + STSStandSlackAngle))
  // {
  //   STSTorque3 = 0.0;
  //   if (STS_in_sit_region == 0)
  //   {
  //     STSSitThreshold = STSSitPreviousMaxAngle;
  //     // STSSitPreviousMaxAngle = 30.0;
  //   }
  //   STS_in_stand_region = 1;
  //   STSStandPreviousMinAngle = (thigh_average_angle < STSStandPreviousMinAngle) ? thigh_average_angle : STSStandPreviousMinAngle;
  //   // if (thigh_average_angle < STSStandThreshold)
  //   // {
  //   //   STSStandPreviousMinAngle = (thigh_average_angle < STSStandPreviousMinAngle) ? thigh_average_angle : STSStandPreviousMinAngle;
  //   // }
  //   // slopeLTx = 0.0;
  //   // Serial.println(0);
  // }
  // else if (thigh_average_angle > (STSSitThreshold - STSSitSlackAngle))
  // {
  //   STSTorque3 = 0.0;
  //   STS_in_sit_region = 1;
  //   STSSitPreviousMaxAngle = (thigh_average_angle > STSSitPreviousMaxAngle) ? thigh_average_angle : STSSitPreviousMaxAngle;
  //   // if (thigh_average_angle > STSSitThreshold)
  //   // {
  //   //   STSSitPreviousMaxAngle = (thigh_average_angle > STSSitPreviousMaxAngle) ? thigh_average_angle : STSSitPreviousMaxAngle;
  //   // }
  // }
  // else
  // {
  //   // if ((thigh_average_angle >= (STSStandThreshold + STSStandSlackAngle)) & (STS_in_stand_region == 1))
  //   // {
  //   //   STS_in_stand_region = 0;
  //   //   STSStandThreshold = STSStandPreviousMinAngle;
  //   //   STSStandPreviousMinAngle = 60.0;
  //   // }
  //   if ((thigh_average_angle <= (STSSitThreshold - STSSitSlackAngle)) & (STS_in_sit_region == 1))
  //   {
  //     STS_in_sit_region = 0;
  //     // STSSitThreshold = STSSitPreviousMaxAngle;
  //     // STSSitPreviousMaxAngle = 30.0;
  //   }
  //   if ((thigh_average_angle <= (STSStandThreshold + STSStandSlackAngle)) & (STS_in_sit_region == 0))
  //   {
  //     // STS_in_sit_region = 0;
  //     // STSSitThreshold = STSSitPreviousMaxAngle;
  //     // STSSitPreviousMaxAngle = 30.0;
  //   }
  //   numeratorLTx = 3 * (1 * thighXHistory[0] + 2 * thighXHistory[10] + 3 * thighXHistory[20]) - (1 + 2 + 3) * (thighXHistory[0] + thighXHistory[10] + thighXHistory[20]);
  //   denominator = 3 * (1 + 4 + 9) - pow(1 + 2 + 3, 2);
  //   slopeLTx = numeratorLTx / denominator;
  //   theta = (knee_average_angle - STSStandThreshold) / (STSSitThreshold - STSStandThreshold);
  //   thetaIndex = round(theta * 100);
  //   if (slopeLTx >= thighXSlopeStandThreshold) // is standing
  //   {
  //     STSTorque3 = Sit2StandProfileList[thetaIndex] * STS_Gain_Stand;
  //     // Serial.println(15);
  //   }
  //   else if (slopeLTx <= thighXSlopeSitThreshold) // is sitting
  //   {
  //     STSTorque3 = Sit2StandProfileList[thetaIndex] * STS_Gain_Sit;

  //     // Serial.println(10);
  //   }
  //   else
  //   {
  //     // Serial.println(5);
  //   }

  // }
  // Serial.print(thigh_average_angle);
  // Serial.print(" ");
  // // Serial.println(STSSitThreshold);
  // Serial.print(STSSitPreviousMaxAngle);
  // Serial.print(" ");
  // Serial.print(STSSitThreshold);
  // Serial.print(" ");
  // Serial.println(STS_in_sit_region * 20);
}

void IMU::STSTorqueCommand()
{
  knee_average_angle = (LKx + RKx) / 2;
  thigh_average_angle = (LTx + RTx) / 2;
  knee_average_speed = (LSAVx - LTAVx + RSAVx - RTAVx) / 2;
  double hipX = thigh_average_angle - TKx;
  for (int ii = 0; ii <= 9; ii++)
  {
    hipXHistory[ii] = hipXHistory[ii + 1];
  }
  hipXHistory[10] = hipX;

  for (int ii = 0; ii <= 9; ii++)
  {
    kneeXHistory[ii] = kneeXHistory[ii + 1];
  }
  kneeXHistory[10] = knee_average_angle;

  // calculate slope of three points using least squares: https://www.mathsisfun.com/data/least-squares-regression.html
  // slope = (N*sum(x*y) - sum(x)*sum(y)) / (N*sum(x^2) - sum(x)^2)
  double numeratorHipX = 3 * (1 * hipXHistory[0] + 2 * hipXHistory[5] + 3 * hipXHistory[10]) - (1 + 2 + 3) * (hipXHistory[0] + hipXHistory[5] + hipXHistory[10]);
  double denominator = 3 * (1 + 4 + 9) - pow(1 + 2 + 3, 2);
  double slopeLHx = numeratorHipX / denominator;

  double numeratorKneeX = 3 * (1 * kneeXHistory[0] + 2 * kneeXHistory[5] + 3 * kneeXHistory[10]) - (1 + 2 + 3) * (kneeXHistory[0] + kneeXHistory[5] + kneeXHistory[10]);
  double slopeLKx = numeratorKneeX / denominator;

  if ((knee_average_angle >= 50) & (hipX <= -50) & !isTriggered)
  {
    isSitting = 1;
  }

  if (isSitting & (slopeLHx >= STSSlopeThreshold) & (slopeLKx <= -STSSlopeThreshold) & (knee_average_speed <= 0) & !isTriggered)
  {
    isTriggered = 1;
    isSitting = 0;
    angleThreshold = knee_average_angle + 0;
  }
  else if (isSitting & (slopeLHx <= -STSSlopeThreshold) & (slopeLKx <= -STSSlopeThreshold) & (knee_average_speed <= 0) & !isTriggered)
  {
    isTriggered = 1;
    isSitting = 0;
    angleThreshold = knee_average_angle + 0;
  }

  if (knee_average_angle <= 5)
  {
    isTriggered = 0;
  }

  if (isTriggered)
  {
    if (knee_average_angle < 0)
    {
      STSTorque = 0; //(unit Amp);
      STSTorque2 = 0;
    }
    else if (knee_average_angle > angleThreshold)
    {
      STSTorque = 0; //(unit Amp);
      STSTorque2 = 0;
    }
    else
    {
      theta = knee_average_angle / angleThreshold;
      thetaIndex = round(theta * 100);
      // STS_Gain = 0;
      // larger alpha leads to steeper rising edge
      // smaller beta leads to steeper dropping edge
      STSTorque = pow(theta, (alpha - 1)) * pow((1 - theta), (beta - 1)) * 12.15 * 0.95 * STS_Gain; // the maximum value is 1Nm
      STSTorque2 = Sit2StandProfileList[thetaIndex] * STS_Gain;
    }
  }
  else
  {
    STSTorque = 0;
    STSTorque2 = 0;
  }

  // if idx >= 11
  //       p = polyfit([1,2,3],[hipXHistory(idx-10),hipXHistory(idx-5),hipXHistory(idx)],1);
  //       slopeLHxList(idx) = p(1);
  //   end
  //   if LKx >= 50 && LHx <= -50 && ~isTriggered
  //       isSittingList(idx) = 1;
  //   end
  //   if isSittingList(idx) == 1 && slopeLHxList(idx) >= 0.5 && LKAVx <= 0 && ~isTriggered
  //       triggerList = [triggerList,idx];
  //       isTriggered = 1;
  //       isSittingList(idx) = 0;
  //       isRisingList(idx) = 1;
  //   end
  //   if LKx < 20
  //       isTriggered = 0;
  //   end
}

//******************Stair Ascending Algorithm********************//
void IMU::Stair_Ascending()
{
  y_filtered_ascending_right = (sin(RKx_filtered * PI / 180) - 5 * sin(LKx_filtered * PI / 180));
  y_filtered_ascending_left = (sin(LKx_filtered * PI / 180) - 5 * sin(RKx_filtered * PI / 180));
  y_delay_ascending_right[doi_ascending] = y_filtered_ascending_right;
  y_delay_ascending_left[doi_ascending] = y_filtered_ascending_left;

  currentpoint_ascending = doi_ascending;
  delayindex_ascending = doi_ascending - delaypoint;
  if (delayindex_ascending < 0)
  {
    delayindex_ascending = delayindex_ascending + 100;
  }
  doi_ascending++;
  doi_ascending = doi_ascending % 100;

  if (y_delay_ascending_left[doi_ascending] > 0) // left leg torque, positive = extension
  {
    DOTC_ascending[0] = 2 * Gain_E * y_delay_ascending_left[doi_ascending];
  }
  else
  {
    DOTC_ascending[0] = 0.2 * Gain_F * y_delay_ascending_left[doi_ascending];
  }

  if (y_delay_ascending_right[doi_ascending] > 0) // right leg torque, positive = extension
  {
    DOTC_ascending[1] = 2 * Gain_E * y_delay_ascending_right[doi_ascending];
  }
  else
  {
    DOTC_ascending[1] = 0.2 * Gain_F * y_delay_ascending_right[doi_ascending];
  }
}
