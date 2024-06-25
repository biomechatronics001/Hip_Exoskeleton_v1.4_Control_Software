#include "Controller.h"

Controller::Controller()
{
  KP = 39.4784; // KP=wn^2=(2*PI)^2
  KD = 50.2655; // KD=2*zeta*wn
  ddqTd_L = 0;
  dqTd_L = 0;
  qTd_L = 0;
  ddqTd_R = 0;
  dqTd_R = 0;
  qTd_R = 0;
  Ts = 0.002; // !!!
              //  P_ahead=3;
  qTd_L_mean = 0;
  qTd_R_mean = 0;
  qTd_L_var = 25;
  qTd_R_var = 25;
  qTd_L_std = 5;
  qTd_R_std = 5;
  qTd_L_nor = 1;
  dqTd_L_nor = 0;
  qTd_R_nor = 1;
  dqTd_R_nor = 0;
  GP_IMU_L = 0;
  GP_IMU_R = 0;
  GP_IMU_L_ahead = 0;
  GP_IMU_R_ahead = 0;
}
Controller::~Controller()
{
}
void Controller::Controller_update(double TAV, double MotorAnlge_L, double MotorAnlge_R, double MotorVelocity_L, double MotorVelocity_R)
{
  qm_L = MotorAnlge_L;
  dqm_L = MotorVelocity_L;
  qm_R = MotorAnlge_R;
  dqm_R = MotorVelocity_R;
  pos_vel_estimate(TAV);
  gait_estimation();
  acc_estimation();
  Controller_caculation();
}
void Controller::pos_vel_estimate(double TAV)
{
  //  TAV_mean=(TAV_mean*filter_n+TAV)/(filter_n+1);
  //  TAV_offset=TAV-TAV_mean;
  //  TA=TA+TAV_offset*Ts;
  // use bandpass filter on thigh angular speed (0.2-3 Hz)
  // parameter a and b can be found at "\Dropbox\BiomechatronicsLab\Projects\QuasiDirectDriveActuation\Collocated Control\code\Hip Driven Gait Phase Estimation\Hip Driven Gait Phase Estimation by Angular Velocity\IMU_GP_All_Method_Compare_Final_DEC132021.m"
  TA = TA + TAV * Ts;
  // x[2]=x[1];
  // x[1]=x[0];
  // x[0]=TA;
  // y[2]=y[1];
  // y[1]=y[0];
  // y[0]=b[0]*x[0]+b[1]*x[1]+b[2]*x[2]-a[1]*y[1]-a[2]*y[2];
  x[4] = x[3];
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];
  x[0] = TA;
  y[4] = y[3];
  y[3] = y[2];
  y[2] = y[1];
  y[1] = y[0];
  y[0] = (b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3] + b[4] * x[4] - a[1] * y[1] - a[2] * y[2] - a[3] * y[3] - a[4] * y[4]) / a[0];
  TA_filter = y[0];
  // Use second order filter to compute acceleration through integration (angle and speed) to reduce noise
  // See Tairan's Collocated Control documentation Sec 1.6.2 for derivation: https://www.overleaf.com/project/61ae91b6af2cec20d24b96c1
  // wn is chosen to be the desired walking speed (1 Hz), zeta (damping ratio) is chosen to be large enough (=4) so that the filter phase delay near the desired walking speed (0.5-1 Hz) is roughly linear and thus easy to compensate
  ddqTd_L = KP * (TA_filter - qTd_L) - KD * dqTd_L;
  qTd_L = qTd_L + Ts * dqTd_L + 0.5 * Ts * Ts * ddqTd_L;
  dqTd_L = dqTd_L + Ts * ddqTd_L;
  ddqTd_R = -ddqTd_L;
  qTd_R = -qTd_L;
  dqTd_R = -dqTd_L;
}
void Controller::acc_estimation()
{
  // same as pos_vel_estimate() above
  ddqme_L = KPqm * (qm_L - qme_L) - KDqm * dqme_L;
  qme_L = qme_L + Ts * dqme_L + 0.5 * Ts * Ts * ddqme_L;
  dqme_L = dqme_L + Ts * ddqme_L;
  ddqme_R = KPqm * (qm_R - qme_R) - KDqm * dqme_R;
  qme_R = qme_R + Ts * dqme_R + 0.5 * Ts * Ts * ddqme_R;
  dqme_R = dqme_R + Ts * ddqme_R;
}
void Controller::gait_estimation()
{
  // update thigh angle and angular speed std and mean
  // See Tairan's Collocated Control documentation Sec 1.7.2 for derivation: https://www.overleaf.com/project/61ae91b6af2cec20d24b96c1
  qTd_L_mean = (qTd_L_mean * filter_n + qTd_L) / (filter_n + 1);
  qTd_R_mean = (qTd_R_mean * filter_n + qTd_R) / (filter_n + 1);
  qTd_L_var = ((qTd_L - qTd_L_mean) * (qTd_L - qTd_L_mean) + qTd_L_var * filter_n) / (filter_n + 1);
  qTd_R_var = ((qTd_R - qTd_R_mean) * (qTd_R - qTd_R_mean) + qTd_R_var * filter_n) / (filter_n + 1);
  qTd_L_std = sqrt(qTd_L_var);
  qTd_R_std = sqrt(qTd_R_var);
  dqTd_L_var = (dqTd_L * dqTd_L + dqTd_L_var * filter_n) / (filter_n + 1);
  dqTd_R_var = (dqTd_R * dqTd_R + dqTd_R_var * filter_n) / (filter_n + 1);
  dqTd_L_std = sqrt(dqTd_L_var);
  dqTd_R_std = sqrt(dqTd_R_var);
  if (qTd_L_std < 1)
  {
    qTd_L_std = 1;
  }
  if (qTd_R_std < 1)
  {
    qTd_R_std = 1;
  }
  // angle = A*sin(w*t)+b, angular speed = w*A*cos(w*t)
  // In order to form a circular limit cycle, angular speed needs to be normalized by w, which is approximated by the ratio of std of angle and angular speed
  // Howard said the normalization by max-min method in Gregg's paper is difficult to implement
  w = dqTd_L_std / qTd_L_std;
  // Set lower bound on w so that the phase trajectory falls into the small "STOP" circle
  // See Gregg's paper for detail: "E:\Dropbox\BiomechatronicsLab\Projects\QuasiDirectDriveActuation\Collocated Control\code\Hip Driven Gait Phase Estimation\Hip Driven Gait Phase Estimation by Angular Velocity\Gait_Estimation.pdf"
  if (w < 1)
  {
    w = 1;
  }
  qTd_L_nor = qTd_L - qTd_L_mean;
  dqTd_L_nor = (dqTd_L) / w;
  qTd_R_nor = qTd_R - qTd_R_mean;
  dqTd_R_nor = (dqTd_R) / w;
  // convert to phase angle to gait phase percentage. -90 is a constant offset such that zero phase angle roughly corresponds to zero gait phase percentage
  GP_IMU_L = (atan2(-dqTd_L_nor, qTd_L_nor) * 180 / PI) - 140;// + (20.0 / 100.0 * 360.0);

  if (GP_IMU_L < 0)
  {
    GP_IMU_L = GP_IMU_L + 360;
  }
  else if (GP_IMU_L > 360)
  {
    GP_IMU_L = GP_IMU_L - 360;
  }
  GP_IMU_L = 100 * GP_IMU_L / 360;
  // GP_IMU_L = GP_IMU_L - 20;
  // Serial.println(GP_IMU_L);

  GP_IMU_R = (atan2(-dqTd_R_nor, qTd_R_nor) * 180 / PI) - 140;// + (20.0 / 100.0 * 360.0);
  if (GP_IMU_R < 0)
  {
    GP_IMU_R = GP_IMU_R + 360;
  }
  else if (GP_IMU_R > 360)
  {
    GP_IMU_R = GP_IMU_R - 360;
  }
  GP_IMU_R = 100 * GP_IMU_R / 360;
  // GP_IMU_R = GP_IMU_R - 20;

  mag = sqrt(dqTd_L_nor * dqTd_L_nor + qTd_L_nor * qTd_L_nor);
  delta_GP_IMU_L = GP_IMU_L - GP_IMU_L_Pre;
  if (delta_GP_IMU_L < -90)
  {
    delta_GP_IMU_L = delta_GP_IMU_L + 100;
  }
  // stop walking assistance when estimated walking speed is lower than a threshold
  if (mag < mag_close && state == 1)
  {
    state = 0;
  }
  // resume walking assistance after estimated walking speed exceeds a threshold
  else if (mag > mag_close * 1.5 && state == 0 && delta_GP_IMU_L < 1 && delta_GP_IMU_L > 0)
  {
    state = 1;
  }
  if (state == 0)
  {
    GP_IMU_L = GP_IMU_L_Pre;
    GP_IMU_R = GP_IMU_R_Pre;
  }
  else if (delta_GP_IMU_L < 0 && state == 1)
  {
    GP_IMU_L = GP_IMU_L_Pre;
    GP_IMU_R = GP_IMU_R_Pre;
  }
  else
  {
    GP_IMU_L_Pre = GP_IMU_L;
    GP_IMU_R_Pre = GP_IMU_R;
  }

  GP_IMU_L_ahead = GP_IMU_L + P_ahead;
  GP_IMU_L_ahead_imp = GP_IMU_L + P_ahead_imp;
  GP_IMU_L_ahead_ff = GP_IMU_L + P_ahead_ff;
  if (GP_IMU_L_ahead > 100)
  {
    GP_IMU_L_ahead = GP_IMU_L_ahead - 100;
  }
  else if (GP_IMU_L_ahead < 0)
  {
    GP_IMU_L_ahead = GP_IMU_L_ahead + 100;
  }
  // impedance
  if (GP_IMU_L_ahead_imp > 100)
  {
    GP_IMU_L_ahead_imp = GP_IMU_L_ahead_imp - 100;
  }
  else if (GP_IMU_L_ahead_imp < 0)
  {
    GP_IMU_L_ahead_imp = GP_IMU_L_ahead_imp + 100;
  }
  // feedforward
  if (GP_IMU_L_ahead_ff > 100)
  {
    GP_IMU_L_ahead_ff = GP_IMU_L_ahead_ff - 100;
  }
  else if (GP_IMU_L_ahead_ff < 0)
  {
    GP_IMU_L_ahead_ff = GP_IMU_L_ahead_ff + 100;
  }

  GP_IMU_R_ahead = GP_IMU_R + P_ahead;
  GP_IMU_R_ahead_imp = GP_IMU_R + P_ahead_imp;
  GP_IMU_R_ahead_ff = GP_IMU_R + P_ahead_ff;
  if (GP_IMU_R_ahead > 100)
  {
    GP_IMU_R_ahead = GP_IMU_R_ahead - 100;
  }
  else if (GP_IMU_R_ahead < 0)
  {
    GP_IMU_R_ahead = GP_IMU_R_ahead + 100;
  }
  // impedance
  if (GP_IMU_R_ahead_imp > 100)
  {
    GP_IMU_R_ahead_imp = GP_IMU_R_ahead_imp - 100;
  }
  else if (GP_IMU_R_ahead_imp < 0)
  {
    GP_IMU_R_ahead_imp = GP_IMU_R_ahead_imp + 100;
  }
  // feedforward
  if (GP_IMU_R_ahead_ff > 100)
  {
    GP_IMU_R_ahead_ff = GP_IMU_R_ahead_ff - 100;
  }
  else if (GP_IMU_R_ahead_ff < 0)
  {
    GP_IMU_R_ahead_ff = GP_IMU_R_ahead_ff + 100;
  }
}
void Controller::Controller_caculation()
{
  index_cur_L = round(GP_IMU_L_ahead * 10);
  index_cur_R = round(GP_IMU_R_ahead * 10);
  index_cur_imp_L = round(GP_IMU_L_ahead_imp * 10);
  index_cur_imp_R = round(GP_IMU_R_ahead_imp * 10);
  index_cur_ff_L = round(GP_IMU_L_ahead_ff * 10);
  index_cur_ff_R = round(GP_IMU_R_ahead_ff * 10);
  // original
  // qhr_L=qhr_arr[index_cur_L];
  // qhr_R=qhr_arr[index_cur_R];
  // Tauhr_L=Tauhr_arr[index_cur_L];
  // Tauhr_R=Tauhr_arr[index_cur_R];
  // kd_L=kd_arr[index_cur_L];
  // bd_L=bd_arr[index_cur_L];
  // kd_R=kd_arr[index_cur_R];
  // bd_R=bd_arr[index_cur_R];
  // dqhr_L=dqhr_arr[index_cur_L];
  // dqhr_R=dqhr_arr[index_cur_R];
  // modified
  qhr_L = qhr_arr[index_cur_imp_L];
  qhr_R = qhr_arr[index_cur_imp_R];
  Tauhr_L = Tauhr_arr[index_cur_ff_L];
  Tauhr_R = Tauhr_arr[index_cur_ff_R];
  kd_L = kd_arr[index_cur_imp_L];
  kd_L_Junxi = kd_arr_Junxi[index_cur_imp_L];
  bd_L = bd_arr[index_cur_imp_L];
  bd_L_Junxi = bd_arr_Junxi[index_cur_imp_L];
  kd_R = kd_arr[index_cur_imp_R];
  kd_R_Junxi = kd_arr_Junxi[index_cur_imp_R];
  bd_R = bd_arr[index_cur_imp_R];
  bd_R_Junxi = bd_arr_Junxi[index_cur_imp_R];
  sigmoid_L = sigmoid[index_cur_ff_L];
  sigmoid_R = sigmoid[index_cur_ff_R];
  dqhr_L = dqhr_arr[index_cur_imp_L];
  dqhr_R = dqhr_arr[index_cur_imp_R];

  //
  k_col_imp_L = (1 / n) * (ktrans / (ktrans - kd_L));
  k_col_imp_R = (1 / n) * (ktrans / (ktrans - kd_R));
  // bd_L*dqm_L/n is excluded because it contains large noise
  // motor dynamics is ignored here (Jm=bm=0) to ensure smoothness of the assistance torque profile
  //  T_command_L=(kd_L*(qhr_L-(qm_L/n))+bd_L*(dqhr_L))*k_col_imp_L+Jm*ddqhr_L+bm*dqhr_L; //-dqme_L/n
  //  T_command_R=(kd_R*(qhr_R-(qm_R/n))+bd_R*(dqhr_R))*k_col_imp_R+Jm*ddqhr_R+bm*dqhr_R; //-dqme_R/n
  kd_torque_L = kd_L_Junxi * (qhr_L - (qm_L / n)) * k_col_imp_L;                                                        //-dqme_L/n
  kd_torque_R = kd_R_Junxi * (qhr_R - (qm_R / n)) * k_col_imp_R;                                                        //-dqme_R/n
  bd_torque_L = bd_L_Junxi * (dqhr_L)*k_col_imp_L + Jm * ddqhr_L + bm * dqhr_L;                                         //-dqme_L/n
  bd_torque_R = bd_R_Junxi * (dqhr_R)*k_col_imp_R + Jm * ddqhr_R + bm * dqhr_R;                                         //-dqme_R/n
  T_command_L = (kd_L_Junxi * (qhr_L - (qm_L / n)) + bd_L_Junxi * (dqhr_L)) * k_col_imp_L + Jm * ddqhr_L + bm * dqhr_L; //-dqme_L/n
  T_command_R = (kd_R_Junxi * (qhr_R - (qm_R / n)) + bd_R_Junxi * (dqhr_R)) * k_col_imp_R + Jm * ddqhr_R + bm * dqhr_R; //-dqme_R/n
  // impedance portion essentially tracks a reference position trajectory (from able-bodied subject dataset)
  // for testing on able-bodied subject, this parts is close to zero because the actual position trajectory is close to the reference. To actually make the able-bodied subject feel the assistance, a biological torque profile is added through feedforward term below
  impedanceTorque_L = ControllerGain_L * T_command_L;
  impedanceTorque_R = ControllerGain_L * T_command_R;
  // feedforward moment is generated by "E:\Dropbox\BiomechatronicsLab\Projects\Knee Exoskeleton Superhuman\Code\20211123 Impedance collocated control - for Howard\1. Teensy code\feedforwardMoment.m"
  feedforwardTorque_L = FeedForwardGain_L * Weight * Tauhr_L * sigmoid_L;
  feedforwardTorque_R = FeedForwardGain_R * Weight * Tauhr_R * sigmoid_R;
  T_command_L = impedanceTorque_L + feedforwardTorque_L;
  T_command_R = impedanceTorque_R + feedforwardTorque_R;

  if (state == 1)
  {
    // the torque T_command_L here is the motor torque. Actuator torque = motor torque * gear ratio
    // the current command Cur_command_L also is the motor current
    Cur_command_L = T_command_L / kt;
    Cur_command_R = T_command_R / kt;
  }
  else
  {
    Cur_command_L = 0;
    Cur_command_R = 0;
    initialStepCounter = 0;
  }

  //
}
void Controller::set_P_ahead(double leading)
{
  P_ahead = leading;
}

void Controller::calculateSigmoid(double maxValue, double minValue, double *sigmoid_array)
{
  memset(sigmoid_array, 0, sizeof(sigmoid_array));
  memset(sigmoid_array_temp, 0, sizeof(sigmoid_array_temp));
  for (int i = 0; i < stanceLength; i++)
  {
    sigmoid_array_temp[i] = 1;
  }
  for (int i = strideLength; i < strideLength + stanceLength; i++)
  {
    sigmoid_array_temp[i] = 1;
  }
  for (int i = 0; i < strideLength * 2; i++)
  {
    int idx = i % strideLength;
    int idx1 = abs(idx - midSwingLoc);
    int idx2 = abs(idx - midStanceLoc);
    if (idx1 >= idx2)
    {
      double percentage = idx2 / (strideLength * 1.0 / 2);
      idxNormalized = -6 + percentage * 12;
    }
    else
    {
      double percentage = idx1 / (strideLength * 1.0 / 2);
      idxNormalized = 6 - percentage * 12;
    }
    sigmoid_array_temp[i] = 1.0 / (1.0 + exp(sigmoidShapeFactor * idxNormalized));
  }
  for (int i = 0; i < strideLength; i++)
  {
    sigmoid_array[i] = sigmoid_array_temp[i] * (maxValue - minValue) + minValue;
  }
}

void Controller::computeGaitPhase(double TAV, double MotorAnlge_L, double MotorAnlge_R, double MotorVelocity_L, double MotorVelocity_R)
{
  qm_L = MotorAnlge_L;
  dqm_L = MotorVelocity_L;
  qm_R = MotorAnlge_R;
  dqm_R = MotorVelocity_R;
  pos_vel_estimate(TAV);
  gait_estimation();
  acc_estimation();
}

void Controller::addSineAssistanceProfile(int sineAssistanceTypeIdx, double sineAssistanceMagnitude, int sineAssistanceShift, int sineAssistanceDuration, double sineAssistanceSaturation)
{
  if (sineAssistanceTypeIdx == 1) // Flexion
  {
    sineAssistanceFlexionParameterList[sineAssistanceFlexionProfileCounter][0] = sineAssistanceMagnitude;
    sineAssistanceFlexionParameterList[sineAssistanceFlexionProfileCounter][1] = sineAssistanceShift;
    sineAssistanceFlexionParameterList[sineAssistanceFlexionProfileCounter][2] = sineAssistanceDuration;
    sineAssistanceFlexionParameterList[sineAssistanceFlexionProfileCounter][3] = sineAssistanceSaturation;

    for (int i = 0; i < sineAssistanceDuration + 1; i++)
    {
      if (sineAssistanceShift + i < 100)
      {
        sineAssistanceFlexionProfileList[sineAssistanceFlexionProfileCounter][sineAssistanceShift + i] = -min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
      else
      {
        sineAssistanceFlexionProfileList[sineAssistanceFlexionProfileCounter][sineAssistanceShift + i - 100] = -min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
    }
    sineAssistanceFlexionProfileCounter += 1;
  }
  else if (sineAssistanceTypeIdx == 2) // Extension
  {
    sineAssistanceExtensionParameterList[sineAssistanceExtensionProfileCounter][0] = sineAssistanceMagnitude;
    sineAssistanceExtensionParameterList[sineAssistanceExtensionProfileCounter][1] = sineAssistanceShift;
    sineAssistanceExtensionParameterList[sineAssistanceExtensionProfileCounter][2] = sineAssistanceDuration;
    sineAssistanceExtensionParameterList[sineAssistanceExtensionProfileCounter][3] = sineAssistanceSaturation;

    for (int i = 0; i < sineAssistanceDuration + 1; i++)
    {
      if (sineAssistanceShift + i < 100)
      {
        sineAssistanceExtensionProfileList[sineAssistanceExtensionProfileCounter][sineAssistanceShift + i] = min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
      else
      {
        sineAssistanceExtensionProfileList[sineAssistanceExtensionProfileCounter][sineAssistanceShift + i - 100] = min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
    }

    sineAssistanceExtensionProfileCounter += 1;
  }
}

void Controller::removeSineAssistanceProfile(int sineAssistanceTypeIdx, int sineAssistanceProfileIdx)
{
  if (sineAssistanceTypeIdx == 1)
  {
    for (int i = sineAssistanceProfileIdx + 1; i < sineAssistanceFlexionProfileCounter; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        sineAssistanceFlexionParameterList[i - 1][j] = sineAssistanceFlexionParameterList[i][j];
      }
      for (int j = 0; j < 100; j++)
      {
        sineAssistanceFlexionProfileList[i - 1][j] = sineAssistanceFlexionProfileList[i][j];
      }
    }
    // Serial.println(sineAssistanceFlexionProfileCounter);
    for (int i = 0; i < 4; i++)
    {
      sineAssistanceFlexionParameterList[sineAssistanceFlexionProfileCounter - 1][i] = 0.0;
    }
    for (int i = 0; i < 100; i++)
    {
      sineAssistanceFlexionProfileList[sineAssistanceFlexionProfileCounter - 1][i] = 0.0;
    }
    sineAssistanceFlexionProfileCounter -= 1;
  }
  else if (sineAssistanceTypeIdx == 2)
  {
    for (int i = sineAssistanceProfileIdx + 1; i < sineAssistanceExtensionProfileCounter; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        sineAssistanceExtensionParameterList[i - 1][j] = sineAssistanceExtensionParameterList[i][j];
      }
      for (int j = 0; j < 100; j++)
      {
        sineAssistanceExtensionProfileList[i - 1][j] = sineAssistanceExtensionProfileList[i][j];
      }
    }
    for (int i = 0; i < 4; i++)
    {
      sineAssistanceExtensionParameterList[sineAssistanceExtensionProfileCounter - 1][i] = 0.0;
    }
    for (int i = 0; i < 100; i++)
    {
      sineAssistanceExtensionProfileList[sineAssistanceExtensionProfileCounter - 1][i] = 0.0;
    }
    sineAssistanceExtensionProfileCounter -= 1;
  }
  computeSineAssistanceTorque();
}

void Controller::editSineAssistanceProfile(int sineAssistanceTypeIdx, int sineAssistanceProfileIdx, double sineAssistanceMagnitude, int sineAssistanceShift, int sineAssistanceDuration, double sineAssistanceSaturation)
{
  if (sineAssistanceTypeIdx == 1) // Flexion
  {
    sineAssistanceFlexionParameterList[sineAssistanceProfileIdx][0] = sineAssistanceMagnitude;
    sineAssistanceFlexionParameterList[sineAssistanceProfileIdx][1] = sineAssistanceShift;
    sineAssistanceFlexionParameterList[sineAssistanceProfileIdx][2] = sineAssistanceDuration;
    sineAssistanceFlexionParameterList[sineAssistanceProfileIdx][3] = sineAssistanceSaturation;

    for (int i = 0; i < sineAssistanceDuration + 1; i++)
    {
      if (sineAssistanceShift + i < 100)
      {
        sineAssistanceFlexionProfileList[sineAssistanceProfileIdx][sineAssistanceShift + i] = -min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
      else
      {
        sineAssistanceFlexionProfileList[sineAssistanceProfileIdx][sineAssistanceShift + i - 100] = -min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
    }
  }
  else if (sineAssistanceTypeIdx == 2) // Extension
  {
    sineAssistanceExtensionParameterList[sineAssistanceProfileIdx][0] = sineAssistanceMagnitude;
    sineAssistanceExtensionParameterList[sineAssistanceProfileIdx][1] = sineAssistanceShift;
    sineAssistanceExtensionParameterList[sineAssistanceProfileIdx][2] = sineAssistanceDuration;
    sineAssistanceExtensionParameterList[sineAssistanceProfileIdx][3] = sineAssistanceSaturation;

    for (int i = 0; i < sineAssistanceDuration + 1; i++)
    {
      if (sineAssistanceShift + i < 100)
      {
        sineAssistanceExtensionProfileList[sineAssistanceProfileIdx][sineAssistanceShift + i] = min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
      else
      {
        sineAssistanceExtensionProfileList[sineAssistanceProfileIdx][sineAssistanceShift + i - 100] = min(sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i), sineAssistanceSaturation / 1.0);
      }
    }
  }
}

void Controller::computeSineAssistanceTorque()
{
  // double totalTorque = 0.0;
  sineAssistanceTotalTorqueLeft = 0;
  sineAssistanceTotalTorqueRight = 0;
  for (int i = 0; i < sineAssistanceFlexionProfileCounter; i++)
  {
    sineAssistanceMagnitude = sineAssistanceFlexionParameterList[i][0];
    sineAssistanceShift = (int)sineAssistanceFlexionParameterList[i][1];
    sineAssistanceDuration = (int)sineAssistanceFlexionParameterList[i][2];
    sineAssistanceGaitPhaseLeft = round(GP_IMU_L / 100.0 * 99.0);
    sineAssistanceTotalTorqueLeft += sineAssistanceFlexionProfileList[i][sineAssistanceGaitPhaseLeft];
  }
  for (int i = 0; i < sineAssistanceExtensionProfileCounter; i++)
  {
    sineAssistanceMagnitude = sineAssistanceExtensionParameterList[i][0];
    sineAssistanceShift = (int)sineAssistanceExtensionParameterList[i][1];
    sineAssistanceDuration = (int)sineAssistanceExtensionParameterList[i][2];
    sineAssistanceGaitPhaseLeft = round(GP_IMU_L / 100.0 * 99.0);
    sineAssistanceTotalTorqueLeft += sineAssistanceExtensionProfileList[i][sineAssistanceGaitPhaseLeft];
  }

  for (int i = 0; i < sineAssistanceFlexionProfileCounter; i++)
  {
    sineAssistanceMagnitude = sineAssistanceFlexionParameterList[i][0];
    sineAssistanceShift = (int)sineAssistanceFlexionParameterList[i][1];
    sineAssistanceDuration = (int)sineAssistanceFlexionParameterList[i][2];
    sineAssistanceGaitPhaseRight = round(GP_IMU_R / 100.0 * 99.0);
    sineAssistanceTotalTorqueRight += sineAssistanceFlexionProfileList[i][sineAssistanceGaitPhaseRight];
  }
  for (int i = 0; i < sineAssistanceExtensionProfileCounter; i++)
  {
    sineAssistanceMagnitude = sineAssistanceExtensionParameterList[i][0];
    sineAssistanceShift = (int)sineAssistanceExtensionParameterList[i][1];
    sineAssistanceDuration = (int)sineAssistanceExtensionParameterList[i][2];
    sineAssistanceGaitPhaseRight = round(GP_IMU_R / 100.0 * 99.0);
    sineAssistanceTotalTorqueRight += sineAssistanceExtensionProfileList[i][sineAssistanceGaitPhaseRight];
  }
  if (state == 1)
  {
    // Cur_command_L = sineAssistanceTotalTorqueLeft / kt;
    // Cur_command_R = sineAssistanceTotalTorqueRight / kt;
  }
  else
  {
    sineAssistanceTotalTorqueLeft = 0;
    sineAssistanceTotalTorqueRight = 0;
  }
}

void Controller::updateSineAssistanceProfile(double sineAssistanceMagnitude, int sineAssistanceShift, int sineAssistanceDuration)
{
  for (int i = 0; i < 100 + 1; i++)
  {
    sineAssistanceProfileList[i] = 0.0;
  }
  for (int i = 0; i < sineAssistanceDuration + 1; i++)
  {
    if (sineAssistanceShift + i < 100)
    {
      sineAssistanceProfileList[sineAssistanceShift + i] = sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i);
    }
    else
    {
      sineAssistanceProfileList[sineAssistanceShift + i - 100] = sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * i);
    }
  }
  // sineAssistanceGaitPhaseLeft = round(GP_IMU_L / 100.0 * 99.0);
  // if ((sineAssistanceGaitPhaseLeft >= sineAssistanceShift) | (sineAssistanceGaitPhaseLeft <= sineAssistanceShift + sineAssistanceDuration))
  // {
  //   sineAssistanceTotalTorqueLeft = sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * (sineAssistanceGaitPhaseLeft - sineAssistanceShift));
  // }
  // else
  // {
  //   sineAssistanceTotalTorqueLeft = 0.0;
  // }
  
  // sineAssistanceGaitPhaseRight = round(GP_IMU_R / 100.0 * 99.0);
  // if ((sineAssistanceGaitPhaseRight >= sineAssistanceShift) | (sineAssistanceGaitPhaseRight <= sineAssistanceShift + sineAssistanceDuration))
  // {
  //   sineAssistanceTotalTorqueRight = sineAssistanceMagnitude / 1.0 * sin(3.1415926 / sineAssistanceDuration * (sineAssistanceGaitPhaseRight - sineAssistanceShift));
  // }
  // else
  // {
  //   sineAssistanceTotalTorqueRight = 0.0;
  // }
}

void Controller::computeSingleSineAssistanceProfile()
{
  sineAssistanceGaitPhaseLeft = round(GP_IMU_L / 100.0 * 99.0);
  sineAssistanceTotalTorqueLeft = sineAssistanceProfileList[sineAssistanceGaitPhaseLeft];
  sineAssistanceGaitPhaseRight = round(GP_IMU_R / 100.0 * 99.0);
  sineAssistanceTotalTorqueRight = sineAssistanceProfileList[sineAssistanceGaitPhaseRight];
}
