//#ifndef _MOTOR_CONTROL_TMOTOR_H
//#define _MOTOR_CONTROL_TMOTOR_H
//#include <FlexCAN.h>
#include <FlexCAN_T4.h>
#include <Arduino.h>
#ifndef __IMXRT1062__
#error "Teensy 3.6 with dual CAN bus is required to run this code"
#endif

//FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

class Motor_Control_Tmotor
{
  public:

    Motor_Control_Tmotor(uint8_t id, int c0);
    ~Motor_Control_Tmotor();
    void send_CAN_message();
    void enter_control_mode();
    void exit_control_mode();
    void set_origin();
    void send_cmd(float p_des, float v_des, float kp, float kd, float t_ff);
    int float_to_uint(float x, float x_min, float x_max, uint8_t nbits);
    float uint_to_float(int x_int, float x_min, float x_max, uint8_t nbits);
    void unpack_reply(CAN_message_t msgR);
    void initial_CAN();
    float pos; // rad
    float spe; // rad/s
    float torque; // Nm
    float temp;
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
  private:
    CAN_message_t msgW;
    CAN_message_t msgR;
    // struct CAN_filter_t defaultMask;
    uint8_t ID;
    float P_MIN = -12.5; //rad
    float P_MAX = 12.5; //rad
    float V_MIN = -30;  //rad/s
    float V_MAX = 30;   //rad/s
    float T_MIN = -18;  //Nm
    float T_MAX = 18;   //Nm
    float Temp_MIN = 0;  //Nm
    float Temp_MAX = 100;   //Nm
    float Kp_MIN = 0;
    float Kp_MAX = 500;
    float Kd_MIN = 0;
    float Kd_MAX = 5;
    float Test_Pos = 0.0;

};
//#endif
