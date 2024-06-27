//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor).
#include "Serial_Com.h"
#include "Wireless_IMU.h"
#include <Arduino.h>
#include "MovingAverage.h"
/*MOTOR*/
#include <FlexCAN_T4.h>
#include "Motor_Control_Tmotor.h"
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

/*Filter*/
MovingAverage LTAVx(12);
MovingAverage RTAVx(12);
float f_LTAVx = 0;
float f_RTAVx = 0;

CAN_message_t msgR;

int Motor_ID = 1;
int Motor_ID2 = 2;
int CAN_ID = 3; // Teensy 4.1 CAN bus port

double torque_command = 0;
double velocity_command = 0;
double position_command = 0;

double M1_torque_command = 0;
double M2_torque_command = 0;

int LimitInf = -18;
int LimitSup = 18;

float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff = 0;

Motor_Control_Tmotor m1(Motor_ID, CAN_ID);
Motor_Control_Tmotor m2(Motor_ID2, CAN_ID);
/*MOTOR*/

/*Isra Serial Class Setup*/
Serial_Com Serial_Com;

/*Sensors Setup*/
IMU imu;

/*Serial Send*/
size_t Send_Length = 11;
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;
uint16_t R_IMUX_int = 0x00;

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00;

uint16_t L_CMD_int16 = 0x7fff;
float L_CMD_serial = 0.0;

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial = 0.0;

float IMUX_float = 0;
float IMU11 = 0;
float IMU22 = 0;
float IMU33 = 0;
float IMU44 = 0;

/*Controller variables*/
double t;
int doi          = 0;
int currentpoint = 0;
int delayindex   = 0;
// double Rescaling_gain    = 0.01; // 1.6 for max 4 Nm
double LTx_filtered      = 0;
double LTx_filtered_last = 0;
double RTx_filtered      = 0;
double RTx_filtered_last = 0;
double torque_filtered   = 0;
double RLTx              = 0;
double RLTx_filtered     = 0;
double RLTx_delay[100]   = {};
double torque_delay[100] = {};


//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;  // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************


//***Data sent via bluetooth
char datalength_ble    = 32;   // Bluetooth Data Length (32)
char data_ble[60]      = {0};  // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int t_teensy = 0;
int L_leg_IMU_angle = 0;
int R_leg_IMU_angle = 0;
int L_motor_torque  = 0;
int R_motor_torque  = 0;
int L_motor_torque_command = 0;
int R_motor_torque_command = 0;
double Rescaling_gain    = 0;
double Flex_Assist_gain  = 0;
double Ext_Assist_gain   = 0;
double Assist_delay_gain = 0;
//**************************


void setup() {
  delay(3000);
  Serial.begin(115200);   //115200/9600=12
  Serial2.begin(115200);  //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  Serial_Com.INIT();
  //#################
  Serial.println("SETUP DONE");
  Serial.print("Controller executed at ");
  Serial.print(String(cyclespersec_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(cyclespersec_ble));
  Serial.println(" Hz");
  //####################
  initial_CAN();
  initial_M1();
  initial_M2();
  delay(100);
  IMUSetup();
  t_0 = micros();
}


void loop() {

  imu.READ();
  Serial_Com.READ2();
  
  current_time = micros() - t_0;
  t = current_time / 1000000.0;

  if (current_time - previous_time > Tinterval_ctrl_micros) {
    
    if (current_time - previous_time_ble > Tinterval_ble_micros) {

      Receive_ble_Data();
      Transmit_ble_Data(); // send the BLE data

      previous_time_ble = current_time;
    }

    //fakeIMU();
    RealIMU();

    //##### Samsung Controller #####

    RLTx = imu.RTx - imu.LTx; // right-left thigh angle difference
    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = ((0.95 * LTx_filtered_last) + (0.05 * imu.LTx));
    RTx_filtered_last = RTx_filtered;
    RTx_filtered      = ((0.95 * RTx_filtered_last) + (0.05 * imu.RTx));
    RLTx_filtered     = RTx_filtered - LTx_filtered;
    RLTx_delay[doi]   = RLTx_filtered;
    torque_filtered   = (sin(RTx_filtered * PI / 180) - sin(LTx_filtered * PI / 180));
    torque_delay[doi] = torque_filtered;

    currentpoint = doi;
    delayindex   = doi - Assist_delay_gain;

    if (delayindex < 0) {
      delayindex = delayindex + 100;
    }
    else if (delayindex >= 100) {
      delayindex = delayindex - 100;
    }

    doi++;
    doi = doi % 100;
    
    if ((RLTx_delay[delayindex] >= 0) && (RLTx_delay[delayindex] < 120)) {
      M1_torque_command = - Rescaling_gain * Ext_Assist_gain * torque_delay[delayindex]; // left hip torque
      M2_torque_command = Rescaling_gain * Flex_Assist_gain * torque_delay[delayindex];  // right hip torque
    }
    else if ((RLTx_delay[delayindex] < 0) && (RLTx_delay[delayindex] > -120)) {
      M2_torque_command = Rescaling_gain * Ext_Assist_gain * torque_delay[delayindex];  // right hip torque
      M1_torque_command = -Rescaling_gain * Flex_Assist_gain * torque_delay[delayindex]; // left hip torque
    }
    else {
      // M2_torque_command=0;
      // M1_torque_command=0;
    }

    int max_allowed_torque = 30; // Safety measurement to limit the commanded torque

    if (abs(M1_torque_command) > max_allowed_torque || abs(M2_torque_command) > max_allowed_torque) {
      M1_torque_command = 0;
      M2_torque_command = 0;
      Serial.println("ERROR 1");
      Serial.println("You have exeded the maximum allowed torque.");
      Serial.println("Yoo must adjust the gains of the controller and redo the whole experiment.");
      RealIMU_Reset();
      Serial_Com.WRITE(Send, Send_Length);
      Wait(10000);
      delay(1000);
    }    

    M1_Torque_Control_Example();
    Wait(1100);
    M2_Torque_Control_Example();
    Wait(1100);

    // Print_Data_Basic();

    previous_time = current_time;
  }
}


void Print_Data_Basic() {
  Serial.print(t);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.RTx);
  Serial.print(" ");
  Serial.print(m1.torque);
  Serial.print(" ");
  Serial.print(m1.torque);
  Serial.println(" ");
}


void print_Data_IMU() {
  Serial.print(-180);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  //  Serial.print(IMU22);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.LTAVx);
  Serial.print(" ");
  Serial.print(imu.RTx);
  Serial.print(" ");
  Serial.print(imu.RTAVx);
  Serial.println(" ");
}


void print_Data_Received() {

  Serial.print(20);
  Serial.print(" ");
  Serial.print(-20);
  Serial.print(" ");
  Serial.print(L_CMD_serial);
  Serial.print(" ");
  Serial.print(R_CMD_serial);
  Serial.print(" ");
  Serial.println(" ");
}


void print_data_motor() {
  //  double v1 = 90;
  //  double v2 = -v1;
  //  Serial.print(v1);
  //  Serial.print("   ");
  //Serial.print(current_time);
  Serial.print(" ; ");
  Serial.print(" M1_tor ; "); //M1 is left, M2 is right
  Serial.print(m1.torque);
  Serial.print(" ; M1_cmd ; ");
  Serial.print(M1_torque_command);
  Serial.print(" ; M2_tor ; ");
  Serial.print(m2.torque);
  Serial.print(" ; M2_cmd ; ");
  Serial.print(M2_torque_command);
  Serial.print(" ; M1_pos ; ");
  Serial.print(m1.pos);
  Serial.println(" ;  ");
}


void IMUSetup() {
  imu.INIT();
  delay(500);
  imu.INIT_MEAN();
}


void M1_Torque_Control_Example() {
  p_des = 0;  //dont change this
  v_des = 0;  //dont change this
  kp = 0;     //dont change this
  kd = 0;     //dont change this
  t_ff = M1_torque_command;
  m1.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}


void M2_Torque_Control_Example() {
  p_des = 0;  //dont change this
  v_des = 0;  //dont change this
  kp = 0;     //dont change this
  kd = 0;     //dont change this
  t_ff = M2_torque_command;
  m2.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}


void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}


void initial_M1() {
  //m1.initial_CAN();
  m1.exit_control_mode();
  delay(200);
  m1.exit_control_mode();
  delay(1000);
  m1.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  position_command = 0;
  M1_Position_Control_Example();
  Serial.println("M1 Done");
  delay(100);
}


void initial_M2() {
  //m2.initial_CAN();
  m2.exit_control_mode();
  delay(200);
  m2.exit_control_mode();
  delay(1000);
  m2.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m2.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  position_command = 0;
  M2_Position_Control_Example();
  Serial.println("M2 Done");
  delay(100);
}


void receive_CAN_data() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);
    int id = msgR.buf[0];
    //Serial.print(msgR.id, HEX );
    if (id == Motor_ID) {
      m1.unpack_reply(msgR);
    }
    if (id == Motor_ID2) {
      m2.unpack_reply(msgR);
    }
  }
}


void fakeIMU() {
  IMU11 = 150.0 * sin(t / 5.0);
  IMU22 = 150.0 * cos(t / 5.0);
  IMU33 = 700.0 * sin(t / 5.0);
  IMU44 = 700.0 * cos(t / 5.0);

  L_IMUX_int = Serial_Com.float_to_uint(IMU11, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(IMU22, -180, 180, 16);

  L_IMUV_int = Serial_Com.float_to_uint(IMU33, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(IMU44, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}


void RealIMU() {
  f_LTAVx = LTAVx.addSample(imu.LTAVx);
  f_RTAVx = RTAVx.addSample(imu.RTAVx);

  L_IMUX_int = Serial_Com.float_to_uint(imu.LTx, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(imu.RTx, -180, 180, 16);

  L_IMUV_int = Serial_Com.float_to_uint(f_LTAVx, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(f_RTAVx, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}


void RealIMU_Reset() {
  float reset_imu = 0;

  L_IMUX_int = Serial_Com.float_to_uint(reset_imu, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(reset_imu, -180, 180, 16);

  L_IMUV_int = Serial_Com.float_to_uint(reset_imu, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(reset_imu, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}


void M1_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  m1.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}


void M2_Position_Control_Example() {
  position_command = 0;
  p_des = position_command * PI / 180;
  v_des = 0;  //dont change this
  kp = 30;    //max 450 min 0
  kd = 1.5;   //max 5 min 0
  t_ff = 0;   //dont change this
  m2.send_cmd(p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
}


void Wait(unsigned long delay_control) {
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  } while (Time_Control < Time_Delta);
}


void Receive_ble_Data(){
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          Rescaling_gain   = ((int16_t)(data_rs232_rx[3] | (data_rs232_rx[4] << 8))) / 100.0;
          Flex_Assist_gain = ((int16_t)(data_rs232_rx[5] | (data_rs232_rx[6] << 8))) / 100.0;
          Ext_Assist_gain  = ((int16_t)(data_rs232_rx[7] | (data_rs232_rx[8] << 8))) / 100.0;
          Assist_delay_gain = data_rs232_rx[9];

          Serial.print(" | ");
          Serial.print(Rescaling_gain);
          Serial.print(" | ");
          Serial.print(Flex_Assist_gain);
          Serial.print(" | ");
          Serial.print(Ext_Assist_gain);
          Serial.print(" | ");
          Serial.print(Assist_delay_gain);
          Serial.println(" | ");
        }
      }
    }
  }
}


void Transmit_ble_Data() {
  t_teensy        = t * 100;
  L_leg_IMU_angle = imu.LTx * 100;
  R_leg_IMU_angle = imu.RTx * 100;
  L_motor_torque  = m1.torque * 100;
  R_motor_torque  = m2.torque * 100;
  L_motor_torque_command = M1_torque_command *100;
  R_motor_torque_command = M2_torque_command *100;

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = L_leg_IMU_angle;
  data_ble[6]  = L_leg_IMU_angle >> 8;
  data_ble[7]  = R_leg_IMU_angle;
  data_ble[8]  = R_leg_IMU_angle >> 8;
  data_ble[9]  = L_motor_torque;
  data_ble[10] = L_motor_torque >> 8;
  data_ble[11] = R_motor_torque;
  data_ble[12] = R_motor_torque >> 8;
  data_ble[13] = L_motor_torque_command;
  data_ble[14] = L_motor_torque_command >> 8;
  data_ble[15] = R_motor_torque_command;
  data_ble[16] = R_motor_torque_command >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);
}