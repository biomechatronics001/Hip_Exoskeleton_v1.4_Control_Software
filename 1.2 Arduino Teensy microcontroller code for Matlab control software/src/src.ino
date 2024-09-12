#include <SPI.h>
#include "Motor_Control_Tmotor.h"
#include <FlexCAN_T4.h>
#include "Wireless_IMU.h"
#include "Controller.h"
#include <Arduino.h>
#include <stdio.h>
#include <TimeLib.h> // to get date and time
#include "Torque_Sensor.h" //FOR THE TORQUE SENSORS
ads1292r torque_sensor1;//FOR THE TORQUE SENSORS
//#include <SD.h>

// Settings
int assist_mode = 20;
int stopFlag = 0;
int saveDataFlag = 1;
int numOfInitialSteps = 1;
int enableExtensionStop = 0;
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, positive current = extension = encoder value increase // updated on 2023-05-16 2
uint32_t Motor_ID2 = 2; // Motor Can Bus ID, right leg, positive current = flexion = encoder value decrease // updated on 2023-05-16 3
int CAN_ID = 3;         // CAN port from Teensy
double Gear_ratio = 9;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
double torque_constant_before_gear = 0.105; // before gear at 24 V. Ref: https://store.tmotor.com/goods.php?id=982
double torqueCorrectionFactor = 1.09; // actual torque is 1.09x the motor-returned torque through experiments (2023/07/14)
int triggerPin = A9; //reading pin for sync with external mocap device
int groundPin = A8; // for sync with external mocap device

// *** for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h"

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 2000 // 500 Hz

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
// #define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes = 150,000,000 bytes.
// #define LOG_FILE_SIZE 100 * 500 * 80 // Size to log 10 byte lines at 500 Hz for more than 80 seconds =  bytes.
#define LOG_FILE_SIZE 220 * 500 * 600 // Size to log 10 byte lines at 500 Hz for more than 80 seconds = 66,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400 * 512 * 10 / 50 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.

// #define LOG_FILENAME "Walking.csv"
//#define LOG_FILENAME "0407_Powered_Jennifer_Walking_bd0.04_0.00_kd_1_0.6_0_0_0_0_newtau.csv"
#define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;


// *** End for RingBuf *** //

/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

// Begin Tmotor related code //
float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff_L = 0;
float t_ff_R = 0;
// End Tmotor related code //

// #define PI = 3.1415926
double LGPPrevious = 0;
double LGPCurrent = 0;
double dataCounter = 0;
int initialStepCounter = 0;
double neturalKneeAngle = 0.0; // degree, flexion is positive
double neturalKneeAngleSlack = 5.0; // degree; knee considered to be fully extend if knee encoder value <= neturalKneeAngle + neturalKneeAngleSlack
int stopAssistanceInSwingAtNeturalKnee = 0; //set to be always on
int isEnabled = 0;

// double weight = 52; // [kg] weight of the subject
uint32_t ID_offset = 0x140;

int Stop_button = 0;    // Stop function
String mode = "start";
double milli_time_current_double = 0;

Motor_Control_Tmotor m1(Motor_ID1, CAN_ID);
Motor_Control_Tmotor m2(Motor_ID2, CAN_ID);
IMU imu; 
Controller controller;
double Pgain = 7.5;    //P gain of torque control
double Igain = 0.7;    //I gain of torque control
double Dgain = 0;      //D gain of torque control
double MaxPIDout = 10; //Max torque control PID output (Unit Current A, inner loop is current controller)

double Fsample = 500;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                                    // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency

double Cur_command_L = 0;
double Cur_command_R = 0;
int current_limitation = 10; //(unit Amp)//double Tor_command_L = 0;
// Trigger
int triggerOn = 0;
int triggerVal = 0;  // analog trigger pin value
// Data logging
int isLogging = 0;
int taskIdx = 1;
int conditionIdx = 1; // baseline=1, sham=2, powered=3
int trialIdx = 1;
String taskName;
String conditionName;
String logFileName;
String paramFileName;
//
double torque_measured_L = 0.0;
double torque_measured_R = 0.0;

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int LK_ble = 0;                //left knee angle
int RK_ble = 0;                //right knee angle
int current_command_L_ble = 0; //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int current_actual_L_ble = 0;
int current_actual_R_ble = 0;
int torque_command_L_ble = 0; //total torque reference(N-m)
int torque_command_R_ble = 0;
int torque_command_imp_L_ble = 0; // impedance torque
int torque_command_imp_R_ble = 0; // impedance torque
int torque_command_ff_L_ble = 0;  // feedforward torque
int torque_command_ff_R_ble = 0;  // feedforward torque
int torque_estimated_L_ble = 0; //total torque reference(N-m)
int torque_estimated_R_ble = 0;
int torque_measured_L_ble = 0;             //actual torque(N-m)(measured by torque sensor)
int torque_measured_R_ble = 0;
int motor_speed_L_ble = 0;
int motor_speed_R_ble = 0;
double torque_command_L = 0;
double torque_command_R = 0;

int gait_percentage_L_ble = 0;
int insole_torque_command_L_ble = 0;
int insole_torque_command_R_ble = 0;
int insole_gait_percent_L_ble = 0;
int insole_gait_percent_R_ble = 0;
int imu_gait_percent_L_ble = 0;
int imu_gait_percent_R_ble = 0;
int imu_gait_percent_imp_L_ble = 0; // impedance torque
int imu_gait_percent_imp_R_ble = 0; // mpedance torque
int imu_gait_percent_ff_L_ble = 0;  // eedforward biological torque
int imu_gait_percent_ff_R_ble = 0;  // feedforward biological torque
double Insole_gain = 5;

//int percentage_ble = 0;  //gait percentage(%)
//int torque_commandGP_ble = 0;     //torque reference(N-m) generated by gait percentage VS torque reference table
//int current_ble = 0;              //actual current (A)

double relTime = 0.0;
int SDcounter = 0;
int SDtotalCounter = 0;

// int squat_bio[140] = { -16,  -16,  -16,  -16,  -15,  -14,  -12,  -11,  -10,  -9, -8, -6, -4, -2, -1, 1,  3,  4,  5,  8,  9,  10, 11, 13, 14, 16, 17, 18, 20, 22, 23, 25, 26, 28, 29, 30, 31, 32, 33, 36, 37, 38, 39, 41, 42, 43, 44, 46, 47, 48, 48, 50, 51, 51, 52, 54, 54, 56, 56, 58, 58, 59, 60, 61, 62, 63, 64, 65, 66, 66, 67, 68, 68, 69, 69, 70, 71, 72, 72, 73, 74, 75, 76, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 84, 85, 85, 86, 87, 88, 88, 89, 89, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 91, 92, 92, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93};
// int squat_bio_angle = 0;

// Send message to MATLAB GUI
int messageID = 0;
int isSDCardPresent = 0;
int isDataSaved = 0;
int logYear;
int logMonth;
int logDay;
int logHour;
int logMinute;
int logSecond;

// for NYU controller
double NYU_start_time = micros();
double NYU_time = 0.0;


void setup()
{
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);  //used for communication with computer.
  Serial5.begin(115200); //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN2();
  delay(500);
  IMUSetup();
  // m1.init_motor(); // Strat the CAN bus communication & Motor Control
  // delay(100);
  // m2.init_motor(); // Strat the CAN bus communication & Motor Control
  // delay(100);
  reset_motor_angle();
  CurrentControlSetup();
  pinMode(A8, INPUT); // init trigger pin

  // controller.calculateSigmoid();
  // controller.calculateSigmoid(controller.kdMax, controller.kdMin, controller.kd_arr_Junxi);
  // controller.calculateSigmoid(controller.bdMax, controller.bdMin, controller.bd_arr_Junxi);
  // controller.calculateSigmoid(1, 0, controller.sigmoid);

  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  //
  torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.//FOR THE TORQUE SENSORS
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 1.8, 0.0003446 * (-1) * 1.97); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.//FOR THE TORQUE SENSORS. The first in this list is the Left motor (cable with tape further from header).
  torque_sensor1.Torque_sensor_offset_calibration();//FOR THE TORQUE SENSORS
  delay(1000);//FOR THE TORQUE SENSORS

  SPI1.setMOSI(26);//FOR THE TORQUE SENSORS
  SPI1.setMISO(1);//FOR THE TORQUE SENSORS
  SPI1.setSCK(27);//FOR THE TORQUE SENSORS
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void IMUSetup()
{
  imu.Gain_E = 1;
  imu.Gain_F = 1;
  imu.STS_Gain = 0;
  imu.delaypoint = 0;
  imu.alpha = 5;
  imu.beta = 2;
  imu.angleRelativeThreshold = 20;
}

void loop()
{
  while (stopFlag)
  {
  };

  // imu.READ();
  CurrentControl();
  // Serial.print(m1.pos);
  // Serial.println(m2.pos);
}

int SDCardSetup()
{
  sd.remove(LOG_FILENAME);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    return 0;
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return 0;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.println("Data logging initialized.");
  return 1;
}

int SDCardSetup(const char* fileName)
{
  sd.remove(fileName);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    return 0;
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(fileName, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return 0;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.print("Data logging initialized. File name = ");
  Serial.println(fileName);
  return 1;
}

int SaveAssistanceProfileParameters(const char* fileName)
{
  sd.remove(fileName);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    return 0;
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(fileName, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return 0;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.print("Assistance profile parameters saving initialized. File name = ");
  Serial.println(fileName);

  //
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
  {
    Serial.println("File full - quitting.");
    return; // break;
  }
  if (n > maxUsed)
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy())
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512))
    {
      Serial.println("writeOut failed");
      return; //break;
    }
  }
  for (int i = 0; i < controller.sineAssistanceFlexionProfileCounter; i++)
  {
    double sineAssistanceMagnitude = controller.sineAssistanceFlexionParameterList[i][0];
    int sineAssistanceShift = (int)controller.sineAssistanceFlexionParameterList[i][1];
    int sineAssistanceDuration = (int)controller.sineAssistanceFlexionParameterList[i][2];
    double sineAssistanceSaturation = controller.sineAssistanceFlexionParameterList[i][3];
    rb.write("Flexion, Saturation = ");
    rb.print(sineAssistanceSaturation);
    rb.write(", Magnitude = ");
    rb.print(sineAssistanceMagnitude);
    rb.write(", Shift = ");
    rb.print(sineAssistanceShift);
    rb.write(", Duration = ");
    rb.println(sineAssistanceDuration);
  }
  for (int i = 0; i < controller.sineAssistanceExtensionProfileCounter; i++)
  {
    double sineAssistanceMagnitude = controller.sineAssistanceExtensionParameterList[i][0];
    int sineAssistanceShift = (int)controller.sineAssistanceExtensionParameterList[i][1];
    int sineAssistanceDuration = (int)controller.sineAssistanceExtensionParameterList[i][2];
    double sineAssistanceSaturation = controller.sineAssistanceExtensionParameterList[i][3];
    rb.write("Extension, Saturation = ");
    rb.print(sineAssistanceSaturation);
    rb.write(", Magnitude = ");
    rb.print(sineAssistanceMagnitude);
    rb.write(", Shift = ");
    rb.print(sineAssistanceShift);
    rb.write(", Duration = ");
    rb.println(sineAssistanceDuration);
  }
  if (rb.getWriteError())
  {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return 0; //break;
  }
  SDCardSaveToFile();
  Serial.println("Assistance profile parameters saved");
  // Serial.println(fileName);

  return 1;
}

void CurrentControlSetup()
{
  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
  // previous_time_insole = current_time;
}

void CurrentControl()
{
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  current_time = micros();             //query current time (microsencond)
  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    imu.READ();
    if (Stop_button) //stop
    {
      p_des = 0; //dont change this
      v_des = 0; //dont change this
      kp = 0; //dont change this
      kd = 0; //dont change this
      torque_command_L = 0; // torque command
      torque_command_R = 0; // torque command
    }
    else
    {
      Compute_Torque_Commands(); // TODO
    }

    // if (assist_mode == 10)
    // {
    //   // enforceInitialSteps();
    // }
    // Serial.println(torque_command_L);
    m1.send_cmd(p_des, v_des, kp, kd, torque_command_L / torqueCorrectionFactor); //to correct for the torque
    // delay(1);
    for (int  qwe = 0; qwe < 100; qwe++)
    {

    }
    receive_CAN_data();
    // delay(1);
    for (int  qwe = 0; qwe < 100; qwe++)
    {

    }
    m2.send_cmd(p_des, v_des, kp, kd, torque_command_R / torqueCorrectionFactor);//to correct for the torque
    // delay(1);
    for (int  qwe = 0; qwe < 100; qwe++)
    {

    }
    receive_CAN_data();
    for (int  qwe = 0; qwe < 100; qwe++)
    {

    }
    // delay(1);

    // Read trigger signal
    triggerVal = analogRead(triggerPin);
    digitalWrite(groundPin, LOW); //pin converted into GND
    // triggerVal = analogRead(A8); // temp change on 01/17/2023 for the Sinai visit
    if (triggerVal < 400)
    {
      triggerOn = 0;
    }
    else
    {
      triggerOn = 1;
    }
    

    if (isLogging)
    {
      logData3();
    }

    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data(); // send the BLE data
    previous_time_ble = current_time;
    // CustomWait(1000);
    torque_sensor1.Torque_sensor_read(); //FOR THE TORQUE SENSORS
    torque_measured_L = -torque_sensor1.torque[0];
    torque_measured_R = torque_sensor1.torque[1];
    // Serial.print(-torque_sensor1.torque[0]); Serial.print("   ");//FOR THE TORQUE SENSORS. THIS IS THE LEFT MOTOR
    // Serial.println(torque_command_L);
    Serial.print(imu.LTx);
    Serial.print(" ");
    Serial.println(torque_command_L);

    // plot_controller_data();

  }
  if (Serial.available())
  {
    char cmd = (char)Serial.read();
    if (cmd == 's')
    {
      stopFlag = 1;
      Serial.println("Stopped");
      if (saveDataFlag)
      {
        SDCardSaveToFile();
        messageID = 2;
        int messageValueArray[] = {1, logYear, logMonth, logDay, logHour, logMinute, logSecond};
        sendMessage(messageID, messageValueArray, 7);
        String yearStr = String(logYear);
        String monthStr = (logMonth < 10) ? "0" + String(logMonth) : String(logMonth);
        String dayStr = (logDay < 10) ? "0" + String(logDay) : String(logDay);
        String hourStr = (logHour < 10) ? "0" + String(logHour) : String(logHour);
        String minuteStr = (logMinute < 10) ? "0" + String(logMinute) : String(logMinute);
        String secondStr = (logSecond < 10) ? "0" + String(logSecond) : String(logSecond);
        String timeStr = yearStr + "-" + monthStr + "-" + dayStr + "-" + hourStr + "-" + minuteStr + "-" + secondStr;

        String stringOne = taskName + '-';
        String stringTwo = stringOne + conditionName + "-Trial";
        logFileName = timeStr + "-" + stringTwo + String(trialIdx) + ".csv";
        paramFileName = timeStr + "-" + stringTwo + String(trialIdx) + "-Parameters.txt";
        Serial.println(logFileName);
        SaveAssistanceProfileParameters(paramFileName.c_str());
      }
    }
  }
}

void Compute_Torque_Commands()
{
  if (assist_mode == 1) //Sine controller (IMU)
  {
    mode = "Sine (IMU)";
    controller.computeGaitPhase(imu.LTAVx - imu.RTAVx, -m1.pos, m2.pos, -m1.spe, m2.spe);
    controller.computeSingleSineAssistanceProfile();
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = controller.sineAssistanceTotalTorqueLeft;
    torque_command_R = -controller.sineAssistanceTotalTorqueRight;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
    // Serial.println(Cur_command_L);
  }
  else if (assist_mode == 2) //Samsung controller (IMU)
  {
    mode = "Samsung (IMU)";
    controller.computeGaitPhase(imu.LTAVx - imu.RTAVx, -m1.pos, m2.pos, -m1.spe, m2.spe);
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = imu.DOTC[0];
    torque_command_R = -imu.DOTC[1];
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
    
  }
  else if (assist_mode == 3) //Constant torque (for hardware debugging purpose)
  {
    mode = "Constant";
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = 0;
    torque_command_R = 0;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  else if (assist_mode == 20) //NYU controller
  {
    mode = "NYU controller";
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    double x = (imu.LTx - imu.RTx) / 180.0 * 3.1415926;
    double x_dot = (imu.LTAVx - imu.RTAVx) / 180 * 3.1415926;
    NYU_time = (micros() - NYU_start_time) / 1000000.0;
    torque_command_L = - (13.9758 * x + 2.1551 * x_dot) + (-6.3961 * 1 + (-4.3143) * sin(NYU_time) + (-6.2224) * cos(NYU_time) + (-3.9566) * sin(2*NYU_time) + 1.2621 * cos(2*NYU_time) + (-2.4282) * sin(3*NYU_time) + 5.5883 * cos(3*NYU_time) + (-416.1619) * sin(4*NYU_time) + 210.3456 * cos(4*NYU_time) + 10.3936 * sin(5*NYU_time) + (-3.5327) * cos(5*NYU_time));
    torque_command_R = -torque_command_L;
    torque_command_L = -torque_command_L * 0.01;
    torque_command_R = torque_command_R * 0.01;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  

  else if (assist_mode == 100)
  {
    mode = "Stop";
    p_des = 0; //dont change this
    v_des = 0; //dont change this
    kp = 0; //dont change this
    kd = 0; //dont change this
    torque_command_L = 0;
    torque_command_R = 0;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
}



void SDCardSaveToFile()
{
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
}

void CustomWait(unsigned long delay_control)
{
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  }
  while (Time_Control < Time_Delta);

}


// void Cur_limitation()
// {
//   //************* Current limitation *************//

//   Cur_command_L = min(current_limitation, Cur_command_L);
//   Cur_command_L = max(-current_limitation, Cur_command_L);
//   Cur_command_R = min(current_limitation, Cur_command_R);
//   Cur_command_R = max(-current_limitation, Cur_command_R);

//   torque_command_L = Cur_command_L * 0.232 * 9;//2.2;
//   torque_command_R = Cur_command_R * 0.232 * 9;//2.2;
// }

void enforceInitialSteps()
{
  if (dataCounter == 1)
  {
    LGPCurrent = controller.GP_IMU_L;
  }
  else
  {
    LGPPrevious = LGPCurrent;
    LGPCurrent = controller.GP_IMU_L;
  }
  dataCounter += 1;

  if ((LGPPrevious > 95) && (LGPCurrent < 1))
  {
    controller.initialStepCounter += 1;
  }

  if (controller.initialStepCounter <= numOfInitialSteps)
  {
    Cur_command_L = Cur_command_L * (controller.initialStepCounter - 0) / numOfInitialSteps;
    Cur_command_R = Cur_command_R * (controller.initialStepCounter - 0) / numOfInitialSteps;
  }
}

void receive_ble_Data()
{
  if (Serial5.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial5.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial5.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial5.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial5.readBytes(&data_rs232_rx[3], 17);
          if (data_rs232_rx[3] == 0)
          {
            Stop_button = int(data_rs232_rx[4]);
            if (Stop_button)
            {
              Serial.println("STOP button pressed");
            }
            else
            {
              Serial.println("START button pressed");
            }
          }
          else if (data_rs232_rx[3] == 1)
          {
            assist_mode = int(data_rs232_rx[4]);
            Serial.print("Mode: ");
            Serial.println(assist_mode);
            //Serial.print("    ");
            //Serial.println(mode);
          }
          else if (data_rs232_rx[3] == 2)
          {
            float Gain_E = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.Gain_E = Gain_E;

            Serial.print("Extension gain from Matlab: ");
            Serial.println(Gain_E);
          }
          else if (data_rs232_rx[3] == 3)
          {
            float Gain_F = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.Gain_F = Gain_F;

            Serial.print("Flexion gain from matlab: ");
            Serial.println(Gain_F);
          }
          else if (data_rs232_rx[3] == 4)
          { // 10 ms per timepoint, delaypoint needs to be less than 100
            // 1 delaypoint in matlab interface equals 5 timepoints here, that is 50 ms
            int delaypoint = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000;
            imu.delaypoint = delaypoint;

            Serial.print("Delay [ms]: ");
            Serial.println(delaypoint * 10);
            // delay = delaypoint*sample time
          }
          // else if (data_rs232_rx[3] == 5)
          // {
          //   weight = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   Serial.print("Weight [kg]: ");
          //   Serial.println(weight);
          // }
          // else if (data_rs232_rx[3] == 6)
          // {
          //   Insole_gain = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   Serial.print("Insole gain: ");
          //   Serial.println(Insole_gain);
          // }
          else if (data_rs232_rx[3] == 7)
          {
            reset_motor_angle();
            // int m = 0;
            // float sum = 0;
            // int SSize = 20;
            // while (m < SSize) {
            //   sum = (m1.pos + m2.pos) + sum;
            //   m++;
            // }
            neturalKneeAngle = 0; //sum / (SSize * 2);
            Serial.println("The angle of motor has been reset");
            imu.INIT_MEAN();
            Serial.println("The angle of IMUs has been reset");
          }

          // else if (data_rs232_rx[3] == 11)
          // {
          //   float Gain_Sw = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.ControllerGain_L = Gain_Sw;
          //   controller.ControllerGain_R = Gain_Sw;
          //   Serial.print("Swing gain from matlab: ");
          //   Serial.println(Gain_Sw);
          // }
          // else if (data_rs232_rx[3] == 12)
          // {
          //   float Gain_St = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.FeedForwardGain_L = Gain_St;
          //   controller.FeedForwardGain_R = Gain_St;
          //   Serial.print("Stance gain from matlab: ");
          //   Serial.println(Gain_St);
          // }
          // else if (data_rs232_rx[3] == 13)
          // {
          //   float Timing_Sw = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.P_ahead_imp = Timing_Sw;
          //   Serial.print("Swing timing from matlab: ");
          //   Serial.println(Timing_Sw);
          // }
          // else if (data_rs232_rx[3] == 14)
          // {
          //   float Timing_St = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   controller.P_ahead_ff = Timing_St;
          //   Serial.print("Stance timing from matlab: ");
          //   Serial.println(Timing_St);
          // }
          // else if (data_rs232_rx[3] == 15)
          // {
          //   double STSMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.STS_Gain = STSMagnitude;
          //   Serial.print("Sit-to-stand gain from matlab: ");
          //   Serial.println(STSMagnitude);
          // }
          // else if (data_rs232_rx[3] == 16)
          // {
          //   double STSSensitivity = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.STSSlopeThreshold = STSSensitivity;
          //   Serial.print("Sit-to-stand slope threshold from matlab: ");
          //   Serial.println(STSSensitivity);
          // }
          // else if (data_rs232_rx[3] == 17)
          // {
          //   double STSAlpha = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.alpha = STSAlpha;
          //   Serial.print("Sit-to-stand alpha from matlab: ");
          //   Serial.println(STSAlpha);
          // }
          // else if (data_rs232_rx[3] == 18)
          // {
          //   double STSBeta = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
          //   imu.beta = STSBeta;
          //   Serial.print("Sit-to-stand beta from matlab: ");
          //   Serial.println(STSBeta);
          // }
          else if (data_rs232_rx[3] == 20)
          {
            isLogging = int(data_rs232_rx[7]);
            if (isLogging == 1)
            {
              taskIdx = int(data_rs232_rx[4]);
              conditionIdx = int(data_rs232_rx[5]);
              trialIdx = int(data_rs232_rx[6]);
              // String taskName;
              if (taskIdx == 1)
              {
                taskName = "Walking";
              }
              else if (taskIdx == 2)
              {
                taskName = "STS";
              }
              String stringOne = taskName + '-';

              // String conditionName;
              if (conditionIdx == 1)
              {
                conditionName = "Baseline";
              }
              else if (conditionIdx == 2)
              {
                conditionName = "Sham";
              }
              else if (conditionIdx == 3)
              {
                conditionName = "Powered";
              }
              else
              {
                conditionName = "Others";
              }
              logYear = year();
              logMonth = month();
              logDay = day();
              logHour = hour();
              logMinute = minute();
              logSecond = second();
              String yearStr = String(logYear);
              String monthStr = (logMonth < 10) ? "0" + String(logMonth) : String(logMonth);
              String dayStr = (logDay < 10) ? "0" + String(logDay) : String(logDay);
              String hourStr = (logHour < 10) ? "0" + String(logHour) : String(logHour);
              String minuteStr = (logMinute < 10) ? "0" + String(logMinute) : String(logMinute);
              String secondStr = (logSecond < 10) ? "0" + String(logSecond) : String(logSecond);
              String timeStr = yearStr + "-" + monthStr + "-" + dayStr + "-" + hourStr + "-" + minuteStr + "-" + secondStr;

              String stringTwo = stringOne + conditionName + "-Trial";
              logFileName = timeStr + "-" + stringTwo + String(trialIdx) + ".csv";
              paramFileName = timeStr + "-" + stringTwo + String(trialIdx) + "-Parameters.txt";
              Serial.println(logFileName);
              int status = SDCardSetup(logFileName.c_str());
              if (status == 1)
              {
                messageID = 1;
                int messageValueArray[] = {1};
                sendMessage(messageID, messageValueArray, 1);
                relTime = 0.0;
                Serial.println("Data logging started......");
              }
              else
              {
                messageID = 1;
                int messageValueArray[] = {0};
                sendMessage(messageID, messageValueArray, 1);
                isLogging = 0; // prevent logging because there is no SD
                Serial.println("Data logging aborted because there is no SD");
              }

            }
            else if (isLogging == 0)
            {
              SDCardSaveToFile();
              messageID = 2;
              int messageValueArray[] = {1, logYear - 2000, logMonth, logDay, logHour, logMinute, logSecond};
              sendMessage(messageID, messageValueArray, 7);
              Serial.println("Data logging stopped......");
              // SaveAssistanceProfileParameters(paramFileName.c_str());
            }
          }
          // else if (data_rs232_rx[3] == 25)
          // {
          //   int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
          //   String sineAssistanceTypeName;
          //   if (sineAssistanceTypeIdx == 1)
          //   {
          //     sineAssistanceTypeName = "Flexion";
          //   }
          //   else if (sineAssistanceTypeIdx == 2)
          //   {
          //     sineAssistanceTypeName = "Extension";
          //   }
          //   double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[5]) | ((uint16_t)data_rs232_rx[6] << 8))) / 100.0;
          //   int sineAssistanceShift = int(data_rs232_rx[7]);
          //   int sineAssistanceDuration = int(data_rs232_rx[8]);
          //   double sineAssistanceSaturation = ((int16_t)(((uint16_t)data_rs232_rx[9]) | ((uint16_t)data_rs232_rx[10] << 8))) / 100.0;
          //   controller.addSineAssistanceProfile(sineAssistanceTypeIdx, sineAssistanceMagnitude, sineAssistanceShift, sineAssistanceDuration, sineAssistanceSaturation);
          //   Serial.print("Added ");
          //   Serial.print(sineAssistanceTypeName);
          //   Serial.print(" assistance. Magnitude = ");
          //   Serial.print(sineAssistanceMagnitude);
          //   Serial.print(", shift = ");
          //   Serial.print(sineAssistanceShift);
          //   Serial.print(", duration = ");
          //   Serial.print(sineAssistanceDuration);
          //   Serial.print(", saturation = ");
          //   Serial.print(sineAssistanceSaturation);
          //   Serial.print(". Number of flexion profile = ");
          //   Serial.print(controller.sineAssistanceFlexionProfileCounter);
          //   Serial.print(". Number of extension profile = ");
          //   Serial.println(controller.sineAssistanceExtensionProfileCounter);
          // }
          // else if (data_rs232_rx[3] == 26)
          // {
          //   int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
          //   String sineAssistanceTypeName;
          //   if (sineAssistanceTypeIdx == 1)
          //   {
          //     sineAssistanceTypeName = "Flexion";
          //   }
          //   else if (sineAssistanceTypeIdx == 2)
          //   {
          //     sineAssistanceTypeName = "Extension";
          //   }
          //   int sineAssistanceProfileIdx = int(data_rs232_rx[5]);
          //   // for (int i = 0; i < 3; i++)
          //   // {
          //   //   Serial.print(controller.sineAssistanceExtensionParameterList[0][i]);
          //   //   Serial.print(" ");
          //   // }
          //   // Serial.println(" ");
          //   controller.removeSineAssistanceProfile(sineAssistanceTypeIdx, sineAssistanceProfileIdx);
          //   Serial.print("Removed ");
          //   Serial.print(sineAssistanceTypeName);
          //   Serial.print(" assistance. ID = ");
          //   Serial.println(sineAssistanceProfileIdx);

          //   // for (int i = 0; i < 3; i++)
          //   // {
          //   //   Serial.print(controller.sineAssistanceExtensionParameterList[0][i]);
          //   //   Serial.print(" ");
          //   // }
          //   // Serial.println(" ");
          // }
          // else if (data_rs232_rx[3] == 27)
          // {
          //   int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
          //   String sineAssistanceTypeName;
          //   if (sineAssistanceTypeIdx == 1)
          //   {
          //     sineAssistanceTypeName = "Flexion";
          //   }
          //   else if (sineAssistanceTypeIdx == 2)
          //   {
          //     sineAssistanceTypeName = "Extension";
          //   }
          //   int sineAssistanceProfileIdx = int(data_rs232_rx[5]);
          //   double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[6]) | ((uint16_t)data_rs232_rx[7] << 8))) / 100.0;
          //   int sineAssistanceShift = int(data_rs232_rx[8]);
          //   int sineAssistanceDuration = int(data_rs232_rx[9]);
          //   double sineAssistanceSaturation = ((int16_t)(((uint16_t)data_rs232_rx[10]) | ((uint16_t)data_rs232_rx[11] << 8))) / 100.0;
          //   controller.editSineAssistanceProfile(sineAssistanceTypeIdx, sineAssistanceProfileIdx, sineAssistanceMagnitude, sineAssistanceShift, sineAssistanceDuration, sineAssistanceSaturation);
          //   Serial.print("Edit ");
          //   Serial.print(sineAssistanceTypeName);
          //   Serial.print(" assistance. ID = ");
          //   Serial.print(sineAssistanceProfileIdx);
          //   Serial.print(". Magnitude = ");
          //   Serial.print(sineAssistanceMagnitude);
          //   Serial.print(", shift = ");
          //   Serial.print(sineAssistanceShift);
          //   Serial.print(", duration = ");
          //   Serial.print(sineAssistanceDuration);
          //   Serial.print(", saturation = ");
          //   Serial.println(sineAssistanceSaturation);
          // }
          // else if (data_rs232_rx[3] == 28)
          // {
          //   stopAssistanceInSwingAtNeturalKnee = int(data_rs232_rx[4]); //set to be always on
          //   Serial.println(stopAssistanceInSwingAtNeturalKnee);
          // }
          // else if (data_rs232_rx[3] == 30)
          // {
          //   double STSStandUpMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSStandUpMagnitude = STSStandUpMagnitude;
          //   Serial.print("STS magnitude (stand up) = ");
          //   Serial.print(STSStandUpMagnitude);
          //   Serial.println(" Nm");
          // }
          // else if (data_rs232_rx[3] == 31)
          // {
          //   double STSSitDownMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSSitDownMagnitude = STSSitDownMagnitude;
          //   Serial.print("STS magnitude (sit down) = ");
          //   Serial.print(STSSitDownMagnitude);
          //   Serial.println(" Nm");
          // }
          // else if (data_rs232_rx[3] == 32)
          // {
          //   double STSStandSlackAngle = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSStandSlackAngle = STSStandSlackAngle;
          //   Serial.print("STS slack angle (stand) = ");
          //   Serial.print(STSStandSlackAngle);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 33)
          // {
          //   double STSSitSlackAngle = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSSitSlackAngle = STSSitSlackAngle;
          //   Serial.print("STS slack angle (sit) = ");
          //   Serial.print(STSSitSlackAngle);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 34)
          // {
          //   double STSStandThreshold = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSStandThreshold = STSStandThreshold;
          //   Serial.print("STS threshold (stand) = ");
          //   Serial.print(STSStandThreshold);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 35)
          // {
          //   double STSSitThreshold = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
          //   imu.STSSitThreshold = STSSitThreshold;
          //   Serial.print("STS slack angle (sit) = ");
          //   Serial.print(STSSitThreshold);
          //   Serial.println(" deg");
          // }
          // else if (data_rs232_rx[3] == 36)
          // {
          //   imu.STSStandUpMagnitude = 0.0;
          //   imu.STSSitDownMagnitude = 0.0;
          //   imu.STSStandSlackAngle = 7.5;
          //   imu.STSSitSlackAngle = 7.5;
          //   imu.STSStandThreshold = 5.0;
          //   imu.STSSitThreshold = 75.0;
          //   imu.STS_state = 0;
          //   imu.STSSitPreviousMaxAngle = 75;
          //   imu.STSStandPreviousMinAngle = 5;
          //   Serial.println("STS Reset button pressed. All parameters reset to default values.");
          // }
          else if (data_rs232_rx[3] == 50) // Sine controller
          {
            double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 100.0;
            int sineAssistanceShift = int(data_rs232_rx[6]);
            int sineAssistanceDuration = int(data_rs232_rx[7]);

            controller.updateSineAssistanceProfile(sineAssistanceMagnitude, sineAssistanceShift, sineAssistanceDuration);
            Serial.print("Sine assistance. Magnitude = ");
            Serial.print(sineAssistanceMagnitude);
            Serial.print(", shift = ");
            Serial.print(sineAssistanceShift);
            Serial.print(", duration = ");
            Serial.println(sineAssistanceDuration);
           
          }
        }
      }
    }
  }
}

void send_ble_Data()
{
  // Serial.println("Sending...");
  LK_ble = imu.LKx * 100;
  RK_ble = imu.RKx * 100;

  current_command_L_ble = -Cur_command_L * 100; //
  current_command_R_ble = Cur_command_R * 100;  //
  // current_command_L_ble = -m1.iq_A * 100;
  // current_command_R_ble = m2.iq_A * 100;

  torque_command_L_ble = torque_command_L * 100; //
  torque_estimated_L_ble = m1.torque * 100;  //
  torque_measured_L_ble = torque_measured_L * 100;

  torque_command_R_ble = -torque_command_R * 100; //
  torque_estimated_R_ble = -m2.torque * 100;
  torque_measured_R_ble = torque_measured_R * 100;

  //  gait_percentage_L_ble = imu.gait_percentage_L * 100;
  gait_percentage_L_ble = 0;

  // insole_torque_command_L_ble = insole.normalized_torque_command_L * 100;
  // insole_torque_command_R_ble = insole.normalized_torque_command_R * 100;
  // insole_gait_percent_L_ble = insole.gait_percent_L * 100;
  // insole_gait_percent_R_ble = insole.gait_percent_R * 100;
  imu_gait_percent_L_ble = controller.GP_IMU_L_ahead * 100;
  imu_gait_percent_R_ble = controller.GP_IMU_R_ahead * 100;
  //
  imu_gait_percent_imp_L_ble = controller.GP_IMU_L_ahead_imp * 100;
  imu_gait_percent_imp_R_ble = controller.GP_IMU_R_ahead_imp * 100;
  imu_gait_percent_ff_L_ble = controller.GP_IMU_L_ahead_ff * 100;
  imu_gait_percent_ff_R_ble = controller.GP_IMU_R_ahead_ff * 100;
  //
  torque_command_imp_L_ble = -controller.impedanceTorque_L * 100;
  torque_command_imp_R_ble = -controller.impedanceTorque_R * 100;
  torque_command_ff_L_ble = controller.feedforwardTorque_L * 100;
  torque_command_ff_R_ble = controller.feedforwardTorque_R * 100;
  //
  motor_speed_L_ble = m1.spe * 100; // radian
  motor_speed_R_ble = -m2.spe * 100; // radian

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  data_ble[3] = LK_ble;
  data_ble[4] = LK_ble >> 8;
  data_ble[5] = RK_ble;
  data_ble[6] = RK_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] = current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = torque_command_L_ble;
  data_ble[12] = torque_command_L_ble >> 8;
  data_ble[13] = torque_command_R_ble;
  data_ble[14] = torque_command_R_ble >> 8;
  data_ble[15] = torque_estimated_L_ble;
  data_ble[16] = torque_estimated_L_ble >> 8;
  data_ble[17] = torque_estimated_R_ble;
  data_ble[18] = torque_estimated_R_ble >> 8;
  data_ble[19] = torque_measured_L_ble;
  data_ble[20] = torque_measured_L_ble >> 8;
  data_ble[21] = imu_gait_percent_L_ble;
  data_ble[22] = imu_gait_percent_L_ble >> 8;
  data_ble[23] = imu_gait_percent_R_ble;
  data_ble[24] = imu_gait_percent_R_ble >> 8;
  data_ble[25] = motor_speed_L_ble;
  data_ble[26] = motor_speed_L_ble >> 8;
  data_ble[27] = motor_speed_R_ble;
  data_ble[28] = motor_speed_R_ble >> 8;
  data_ble[29] = torque_measured_R_ble;
  data_ble[30] = torque_measured_R_ble >> 8;
  // data_ble[31] = imu.STSSitThreshold - 20;
  //

  Serial5.write(data_ble, datalength_ble);
}

void sendMessage(int messageID, int* messageValueArray, int messageLength)
{
  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 95
  // 2    bluetooth data length
  // 3    Message ID
  // ID = 0  All normal
  // ID = 1  if SD card detected on Teensy
  // ID = 2  if logged data file is saved

  data_ble[0] = 165;
  data_ble[1] = 95;
  data_ble[2] = datalength_ble;
  data_ble[3] = messageID;
  data_ble[4] = messageLength;
  for (int i = 0; i < messageLength; i++)
  {
    data_ble[5 + i] = messageValueArray[i];
  }
  Serial5.write(data_ble, datalength_ble);
  // Serial.println("Message sent");
  // Serial.println((double)data_ble[1]);
}

//*** Ringbuf ***//


void logData3()
{
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX;

  // Start time.
  uint32_t logTime = micros();
  // Log data until Serial input or file full.
  //  while (!Serial.available()) {
  // Amount of data in ringBuf.
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
  {
    Serial.println("File full - quitting.");
    return; // break;
  }
  if (n > maxUsed)
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy())
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512))
    {
      Serial.println("writeOut failed");
      return; //break;
    }
  }
  // Time for next point.
  //  logTime += LOG_INTERVAL_USEC;
  //  int32_t spareMicros = logTime - micros();
  //  if (spareMicros < minSpareMicros) {
  //    minSpareMicros = spareMicros;
  //  }
  //  if (spareMicros <= 0) {
  //    Serial.print("Rate too fast ");
  //    Serial.println(spareMicros);
  //    break;
  //  }
  // Wait until time to log data.
  //  while (micros() < logTime) {}

  // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
  //  uint16_t adc = analogRead(0);
  // Print spareMicros into the RingBuf as test data.
  rb.print(relTime);
  rb.write(" ");
  rb.print(imu.TKx); // trunk angle sagittal [deg]
  rb.write(" ");
  rb.print(imu.TKy); // trunk angle frontal [deg]
  rb.write(" ");
  rb.print(imu.TKz); // trunk angle transverse [deg]
  rb.write(" ");
  rb.print(imu.LTx); // left thigh angle sagittal [deg]
  rb.write(" ");
  rb.print(imu.LTy); // left thigh angle frontal [deg]
  rb.write(" ");
  rb.print(imu.LTz); // left thigh angle transverse [deg]
  rb.write(" ");
  rb.print(imu.RTx); // right thigh angle sagittal [deg]
  rb.write(" ");
  rb.print(imu.RTy); // right thigh angle frontal [deg]
  rb.write(" ");
  rb.print(imu.RTz); // right thigh angle transverse [deg]
  rb.write(" ");
  rb.print(imu.LSx); // left shank angle sagittal [deg]
  rb.write(" ");
  rb.print(imu.LSy); // left shank angle frontal [deg]
  rb.write(" ");
  rb.print(imu.LSz); // left shank angle transverse [deg]
  rb.write(" ");
  rb.print(imu.RSx); // right shank angle sagittal [deg]
  rb.write(" ");
  rb.print(imu.RSy); // right shank angle frontal [deg]
  rb.write(" ");
  rb.print(imu.RSz); // right shank angle transverse [deg]
  rb.write(" ");
  rb.print(imu.TKAVx); // trunk anglular velocity sagittal [deg/s]
  rb.write(" ");
  rb.print(imu.TKAVy); // trunk anglular velocity frontal [deg/s]
  rb.write(" ");
  rb.print(imu.TKAVz); // trunk anglular velocity transverse [deg/s]
  rb.write(" ");
  rb.print(imu.LTAVx); // left thigh anglular velocity sagittal [deg/s]
  rb.write(" ");
  rb.print(imu.LTAVy); // left thigh anglular velocity frontal [deg/s]
  rb.write(" ");
  rb.print(imu.LTAVz); // left thigh anglular velocity transverse [deg/s]
  rb.write(" ");
  rb.print(imu.RTAVx); // right thigh anglular velocity sagittal [deg/s]
  rb.write(" ");
  rb.print(imu.RTAVy); // right thigh anglular velocity frontal [deg/s]
  rb.write(" ");
  rb.print(imu.RTAVz); // right thigh anglular velocity transverse [deg/s]
  rb.write(" ");
  rb.print(imu.LSAVx); // left shank anglular velocity sagittal [deg/s]
  rb.write(" ");
  rb.print(imu.LSAVy); // left shank anglular velocity frontal [deg/s]
  rb.write(" ");
  rb.print(imu.LSAVz); // left shank anglular velocity transverse [deg/s]
  rb.write(" ");
  rb.print(imu.RSAVx); // right shank anglular velocity sagittal [deg/s]
  rb.write(" ");
  rb.print(imu.RSAVy); // right shank anglular velocity frontal [deg/s]
  rb.write(" ");
  rb.print(imu.RSAVz); // right shank anglular velocity transverse [deg/s]
  rb.write(" ");
  rb.print(controller.GP_IMU_L); // estimated gait phase of left leg [0-100, 0 = heel strike, 100 = toe off]
  rb.write(" ");
  rb.print(controller.GP_IMU_R); // estimated gait phase of right leg [0-100, 0 = heel strike, 100 = toe off]
  rb.write(" ");
  rb.print(Cur_command_L); // commanded current to the left motor [A]
  rb.write(" ");
  rb.print(Cur_command_R); // commanded current to the right motor [A]
  rb.write(" ");
  rb.print(m1.torque * torqueCorrectionFactor / torque_constant_before_gear / Gear_ratio); // estimated current to the left motor [A]
  rb.write(" ");
  rb.print(m2.torque * torqueCorrectionFactor / torque_constant_before_gear / Gear_ratio); // estimated current to the right motor [A]
  rb.write(" ");
  rb.print(m1.pos); // motor-returned encoder position of the left motor [rad]
  rb.write(" ");
  rb.print(m2.pos); // motor-returned encoder position of the right motor [rad]
  rb.write(" ");
  rb.print(m1.spe); // motor-returned speed of the left motor [rad/s]
  rb.write(" ");
  rb.print(m2.spe); // motor-returned speed of the right motor [rad/s]
  rb.write(" ");
  rb.print(triggerOn); // binarized trigger on/off signal [0 = off, 1 = on]
  rb.write(" ");
  rb.println(triggerVal); // raw trigger pin readings [0-1023, corresponding to 0-3.3v]
  //  rb.print("\n");
  // Print adc into RingBuf.
  //  rb.println(adc);
  if (rb.getWriteError())
  {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return; //break;
  }

}


//**************Plot Data*****************//

void plot_controller_data()
{
  // double tempVal = analogRead(A8);
  // Serial.println(m1.spe);
  // Serial.print(" ");
  // Serial.print(m2.pos);
  // Serial.print(" ");
  // Serial.println(stopAssistanceInSwingAtNeturalKnee);
  //  Serial.print(" ");
  //  Serial.println(m2.torque);
  // Serial.println(imu.LTx);
  //    Serial.print(m1.pos/3.14*180);
  //    Serial.print(" ");
  //    Serial.print(m2.pos/3.14*180);
  //    Serial.print(" ");
  //    Serial.println(imu.LTx);
  // for (int i = 0; i < 3; i++)
  // {
  //   Serial.print(controller.sineAssistanceExtensionParameterList[0][i]);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  // Serial.print(Cur_command_L);
  // Serial.print(" ");
  // Serial.print(controller.T_command_L/0.232);
  // Serial.print(" ");
  // Serial.println(m1.iq_A);
  // Serial.println(triggerVal);

}


void initial_CAN2()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(100);
  Serial.println("Can bus setup done...");

  // motor 1
  m1.initial_CAN();
  delay(100);
  m1.exit_control_mode();
  delay(200);
  m1.exit_control_mode();
  delay(200);
  m1.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(200);
  // motor 2
  m2.initial_CAN();
  delay(100);
  m2.exit_control_mode();
  delay(200);
  m2.exit_control_mode();
  delay(200);
  m2.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m2.set_origin();
  delay(200);
  receive_CAN_data();
  delay(200);
}
void receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    int id = msgR.buf[0];
    // Serial.print(msgR.id, HEX );
    if (id == Motor_ID1)
    {
      m1.unpack_reply(msgR);
    }
    if (id == Motor_ID2)
    {
      m2.unpack_reply(msgR);
    }
  }
}

void reset_motor_angle()
{
  // TODO: check if this is correct!
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  m2.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);

}
