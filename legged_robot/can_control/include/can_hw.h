#pragma once

#include <cstdint>
#include <vector>
#include "controlcan.h"
#include "math_ops.h"

typedef struct
{
  uint16_t angle_actual_int;
  uint16_t angle_desired_int;
  int16_t speed_actual_int;
  int16_t speed_desired_int;
  int16_t current_actual_int;
  int16_t current_desired_int;
  float speed_actual_rad;
  float speed_desired_rad;
  float angle_actual_rad;
  float angle_desired_rad;
  uint16_t motor_id;
  uint8_t temperature;
  uint8_t error;
  float angle_actual_float;
  float speed_actual_float;
  float current_actual_float;
  float angle_desired_float;
  float speed_desired_float;
  float current_desired_float;
  float power;
  uint16_t acceleration;
  uint16_t linkage_KP;
  uint16_t speed_KI;
  uint16_t feedback_KP;
  uint16_t feedback_KD;
} OD_Motor_Msg;

extern OD_Motor_Msg rv_motor_msg[12];
typedef struct 
{
    double pos_, vel_, tau_;                   // state 位置 速度 力矩
    double pos_des_, vel_des_, kp_, kd_, ff_;  // command 期望位置 期望速度 kp kd 前馈（feedforward）
} YKSMotorData;
// extern YKSMotorData motorDate_recv[12];

YKSMotorData send_cmd[6];

double findMax(double arr[], int size);
int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4]);
void toIntArray(int number, int *res, int size);
int16_t combineBytes(uint8_t low, uint8_t high);
// This function use in can send_data convert.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_data_convert(VCI_CAN_OBJ *message, float kp, float kd, float pos, float spd, float tor);
bool send_fix_body_command();
void sendCommand( int s_id, int numOfActuator,YKSMotorData *mot_data);
//void sendCanCommand(int numOfActuator, int s_id, YKSMotorData *mot_data);
void sendCanCommand(int numOfActuator, YKSMotorData *mot_data);
bool init_can();

