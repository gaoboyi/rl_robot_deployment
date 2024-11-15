#ifndef __SERIAL_PARSE_H_
#define	__SERIAL_PARSE_H_

#include <stdint.h>
#include <string.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <signal.h>



#define IMU_PARSE_STATE_SYNC1_ID      0xAA
#define IMU_PARSE_STATE_SYNC2_ID      0x55


enum 
{
     UART_STREAM_MODE_SEND_AHRS=1,
     UART_COMMAND_MODE=100,
};


typedef enum {
    IMU_PARSE_STATE_WAIT_SYNC1=0,
    IMU_PARSE_STATE_WAIT_SYNC2,
    IMU_PARSE_STATE_WAIT_ID1,
    IMU_PARSE_STATE_WAIT_ID,
    IMU_PARSE_STATE_WAIT_LENGTH1,
    IMU_PARSE_STATE_WAIT_LENGTH2,
    IMU_PARSE_STATE_PAYLOAD,
    IMU_PARSE_STATE_CHECK1,
    IMU_PARSE_STATE_CHECK2,
    IMU_PARSE_STATE_CHECK3,
    IMU_PARSE_STATE_CHECK4
} imu_parse_state_t; 

typedef struct {
    volatile unsigned char state;
    volatile unsigned int count;
    volatile unsigned int id;
    volatile unsigned int length;
    volatile uint32_t check;
    volatile unsigned char id_temp;
    volatile unsigned char length_temp;
    volatile unsigned char check_temp1;
    volatile unsigned char check_temp2;
    volatile unsigned char check_temp3;
    volatile unsigned char check_temp4;

} ParseStruct;

#pragma pack(push) //保存对齐状态
#pragma pack(1)    //设定为1字节对齐

/**
 * 接收AHRS帧负载
 */
struct AHRS_PAYLOAD_9axies
{ 
  uint32_t time_us;
  float pitch;
  float roll;
  float yaw;
  float imu[7];
  float mag[3];

};

struct AHRS_PAYLOAD
{ 
  uint32_t time_us;
  float pitch;
  float roll;
  float yaw;
  float imu[7];

};

/**
 * 命令通用结构体
 */
struct  MULTI_LONG_CMD_STRUCT
{
uint8_t header1;    
uint8_t header2;
uint16_t id;
uint16_t length;
float param1;
float param2;
uint32_t param3;
uint32_t param4;
int32_t param5;
int32_t param6;
uint32_t check_crc;
}__attribute__((packed));

#pragma pack(pop)//恢复对齐状态
typedef union 
{ 
  struct AHRS_PAYLOAD_9axies AHRS_DATA_9axis;
  struct AHRS_PAYLOAD AHRS_DATA;
  unsigned char payload[1000]; 
}ParseUnion;

void imu_rx(unsigned char data);

extern ros::Publisher IMU_pub;
extern ros::Publisher MAG_pub;
extern struct  MULTI_LONG_CMD_STRUCT data_cmd_long;
void  Send_CMD_LONG(uint16_t cmd_id,float cm1,float cm2,uint32_t cm3,uint32_t cm4,int32_t cm5,int32_t cm6);
#endif

