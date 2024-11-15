// serial_demo.cpp

#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
//#include <robotmove.h>
#include <iostream>
#include <cstdint>
#include <thread>
#include <ros/ros.h>	//引入ros
#include "std_msgs/String.h"
#include "math.h"
#include "can_hw.h"


#define MESSAGE_FREQ 200
#define MESSAGE_FREQ_END 120

OD_Motor_Msg rv_motor_msg[12];

double next_velocitie[6] = {0};

double ori_velocitie[6] = {0};
double cha_velocitie[6] = {0};
double now_velocitie[6] = {0};

double next_position[6] = {0};	

double ori_position[6] = {0};
double cha_position[6] = {0};
int now_position[6] = {0};
char buffer[5120];

int r1[500] = {0};
int r2[500] = {0};
int r3[500] = {0};

int r4[500] = {0};
int r5[500] = {0};
int r6[500] = {0};


// 将字节数组中的连续两个字节组合为一个16位整数
int16_t combineBytes(uint8_t low, uint8_t high) {
    return (static_cast<int16_t>(high) << 8) | low;
}

int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4])
{
	std::int32_t result=0;
	for(int i=0;i<8;i++)
		result=(result << 8) | hexArray[i];
	if(result>0x7FFFFFFF)
		result -= 0x100000000;

	return static_cast<std::int32_t>(result);
}

void toIntArray(int number, int *res, int size)
{
	unsigned int unsignedNumber = static_cast<unsigned int>(number);
	
	for (int i = 0; i < size; ++i)
	{
		res[i] = unsignedNumber & 0xFF; 
		unsignedNumber >>= 8;           
	}
}

void send_motor_data_convert(VCI_CAN_OBJ *message, int motor_id, float kp, float kd, float pos, float spd, float tor)
{	
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;
    if (kp > KP_MAXX)
        kp = KP_MAXX;       
    else if (kp < KP_MINX)
        kp = KP_MINX;
    if (kd > KD_MAXX)
        kd = KD_MAXX;
    else if (kd < KD_MINX)
        kd = KD_MINX;
    if (pos > POS_MAXX)
        pos = POS_MAXX;
    else if (pos < POS_MINX)
        pos = POS_MINX;
    if (spd > SPD_MAXX)
        spd = SPD_MAXX;
    else if (spd < SPD_MINX)
        spd = SPD_MINX;
    kp_int = float_to_uint(kp, KP_MINX, KP_MAXX, 12);
    kd_int = float_to_uint(kd, KD_MINX, KD_MAXX, 9);
    pos_int = float_to_uint(pos, POS_MINX, POS_MAXX, 16);
    spd_int = float_to_uint(spd, SPD_MINX, SPD_MAXX, 12);
    
    if (motor_id == 1 || motor_id == 2 || motor_id == 7 || motor_id ==8)
    {	
        if (tor > actuators[LSG_20_90_7090].tMaxX)
            tor = actuators[LSG_20_90_7090].tMaxX;
        else if (tor < actuators[LSG_20_90_7090].tMinX)
            tor = actuators[LSG_20_90_7090].tMinX;
        tor_int = float_to_uint(tor, actuators[LSG_20_90_7090].tMinX, actuators[LSG_20_90_7090].tMaxX, 12);
		// if(motor_id==1){
		// 	ROS_WARN("tor_int FF =%d",tor_int);
		// } 
    }
    else if (motor_id == 3 || motor_id == 4 || motor_id == 9 || motor_id ==10)
    {	
        if (tor > actuators[LSG_10_414].tMaxX)
            tor = actuators[LSG_10_414].tMaxX;
        else if (tor < actuators[LSG_10_414].tMinX)
            tor = actuators[LSG_10_414].tMinX;
        tor_int = float_to_uint(tor, actuators[LSG_10_414].tMinX, actuators[LSG_10_414].tMaxX, 12);
    }
    else if (motor_id == 5 || motor_id == 11)
    {	
        if (tor > actuators[LSG_17_80_6070_new].tMaxX)
            tor = actuators[LSG_17_80_6070_new].tMaxX;
        else if (tor < actuators[LSG_17_80_6070_new].tMinX)
            tor = actuators[LSG_17_80_6070_new].tMinX;
        tor_int = float_to_uint(tor, actuators[LSG_17_80_6070_new].tMinX, actuators[LSG_17_80_6070_new].tMaxX, 12);
    }
    else if (motor_id == 6 || motor_id == 12)
    {
        if (tor > actuators[LSG_14_70_5060].tMaxX)
            tor = actuators[LSG_14_70_5060].tMaxX;
        else if (tor < actuators[LSG_14_70_5060].tMinX)
            tor = actuators[LSG_14_70_5060].tMinX;
        tor_int = float_to_uint(tor, actuators[LSG_14_70_5060].tMinX, actuators[LSG_14_70_5060].tMaxX, 12);
    }
    message->Data[0] = 0x00 | (kp_int >> 7);                             // kp5
    message->Data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    message->Data[2] = kd_int & 0xFF;
    message->Data[3] = pos_int >> 8;
    message->Data[4] = pos_int & 0xFF;
    message->Data[5] = spd_int >> 4;
    message->Data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    message->Data[7] = tor_int & 0xff;

}

bool send_fix_body_command()
{
    std::vector<int> res; // 用于存储接收到的数据
    VCI_CAN_OBJ send[1]; // 用于存储要发送的CAN消息
    send[0].SendType = 0; // 设置发送类型为0sendCanCommand
    send[0].RemoteFlag = 0; // 设置远程标志为0
    send[0].ExternFlag = 0; // 设置扩展帧标志为0
    send[0].DataLen = 5; // 设置数据长度为5
	// ROS_INFO("numOfActuator:%d", numOfActuator);
    for (int i = 12; i < 29; i++) // 循环发送命令
    {
        // if(i >= 18 && i<=22) continue;
		send[0].ID = i + 1; // 设置CAN消息的ID
		int motor_id = i+1;
        if(motor_id == 15){
            send[0].Data[0] = 0x1E; // 设置数据字节0
            send[0].Data[1] = 0x00; // 设置数据字节1
            send[0].Data[2] = 0xAA; // 设置数据字节2
            send[0].Data[3] = 0xFD; // 设置数据字节3
            send[0].Data[4] = 0xFF; // 设置数据字节3
        }
        else{
            send[0].Data[0] = 0x1E; // 设置数据字节0
            send[0].Data[1] = 0x00; // 设置数据字节1
            send[0].Data[2] = 0x00; // 设置数据字节2
            send[0].Data[3] = 0x00; // 设置数据字节3
            send[0].Data[4] = 0x00; // 设置数据字节3
        }
	
        pthread_t threadid; // 声明线程ID变量
        int cc=3;
        while(cc--) 
        {
            if(VCI_Transmit(VCI_USBCAN2, 1, 0, send, 1)==1) std::cout<<"send can command "<<send[0].ID<<"  success"<<std::endl;
        }
    }
	return true;
}

////sendCommand,0,numOfActuator/2,mot_data
void sendCommand( int s_id, int numOfActuator,YKSMotorData *mot_data)
{
    std::vector<int> res; // 用于存储接收到的数据
    VCI_CAN_OBJ send[1]; // 用于存储要发送的CAN消息
    send[0].SendType = 0; // 设置发送类型为0sendCanCommand
    send[0].RemoteFlag = 0; // 设置远程标志为0
    send[0].ExternFlag = 0; // 设置扩展帧标志为0
    send[0].DataLen = 8; // 设置数据长度为8
	// ROS_INFO("numOfActuator:%d", numOfActuator);
    for (int i = s_id; i < s_id+numOfActuator; i++) // 循环发送命令
    {
        send[0].ID = i + 1; // 设置CAN消息的ID
		int motor_id = i+1;
		/*
			send_motor_data_convert
		*/
		// ROS_INFO("send_motor_data_convert s");
		send_motor_data_convert(&send[0], motor_id, mot_data[i].kp_, mot_data[i].kd_, mot_data[i].pos_des_, mot_data[i].vel_des_, mot_data[i].ff_); 
		// ROS_INFO("send_motor_data_convert e");
        pthread_t threadid; // 声明线程ID变量

		// ROS_INFO("VCI_Transmit s");
        if (VCI_Transmit(VCI_USBCAN2, s_id/6, 0, send, 1) == 1) // 如果发送成功
        {	
			// ROS_INFO("VCI_Transmit e");
            int cnt = 5; // 设置尝试接收次数为5
            int reclen = 0, ind = 0; // 声明接收消息长度和索引
            VCI_CAN_OBJ rec[3000]; // 用于存储接收到的CAN消息
			// ROS_INFO("VCI_Receive s");
            while ((reclen = VCI_Receive(VCI_USBCAN2, s_id/6, ind, rec, 3000, 100)) <= 0 && cnt) // 循环接收消息
                cnt--; // 减少尝试接收次数
			// ROS_INFO("VCI_Receive e");
            if (cnt == 0) // 如果尝试了5次仍未接收到消息
                std::cout << "ops! ID " << send[0].ID << " failed after try 5 times." <<std::endl;
            else
            {	//ROS_INFO("uint_to_float s");
			
                for (int j = 0; j < reclen; j++) // 遍历接收到的消息
                {
                    std::uint8_t hexArray[8] = {rec[j]. Data[7], rec[j].Data[6], rec[j].Data[5], 
					rec[j].Data[4],rec[j].Data[3],rec[j].Data[2],
					rec[j].Data[1],rec[j].Data[0]}; // 将接收到的数据转换为十六进制数组
					int pos_int = 0;
					int spd_int = 0;
					int cur_int = 0;
					// 解析位置、速度和电流数据
					pos_int = rec[j].Data[1] << 8 | rec[j].Data[2];
					spd_int = rec[j].Data[3] << 4 | (rec[j].Data[4] & 0xF0) >> 4;
					cur_int = (rec[j].Data[4] & 0x0F) << 8 | rec[j].Data[5];
					// if(motor_id == 1){
					// 	ROS_WARN("cur_int FF =%d",cur_int);
					// }
					// 根据电机ID的不同，使用不同的转换范围
					rv_motor_msg[i].angle_actual_rad = uint_to_float(pos_int, POS_MINX, POS_MAXX, 16);
					rv_motor_msg[i].speed_actual_rad = uint_to_float(spd_int, SPD_MINX, SPD_MAXX, 12);
					if (motor_id == 1 || motor_id == 2 || motor_id == 7 || motor_id ==8)
					{
						rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, actuators[LSG_20_90_7090].iMinX, actuators[LSG_20_90_7090].iMaxX, 12);
					}
					else if (motor_id == 3 || motor_id == 4 || motor_id == 9 || motor_id ==10)
					{
						rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, actuators[LSG_10_414].iMinX, actuators[LSG_10_414].iMaxX, 12);
					}
					else if (motor_id == 5 || motor_id == 11)
					{
						rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, actuators[LSG_17_80_6070_new].iMinX, actuators[LSG_17_80_6070_new].iMaxX, 12);
					}
					else if (motor_id == 6 || motor_id == 12)
					{
						rv_motor_msg[i].current_actual_float = uint_to_float(cur_int, actuators[LSG_14_70_5060].iMinX, actuators[LSG_14_70_5060].iMaxX, 12);
					}
                }
				
            }
        	// ROS_INFO("uint_to_float e");
            // exit(0); // 退出程序
        }
        else{
			ROS_ERROR("send can command fail");
            break; // 如果发送失败，则跳出循环
		}

		
			
    }
}

void sendCanCommand(int numOfActuator,YKSMotorData *mot_data)
{
	std::thread thread1(sendCommand,0,numOfActuator/2,mot_data);
	std::thread thread2(sendCommand,6,numOfActuator/2,mot_data);

	thread1.join();
	thread2.join();	
}

bool init_can()
{
	VCI_BOARD_INFO pInfo; 
	VCI_BOARD_INFO pInfo1[50];
    	
	int nDeviceType = 4;
	int nDeviceInd_1=0,nDeviceInd_2=1;
	int nCANInd = 0;

	int num = VCI_FindUsbDevice2(pInfo1);
	std::cout<<" I find "<<num<<" CAN she ben aaaaaa"<<std::endl;
/*
    if(pInfo1[0].str_Serial_Num[10]=='8')
    {
        pInfo = pInfo1[0];
        pInfo1[0] = pInfo1[1];
        pInfo1[1] = pInfo;

    }
    */
    //pInfo1[1].str_Serial_Num[20]='31F10001EB8\n';
    for(int i=0;i<num;i++)
    {
        for(int j=0;j<20;j++)
        std::cout<<pInfo1[i].str_Serial_Num[j];
        std::cout<<std::endl;
    }

    DWORD dwRel;
	VCI_INIT_CONFIG vic;
	vic.AccCode = 0x80000008;
	vic.AccMask = 0xFFFFFFFF;
	vic.Filter = 1;
	vic.Timing0 = 0x00;
	vic.Timing1 = 0x14;
	vic.Mode = 0;
    
	dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd_1, 0);
    if (dwRel != 1)
	{
		std::cout << "open-fail:" << dwRel << std::endl;
		return FALSE;
    }
    else
		std::cout << "open success:1";

	dwRel = VCI_InitCAN(nDeviceType, nDeviceInd_1, nCANInd, &vic);
	if (dwRel != 1)
	{   
		std::cout << "init-fail:" << dwRel << std::endl;
        VCI_CloseDevice(nDeviceType, nDeviceInd_1);
        return FALSE;
    }
    else
		std::cout << "initsuccess:" << dwRel << std::endl;

    if (VCI_StartCAN(VCI_USBCAN2, nDeviceInd_1, 0) != 1)
    {
		std::cout << "start-fail:" << dwRel << std::endl;
        VCI_CloseDevice(VCI_USBCAN2, nDeviceInd_1);
		return FALSE;
    }
    else
		std::cout << "startsuccess:1" << std::endl;





	dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd_2, 0);
    if (dwRel != 1)
	{
		std::cout << "open-fail:" << dwRel << std::endl;
		return FALSE;
    }
    else
		std::cout << "open success:1";

	dwRel = VCI_InitCAN(nDeviceType, nDeviceInd_2, nCANInd, &vic);
	if (dwRel != 1)
	{   
		std::cout << "init-fail:" << dwRel << std::endl;
        VCI_CloseDevice(nDeviceType, nDeviceInd_2);
        return FALSE;
    }
    else
		std::cout << "initsuccess:" << dwRel << std::endl;

    if (VCI_StartCAN(VCI_USBCAN2, nDeviceInd_2, 0) != 1)
    {
		std::cout << "start-fail:" << dwRel << std::endl;
        VCI_CloseDevice(VCI_USBCAN2, nDeviceInd_2);
		return FALSE;
    }
    else
		std::cout << "startsuccess:222222" << std::endl;


	return true;
}