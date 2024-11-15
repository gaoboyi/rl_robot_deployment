//serial_imu.cpp
#include "serial_parse.h"

#define IMU_SERIAL   "/dev/ttyUSB0"
// #define BAUD         (115200)
#define BAUD         (1500000)
// #define BAUD         (460800)
#define BUF_SIZE     1024

ros::Publisher IMU_pub;

static int frame_rate;
static int frame_count;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}



int main(int argc, char** argv)
{
	int rev = 0;
	ros::init(argc, argv, "serial_imu");
	ros::NodeHandle n;

	IMU_pub = n.advertise<sensor_msgs::Imu>("/imu", 20);

	serial::Serial sp;

	serial::Timeout to = serial::Timeout::simpleTimeout(10);

	sp.setPort(IMU_SERIAL);

	sp.setBaudrate(BAUD);

	sp.setTimeout(to);



	signal(SIGALRM,timer);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
    
	if(sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
	}
	else
	{
		return -1;
	}
	sp.flushInput(); // 清空接收缓冲区
	sp.flushOutput(); // 清空发送缓冲区	
	alarm(1);
	 
	ros::Rate loop_rate(500);

  	// Send_CMD_LONG(14,101,0,4,0,0,0); //设置IMU坐标系，默认为101，前右下
	// for (int i = 0; i < 10; i++)
	// sp.write((uint8_t *)&data_cmd_long,sizeof(data_cmd_long));


	Send_CMD_LONG(3,1,0,0,0,0,0); //获取AHRS数据流
	for (int i = 0; i < 10; i++)
	sp.write((uint8_t *)&data_cmd_long,sizeof(data_cmd_long));

	Send_CMD_LONG(5,0,0,0,0,0,0);//保存到flash
	for (int i = 0; i < 3; i++)
	sp.write((uint8_t *)&data_cmd_long,sizeof(data_cmd_long));

	Send_CMD_LONG(3,1,0,0,0,0,0); //再次获取AHRS数据流
	for (int i = 0; i < 10; i++)
	sp.write((uint8_t *)&data_cmd_long,sizeof(data_cmd_long));
    sp.flushInput(); // 只清除接收缓存
	while(ros::ok())
	{
		size_t num = sp.available();
		if(num!=0)
		{
			uint8_t buffer[BUF_SIZE]; 
	
			if(num > BUF_SIZE)
				num = BUF_SIZE;
			
			num = sp.read(buffer, num);
			if(num > 0)
			{
				for (int i = 0; i < num; i++)
				{
					imu_rx(buffer[i]);
				}
			}
		}
		loop_rate.sleep();
	}
    
	sp.close();
 
	return 0;
}






