# ROS串口例程

本文档介绍如何在ROS下读取IMU的数据，并提供了c++语言例程代码，通过执行ROS命令，运行相应的节点，就可以看到打印到终端上的信息。

* 测试环境：Ubuntu20.04

* ROS版本：ROS_noetic


* 测试设备：imu618

## 安装 ROS serial 软件包，

安装 ROS serial 软件包，本例程依赖 ROS 提供的 serial 包实现串口通信。
首先执行如下命令，下载安装 serial 软件包：
sudo apt-get update
sudo apt-get install ros-noetic-rosserial


如果出现报错，如pkg was interrupted, you must manually run 'sudo dpkg --configure -a' to correct the problem，则进行以下命令解决：
sudo dpkg --configure -a
sudo apt update



注意不同的ros版本安装ROS serial 的命令不一样

1. Kinetic版本ros（适用于Ubuntu 16.04）

   ```bash
   sudo apt-get update
   sudo apt-get install ros-kinetic-rosserial
   ```

2. Melodic版本ros（适用于Ubuntu 18.04）

   ```bash
   sudo apt-get update
   sudo apt-get install ros-melodic-rosserial
   ```

## 安装USB-UART驱动

Ubuntu 认不需要安装串口驱动。将调试版连接到电脑上时，会自动识别设备。识别成功后，会在dev目录下出现一个对应的设备:ttyUSBx

检查USB-UART设备是否被Ubantu识别：

1. 打开终端，输入`ls /dev`,先查看已经存在的串口设备。
2. 查看是否已经存在  ttyUSBx 这个设备文件，便于确认对应的端口号。
4. 接下来插入USB线，连接调试板，然后再次执行`ls /dev`。 dev目录下多了一个设备`ttyUSB0`：

```shell
linux@ubuntu:~$ ls /dev
.....
hpet             net           tty11     tty4   ttyS0      ttyUSB0    vhost-vsock
hugepages        null          tty12     tty40  ttyS1      udmabuf  vmci
......
```

4.打开USB设备的可执行权限：

```shell
   $ sudo chmod 777 /dev/ttyUSB0
```
## 设置串口低延迟
5.为了保证数据传输的实时性，需要设置串口为低延迟
```shell
  安装: sudo apt install setserial
 插入usb线后执行:  setserial /dev/ttyUSB0 low_latency
   ttyUSB0换成实际使用的串口
```

## 修改串口号与波特率
当前驱动代码默认波特率为115200，默认串口号为ttyUSB0。
如需要修改波特率与串口号，请在serial_imu.cpp文件下修改宏定义：

```shell
#define IMU_SERIAL   "/dev/ttyUSB0"
#define BAUD         (115200)
#define BUF_SIZE     1024
```
ROS驱动代码里的波特率需要与我们产品设备波特率保持一致，
产品设备的波特率请通过我们的上位机进行修改.

## 编译代码

cd ahrs_imu_ros1
catkin_make
source devel/setup.bash



###  查看 IMU 数据

1. 打开一个终端，执行：
roscore
回到ahrs_imu_ros1⽂件夹下 执⾏
source devel/setup.bash
执⾏启动 rosrun
rosrun serial_imu serial_imu

2. 打开新的终端:
source devel/setup.bash
输⼊命令查看 IMU 数据
rostopic echo IMU_data

```shell
header: 
  seq: 2102
  stamp: 
    secs: 1712122898
    nsecs: 699805721
  frame_id: "imu"
orientation: 
  x: -0.0010773062987748604
  y: 0.016450232956337168
  z: -0.10690676696280528
  w: 0.9941323716811465
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: 0.00010886170027006417
  y: 0.0020122985332117972
  z: -0.000609063357935883
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: -0.31544684584792704
  y: -0.04979020018458832
  z: 9.545999998950958
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
默认输出的线加速度(linear acceleration)单位为m/s^2，
输出角加速度(angular velocity)单位为rad/s，
输出方向(orientation)为 四元数(Quaternion) 


3. 使⽤rviz 查看 IMU 数据
⾸先安装 imu_tool
sudo apt-get install ros-noetic-imu-tools
打开 rviz，选择 Fixed_Frame 为 imu，Add 添加 imu，且 Topic 为/IMU_data，则可以显
示三轴姿态。

###  如何修改坐标系

默认坐标系为前右下，需要修改坐标系，需要在serial_imu.cpp文件下代码初始化地方添加如下代码：
```shell
Send_CMD_LONG(14,101,0,4,0,0,0); //设置IMU坐标系，默认为101，前右下
for (int i = 0; i < 10; i++)
sp.write((uint8_t *)&data_cmd_long,sizeof(data_cmd_long));
```
其中Send_CMD_LONG函数里第二个参数101为默认朝向，第一个参数14与第三个参数4为固定值。
如需要修改朝向更改第二个参数即可，参数值参考下表：

```shell
101 +Ux +Uy +Uz 默认朝向
102 -Ux -Uy +Uz
103 -Uy +Ux +Uz
104 +Uy -Ux +Uz
105 -Ux +Uy -Uz
106 +Ux -Uy -Uz
107 +Uy +Ux -Uz
108 -Uy -Ux -Uz
109 -Uz +Uy +Ux
110 +Uz -Uy +Ux
111 +Uy +Uz +Ux
112 -Uy -Uz +Ux
113 +Uz +Uy -Ux
114 -Uz -Uy -Ux
115 -Uy +Uz -Ux
116 +Uy -Uz -Ux
117 -Ux +Uz +Uy
118 +Ux -Uz +Uy
119 +Uz +Ux +Uy
120 -Uz -Ux +Uy
121 +Ux +Uz -Uy
122 -Ux -Uz -Uy
123 -Uz +Ux -Uy
124 +Uz -Ux -Uy
```

###  如何查看发布topic的频率

输入以下命令可以查看topic频率
```shell
rostopic hz /IMU_data
```

输入命令后可以看到数据频率在100hz左右
```shell
average rate: 100.510
	min: 0.0098s max: 0.0102s std dev: 0.02638s window: 1400
average rate: 100.469
	min: 0.0098s max: 0.0102s std dev: 0.02639s window: 1504
```

###  录制rosbag并回放

1.录制rosbag
打开串口后，在新的终端中输入以下命令
```shell
rosbag record /IMU_data
```
当完成所需数据的录制后，按Ctrl+C终止rosbag record命令，rosbag文件将自动保存。

2.回放rosbag

 在新的终端窗口中，使用rosbag play命令回放之前录制的rosbag文件。例如，如果您要回放名为my_bag.bag的文件，执行：
```shell
rosbag play my_bag.bag
```

在另一个终端中，可以使用rostopic echo或rqt等工具监控回放的话题数据，确保数据正确回放,输入以下命令:
```shell
rostopic echo /IMU_data
```
在回放期间，相应的ROS节点应能接收到回放的数据，如同实时数据一样。观察您的应用程序或可视化工具是否正确响应这些数据。
按Ctrl+C终止rosbag play命令，停止回放



























 

