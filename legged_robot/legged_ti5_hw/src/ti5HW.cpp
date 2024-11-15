#include "ti5HW.h"
#include "can_hw.h"
#include "utilities.h"
#include <chrono>
namespace legged
{
  Timer timer_;
  bool Ti5HW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {  
    listener_ = new tf::TransformListener(root_nh, ros::Duration(5.0), true);  //创建一个tf::TransformListener对象，并传递root_nh、持续时间和线程安全性参数进行初始化
    /*
        关节/imu话题发布
    */
    motorPosPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("data_analysis/motor_pos", 1);
    motorVelPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("data_analysis/motor_vel", 1);
    motorTorquePublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("data_analysis/motor_torque", 1);
    motor_pos_feedback_.setZero();
    motor_vel_feedback_.setZero();
    motor_tau_feedback_.setZero();
    joint_planned_torque_.setZero();

    /*
        硬件通讯协议初始化
    */
    int attempt; 
    for (attempt=0; attempt<10; attempt++) {
        if(!init_can()){
            std::cerr << "Error: init failed, try again." << std::endl;
            continue;  // 或其他错误处理
        }
        else{
            std::cout<<"init success"<<std::endl;
            break;
        }
    }
    if (attempt == 10) {
        std::cerr << "Error: Maximum attempts reached, initialization failed." << std::endl;
        return -1;  // 退出程序
    }
      send_fix_body_command();
    /*
        IMU话题订阅初始化
    */
    root_nh.getParam("use_imu_topic", use_imu_topic);
    if(use_imu_topic){
      std::cout<<"use imu topic"<<std::endl;
      imu_sub = root_nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &Ti5HW::imuCallback, this);
    }

    root_nh.getParam("leg3_kp_scale", leg3_kp_scale);
    root_nh.getParam("leg3_kd_scale", leg3_kd_scale);
    root_nh.getParam("leg4_kd_scale", leg4_kd_scale);
    root_nh.getParam("leg4_kp_scale", leg4_kp_scale);
    root_nh.getParam("leg5_kd_scale", leg5_kd_scale);
    root_nh.getParam("leg5_kp_scale", leg5_kp_scale);
    /*
        LeggedHW初始化
    */
    if (!LeggedHW::init(root_nh, robot_hw_nh))
    {
      printf("flase_bc_leggedHW::init\n");
      return false;
    }

    /*
        关节 IMU 接触传感器配置初始化
    */
    setupJoints();
    setupImu();
    
    DEBUG_TIMER_START(timer_);
    return true;
  }

  void Ti5HW::read(const ros::Time &time, const ros::Duration &period)
  { 

    /*
    元生IMU数据
    */ 
    if(use_imu_topic){
      ros::spinOnce();  // 处理一次IMU回调队列中的回调函数
    }

    /*
        CAN通讯处理后的关节数据返回值进行单位换算，并发送给控制器   read from hw:1-6Right 7-12Left  controller:1-6 Left 7-12Right
    */
    for (int i = 0; i < 12; ++i) {
      joint_data_[i].pos_ = rv_motor_msg[i].angle_actual_rad * direction_motor[i];
      joint_data_[i].vel_ = rv_motor_msg[i].speed_actual_rad * direction_motor[i];
      joint_data_[i].tau_ = rv_motor_msg[i].current_actual_float * motor_constants[i] * direction_motor[i];
      motor_pos_feedback_(i) = joint_data_[i].pos_;
      motor_vel_feedback_(i) = joint_data_[i].vel_;
      motor_tau_feedback_(i) = joint_data_[i].tau_;

      readPosArrayBeforeConversion_[i] = joint_data_[i].pos_; // Apply direction correction
      readVelArrayBeforeConversion_[i] = joint_data_[i].vel_;
      readTauArrayBeforeConversion_[i] = joint_data_[i].tau_;
    }

    DEBUG_TIMER_DURATION(timer_, "joint_data_");
    motorTorquePublisher_.publish(createFloat64MultiArrayFromVector(motor_tau_feedback_));
    motorPosPublisher_.publish(createFloat64MultiArrayFromVector(motor_pos_feedback_));
    motorVelPublisher_.publish(createFloat64MultiArrayFromVector(motor_vel_feedback_));

    DEBUG_TIMER_DURATION(timer_, "publish");
    // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto &name : names)
    {
      HybridJointHandle handle = hybridJointInterface_.getHandle(name);
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(3.1415);
      handle.setKp(0.);
    }
    DEBUG_TIMER_DURATION(timer_, " Set feedforward and velocity cmd to zero");
  }


  void Ti5HW::write(const ros::Time &time, const ros::Duration &period)
  { 
    /*
        进行一次CAN通讯
    */
    for (int i = 0; i < 12; ++i)  //12个电机
    {   
        send_cmd[i].pos_des_ = joint_data_[i].pos_des_  * direction_motor[i];  //发送命令的位置目标
        send_cmd[i].vel_des_ = joint_data_[i].vel_des_  * direction_motor[i];                  //发送命令的速度目标 好像没有设定速度目标
        if(i == 3 || i == 9){
          send_cmd[i].kp_ = joint_data_[i].kp_ * leg4_kp_scale;
            send_cmd[i].kd_ = joint_data_[i].kd_ * leg4_kd_scale;
          }
        else if( i == 4 || i == 10)
        {
          send_cmd[i].kp_ = joint_data_[i].kp_ * leg5_kp_scale;
          send_cmd[i].kd_ = joint_data_[i].kd_ * leg5_kd_scale;
        }
        else if(i == 2 || i == 8)
        {
          send_cmd[i].kp_ = joint_data_[i].kp_ * leg3_kp_scale;
          send_cmd[i].kd_ = joint_data_[i].kd_ * leg3_kd_scale;
        }
        else{
          send_cmd[i].kp_ = joint_data_[i].kp_;
          send_cmd[i].kd_ = joint_data_[i].kd_;
        }
        send_cmd[i].ff_ = joint_data_[i].ff_ * direction_motor[i];                             //发送前馈力矩
    }
    sendCanCommand(12, (YKSMotorData *)send_cmd);
  }


  bool Ti5HW::setupJoints()
  {
    for (const auto &joint : urdfModel_->joints_)
    {
      int leg_index=0;
      int joint_index=0; 
      int index=0;
      if (joint.first.find("leg_l") != std::string::npos)
      {
        leg_index = 1;
        index+=leg_index*6;
      }
      else if (joint.first.find("leg_r") != std::string::npos) 
      {
        leg_index = 0;
        index+=leg_index*6;
      }

      else
        continue;  // 不是左腿或右腿的关节，跳过

      // 根据关节名称确定关节在腿上的索引
      if (joint.first.find("1_joint") != std::string::npos)
        joint_index = 0;
      else if (joint.first.find("2_joint") != std::string::npos)
        joint_index = 1;
      else if (joint.first.find("3_joint") != std::string::npos)
        joint_index = 2;
      else if (joint.first.find("4_joint") != std::string::npos)
        joint_index = 3;
      else if (joint.first.find("5_joint") != std::string::npos)
        joint_index = 4;
      else if (joint.first.find("6_joint") != std::string::npos)
        joint_index = 5;
      else
        continue;  // 不是1-6号关节的关节，跳过

      // 计算该关节在joint_data_数组中的索引
      index+=joint_index;
      ROS_INFO("joint index = %d", index);

      // 创建JointStateHandle对象并注册到jointStateInterface_
      hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                        &joint_data_[index].tau_);
      jointStateInterface_.registerHandle(state_handle);

      // 创建HybridJointHandle对象并注册到hybridJointInterface_
      hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                            &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                            &joint_data_[index].kd_, &joint_data_[index].ff_));
    } 
    return true;
  }

  void Ti5HW::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  { 
    // 处理接收到的IMU数据
    imu_data_.ori[0] = msg->orientation.x;
    imu_data_.ori[1] = msg->orientation.y;
    imu_data_.ori[2] = msg->orientation.z;
    imu_data_.ori[3] = msg->orientation.w;

    imu_data_.angular_vel[0] = msg->angular_velocity.x;
    imu_data_.angular_vel[1] = msg->angular_velocity.y;
    imu_data_.angular_vel[2] = msg->angular_velocity.z;

    imu_data_.linear_acc[0] = msg->linear_acceleration.x;
    imu_data_.linear_acc[1] = msg->linear_acceleration.y;
    imu_data_.linear_acc[2] = msg->linear_acceleration.z;
  }

  bool Ti5HW::setupImu()
  {
     imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
        "base_imu", "base_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
        imu_data_.linear_acc, imu_data_.linear_acc_cov));
    imu_data_.ori_cov[0] = 0.0012;
    imu_data_.ori_cov[4] = 0.0012;
    imu_data_.ori_cov[8] = 0.0012;

    imu_data_.angular_vel_cov[0] = 0.0004;
    imu_data_.angular_vel_cov[4] = 0.0004;
    imu_data_.angular_vel_cov[8] = 0.0004;

    return true;
  }
} // namespace legged