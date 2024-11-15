
//
// Created by hang on 5/21/24.
//

#pragma once

//legged_base
#include <legged_hw/LeggedHW.h>

// standard include
#include <iostream>
#include <fstream>
#include <ostream>
#include <memory.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <time.h>
#include <iomanip>
#include <cstdio>
// msgs
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <controller_manager_msgs/SwitchController.h>
// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include "Types.h"
#include "TimeDebug.h"  //计算函数运行时间


namespace legged
{

  const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

  struct Ti5MotorData
  {
    double pos_, vel_, tau_;                  // state
    double pos_des_, vel_des_, kp_, kd_, ff_; // command
  };

  struct Ti5ImuData
  {
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
  };

  class Ti5HW : public LeggedHW
  {
  public:
    Ti5HW()
    {
    }
    ~Ti5HW()
    {
      std::cout << "~Ti5HW_END" << std::endl;
    }

    const double motor_constants[12] = {51*0.118, 51*0.118, 51*0.175, 51*0.175, 51*0.096, 51*0.089, 
                                        51*0.118, 51*0.118, 51*0.175, 51*0.175, 51*0.096, 51*0.089};
    float leg3_kp_scale, leg3_kd_scale, 
          leg4_kp_scale, leg4_kd_scale, 
          leg5_kp_scale, leg5_kd_scale;

    /** \brief Get necessary params from param server. Init hardware_interface.
     *
     * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
     * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
     *
     * @param root_nh Root node-handle of a ROS node.
     * @param robot_hw_nh Node-handle for robot hardware.
     * @return True when init successful, False when failed.
     */
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    /** \brief Communicate with hardware. Get data, status of robot.
     *
     * Call @ref Gsmp_LEGGED_SDK::UDP::Recv() to get robot's state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /** \brief Comunicate with hardware. Publish command to robot.
     *
     * Propagate joint state to actuator state for the stored
     * transmission. Limit cmd_effort into suitable value. Call @ref Gsmp_LEGGED_SDK::UDP::Recv(). Publish actuator
     * current state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

  private:

    bool setupJoints();

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    bool setupImu();

    bool setupContactSensor(ros::NodeHandle &nh);
    
    // 定义电机的扭矩常数数组
    bool send_flag = false;
    ros::Subscriber imu_sub;
    bool use_imu_topic = false;
    
    Ti5MotorData joint_data_[12]{};
    Ti5ImuData imu_data_{};
    bool contact_state_[4]{};
    int contact_threshold_{};

    double readPosArrayBeforeConversion_[12];
    double readVelArrayBeforeConversion_[12];
    double readTauArrayBeforeConversion_[12];
    double readPosArrayAfterConversion_[12];
    double readVelArrayAfterConversion_[12];
    double readTauArrayAfterConversion_[12];

    uint64_t hs = 0;
    vector_t motor_pos_feedback_{12};
    vector_t motor_vel_feedback_{12};
    vector_t motor_tau_feedback_{12};
    vector_t joint_planned_torque_{12};
    ros::Subscriber odom_sub_;
    ros::Publisher motorPosPublisher_;
    ros::Publisher motorVelPublisher_;
    ros::Publisher motorTorquePublisher_;

    tf::TransformListener *listener_;

    // void OdomCallBack(const sensor_msgs::Imu::ConstPtr &odom)
    // {
    //   yesenceIMU_ = *odom;
    // }

    const  std::vector<int> direction_motor{-1, 1, -1,
                                       1, -1, 1,
                                       -1, 1,  1,
                                       -1, 1,  1};
    // const std::vector<int> direction_motor{-1, 1, 1, -1, -1, -1, 1, -1, 1, 1};

    float bias_motor[12] = { 0, 0, 0.523599, -1.047197, 0.523599, 0, 0, 0.523599, -1.047197, 0.523599};

  };

} // namespace legged