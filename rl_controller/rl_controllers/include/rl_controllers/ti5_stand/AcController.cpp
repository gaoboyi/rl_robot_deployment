#include "rl_controllers/AcController.h"
#include <pluginlib/class_list_macros.hpp>
#include "rl_controllers/RotationTools.h"
#include <algorithm>
#include <stdlib.h>
#include <ros/package.h>  // 添加这个头文件

namespace legged
{

  void AcController::handleWalkMode()
  {
    // compute observation & actions
    if (std::cout.fail())
    {
      std::cerr << "std::cout is in a bad state!" << std::endl;
      // 可能需要清除错误状态
      std::cout.clear();
    }
    if (loopCount_ % robotCfg_.controlCfg.decimation == 0)
    { 

      // std::cout << "loopCount_:" << loopCount_ << std::endl;

      // std::cout << "robotCfg_.controlCfg.decimation:" << robotCfg_.controlCfg.decimation << std::endl;
      computeObservation();
      computeActions();
      // limit action range
      scalar_t actionMin = -robotCfg_.clipActions;
      scalar_t actionMax = robotCfg_.clipActions;
      std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                     [actionMin, actionMax](scalar_t x)
                     { return std::max(actionMin, std::min(actionMax, x)); });
    }

    vector_t output_torque(actionsSize_);
    // set action
    for (int i = 0; i < actionsSize_; i++)
    {
      std::string partName = hybridJointHandles_[i].getName();
      scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i);
      double stiffness = 1.0 * robotCfg_.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
      double damping = robotCfg_.controlCfg.damping[partName]; // 根据关节名称获取阻尼
      // std::cout << "joint_name:" << partName << "kp:" << stiffness << " kd:" << damping << std::endl;
      hybridJointHandles_[i].setCommand(pos_des, 0, stiffness, damping, 0);
      // std::cout << "action:" << actions_[i] << std::endl;
      lastActions_(i, 0) = actions_[i];
    }
  }

  bool AcController::loadModel(ros::NodeHandle &nh)
  {
    std::string policyFilePath;
    reloadParameters(); //load robot_type_amp.yaml again 
    if(!loadRLCfg(nh)){
      ROS_ERROR_STREAM("loadRLCfg from param server, some error occur!");
    }
    else{
      ROS_INFO_STREAM("loadRLCfg from param server success");
    }
    if (!nh.getParam("/policyFile", policyFilePath))
    {
      ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
      return false;
    }
    policyFilePath_ = policyFilePath;
    ROS_INFO_STREAM("Load MNN model from path : " << policyFilePath);

    // Create the MNN interpreter instance
    interpreter_ = std::unique_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(policyFilePath.c_str()));
    if (!interpreter_)
    {
      ROS_ERROR_STREAM("Failed to create MNN interpreter!");
      return false;
    }

    // Create session with default options
    MNN::ScheduleConfig config;
    config.numThread = 1; // Set the number of threads
    MNN::BackendConfig backendConfig;
    backendConfig.power = MNN::BackendConfig::Power_High;
    config.backendConfig = &backendConfig;

    session_ = interpreter_->createSession(config);
    if (!session_)
    {
      ROS_ERROR_STREAM("Failed to create MNN session!");
      return false;
    }
    // Get input info
    inputNames_.clear();
    outputNames_.clear();
    inputShapes_.clear();
    auto inputTensor = interpreter_->getSessionInputAll(session_);

    // 获取输入名称
    for (const auto &pair : inputTensor)
    {
      inputNames_.push_back(pair.first);
      std::vector<int> shape = pair.second->shape();
      inputShapes_.push_back(shape);
      ROS_INFO_STREAM("Input Shape: [");
      for (size_t j = 0; j < shape.size(); ++j)
      {
          std::cout << shape[j] << (j != shape.size() - 1 ? ", " : "");
      }
      std::cout << "]" << std::endl;
}
    
    // Get output info
    outputNames_.clear();
    outputShapes_.clear();
    auto outputTensor = interpreter_->getSessionOutputAll(session_);
    for (const auto &pair : outputTensor)
    {
      outputNames_.push_back(pair.first);
      std::vector<int> shape = pair.second->shape();
      outputShapes_.push_back(shape);
      ROS_INFO_STREAM("Output Shape: [");
      for (size_t j = 0; j < shape.size(); ++j)
      {
          std::cout << shape[j] << (j != shape.size() - 1 ? ", " : "");
      }
      std::cout << "]" << std::endl;
    }
ROS_INFO_STREAM("MNN model loaded successfully!");
    return true;
  }

  void AcController::reloadParameters()
  {
      // 获取参数
      std::string robot_type;
      if (!ros::param::get("/robot_type", robot_type))
      { 
          ROS_ERROR("Failed to get parameter: robot_type");
          return;
      }

      // 构建参数文件路径
      std::string package_path = ros::package::getPath("rl_controllers");
      std::string param_file = package_path + "/config/" + robot_type + "_amp.yaml";

      // 构建命令
      std::string command = "rosparam load " + param_file;

      // 执行命令
      int result = system(command.c_str());
      if (result == 0)
      {
          ROS_INFO("Parameters reloaded successfully.");
      }
      else
      {
          ROS_ERROR("Failed to reload parameters.");
      }
  }

  bool AcController::loadRLCfg(ros::NodeHandle &nh)
  {
    RLRobotCfg::InitState &initState = robotCfg_.initState;
    RLRobotCfg::ControlCfg &controlCfg = robotCfg_.controlCfg;
    RLRobotCfg::ObsScales &obsScales = robotCfg_.obsScales;

    int error = 0;
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l1_joint", initState.leg_l1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l2_joint", initState.leg_l2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l3_joint", initState.leg_l3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l4_joint", initState.leg_l4_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l5_joint", initState.leg_l5_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l6_joint", initState.leg_l6_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r1_joint", initState.leg_r1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r2_joint", initState.leg_r2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r3_joint", initState.leg_r3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r4_joint", initState.leg_r4_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r5_joint", initState.leg_r5_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r6_joint", initState.leg_r6_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness", controlCfg.stiffness));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping", controlCfg.damping));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", controlCfg.actionScale));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/cycle_time", controlCfg.cycle_time));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robotCfg_.clipObs));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robotCfg_.clipActions));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/quat", obsScales.quat));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/num_hist", numHist_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/mode/sw_mode", sw_mode_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/mode/cmd_threshold", cmd_threshold_));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));

    actions_.resize(actionsSize_);
    std::cout << "actionsSize_  " << actionsSize_ << std::endl;
    std::cout << "observationSize_  " << observationSize_ << std::endl;
    observations_.resize(observationSize_ * numHist_);
    std::fill(observations_.begin(), observations_.end(), 0.0f);
    command_.x = 0;
    command_.y = 0;
    command_.yaw = 0;
    baseLinVel_.setZero();
    basePosition_.setZero();
    std::vector<scalar_t> defaultJointAngles{
        robotCfg_.initState.leg_l1_joint, robotCfg_.initState.leg_l2_joint, robotCfg_.initState.leg_l3_joint,
        robotCfg_.initState.leg_l4_joint, robotCfg_.initState.leg_l5_joint, robotCfg_.initState.leg_l6_joint,
        robotCfg_.initState.leg_r1_joint, robotCfg_.initState.leg_r2_joint, robotCfg_.initState.leg_r3_joint,
        robotCfg_.initState.leg_r4_joint, robotCfg_.initState.leg_r5_joint, robotCfg_.initState.leg_r6_joint};
    lastActions_.resize(actionsSize_);
    lastActions_.setZero();
    const int inputSize = numHist_ * observationSize_;
    proprioHistoryBuffer_.resize(inputSize);
    proprioHistoryBuffer_.setZero();
    defaultJointAngles_.resize(actuatedDofNum_);
    for (int i = 0; i < actuatedDofNum_; i++)
    {
      defaultJointAngles_(i) = defaultJointAngles[i];
    }

    return (error == 0);
  }

  void AcController::computeActions()
  {
// 运行模型前的输入数据设置
    for (const auto &inputName : inputNames_)
    {
      auto inputTensor = interpreter_->getSessionInput(session_, inputName.c_str());
      // interpreter_->resizeTensor(inputTensor, {1,705}); // 确保输入形状正确
      interpreter_->resizeSession(session_);
      memcpy(inputTensor->host<float>(), observations_.data(), observations_.size() * sizeof(float));
    }

    // 运行模型
  
    interpreter_->runSession(session_);
    // 处理输出
    for (const auto &outputName : outputNames_)
    {
      auto outputTensor = interpreter_->getSessionOutput(session_, outputName.c_str());
      MNN::Tensor outputTensorCopy(outputTensor, outputTensor->getDimensionType());

      // 从设备复制输出张量到主机
      outputTensor->copyToHostTensor(&outputTensorCopy);

      // 重新解释输出数据，填充到 actions_ 向量
      float *data = outputTensorCopy.host<float>();
      size_t numElements = outputTensorCopy.elementSize(); // 获取元素总数
      memcpy(actions_.data(), data, numElements * sizeof(float));
    }
  }

  void AcController::computeObservation()
  {
    // command
    vector_t command(5);
    if (sw_mode_)
    {
      cmd_norm_ = std::sqrt(command_.x * command_.x + command_.y * command_.y + command_.yaw * command_.yaw);
      if (cmd_norm_ <= cmd_threshold_)
      {
        current_time_ = ros::Time::now().toSec();
        phase_ = 0;
      }
      else
      {
        phase_ -= current_time_;
      }
    }
    phase_ = phase_ / robotCfg_.controlCfg.cycle_time;
    command[0] = sin(2 * M_PI * phase_);
    command[1] = cos(2 * M_PI * phase_);
    command[2] = command_.x;
    command[3] = command_.y;
    command[4] = command_.yaw;

    // actions
    vector_t actions(lastActions_);

    RLRobotCfg::ObsScales &obsScales = robotCfg_.obsScales;
    matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

    vector_t proprioObs(observationSize_);

    proprioObs << command, // 5
        (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos,  // 10
        propri_.jointVel * obsScales.dofVel,  // 10
        actions,  // 10
        propri_.baseAngVel * obsScales.angVel,  // 3
        propri_.baseEulerXyz * obsScales.quat;  // 3


    if (isfirstRecObs_)
    {      
      for (
         int i = 29; i < 41; i++)
      {
        /* code */
        proprioObs(i,0) = 0.0;
      }

      for (size_t i = 0; i < numHist_; i++)
      {
        proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = proprioObs.cast<tensor_element_t>();
      }
      isfirstRecObs_ = false;
    }

    proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
        proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
    proprioHistoryBuffer_.tail(observationSize_) = proprioObs.cast<tensor_element_t>();


    // clang-format on

    for (size_t i = 0; i < (observationSize_ * numHist_); i++){
      observations_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
      // if(i < observationSize_)
      // std::cout << i << "obs:::" << observations_[i] << std::endl;
    }
    // Limit observation range
    scalar_t obsMin = -robotCfg_.clipObs;
    scalar_t obsMax = robotCfg_.clipObs;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                   [obsMin, obsMax](scalar_t x)
                   { return std::max(obsMin, std::min(obsMax, x)); });
  }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::AcController, controller_interface::ControllerBase)
