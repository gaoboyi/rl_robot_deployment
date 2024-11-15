#include "rl_controllers/StudentStmController.h"
#include <pluginlib/class_list_macros.hpp>
#include "rl_controllers/RotationTools.h"
#include <numeric>

namespace legged
{
    void StudentStmController::handleWalkMode()
    {
        // compute observation & actions
        if (loopCount_ % robotCfg_.controlCfg.decimation == 0)
        {
            computeObservation();
            // walkCount_++;
            computeActions();
            // limit action range
            scalar_t actionMin = -robotCfg_.clipActions;
            scalar_t actionMax = robotCfg_.clipActions;
            std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                           [actionMin, actionMax](scalar_t x)
                           { return std::max(actionMin, std::min(actionMax, x)); });
        }

        // set action
        for (int i = 0; i < hybridJointHandles_.size(); i++)
        {
            scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i, 0);
            // hybridJointHandles_[i].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness, robotCfg_.controlCfg.damping, 0);
            lastActions_(i, 0) = actions_[i];
        }
    }

    bool StudentStmController::loadModel(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("load student policy model");

        std::string policyModelPath;
        std::string encoderModelPath;
        if (!nh.getParam("/policyModelPath", policyModelPath) || !nh.getParam("/encoderModelPath", encoderModelPath))
        {
            ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
            return false;
        }

        // create env
        onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        // create session
        Ort::SessionOptions sessionOptions;
        sessionOptions.SetIntraOpNumThreads(1);
        sessionOptions.SetInterOpNumThreads(1);

        Ort::AllocatorWithDefaultOptions allocator;
        // policy session
        policySessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyModelPath.c_str(), sessionOptions);
        policyInputNames_.clear();
        policyOutputNames_.clear();
        policyInputShapes_.clear();
        policyOutputShapes_.clear();
        for (int i = 0; i < policySessionPtr_->GetInputCount(); i++)
        {
            auto inputPolicynamePtr = policySessionPtr_->GetInputNameAllocated(i, allocator);
            inputPolicyNameAllocatedStrings.push_back(std::move(inputPolicynamePtr));
            policyInputNames_.push_back(inputPolicyNameAllocatedStrings.back().get());
            policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            std::vector<int64_t> shape = policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            std::cerr << "Shape: [";
            for (size_t j = 0; j < shape.size(); ++j)
            {
                std::cout << shape[j];
                if (j != shape.size() - 1)
                {
                    std::cerr << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
        for (int i = 0; i < policySessionPtr_->GetOutputCount(); i++)
        {
            auto outputPolicynamePtr = policySessionPtr_->GetOutputNameAllocated(i, allocator);
            outputPolicyNameAllocatedStrings.push_back(std::move(outputPolicynamePtr));
            policyOutputNames_.push_back(outputPolicyNameAllocatedStrings.back().get());
            policyOutputShapes_.push_back(policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            std::vector<int64_t> shape = policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            std::cerr << "Shape: [";
            for (size_t j = 0; j < shape.size(); ++j)
            {
                std::cout << shape[j];
                if (j != shape.size() - 1)
                {
                    std::cerr << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
        // encoder session
        encoderSessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, encoderModelPath.c_str(), sessionOptions);
        encoderInputNames_.clear();
        encoderOutputNames_.clear();
        encoderInputShapes_.clear();
        encoderOutputShapes_.clear();
        for (int i = 0; i < encoderSessionPtr_->GetInputCount(); i++)
        {
            auto inputEcodernamePtr = encoderSessionPtr_->GetInputNameAllocated(i, allocator);
            inputEncoderNameAllocatedStrings.push_back(std::move(inputEcodernamePtr));
            encoderInputNames_.push_back(inputEncoderNameAllocatedStrings.back().get());
            encoderInputShapes_.push_back(encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            std::vector<int64_t> shape = encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            std::cerr << "EncoderIputShape: [";
            for (size_t j = 0; j < shape.size(); ++j)
            {
                std::cout << shape[j];
                if (j != shape.size() - 1)
                {
                    std::cerr << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
        for (int i = 0; i < encoderSessionPtr_->GetOutputCount(); i++)
        {
            auto outputEcodernamePtr = encoderSessionPtr_->GetOutputNameAllocated(i, allocator);
            outputEncoderNameAllocatedStrings.push_back(std::move(outputEcodernamePtr));
            encoderOutputNames_.push_back(outputEncoderNameAllocatedStrings.back().get());
            //
            std::cout << "name:" << encoderOutputNames_[i] << std::endl;
            //
            encoderOutputShapes_.push_back(encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
            std::vector<int64_t> shape = encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            std::cerr << "EcoderOutputShape: [";
            for (size_t j = 0; j < shape.size(); ++j)
            {
                std::cout << shape[j];
                if (j != shape.size() - 1)
                {
                    std::cerr << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
        hidden_.assign(3 * 256, 0);
        cell_.assign(3 * 256, 0);
        lastActions_ = vector_t::Zero(12);
        const int64_t inputSize = 5 * 45;
        proprioHistoryBuffer_.resize(inputSize);
        ROS_INFO_STREAM("Load Onnx model successfully !!!");
        std::cout << "Stm!!!walk!!!" << std::endl;
        return true;
    }

    bool StudentStmController::loadRLCfg(ros::NodeHandle &nh)
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

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r1_joint", initState.leg_r1_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r2_joint", initState.leg_r2_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r3_joint", initState.leg_r3_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r4_joint", initState.leg_r4_joint));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r5_joint", initState.leg_r5_joint));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness", controlCfg.stiffness));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping", controlCfg.damping));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", controlCfg.actionScale));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robotCfg_.clipObs));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robotCfg_.clipActions));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));

        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
        error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));

        actions_.resize(actionsSize_);
        observations_.resize(observationSize_);

        command_.x = 0;
        command_.y = 0;
        command_.yaw = 0;
        baseLinVel_.setZero();
        basePosition_.setZero();
        std::vector<scalar_t> defaultJointAngles{
            robotCfg_.initState.leg_l1_joint, robotCfg_.initState.leg_l2_joint, robotCfg_.initState.leg_l3_joint,
            robotCfg_.initState.leg_l4_joint, robotCfg_.initState.leg_l5_joint,
            robotCfg_.initState.leg_r1_joint, robotCfg_.initState.leg_r2_joint, robotCfg_.initState.leg_r3_joint,
            robotCfg_.initState.leg_r4_joint, robotCfg_.initState.leg_r5_joint};
        lastActions_.resize(actuatedDofNum_);
        defaultJointAngles_.resize(actuatedDofNum_);
        for (int i = 0; i < actuatedDofNum_; i++)
        {
            defaultJointAngles_(i) = defaultJointAngles[i];
        }

        return (error == 0);
    }

    // stm_obs
    void StudentStmController::computeObservation()
    {

        // command
        vector3_t command;
        command[0] = command_.x;
        command[1] = command_.y;
        command[2] = command_.yaw;

        // actions
        vector_t actions(lastActions_);

        RLRobotCfg::ObsScales &obsScales = robotCfg_.obsScales;
        matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

        vector_t proprioObs(45);

        proprioObs << propri_.projectedGravity,
            propri_.baseAngVel * obsScales.angVel,
            (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos,
            propri_.jointVel * obsScales.dofVel,
            actions,
            commandScaler * command;

        if (isfirstRecObs_)
        {
            for (size_t i = 0; i < 5; i++)
            {
                proprioHistoryBuffer_.segment(i * 45, 45) = proprioObs.cast<tensor_element_t>();
            }
            isfirstRecObs_ = false;
        }

        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - 45) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - 45);
        proprioHistoryBuffer_.tail(45) = proprioObs.cast<tensor_element_t>();

        // clang-format on

        for (size_t i = 0; i < proprioObs.size(); i++)
            observations_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_.tail(45)(i));
        // Limit observation range
        scalar_t obsMin = -robotCfg_.clipObs;
        scalar_t obsMax = robotCfg_.clipObs;
        std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                       [obsMin, obsMax](scalar_t x)
                       { return std::max(obsMin, std::min(obsMax, x)); });
    }

    // stm_action
    void StudentStmController::computeActions()
    {
        // create input tensor object
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
        std::vector<Ort::Value> encoderInputValues;
        encoderInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, proprioHistoryBuffer_.data(), proprioHistoryBuffer_.size(),
                                                                                encoderInputShapes_[0].data(), encoderInputShapes_[0].size()));

        Ort::RunOptions runOptions;
        std::vector<Ort::Value> encoderOutputValues = encoderSessionPtr_->Run(runOptions, encoderInputNames_.data(), encoderInputValues.data(), 1, encoderOutputNames_.data(), 1);
        std::vector<tensor_element_t> encoderOutput(45);
        std::copy(encoderOutputValues[0].GetTensorMutableData<tensor_element_t>(), encoderOutputValues[0].GetTensorMutableData<tensor_element_t>() + encoderOutput.size(), encoderOutput.begin());

        std::vector<tensor_element_t> mergedInput(observations_.size() + encoderOutput.size());
        std::copy(observations_.begin(), observations_.end(), mergedInput.begin());
        std::copy(encoderOutput.begin(), encoderOutput.end(), mergedInput.begin() + observations_.size());

        std::vector<Ort::Value> policyInputValues;
        Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, mergedInput.data(), mergedInput.size(), policyInputShapes_[0].data(), policyInputShapes_[0].size());
        policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, mergedInput.data(), mergedInput.size(), policyInputShapes_[0].data(), policyInputShapes_[0].size()));
        // Run policy inference
        std::vector<Ort::Value> policyOutputValues = policySessionPtr_->Run(runOptions, policyInputNames_.data(), policyInputValues.data(), 1, policyOutputNames_.data(), 1);

        for (int i = 0; i < actions_.size(); i++)
        {
            actions_[i] = *(policyOutputValues[0].GetTensorMutableData<tensor_element_t>() + i);
            // std::cout << "actions_[" << i << "]:" << actions_[i] << std::endl;
        }
    }

}
PLUGINLIB_EXPORT_CLASS(legged::StudentStmController, controller_interface::ControllerBase)