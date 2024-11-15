#pragma once

#include "rl_controllers/RLControllerBase.h"

namespace legged
{

  class AcController : public RLControllerBase
  {
    using tensor_element_t = float;

  public:
    AcController() : interpreter_(nullptr) {}

    ~AcController() override = default;
    void reloadParameters();

  protected:
    bool loadModel(ros::NodeHandle &nh) override;
    bool loadRLCfg(ros::NodeHandle &nh) override;
    void computeActions() override;
    void computeObservation() override;
    void handleWalkMode() override;

  private:
    // onnx policy model

    std::unique_ptr<MNN::Interpreter> interpreter_;
    MNN::Session *session_ = nullptr;
    std::vector<std::string> inputNames_;
    std::vector<std::string> outputNames_;
    std::vector<std::vector<int>> inputShapes_;
    std::vector<std::vector<int>> outputShapes_;
    MNN::Tensor *inputTensor;
    MNN::Tensor *outputTensor;

    std::string policyFilePath_;
    
    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t lastActions_;
    vector_t defaultJointAngles_;

    bool isfirstRecObs_{true};
    int actionsSize_;
    int observationSize_;
    int numHist_;
    bool sw_mode_;
    double cmd_threshold_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;
        Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
    bool isfirstCompAct_{true};
  };

} // namespace legged