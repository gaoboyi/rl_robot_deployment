#pragma once

#include "rl_controllers/RLControllerBase.h"

namespace legged
{

  class StudentStmController : public RLControllerBase
  {
    using tensor_element_t = float;

  public:
    StudentStmController() = default;
    ~StudentStmController() override = default;

  protected:
    bool loadModel(ros::NodeHandle &nh) override;
    bool loadRLCfg(ros::NodeHandle &nh) override;
    void computeActions() override;
    void computeObservation() override;
    void handleWalkMode() override;

  private:
    std::string policyModelPath_;
    std::string encoderModelPath_;
    std::shared_ptr<Ort::Env> onnxEnvPrt_;
    std::unique_ptr<Ort::Session> policySessionPtr_;
    std::unique_ptr<Ort::Session> encoderSessionPtr_;
    std::vector<const char *> policyInputNames_;
    std::vector<const char *> policyOutputNames_;
    std::vector<std::vector<int64_t>> policyInputShapes_;
    std::vector<std::vector<int64_t>> policyOutputShapes_;
    std::vector<const char *> encoderInputNames_;
    std::vector<const char *> encoderOutputNames_;
    std::vector<std::vector<int64_t>> encoderInputShapes_;
    std::vector<std::vector<int64_t>> encoderOutputShapes_;
    int64_t encoderBatchSize_, encoderSeqLength_, encoderInputDim_;
    std::vector<Ort::AllocatedStringPtr> inputPolicyNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> outputPolicyNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> inputEncoderNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> outputEncoderNameAllocatedStrings;

    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t lastActions_;
    vector_t defaultJointAngles_;

    bool isfirstRecObs_{true};
    int actionsSize_;
    int observationSize_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;
    std::vector<tensor_element_t> hidden_;
    std::vector<tensor_element_t> cell_;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
  };

} // namespace legged