#ifndef X2_RL_CONTROLLER__MLP_POLICY_HPP_
#define X2_RL_CONTROLLER__MLP_POLICY_HPP_

#include <vector>
#include <string>
#include <array>
#include <onnxruntime_cxx_api.h>

class MlpPolicy {
public:
    // 构造函数：传入模型路径进行初始化
    explicit MlpPolicy(const std::string& model_path);

    // 核心推理函数：传入当前步的拆解观测，自动维护历史并返回 12 维动作
    std::vector<float> compute_action(
        const std::vector<float>& base_ang_vel,       // 3 dims
        const std::vector<float>& projected_gravity,  // 3 dims
        const std::vector<float>& velocity_commands,  // 3 dims
        const std::vector<float>& joint_pos_rel,      // 12 dims
        const std::vector<float>& joint_vel_rel,      // 12 dims
        const std::vector<float>& last_action         // 12 dims
    );

private:
    // 辅助函数：更新单个特征的历史队列 滑动窗口更新
    void update_history(std::vector<float>& history_buffer, const std::vector<float>& new_data, int dim);
    // 辅助函数：冷启动预填充
    void prefill_history(std::vector<float>& history_buffer, const std::vector<float>& initial_data, int dim);
    // 冷启动标志位
    bool is_first_frame_ = true;
    // ONNX Runtime 核心组件
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "X2_RL_Policy"};
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)};

    // 内存预分配，坚决避免在 50Hz 循环中动态分配内存！
    std::vector<float> input_tensor_values_;
    std::vector<float> output_tensor_values_;

    // I/O 节点信息
    std::vector<const char*> input_node_names_ = {"obs"};
    std::vector<const char*> output_node_names_ = {"actions"};

    // 按照 IsaacLab 设定的各 Term 独立历史 Buffer (历史长度 = 5)
    std::vector<float> hist_base_ang_vel_;      // 3 * 5 = 15
    std::vector<float> hist_projected_gravity_; // 3 * 5 = 15
    std::vector<float> hist_velocity_commands_; // 3 * 5 = 15
    std::vector<float> hist_joint_pos_rel_;     // 12 * 5 = 60
    std::vector<float> hist_joint_vel_rel_;     // 12 * 5 = 60
    std::vector<float> hist_last_action_;       // 12 * 5 = 60
};

#endif // X2_RL_CONTROLLER__MLP_POLICY_HPP_