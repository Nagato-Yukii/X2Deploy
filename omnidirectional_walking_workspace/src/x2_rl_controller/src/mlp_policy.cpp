#include "x2_rl_controller/mlp_policy.hpp"
#include <iostream>
#include <algorithm>
#include <stdexcept>

MlpPolicy::MlpPolicy(const std::string& model_path) {
    // 1. 初始化 ONNX Session 配置 (建议开启多线程优化，视 RK3588 核心情况而定)
    session_options_.SetIntraOpNumThreads(2); 
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // 加载模型
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    // 2. 预分配内存，总输入 225 维，输出 12 维
    input_tensor_values_.resize(225, 0.0f);
    output_tensor_values_.resize(12, 0.0f);

    // 3. 初始化所有历史 Buffer 为 0 (冷启动策略)
    hist_base_ang_vel_.resize(15, 0.0f);
    hist_projected_gravity_.resize(15, 0.0f);
    hist_velocity_commands_.resize(15, 0.0f);
    hist_joint_pos_rel_.resize(60, 0.0f);
    hist_joint_vel_rel_.resize(60, 0.0f);
    hist_last_action_.resize(60, 0.0f);
}

void MlpPolicy::update_history(std::vector<float>& history_buffer, const std::vector<float>& new_data, int dim) {
    // 将旧数据往前平移 (丢弃最老的 frame)
    std::copy(history_buffer.begin() + dim, history_buffer.end(), history_buffer.begin());
    // 将最新的一帧放到末尾
    std::copy(new_data.begin(), new_data.end(), history_buffer.end() - dim);
}

// 冷启动
void MlpPolicy::prefill_history(std::vector<float>& history_buffer, const std::vector<float>& initial_data, int dim) {
    for (int i = 0; i < 5; ++i) { // 5 是 history_length
        std::copy(initial_data.begin(), initial_data.end(), history_buffer.begin() + i * dim);
    }
}

std::vector<float> MlpPolicy::compute_action(
    const std::vector<float>& base_ang_vel, 
    const std::vector<float>& projected_gravity,
    const std::vector<float>& velocity_commands, 
    const std::vector<float>& joint_pos_rel,
    const std::vector<float>& joint_vel_rel, 
    const std::vector<float>& last_action) 
{
    // 冷启动路由
    if (is_first_frame_) {
        prefill_history(hist_base_ang_vel_, base_ang_vel, 3);
        prefill_history(hist_projected_gravity_, projected_gravity, 3);
        prefill_history(hist_velocity_commands_, velocity_commands, 3);
        prefill_history(hist_joint_pos_rel_, joint_pos_rel, 12);
        prefill_history(hist_joint_vel_rel_, joint_vel_rel, 12);
        prefill_history(hist_last_action_, last_action, 12);
        
        is_first_frame_ = false; // 翻转世界线，以后全走滑动窗口
    }
    else {
        // 1. 更新各自的历史 Buffer
        update_history(hist_base_ang_vel_, base_ang_vel, 3);
        update_history(hist_projected_gravity_, projected_gravity, 3);
        update_history(hist_velocity_commands_, velocity_commands, 3);
        update_history(hist_joint_pos_rel_, joint_pos_rel, 12);
        update_history(hist_joint_vel_rel_, joint_vel_rel, 12);
        update_history(hist_last_action_, last_action, 12);
    }
    

    // 2. 极其残暴但高效的内存拼接 (拼成 225 维的一维张量)
    size_t offset = 0;
    auto copy_to_input = [&](const std::vector<float>& hist) {
        std::copy(hist.begin(), hist.end(), input_tensor_values_.begin() + offset);
        offset += hist.size();
    };

    copy_to_input(hist_base_ang_vel_);
    copy_to_input(hist_projected_gravity_);
    copy_to_input(hist_velocity_commands_);
    copy_to_input(hist_joint_pos_rel_);
    copy_to_input(hist_joint_vel_rel_);
    copy_to_input(hist_last_action_);

    // 3. 构建 ONNX 要求的 Tensor 对象
    std::vector<int64_t> input_shape = {1, 225}; // Batch size = 1
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_tensor_values_.data(), input_tensor_values_.size(), 
        input_shape.data(), input_shape.size());

    // 4. 执行推理 (Forward Pass)
    std::vector<Ort::Value> output_tensors = session_->Run(
        Ort::RunOptions{nullptr}, 
        input_node_names_.data(), &input_tensor, 1, 
        output_node_names_.data(), 1);

    // 5. 提取并返回 12 维 action
    const float* output_data = output_tensors.front().GetTensorMutableData<float>();
    std::copy(output_data, output_data + 12, output_tensor_values_.begin());

    return output_tensor_values_;
}