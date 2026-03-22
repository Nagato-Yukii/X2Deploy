#ifndef X2_RL_CONTROLLER__RL_NODE_HPP_
#define X2_RL_CONTROLLER__RL_NODE_HPP_

#define _USE_MATH_DEFINES
#include <cmath>

// 看门狗，机器的倾角不能高过这个值
static constexpr float MAX_PITCH    = 35.0f * M_PI / 180.0f;
static constexpr float MAX_ROLL     = 35.0f * M_PI / 180.0f;
static constexpr float SAFE_LIMIT   = 35.0f * M_PI / 180.0f;  // 统一入口，与 MAX_ROLL/PITCH 相同

#include <rclcpp/rclcpp.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <memory>
#include "x2_rl_controller/mlp_policy.hpp"
#include <std_msgs/msg/empty.hpp>

class RLNode : public rclcpp::Node {
public:
    RLNode();

private:
    // 回调与定时器
    void leg_state_cb(const aimdk_msgs::msg::JointStateArray::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
    void control_loop_cb();

    // 内部辅助函数
    void init_parameters();
    void build_command_message();
    void emergency_shutdown();

    // ROS 2 接口
    rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr leg_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr leg_cmd_pub_;
    // 非rl目标
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr waist_cmd_pub_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr head_cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // 核心组件
    std::unique_ptr<MlpPolicy> policy_;
    std::mutex state_mutex_; // 极度关键：保护跨线程状态不被撕裂

    // 降采样计数器 (Decimation = 10)
    int decimation_counter_ = 0;

    // IsaacLab 严格指定的关节顺序
    std::vector<std::string> joint_names_ = {
        "left_hip_pitch_joint", "right_hip_pitch_joint",
        "left_hip_roll_joint", "right_hip_roll_joint",
        "left_hip_yaw_joint", "right_hip_yaw_joint",
        "left_knee_joint", "right_knee_joint",
        "left_ankle_pitch_joint", "right_ankle_pitch_joint",
        "left_ankle_roll_joint", "right_ankle_roll_joint"
    };
    std::vector<std::string> arm_joint_names_ = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint" , 
    "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint" , 
    "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint"
    };
    std::vector<std::string> waist_joint_names_ = {
        "waist_yaw_joint", "waist_pitch_joint" , "waist_roll_joint"
    };
    std::vector<std::string> head_joint_names_ = {
        "head_yaw_joint", "head_pitch_joint"
    };


    // 最新读取到的传感器状态 (受 mutex_ 保护)
    std::vector<float> latest_joint_pos_;
    std::vector<float> latest_joint_vel_;
    std::vector<float> latest_base_ang_vel_;
    std::vector<float> latest_projected_gravity_;

    // 零阶保持器：存放上一次推理出的 Action
    std::vector<float> last_action_;

    // 控制参数与默认偏置 (从 YAML 读取)
    std::vector<float> default_pos_;
    std::vector<float> kps_;
    std::vector<float> kds_;

    // 欧拉角（IMU 回调中计算，供看门狗使用）
    float latest_roll_  = 0.0f;
    float latest_pitch_ = 0.0f;
    float roll  = 0.0f;
    float pitch = 0.0f;

    // 紧急停机标志位
    bool is_emergency_stopped_ = false;

    // 缓存发送消息以节省内存分配开销
    aimdk_msgs::msg::JointCommandArray cmd_msg_;
    aimdk_msgs::msg::JointCommandArray arm_cmd_msg_;
    aimdk_msgs::msg::JointCommandArray waist_cmd_msg_;
    aimdk_msgs::msg::JointCommandArray head_cmd_msg_;

    // 上位机心跳检测
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
    rclcpp::Time last_heartbeat_received_;
    bool is_comm_lost_ = false;
    const double COMM_TIMEOUT = 0.1; // 100ms
};

#endif // X2_RL_CONTROLLER__RL_NODE_HPP_