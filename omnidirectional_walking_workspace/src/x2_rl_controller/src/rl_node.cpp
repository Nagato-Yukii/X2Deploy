#include "x2_rl_controller/rl_node.hpp"
#include <chrono>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std::chrono_literals;

RLNode::RLNode() : Node("rl_locomotion_node") {
    // 1. 初始化容器尺寸 (12 个关节)
    latest_joint_pos_.resize(12, 0.0f);
    latest_joint_vel_.resize(12, 0.0f);
    latest_base_ang_vel_.resize(3, 0.0f);
    latest_projected_gravity_.resize(3, 0.0f);
    last_action_.resize(12, 0.0f);

    // 2. 加载参数与策略
    init_parameters();
    build_command_message();
    
    // 替换为你在 RK3588 上的实际模型路径
    std::string pkg_share = ament_index_cpp::get_package_share_directory("x2_rl_controller");
    std::string model_path = pkg_share + "/models/policy.onnx";
    
    RCLCPP_INFO(this->get_logger(), "Loading ONNX model from: %s", model_path.c_str());
    policy_ = std::make_unique<MlpPolicy>(model_path);

    last_heartbeat_received_ = this->get_clock()->now();
    heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/aima/remote/heartbeat", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            this->last_heartbeat_received_ = this->get_clock()->now();
            if (this->is_comm_lost_) {
                RCLCPP_INFO(this->get_logger(), "Heartbeat restored. Worldline re-stabilized.");
                this->is_comm_lost_ = false;
            }
        });

    // 3. 建立神经连接 (Sub/Pub)
    // 注意：运控板底层的状态话题频率极高
    leg_state_sub_ = this->create_subscription<aimdk_msgs::msg::JointStateArray>(
        "/aima/hal/joint/leg/state", 10, std::bind(&RLNode::leg_state_cb, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/aima/hal/imu/torso/state", 10, std::bind(&RLNode::imu_cb, this, std::placeholders::_1));

    leg_cmd_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>(
        "/aima/hal/joint/leg/command", 10);
    arm_cmd_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>(
    "/aima/hal/joint/arm/command", 10);
    waist_cmd_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>(
    "/aima/hal/joint/waist/command", 10);
    head_cmd_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>(
    "/aima/hal/joint/head/command", 10);

    // 4. 起搏器：严格的 500Hz 控制循环 (2ms)
    timer_ = this->create_wall_timer(2ms, std::bind(&RLNode::control_loop_cb, this));

    RCLCPP_INFO(this->get_logger(), "El Psy Kongroo. RL Locomotion Node Activated.");
}

void RLNode::init_parameters() {
    // 从 locomotion.yaml 加载参数
    default_pos_.resize(12);
    kps_.resize(12);
    kds_.resize(12);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        std::string name = joint_names_[i];
        this->declare_parameter("joints." + name + ".position", 0.0);
        this->declare_parameter("joints." + name + ".kp", 40.0);
        this->declare_parameter("joints." + name + ".kd", 1.0);

        default_pos_[i] = this->get_parameter("joints." + name + ".position").as_double();
        kps_[i] = this->get_parameter("joints." + name + ".kp").as_double();
        kds_[i] = this->get_parameter("joints." + name + ".kd").as_double();
    }

    auto setup_static_joints = [this](const std::vector<std::string>& names, aimdk_msgs::msg::JointCommandArray& msg) {
    for (const auto& name : names) {
        this->declare_parameter("joints." + name + ".position", 0.0);
        this->declare_parameter("joints." + name + ".kp", 40.0);
        this->declare_parameter("joints." + name + ".kd", 1.0);

        aimdk_msgs::msg::JointCommand joint;
        joint.name = name;
        joint.position = this->get_parameter("joints." + name + ".position").as_double();
        joint.velocity = 0.0;
        joint.effort = 0.0;
        joint.stiffness = this->get_parameter("joints." + name + ".kp").as_double();
        joint.damping = this->get_parameter("joints." + name + ".kd").as_double();
        msg.joints.push_back(joint);
    }
    };
    setup_static_joints(arm_joint_names_, arm_cmd_msg_);
    setup_static_joints(waist_joint_names_, waist_cmd_msg_);
    setup_static_joints(head_joint_names_, head_cmd_msg_);
}

void RLNode::leg_state_cb(const aimdk_msgs::msg::JointStateArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 将底层发来的乱序状态，严格映射到 IsaacLab 的 12 维数组中
    for (const auto& joint : msg->joints) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (joint.name == joint_names_[i]) {
                latest_joint_pos_[i] = joint.position;
                latest_joint_vel_[i] = joint.velocity;
                break;
            }
        }
    }
}

void RLNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 1. 角速度缩放 (保持你原来的 IsaacLab 逻辑)
    latest_base_ang_vel_[0] = msg->angular_velocity.x * 0.2f;
    latest_base_ang_vel_[1] = msg->angular_velocity.y * 0.2f;
    latest_base_ang_vel_[2] = msg->angular_velocity.z * 0.2f;

    // 2. 投影重力计算 (保持不变)
    float w = msg->orientation.w;
    float x = msg->orientation.x;
    float y = msg->orientation.y;
    float z = msg->orientation.z;
    latest_projected_gravity_[0] = -2.0f * (x * z - w * y);
    latest_projected_gravity_[1] = -2.0f * (y * z + w * x);
    latest_projected_gravity_[2] = -(1.0f - 2.0f * (x * x + y * y));

    // 3. 【新增】计算欧拉角用于看门狗
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double r, p, y_unused;
    m.getRPY(r, p, y_unused);
    
    latest_roll_ = static_cast<float>(r);
    latest_pitch_ = static_cast<float>(p);
}

void RLNode::control_loop_cb() {
    auto now = this->get_clock()->now();
    double elapsed = (now - last_heartbeat_received_).seconds();
    if (elapsed > COMM_TIMEOUT) {
        if (!is_comm_lost_) {
            RCLCPP_ERROR(this->get_logger(), "!!! COMM LOSS DETECTED !!! No heartbeat for %f s", elapsed);
            is_comm_lost_ = true;
        }
        emergency_shutdown(); // 强制脱力
        return;               // 停止后续所有计算
    }
    
    if (is_emergency_stopped_) {
        emergency_shutdown(); // 持续发送零力矩，防止底层复位
        return;
    }
    // 1. 安全抓取当前状态快照
    std::vector<float> curr_pos(12), curr_vel(12), curr_ang_vel(3), curr_grav(3);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        curr_pos = latest_joint_pos_;
        curr_vel = latest_joint_vel_;
        curr_ang_vel = latest_base_ang_vel_;
        curr_grav = latest_projected_gravity_;
        roll = latest_roll_;   // 确保你在 IMU 回调里已经计算好了欧拉角
        pitch = latest_pitch_;
    }

    if (std::abs(roll) > SAFE_LIMIT || std::abs(pitch) > SAFE_LIMIT) {
        emergency_shutdown();
        return;
    }

    // 2. 核心状态机：Decimation = 10 (每 20ms 触发一次推理)
    if (decimation_counter_ % 10 == 0) {
        // 构建网络的相对观测值
        std::vector<float> joint_pos_rel(12);
        std::vector<float> joint_vel_rel(12);
        
        for (int i = 0; i < 12; ++i) {
            joint_pos_rel[i] = curr_pos[i] - default_pos_[i]; // 无缩放
            joint_vel_rel[i] = curr_vel[i] * 0.05f;           // 速度缩放 0.05
        }

        // 速度指令 [vx, vy, wz] (这里先硬编码为原点踏步，你可以后续接入遥控器 topic)
        std::vector<float> vel_cmds = {0.0f, 0.0f, 0.0f};

        // 呼叫孤立大脑进行前向传播
        last_action_ = policy_->compute_action(
            curr_ang_vel, curr_grav, vel_cmds, joint_pos_rel, joint_vel_rel, last_action_
        );
    }
    decimation_counter_ = (decimation_counter_ + 1) % 10000; // 防止 int 溢出

    //一直保持指令发送

    auto current_time = this->now(); // 统一获取当前时间戳
    
    cmd_msg_.header.stamp = current_time;
    for (int i = 0; i < 12; ++i) {
        float target_pos = last_action_[i] * 0.15f + default_pos_[i];
        cmd_msg_.joints[i].position = target_pos;
    }



    // 【新增】更新上半身静态指令的时间戳
    arm_cmd_msg_.header.stamp = current_time;
    waist_cmd_msg_.header.stamp = current_time;
    head_cmd_msg_.header.stamp = current_time;

    // 4. 全身控制指令并发下发
    leg_cmd_pub_->publish(cmd_msg_);
    arm_cmd_pub_->publish(arm_cmd_msg_);     // 【新增】锁死手臂
    waist_cmd_pub_->publish(waist_cmd_msg_); // 【新增】锁死腰部
    head_cmd_pub_->publish(head_cmd_msg_);
}

// 辅助函数：紧急停机
void RLNode::emergency_shutdown() {
    if (!is_emergency_stopped_) {
        RCLCPP_ERROR(this->get_logger(), "!!! EMERGENCY STOP TRIGGERED: FULL SYSTEM SHUTDOWN !!!");
        is_emergency_stopped_ = true;
    }

    auto current_time = this->now();

    // 构建一个万能的“瘫痪射线”发射器
    auto make_limp_msg = [&](const std::vector<std::string>& names) {
        aimdk_msgs::msg::JointCommandArray msg;
        msg.header.stamp = current_time;
        for (const auto& name : names) {
            aimdk_msgs::msg::JointCommand j;
            j.name = name;
            j.stiffness = 0.0; 
            j.damping = 0.0; 
            j.effort = 0.0; 
            msg.joints.push_back(j);
        }
        return msg;
    };

    // 无差别全覆盖打击，物理抹除所有伺服刚性
    leg_cmd_pub_->publish(make_limp_msg(joint_names_));
    arm_cmd_pub_->publish(make_limp_msg(arm_joint_names_));
    waist_cmd_pub_->publish(make_limp_msg(waist_joint_names_));
    head_cmd_pub_->publish(make_limp_msg(head_joint_names_));
}

void RLNode::build_command_message() {
    // 预先构建好消息体，避免在 500Hz 循环里重新分配内存
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        aimdk_msgs::msg::JointCommand joint;
        joint.name = joint_names_[i];
        joint.velocity = 0.0;
        joint.effort = 0.0;
        joint.stiffness = kps_[i];
        joint.damping = kds_[i];
        cmd_msg_.joints.push_back(joint);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RLNode>());
    rclcpp::shutdown();
    return 0;
}