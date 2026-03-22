#include "x2_stand_controller/stand_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

// 注意这里！没有 class StandNode {} 的包裹！直接用 StandNode:: 来实现构造函数
StandNode::StandNode() : Node("stand_node") {
    arm_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>("/aima/hal/joint/arm/command", 10);
    waist_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>("/aima/hal/joint/waist/command", 10);
    leg_pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>("/aima/hal/joint/leg/command", 10);
    
    std::vector<std::string> arm_joint_names = {
        "left_shoulder_pitch_joint", "left_elbow_joint" , "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_elbow_joint" , "right_wrist_yaw_joint",
    };
    std::vector<std::string> waist_joint_names = {
        "waist_yaw_joint", "waist_pitch_joint" , "waist_roll_joint",
    };
    std::vector<std::string> leg_joint_names = {
        "left_hip_pitch_joint", "left_hip_roll_joint" , "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint" , "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    };
    
    auto declare_joints = [this](const std::vector<std::string>& names) {
        for (const auto& name : names) {
            this->declare_parameter("joints." + name + ".position", 0.0);
            this->declare_parameter("joints." + name + ".kp", 20.0);
            this->declare_parameter("joints." + name + ".kd", 1.0);
        }
    };
    
    declare_joints(arm_joint_names);
    declare_joints(waist_joint_names);
    declare_joints(leg_joint_names);

    build_stand_command(arm_joint_names, arm_cmd_msg_);
    build_stand_command(waist_joint_names, waist_cmd_msg_);
    build_stand_command(leg_joint_names, leg_cmd_msg_);
    
    timer_ = this->create_wall_timer(2ms, std::bind(&StandNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "El Psy Kongroo. Stand Node Initialized properly.");
}

// 同样，直接用 StandNode:: 来实现成员函数，不要再加 class 的大括号
void StandNode::build_stand_command(const std::vector<std::string>& joint_names, aimdk_msgs::msg::JointCommandArray& target_msg) {
    for (const auto& name : joint_names) {
        aimdk_msgs::msg::JointCommand joint;
        joint.name = name;
        joint.position = this->get_parameter("joints." + name + ".position").as_double();
        joint.velocity = 0.0; 
        joint.effort = 0.0;   
        joint.stiffness = this->get_parameter("joints." + name + ".kp").as_double();
        joint.damping = this->get_parameter("joints." + name + ".kd").as_double();
        
        target_msg.joints.push_back(joint);
    }
}

void StandNode::timer_callback() {
    auto current_time = this->now();
    
    arm_cmd_msg_.header.stamp = current_time;
    arm_pub_->publish(arm_cmd_msg_);

    waist_cmd_msg_.header.stamp = current_time;
    waist_pub_->publish(waist_cmd_msg_);

    leg_cmd_msg_.header.stamp = current_time;
    leg_pub_->publish(leg_cmd_msg_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandNode>());
    rclcpp::shutdown();
    return 0;
}