#ifndef X2_STAND_CONTROLLER__STAND_NODE_HPP_
#define X2_STAND_CONTROLLER__STAND_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <aimdk_msgs/msg/joint_command.hpp>
#include <vector>
#include <string>

class StandNode : public rclcpp::Node {
public:
    StandNode();

private:
    void build_stand_command(const std::vector<std::string>& joint_names, aimdk_msgs::msg::JointCommandArray& target_msg);
    void timer_callback();

    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr arm_pub_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr waist_pub_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr leg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    aimdk_msgs::msg::JointCommandArray arm_cmd_msg_;
    aimdk_msgs::msg::JointCommandArray waist_cmd_msg_;
    aimdk_msgs::msg::JointCommandArray leg_cmd_msg_;
};

#endif // X2_STAND_CONTROLLER__STAND_NODE_HPP_