#include <chrono>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

class MoveitTestNode : public rclcpp::Node {
public:
  MoveitTestNode() : rclcpp::Node("moveit_test") {}

  void init() {
    arm =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    arm->setMaxVelocityScalingFactor(1.0);
    arm->setMaxAccelerationScalingFactor(1.0);

    gripper = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),
                                                                               "gripper");
    gripper->setMaxVelocityScalingFactor(1.0);
    gripper->setMaxAccelerationScalingFactor(1.0);
  }

  bool move_arm_to_named_target(const std::string target_name) {
    if (!arm) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    return move_interface_to_named_target(*arm, target_name);
  }

  bool move_gripper_to_named_target(const std::string target_name) {
    if (!gripper) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    return move_interface_to_named_target(*gripper, target_name);
  }

private:
  bool move_interface_to_named_target(moveit::planning_interface::MoveGroupInterface &interface,
                                      const std::string target_name) {
    interface.setStartStateToCurrentState();
    interface.setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
      success = interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    return success;
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveitTestNode>();
  node->init();

  node->move_arm_to_named_target("pose_1");
  rclcpp::sleep_for(std::chrono::seconds(1));
  node->move_arm_to_named_target("home");
  rclcpp::sleep_for(std::chrono::seconds(1));
  node->move_gripper_to_named_target("gripper_closed");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
