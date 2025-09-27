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
    arm = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    arm->setMaxVelocityScalingFactor(1.0);
    arm->setMaxAccelerationScalingFactor(1.0);
  }

  bool move_to_named_target(std::string target_name) {
    if (!arm) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }

    arm->setStartStateToCurrentState();
    arm->setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = arm->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
      success = arm->execute(plan1) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    return success;
  }

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveitTestNode>();
  node->init();

  node->move_to_named_target("pose_1");
  rclcpp::sleep_for(std::chrono::seconds(2));
  node->move_to_named_target("home");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
