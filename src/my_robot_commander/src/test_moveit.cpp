#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

class MoveitTestNode : public rclcpp::Node {
public:
  MoveitTestNode() : rclcpp::Node("moveit_test") {}

  void init() {
    arm = std::make_unique<MoveGroupInterface>(shared_from_this(), "arm");

    // Following two are there to make robot go fast.
    arm->setMaxVelocityScalingFactor(1.0);
    arm->setMaxAccelerationScalingFactor(1.0);

    gripper = std::make_unique<MoveGroupInterface>(shared_from_this(), "gripper");
    gripper->setMaxVelocityScalingFactor(1.0);
    gripper->setMaxAccelerationScalingFactor(1.0);
  }

  bool move_arm_to_named_target(const std::string target_name) {
    if (!arm) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    return move_group_to_named_target(*arm, target_name);
  }

  bool move_gripper_to_named_target(const std::string target_name) {
    if (!gripper) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    return move_group_to_named_target(*gripper, target_name);
  }

  bool move_arm_to_joint_value_target(const std::vector<double> joint_values) {
    if (!arm) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    if (joint_values.size() != 6) {
      RCLCPP_ERROR(get_logger(), "Arm group requires 6 joints for joint value target!");
      return false;
    }
    return move_group_to_joint_value_target(*arm, joint_values);
  }

  bool move_arm_to_pose_target(const geometry_msgs::msg::PoseStamped pose_target) {
    if (!arm) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    return move_group_to_pose_target(*arm, pose_target);
  }

  bool move_arm_cartesian_path(const std::vector<geometry_msgs::msg::Pose> &waypoints) {
    if (!arm) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized!");
      return false;
    }
    return move_group_cartesian_path(*arm, waypoints);
  }

  geometry_msgs::msg::Pose get_end_effector_pose() { return arm->getCurrentPose().pose; }

private:
  bool move_group_to_named_target(MoveGroupInterface &group, const std::string target_name) {
    group.setStartStateToCurrentState();
    group.setNamedTarget(target_name);

    MoveGroupInterface::Plan plan;
    bool success = group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
      success = group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return success;
  }

  bool move_group_to_joint_value_target(MoveGroupInterface &group,
                                        const std::vector<double> joint_values) {
    group.setStartStateToCurrentState();
    group.setJointValueTarget(joint_values);

    MoveGroupInterface::Plan plan;
    bool success = group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
      success = group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return success;
  }

  bool move_group_to_pose_target(MoveGroupInterface &group,
                                 const geometry_msgs::msg::PoseStamped pose_target) {
    group.setStartStateToCurrentState();
    group.setPoseTarget(pose_target);

    MoveGroupInterface::Plan plan;
    bool success = group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
      success = group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return success;
  }

  bool move_group_cartesian_path(MoveGroupInterface &group,
                                 const std::vector<geometry_msgs::msg::Pose> &waypoints) {
    group.setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, traj);

    if (fraction > 0.9)
      return arm->execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;

    RCLCPP_ERROR(get_logger(), "Cartesian path planning failed, fraction: %f", fraction);
    return false;
  }

  std::unique_ptr<MoveGroupInterface> arm;
  std::unique_ptr<MoveGroupInterface> gripper;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveitTestNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // This kind of executor is required for working while MoveIt 2.
  // Because it helps to be able to continue callbacks to execute while moveit stuff happens.
  // To understand what is this, take a look at the ROS 2 executors section in
  // `Tech Support/ROS 2 basics.md` at Obsidian.
  auto spinner = std::thread([&executor]() { executor.spin(); });

  node->init();

  node->move_arm_to_named_target("pose_1");
  rclcpp::sleep_for(std::chrono::seconds(1));
  node->move_arm_to_named_target("home");
  rclcpp::sleep_for(std::chrono::seconds(1));
  node->move_gripper_to_named_target("gripper_closed");
  rclcpp::sleep_for(std::chrono::seconds(1));
  node->move_arm_to_joint_value_target({1.5, 0.5, 0.0, 1.5, 0.0, -0.7});
  rclcpp::sleep_for(std::chrono::seconds(1));

  tf2::Quaternion q;
  q.setRPY(3.14, 0, 0);
  geometry_msgs::msg::PoseStamped pose_target;
  pose_target.header.frame_id = "base_link";
  pose_target.pose.position.x = 0.0;
  pose_target.pose.position.y = -0.7;
  pose_target.pose.position.z = 0.4;
  pose_target.pose.orientation.x = q.getX();
  pose_target.pose.orientation.y = q.getY();
  pose_target.pose.orientation.z = q.getZ();
  pose_target.pose.orientation.w = q.getW();
  node->move_arm_to_pose_target(pose_target);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose pose1 = node->get_end_effector_pose();
  pose1.position.z -= 0.2;
  waypoints.push_back(pose1);
  geometry_msgs::msg::Pose pose2 = pose1;
  pose2.position.y += 0.2;
  waypoints.push_back(pose2);
  geometry_msgs::msg::Pose pose3 = pose2;
  pose3.position.y -= 0.2;
  pose3.position.z += 0.2;
  waypoints.push_back(pose3);
  node->move_arm_cartesian_path(waypoints);

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
