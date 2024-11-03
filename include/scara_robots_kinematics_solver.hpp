#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

namespace scara_robots_kinematics_solver
{

  class ScaraRobotsKinematicsSolver : public kinematics::KinematicsBase
  {
  public:
    ScaraRobotsKinematicsSolver();

    bool getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
                       moveit_msgs::msg::MoveItErrorCodes &error_code,
                       const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          moveit_msgs::msg::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose, 
                          const std::vector<double> &ik_seed_state, double timeout,
                          const std::vector<double> &consistency_limits, 
                          std::vector<double> &solution,
                          moveit_msgs::msg::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, 
                          const std::vector<double>& ik_seed_state, double timeout,
                          std::vector<double>& solution, 
                          const IKCallbackFn& solution_callback,
                          moveit_msgs::msg::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, 
                          const std::vector<double>& ik_seed_state, double timeout,
                          const std::vector<double>& consistency_limits, 
                          std::vector<double>& solution,
                          const IKCallbackFn& solution_callback, 
                          moveit_msgs::msg::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;


    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles,
                       std::vector<geometry_msgs::msg::Pose> &poses) const override;

    bool initialize(const rclcpp::Node::SharedPtr &node,
                    const moveit::core::RobotModel &robot_model,
                    const std::string &group_name,
                    const std::string &base_frame,
                    const std::vector<std::string> &tip_frames,
                    double search_discretization) override;

    const std::vector<std::string> &getJointNames() const override;
    const std::vector<std::string> &getLinkNames() const override;

  private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;

    std::array<double, 4> joint_min_;
    std::array<double, 4> joint_max_;

    const moveit::core::JointModelGroup *joint_model_group_;

    moveit::core::RobotStatePtr robot_state_;

    unsigned int dimension_;

    rclcpp::Node::SharedPtr node_;
  };

} // namespace custom_kinematics_plugin
