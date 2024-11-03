/* Author: Jure Hudoklin */

#include "scara_robots_kinematics_solver.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace scara_robots_kinematics_solver
{

ScaraRobotsKinematicsSolver::ScaraRobotsKinematicsSolver() : KinematicsBase()
{
}

bool ScaraRobotsKinematicsSolver::initialize(const rclcpp::Node::SharedPtr& node,
                                        const moveit::core::RobotModel& robot_model,
                                        const std::string& group_name,
                                        const std::string& base_frame,
                                        const std::vector<std::string>& tip_frames,
                                        double search_discretization)
{
  node_ = node;
  bool debug = true;

  auto logger = node_->get_logger();

  RCLCPP_WARN(logger, "Initializing SCARA kinematics solver");
  std::cout << group_name.c_str() << '\n';
  std::cout << base_frame.c_str() << '\n';
  std::cout << tip_frames.size() << '\n';

  // Store the input parameters
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
  if (!joint_model_group_)
  {
    RCLCPP_ERROR(logger, "Invalid group name:");
    return false;
  }

  // Print the joint model variable names
  if (debug)
  {
    std::cout << "Joint Model Variable Names: ------------------------------------------- \n ";
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << '\n';
  }

  // Verify that we have only one tip frame for the SCARA robot
  if (tip_frames_.size() != 1)
  {
    RCLCPP_ERROR(logger, "This kinematics solver only supports one tip frame, got %zu", tip_frames_.size());
    return false;
  }

  // Setup the joint state groups that we need
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();

  // Get the joint names and link names for the group
  joint_names_ = joint_model_group_->getActiveJointModelNames();
  link_names_ = joint_model_group_->getLinkModelNames();

  if (debug)
  {
    std::cout << "Joint Names: ------------------------------------------- \n ";
    std::copy(joint_names_.begin(), joint_names_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << '\n';

    std::cout << "Link Names: ------------------------------------------- \n ";
    std::copy(link_names_.begin(), link_names_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << '\n';
  }

  std::vector<Eigen::Isometry3d> link_transforms(link_names_.size());
  // Get transformations of links 
  for (int i = 0; i < link_transforms.size(); ++i)
  {
    const auto link_model = robot_model_->getLinkModel(link_names_[i]);
    const auto& link_transform = link_model->getJointOriginTransform();
    link_transforms[i] = link_transform;
  }
  if (debug)
  {
    std::cout << link_transforms.size() << '\n';
    std::cout << link_names_.size() << '\n';
    std::cout << "Link Transforms: ------------------------------------------- \n ";
    for (int i = 0; i < link_transforms.size(); ++i)
    {
      // Link name
      std::cout << link_names_[i].c_str() << '\n';

      // Rotation matrix
      std::cout << "Rotation matrix: \n";
      std::cout << link_transforms[i].rotation() << '\n';

      // Translation vector
      std::cout << "Translation vector: \n";
      std::cout << link_transforms[i].translation() << '\n';
    }
    std::cout << '\n';
  }

  // Verify that we have 4 joints for the SCARA robot
  if (joint_names_.size() != 4)
  {
    RCLCPP_ERROR(logger, "This kinematics solver expects 4 joints, got %zu", joint_names_.size());
    return false;
  }

  // Set up the joint limits
  for (int i = 0; i < 4; ++i)
  {
    auto const* joint_model = joint_model_group_->getJointModel(joint_names_[i]);
    auto const& bounds = joint_model->getVariableBounds()[0];
    if (bounds.position_bounded_)
    {
      joint_min_[i] = bounds.min_position_;
      joint_max_[i] = bounds.max_position_;
    }
    else
    {
      joint_min_[i] = -M_PI;
      joint_max_[i] = M_PI;
    }
  }

  // Set up the dimension of the robot
  dimension_ = joint_model_group_->getVariableCount();

  // Log successful initialization
  RCLCPP_WARN(logger, "SCARA kinematics solver initialized successfully");

  return true;
}


bool ScaraRobotsKinematicsSolver::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{

  error_code.set__val(moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  // Implement your inverse kinematics algorithm here
  // Calculate joint angles (solution) for the given end-effector pose (ik_pose)
  return true;
}

bool ScaraRobotsKinematicsSolver::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state,
                                              double timeout,
                                              std::vector<double>& solution,
                                              moveit_msgs::msg::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  // Implement a search-based inverse kinematics algorithm if needed
  // This can be useful for handling multiple IK solutions or dealing with joint limits
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}


bool ScaraRobotsKinematicsSolver::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose, 
                                      const std::vector<double> &ik_seed_state, 
                                      double timeout,
                                      const std::vector<double> &consistency_limits, 
                                      std::vector<double> &solution,
                                      moveit_msgs::msg::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options) const
{
    // Implement this function
    return false;
}

bool ScaraRobotsKinematicsSolver::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, 
                                      const std::vector<double>& ik_seed_state, 
                                      double timeout,
                                      std::vector<double>& solution, 
                                      const IKCallbackFn& solution_callback,
                                      moveit_msgs::msg::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
    // Implement this function
    return false;
}


bool ScaraRobotsKinematicsSolver::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, 
                                      const std::vector<double>& ik_seed_state, 
                                      double timeout,
                                      const std::vector<double>& consistency_limits, 
                                      std::vector<double>& solution,
                                      const IKCallbackFn& solution_callback, 
                                      moveit_msgs::msg::MoveItErrorCodes& error_code,
                                      const kinematics::KinematicsQueryOptions& options) const
{
    // Implement this function
    return false;
}





bool ScaraRobotsKinematicsSolver::getPositionFK(const std::vector<std::string>& link_names,
                                           const std::vector<double>& joint_angles,
                                           std::vector<geometry_msgs::msg::Pose>& poses) const
{
  // Implement your forward kinematics algorithm here
  // Calculate end-effector poses for the given joint angles
  return true;
}

const std::vector<std::string>& ScaraRobotsKinematicsSolver::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string>& ScaraRobotsKinematicsSolver::getLinkNames() const
{
  return link_names_;
}

}  // namespace custom_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(scara_robots_kinematics_solver::ScaraRobotsKinematicsSolver, kinematics::KinematicsBase)
