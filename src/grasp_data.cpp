/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Univ of CO, Boulder, PAL Robotics, S.L.
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Authors: Dave Coleman, Bence Magyar
   Description: Data class used by the grasp generator.
*/

#include <moveit_grasps/grasp_data.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

// C++
#include <cmath>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Pose conversion
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_grasps
{
const std::string LOGNAME = "grasp_data";

GraspData::GraspData(const rclcpp::Node::SharedPtr nh, const std::string& end_effector,
                     const moveit::core::RobotModelConstPtr& robot_model)
  : base_link_("/base_link"), robot_model_(robot_model)
{
}

bool GraspData::loadGraspData(const rclcpp::Node::SharedPtr nh, const std::string& end_effector)
{
  std::vector<std::string> joint_names;
  std::vector<double> pre_grasp_posture;  // todo: remove all underscore post-fixes
  std::vector<double> grasp_posture;
  double pregrasp_time_from_start;
  double grasp_time_from_start;
  std::string end_effector_name;

  // Helper to let user know what is wrong
  if (!nh->has_parameter("base_link"))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("grasp_data"),
                        "Grasp configuration parameter `base_link` missing from rosparam "
                        "server. Did you load your end effector's configuration yaml file? "
                        "Searching in namespace: "
                            << nh->get_namespace());
    return false;
  }

  // Load all other parameters
  const std::string parent_name = "grasp_data";  // for namespacing logging messages
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(nh, "base_link", base_link_);

  // Search within the sub-namespace of this end effector name
  // rclcpp::Node::SharedPtr child_nh =
  //     rclcpp::Node::make_shared(end_effector, nh->get_namespace()); // TODO!!!

  error += !rosparam_shortcuts::get(nh, end_effector + ".pregrasp_time_from_start", pregrasp_time_from_start);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_time_from_start", grasp_time_from_start);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_resolution", grasp_resolution_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_min_depth", grasp_min_depth_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_max_depth", grasp_max_depth_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_depth_resolution", grasp_depth_resolution_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".approach_distance_desired", approach_distance_desired_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".retreat_distance_desired", retreat_distance_desired_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".lift_distance_desired", lift_distance_desired_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".angle_resolution", angle_resolution_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".end_effector_name", end_effector_name);
  error += !rosparam_shortcuts::get(nh, end_effector + ".joints", joint_names);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_padding_on_approach", grasp_padding_on_approach_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".grasp_posture", grasp_posture);
  error += !rosparam_shortcuts::get(nh, end_effector + ".pregrasp_posture", pre_grasp_posture);

  bool define_tcp_by_name;
  nh->get_parameter_or<bool>(end_effector + ".define_tcp_by_name", define_tcp_by_name, false);
  if (define_tcp_by_name)
    error += !rosparam_shortcuts::get(nh, end_effector + ".tcp_name", tcp_name_);
  else
    error += !rosparam_shortcuts::get(nh, end_effector + ".tcp_to_eef_mount_transform", tcp_to_eef_mount_);

  rosparam_shortcuts::shutdownIfError(error);

  // Create pre-grasp and grasp postures
  pre_grasp_posture_.header.frame_id = base_link_;
  pre_grasp_posture_.header.stamp = nh->get_clock()->now();
  // Name of joints:
  pre_grasp_posture_.joint_names = joint_names;
  // Position of joints
  pre_grasp_posture_.points.resize(1);
  pre_grasp_posture_.points[0].positions = pre_grasp_posture;
  pre_grasp_posture_.points[0].time_from_start = rclcpp::Duration(pregrasp_time_from_start);

  // Create grasp posture
  grasp_posture_.header.frame_id = base_link_;
  grasp_posture_.header.stamp = nh->get_clock()->now();
  // Name of joints:
  grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = grasp_posture;
  grasp_posture_.points[0].time_from_start = rclcpp::Duration(grasp_time_from_start);

  // Copy values from RobotModel
  ee_jmg_ = robot_model_->getJointModelGroup(end_effector_name);
  arm_jmg_ = robot_model_->getJointModelGroup(ee_jmg_->getEndEffectorParentGroup().first);
  parent_link_ = robot_model_->getLinkModel(ee_jmg_->getEndEffectorParentGroup().second);

  RCLCPP_INFO(rclcpp::get_logger("grasp_data"), "ee_name: %s, arm_jmg: %s, parent_link: %s", ee_jmg_->getName().c_str(),
              arm_jmg_->getName().c_str(), parent_link_->getName().c_str());

  if (define_tcp_by_name)
  {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.update();
    if (!state.knowsFrameTransform(parent_link_->getName()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("grasp_data"),
                   "Robot Model does not know the frame transform for the end "
                   "effector group parent "
                   "frame: %s. Did you set a parent link in the srdf?",
                   parent_link_->getName().c_str());
    }
    if (!state.knowsFrameTransform(tcp_name_))
    {
      RCLCPP_ERROR(rclcpp::get_logger("grasp_data"),
                   "Robot Model does not know the frame transform for the tcp "
                   "frame: %s. Is it "
                   "available in the urdf?",
                   tcp_name_.c_str());
    }
    Eigen::Isometry3d eef_mount_pose = state.getGlobalLinkTransform(parent_link_);
    Eigen::Isometry3d tcp_mount_pose = state.getGlobalLinkTransform(tcp_name_);
    tcp_to_eef_mount_ = tcp_mount_pose.inverse() * eef_mount_pose;
  }

  return true;
}

bool GraspData::setRobotStatePreGrasp(moveit::core::RobotStatePtr& robot_state)
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("grasp_data"), "setRobotStatePreGrasp is probably wrong");
  return setRobotState(robot_state, pre_grasp_posture_);
}

bool GraspData::setRobotStateGrasp(moveit::core::RobotStatePtr& robot_state)
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("grasp_data"), "setRobotStateGrasp is probably wrong");
  return setRobotState(robot_state, grasp_posture_);
}

bool GraspData::setRobotState(moveit::core::RobotStatePtr& robot_state,
                              const trajectory_msgs::msg::JointTrajectory& posture)
{
  // TODO(davetcoleman): make this more efficient
  // Do for every joint in end effector
  for (std::size_t i = 0; i < posture.joint_names.size(); ++i)
  {
    // Set joint position
    robot_state->setJointPositions(posture.joint_names[i], posture.points[0].positions);
  }
  return true;
}

void GraspData::print()
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("grasp_data"), "Debug Grasp Data variable values:");
  std::cout << "tcp_to_eef_mount_: \n"
            << tcp_to_eef_mount_.translation() << "\n"
            << tcp_to_eef_mount_.rotation() << std::endl;
  // std::cout << "pre_grasp_posture_: \n" << pre_grasp_posture_ << std::endl;
  // std::cout << "grasp_posture_: \n" << grasp_posture_ << std::endl;
  std::cout << "base_link_: " << base_link_ << std::endl;
  std::cout << "ee_group_: " << ee_jmg_->getName() << std::endl;
  std::cout << "angle_resolution_: " << angle_resolution_ << std::endl;
  std::cout << "grasp_max_depth_: " << grasp_max_depth_ << std::endl;
  std::cout << "grasp_padding_on_approach_: " << grasp_padding_on_approach_ << std::endl;
}

}  // namespace moveit_grasps
