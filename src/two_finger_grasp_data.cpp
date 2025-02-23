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

#include <moveit_grasps/two_finger_grasp_data.h>

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

#include <rcpputils/asserts.hpp>
#include <memory>
#include <string>
#include <stdexcept>

template <typename... Args>
std::string string_format(const std::string& format, Args... args)
{
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
  if (size_s <= 0)
  {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

namespace moveit_grasps
{
const std::string LOGNAME = "grasp_data.two_finger_gripper";
const rclcpp::Logger LOGGER = rclcpp::get_logger(LOGNAME);

TwoFingerGraspData::TwoFingerGraspData(const rclcpp::Node::SharedPtr nh, const std::string& end_effector,
                                       const moveit::core::RobotModelConstPtr& robot_model)
  : GraspData(nh, end_effector, robot_model)
{
}

bool TwoFingerGraspData::loadGraspData(const rclcpp::Node::SharedPtr nh, const std::string& end_effector)
{
  if (!GraspData::loadGraspData(nh, end_effector))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "GraspData::loadGraspData failed");
    return false;
  }
  // Load all other parameters
  const std::string parent_name = "grasp_data";  // for namespacing logging messages
  std::size_t error = 0;

  // Search within the sub-namespace of this end effector name
  // rclcpp::Node::SharedPtr child_nh = rclcpp::Node::make_shared(end_effector, nh->get_namespace());

  error += !rosparam_shortcuts::get(nh, end_effector + ".gripper_finger_width", gripper_finger_width_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".max_grasp_width", max_grasp_width_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".max_finger_width", max_finger_width_);
  error += !rosparam_shortcuts::get(nh, end_effector + ".min_finger_width", min_finger_width_);
  rosparam_shortcuts::shutdownIfError(error);

  return true;
}

bool TwoFingerGraspData::setGraspWidth(double fraction_open, double min_finger_width,
                                       trajectory_msgs::msg::JointTrajectory& grasp_posture)
{
  if (fraction_open < 0 || fraction_open > 1)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("grasp_data"), "Invalid fraction passed in " << fraction_open);
    return false;
  }

  // Ensure min_finger_width is not less than actual min finger width
  double min_finger_width_adjusted = std::max(min_finger_width, min_finger_width_);

  // Max width = max_finger_width_
  // Min width = min_finger_width_adjusted
  double distance_btw_fingers =
      min_finger_width_adjusted + (max_finger_width_ - min_finger_width_adjusted) * fraction_open;
  return fingerWidthToGraspPosture(distance_btw_fingers, grasp_posture);
}

bool TwoFingerGraspData::fingerWidthToGraspPosture(double distance_btw_fingers,
                                                   trajectory_msgs::msg::JointTrajectory& grasp_posture)
{
  // TODO(mlautman): Change this function to take in a method for translating joint values to grasp width
  //       Currently this function simply interpolates between max open and max closed
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("grasp_data"),
                      "Setting grasp posture to have distance_between_fingers of " << distance_btw_fingers);

  // Error check
  if (distance_btw_fingers > max_finger_width_ + std::numeric_limits<double>::epsilon() ||
      distance_btw_fingers < min_finger_width_ - std::numeric_limits<double>::epsilon())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("grasp_data"), "Requested " << distance_btw_fingers
                                                                       << " is beyond limits of " << min_finger_width_
                                                                       << "," << max_finger_width_);
    return false;
  }

  // NOTE: We assume a linear relationship between the actuated joint values and the distance between fingers.
  //       This is probably incorrect but until we expose an interface for passing in a function to translate from
  //       joint values to grasp width, it's the best we got...
  // TODO(mlautman): Make it so that a user can pass in a function here.
  const std::vector<std::string>& joint_names = pre_grasp_posture_.joint_names;
  const std::vector<double>& grasp_pose = grasp_posture_.points[0].positions;
  const std::vector<double>& pre_grasp_pose = pre_grasp_posture_.points[0].positions;
  if (joint_names.size() != grasp_pose.size() || grasp_pose.size() != pre_grasp_pose.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("grasp_data"),
                 "Mismatched vector sizes joint_names.size()=%zu, grasp_pose.size()=%zu, and "
                 "pre_grasp_pose.size()=%zu",
                 joint_names.size(), grasp_pose.size(), pre_grasp_pose.size());
    return false;
  }

  std::vector<double> slope(joint_names.size());
  std::vector<double> intercept(joint_names.size());
  std::vector<double> joint_positions(joint_names.size());
  for (std::size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
  {
    slope[joint_index] =
        (max_finger_width_ - min_finger_width_) / (pre_grasp_pose[joint_index] - grasp_pose[joint_index]);
    intercept[joint_index] = max_finger_width_ - slope[joint_index] * pre_grasp_pose[joint_index];

    // Sanity check
    rcpputils::assert_true(intercept[joint_index] == min_finger_width_ - slope[joint_index] * grasp_pose[joint_index],
                           string_format("we got different y intercept!! %.3f and %.3f", intercept[joint_index],
                                         min_finger_width_ - slope[joint_index] * grasp_pose[joint_index]));

    joint_positions[joint_index] = (distance_btw_fingers - intercept[joint_index]) / slope[joint_index];

    RCLCPP_DEBUG(rclcpp::get_logger("grasp_data"), "Converted joint %s to position %.3f",
                 joint_names[joint_index].c_str(), joint_positions[joint_index]);
  }

  return jointPositionsToGraspPosture(joint_positions, grasp_posture);
}

bool TwoFingerGraspData::jointPositionsToGraspPosture(const std::vector<double>& joint_positions,
                                                      trajectory_msgs::msg::JointTrajectory& grasp_posture)
{
  for (std::size_t joint_index = 0; joint_index < pre_grasp_posture_.joint_names.size(); joint_index++)
  {
    const moveit::core::JointModel* joint = robot_model_->getJointModel(pre_grasp_posture_.joint_names[joint_index]);
    rcpputils::assert_true(joint->getVariableBounds().size() > 0,
                           string_format("joint->getVariableBounds() is empty for %s",
                                         pre_grasp_posture_.joint_names[joint_index].c_str()));
    const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

    if (joint_positions[joint_index] > bound.max_position_ || joint_positions[joint_index] < bound.min_position_)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("grasp_data"),
                          "Requested joint " << pre_grasp_posture_.joint_names[joint_index].c_str() << "at index"
                                             << joint_index << " with value " << joint_positions[joint_index]
                                             << " is beyond limits of " << bound.min_position_ << ", "
                                             << bound.max_position_);
      return false;
    }
  }

  // Get default grasp posture
  grasp_posture = grasp_posture_;

  // Error check
  if (joint_positions.size() != grasp_posture.points.front().positions.size())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("grasp_data"),
                        "Not enough finger joints passed in: " << joint_positions.size() << " positions but expect "
                                                               << grasp_posture.points.front().positions.size());
    return false;
  }

  // Set joint positions
  grasp_posture.points.front().positions = joint_positions;

  return true;
}

void TwoFingerGraspData::print()
{
  GraspData::print();

  std::cout << "Finger Gripper Parameters: " << std::endl;
  // std::cout << "\tpre_grasp_posture_: \n" << pre_grasp_posture_ << std::endl;
  // std::cout << "\tgrasp_posture_: \n" << grasp_posture_ << std::endl;
  std::cout << "\tgripper_finger_width_: " << gripper_finger_width_ << std::endl;
  std::cout << "\tmax_grasp_width_: " << max_grasp_width_ << std::endl;
  std::cout << "\tmax_finger_width_: " << max_finger_width_ << std::endl;
  std::cout << "\tmin_finger_width_: " << min_finger_width_ << std::endl;
}

}  // namespace moveit_grasps
