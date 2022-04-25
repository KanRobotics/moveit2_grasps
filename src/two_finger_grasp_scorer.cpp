/*********************************************************************
 * Software License Agreement ("Modified BSD License")
 *
 * Copyright (c) 2014, University of Colorado, Boulder
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the Univ of CO, Boulder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * Authors : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc    : Functions for scoring grasps. See *.h file for documentation
 */

#include <moveit_grasps/two_finger_grasp_scorer.h>

namespace moveit_grasps
{
double TwoFingerGraspScoreWeights::computeScore(const Eigen::Vector3d& orientation_scores,
                                                const Eigen::Vector3d& translation_scores, double depth_score,
                                                double width_score, bool verbose) const
{
  double total_score = GraspScoreWeights::computeScore(orientation_scores, translation_scores, false) *
                       GraspScoreWeights::getWeightTotal();
  total_score += depth_score * depth_score_weight_ + width_score * width_score_weight_;

  total_score /= getWeightTotal();

  if (verbose)
  {
    static const std::string logger_name = "grasp_scorer.compute_score";
    static const rclcpp::Logger LOGGER = rclcpp::get_logger(logger_name);
    // clang-format off
    RCLCPP_DEBUG_STREAM(LOGGER, "Two Finger Grasp score: ");
    RCLCPP_DEBUG_STREAM(LOGGER, "\torientation_score.x = " << orientation_scores[0] << "\tweight = "<< orientation_x_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\torientation_score.y = " << orientation_scores[1] << "\tweight = "<< orientation_y_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\torientation_score.z = " << orientation_scores[2] << "\tweight = "<< orientation_z_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\ttranslation_score.x = " << translation_scores[0] << "\tweight = "<< translation_x_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\ttranslation_score.y = " << translation_scores[1] << "\tweight = "<< translation_y_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\ttranslation_score.z = " << translation_scores[2] << "\tweight = "<< translation_z_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\tdepth_score         = " << depth_score           << "\tweight = "<< depth_score_weight_);
    RCLCPP_DEBUG_STREAM(LOGGER, "\twidth_score         = " << width_score           << "\tweight = "<< width_score_weight_);
    // Total
    RCLCPP_DEBUG_STREAM(LOGGER, "\ttotal_score = " << total_score);
    // clang-format on
  }
  return total_score;
}

double TwoFingerGraspScoreWeights::getWeightTotal() const
{
  return GraspScoreWeights::getWeightTotal() + depth_score_weight_ + width_score_weight_;
}

double TwoFingerGraspScorer::scoreGraspWidth(const TwoFingerGraspDataPtr& grasp_data, double percent_open)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("grasp_scorer.graspWidth"), "raw score = " << percent_open);
  return pow(percent_open, 2);
}

double TwoFingerGraspScorer::scoreDistanceToPalm(const Eigen::Isometry3d& grasp_pose_tcp,
                                                 const TwoFingerGraspDataPtr& grasp_data,
                                                 const Eigen::Isometry3d& object_pose, double min_grasp_distance,
                                                 double max_grasp_distance)
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("grasp_scorer.distance");
  double distance = (grasp_pose_tcp.translation() - object_pose.translation()).norm();
  RCLCPP_DEBUG_STREAM(LOGGER, "distance = " << distance << ", " << min_grasp_distance << ":" << max_grasp_distance);

  double score = 1.0 - (distance - min_grasp_distance) / (max_grasp_distance - min_grasp_distance);

  RCLCPP_DEBUG_STREAM(LOGGER, "raw score = " << score);
  if (score < 0)
    RCLCPP_WARN_STREAM(LOGGER, "score < 0!");
  return pow(score, 4);
}

}  // namespace moveit_grasps
