/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/

/* Author: Mike Lautman
   Desc:   Demonstrates a full pick using MoveIt Grasps
*/

// ROS
#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp
#include <moveit_grasps/two_finger_grasp_generator.h>
#include <moveit_grasps/two_finger_grasp_data.h>
#include <moveit_grasps/two_finger_grasp_filter.h>
#include <moveit_grasps/grasp_planner.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps_demo
{
static const std::string LOGNAME = "grasp_pipeline_demo";

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene,
                  const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, moveit::core::RobotState* robot_state,
                  const moveit::core::JointModelGroup* group, const double* ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();
  return !planning_scene->isStateColliding(*robot_state, group->getName());
}

void waitForNextStep(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, const std::string& prompt)
{
  visual_tools->prompt(prompt);
}

}  // namespace

class GraspPipelineDemo
{
public:
  // Constructor
  GraspPipelineDemo()
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    nh_ = rclcpp::Node::make_shared("grasp_test", node_options);
    // Get arm info from param server
    const std::string parent_name = "grasp_filter_demo";  // for namespacing logging messages
    rosparam_shortcuts::get(nh_, "planning_group_name", planning_group_name_);
    rosparam_shortcuts::get(nh_, "ee_group_name", ee_group_name_);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "End Effector: " << ee_group_name_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "Planning Group: " << planning_group_name_);

    loadScene();
    setupGraspPipeline();
  }

  void loadScene()
  {
    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nh_, "robot_description");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGNAME), "Planning scene not configured");
      return;
    }
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "grasping_planning_scene");
    planning_scene_monitor_->getPlanningScene()->setName("grasping_planning_scene");

    robot_model_loader::RobotModelLoaderPtr robot_model_loader;
    robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(nh_, "robot_description");

    // Load the robot model
    robot_model_ = robot_model_loader->getModel();
    arm_jmg_ = robot_model_->getJointModelGroup(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        nh_, robot_model_->getModelFrame(), "/rviz_visual_tools", planning_scene_monitor_);
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("/display_robot_state");
    visual_tools_->loadTrajectoryPub("/display_planned_path");
    visual_tools_->loadSharedRobotState();
    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    // Publish the global frame
    visual_tools_->publishAxis(Eigen::Isometry3d::Identity());
    visual_tools_->trigger();
  }

  void setupGraspPipeline()
  {
    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_ =
        std::make_shared<moveit_grasps::TwoFingerGraspData>(nh_, ee_group_name_, visual_tools_->getRobotModel());
    if (!grasp_data_->loadGraspData(nh_, ee_group_name_))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGNAME), "Failed to load Grasp Data parameters.");
      exit(-1);
    }

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_ = std::make_shared<moveit_grasps::TwoFingerGraspGenerator>(nh_, visual_tools_);

    // Set the ideal grasp orientation for scoring
    std::vector<double> ideal_grasp_rpy = { 3.14, 0.0, 0.0 };
    grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

    // Set custom grasp score weights
    auto grasp_score_weights = std::make_shared<moveit_grasps::TwoFingerGraspScoreWeights>();
    grasp_score_weights->orientation_x_score_weight_ = 2.0;
    grasp_score_weights->orientation_y_score_weight_ = 2.0;
    grasp_score_weights->orientation_z_score_weight_ = 2.0;
    grasp_score_weights->translation_x_score_weight_ = 1.0;
    grasp_score_weights->translation_y_score_weight_ = 1.0;
    grasp_score_weights->translation_z_score_weight_ = 1.0;
    // Finger gripper specific weights.
    grasp_score_weights->depth_score_weight_ = 2.0;
    grasp_score_weights->width_score_weight_ = 2.0;
    // Assign the grasp score weights in the grasp_generator
    grasp_generator_->setGraspScoreWeights(grasp_score_weights);

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    grasp_filter_ =
        std::make_shared<moveit_grasps::TwoFingerGraspFilter>(nh_, visual_tools_->getSharedRobotState(), visual_tools_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp planner for approach, lift and retreat planning
    grasp_planner_ = std::make_shared<moveit_grasps::GraspPlanner>(nh_, visual_tools_);

    // MoveIt Grasps allows for a manual breakpoint debugging tool to be optionally passed in
    grasp_planner_->setWaitForNextStepCallback(boost::bind(&waitForNextStep, visual_tools_, _1));

    // -----------------------------------------------------
    // Load the motion planning pipeline
    planning_pipeline_ =
        std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, nh_, "planning_plugin", "request_adapter");
  }

  bool demoRandomGrasp()
  {
    // -----------------------------------
    // Generate random object to grasp
    geometry_msgs::msg::Pose object_pose;
    double object_x_depth;
    double object_y_width;
    double object_z_height;
    std::string object_name;
    if (!generateRandomCuboid(object_name, object_pose, object_x_depth, object_y_width, object_z_height))
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "Failed to add random cuboid ot planning scene");
      return false;
    }

    // -----------------------------------
    // Generate grasp candidates
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;

    // Configure the desired types of grasps
    moveit_grasps::TwoFingerGraspCandidateConfig grasp_generator_config =
        moveit_grasps::TwoFingerGraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    grasp_generator_config.generate_y_axis_grasps_ = true;
    grasp_generator_config.generate_x_axis_grasps_ = true;
    grasp_generator_config.generate_z_axis_grasps_ = true;

    grasp_generator_->setGraspCandidateConfig(grasp_generator_config);
    if (!grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), object_x_depth, object_y_width,
                                          object_z_height, grasp_data_, grasp_candidates))
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "Grasp generator failed to generate any valid grasps");
      return false;
    }

    // --------------------------------------------
    // Generating a seed state for filtering grasps
    moveit::core::RobotStatePtr seed_state =
        std::make_shared<moveit::core::RobotState>(*visual_tools_->getSharedRobotState());
    Eigen::Isometry3d eef_mount_grasp_pose =
        visual_tools_->convertPose(object_pose) * grasp_data_->tcp_to_eef_mount_.inverse();
    if (!getIKSolution(arm_jmg_, eef_mount_grasp_pose, *seed_state, grasp_data_->parent_link_->getName()))
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGNAME),
                         "The ideal seed state is not reachable. Using start state as seed.");
    }

    // --------------------------------------------
    // Filtering grasps
    // Note: This step also solves for the grasp and pre-grasp states and stores them in grasp candidates)
    bool filter_pregrasps = true;
    if (!grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_, seed_state, filter_pregrasps,
                                     object_name))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGNAME), "Filter grasps failed");
      return false;
    }
    if (!grasp_filter_->removeInvalidAndFilter(grasp_candidates))
    {
      RCLCPP_WARN(rclcpp::get_logger(LOGNAME), "Grasp filtering removed all grasps");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGNAME), "" << grasp_candidates.size() << " remain after filtering");

    // Plan free-space approach, cartesian approach, lift and retreat trajectories
    moveit_grasps::GraspCandidatePtr selected_grasp_candidate;
    moveit_msgs::msg::MotionPlanResponse pre_approach_plan;
    if (!planFullGrasp(grasp_candidates, selected_grasp_candidate, pre_approach_plan, object_name))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGNAME), "Failed to plan grasp motions");
      return false;
    }

    visualizePick(selected_grasp_candidate, pre_approach_plan);

    return true;
  }

  void visualizePick(const moveit_grasps::GraspCandidatePtr& valid_grasp_candidate,
                     const moveit_msgs::msg::MotionPlanResponse& pre_approach_plan)
  {
    EigenSTL::vector_Isometry3d waypoints;
    moveit_grasps::GraspGenerator::getGraspWaypoints(valid_grasp_candidate, waypoints);

    // Visualize waypoints
    visual_tools_->publishAxisLabeled(waypoints[0], "pregrasp");
    visual_tools_->publishAxisLabeled(waypoints[1], "grasp");
    visual_tools_->publishAxisLabeled(waypoints[2], "lifted");
    visual_tools_->publishAxisLabeled(waypoints[3], "retreat");
    visual_tools_->trigger();

    // Get the pre and post grasp states
    visual_tools_->prompt("pre_grasp");
    moveit::core::RobotStatePtr pre_grasp_state =
        std::make_shared<moveit::core::RobotState>(*visual_tools_->getSharedRobotState());
    valid_grasp_candidate->getPreGraspState(pre_grasp_state);
    visual_tools_->publishRobotState(pre_grasp_state, rviz_visual_tools::ORANGE);
    moveit::core::RobotStatePtr grasp_state =
        std::make_shared<moveit::core::RobotState>(*visual_tools_->getSharedRobotState());
    if (valid_grasp_candidate->getGraspStateClosed(grasp_state))
    {
      visual_tools_->prompt("grasp");
      visual_tools_->publishRobotState(grasp_state, rviz_visual_tools::YELLOW);
    }
    if (valid_grasp_candidate->segmented_cartesian_traj_.size() > 1 &&
        valid_grasp_candidate->segmented_cartesian_traj_[1].size())
    {
      visual_tools_->prompt("lift");
      visual_tools_->publishRobotState(valid_grasp_candidate->segmented_cartesian_traj_[1].back(),
                                       rviz_visual_tools::BLUE);
    }
    if (valid_grasp_candidate->segmented_cartesian_traj_.size() > 2 &&
        valid_grasp_candidate->segmented_cartesian_traj_[2].size())
    {
      visual_tools_->prompt("retreat");
      visual_tools_->publishRobotState(valid_grasp_candidate->segmented_cartesian_traj_[2].back(),
                                       rviz_visual_tools::PURPLE);
    }

    visual_tools_->prompt("show free space approach");
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    bool wait_for_animation = true;
    visual_tools_->publishTrajectoryPath(pre_approach_plan.trajectory, pre_grasp_state, wait_for_animation);
    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);
    if (valid_grasp_candidate->segmented_cartesian_traj_.size() > moveit_grasps::APPROACH)
      visual_tools_->publishTrajectoryPath(valid_grasp_candidate->segmented_cartesian_traj_[moveit_grasps::APPROACH],
                                           valid_grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    rclcpp::sleep_for(1s);

    if (valid_grasp_candidate->segmented_cartesian_traj_.size() > moveit_grasps::LIFT)
      visual_tools_->publishTrajectoryPath(valid_grasp_candidate->segmented_cartesian_traj_[moveit_grasps::LIFT],
                                           valid_grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    rclcpp::sleep_for(1s);

    if (valid_grasp_candidate->segmented_cartesian_traj_.size() > moveit_grasps::RETREAT)
      visual_tools_->publishTrajectoryPath(valid_grasp_candidate->segmented_cartesian_traj_[moveit_grasps::RETREAT],
                                           valid_grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    rclcpp::sleep_for(1s);
  }

  bool planFullGrasp(std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidates,
                     moveit_grasps::GraspCandidatePtr& valid_grasp_candidate,
                     moveit_msgs::msg::MotionPlanResponse& pre_approach_plan, const std::string& object_name)
  {
    moveit::core::RobotStatePtr current_state;
    {
      boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
          new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));
      current_state = std::make_shared<moveit::core::RobotState>((*ls)->getCurrentState());
    }

    bool success = false;
    for (; !grasp_candidates.empty(); grasp_candidates.erase(grasp_candidates.begin()))
    {
      valid_grasp_candidate = grasp_candidates.front();
      valid_grasp_candidate->getPreGraspState(current_state);
      if (!grasp_planner_->planApproachLiftRetreat(valid_grasp_candidate, current_state, planning_scene_monitor_, false,
                                                   object_name))
      {
        RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "failed to plan approach lift retreat");
        continue;
      }

      moveit::core::RobotStatePtr pre_grasp_state =
          valid_grasp_candidate->segmented_cartesian_traj_[moveit_grasps::APPROACH].front();
      if (!planPreApproach(*pre_grasp_state, pre_approach_plan))
      {
        RCLCPP_WARN(rclcpp::get_logger(LOGNAME), "failed to plan to pregrasp_state");
        continue;
      }

      success = true;
      break;
    }
    return success;
  }

  bool planPreApproach(const moveit::core::RobotState& goal_state,
                       moveit_msgs::msg::MotionPlanResponse& pre_approach_plan)
  {
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    double tolerance_above = 0.01;
    double tolerance_below = 0.01;
    // req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = arm_jmg_->getName();
    req.num_planning_attempts = 5;
    req.allowed_planning_time = 1.5;
    moveit_msgs::msg::Constraints goal =
        kinematic_constraints::constructGoalConstraints(goal_state, arm_jmg_, tolerance_below, tolerance_above);

    req.goal_constraints.push_back(goal);
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
        new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));

    // ---------------------------------
    // Change the robot current state
    // NOTE: We have to do this since Panda start configuration is in self collision.
    moveit::core::RobotState rs = (*ls)->getCurrentState();
    std::vector<double> starting_joint_values = { 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785 };
    std::vector<std::string> joint_names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                             "panda_joint5", "panda_joint6", "panda_joint7" };
    // arm_jmg_->getActiveJointModelNames();
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      rs.setJointPositions(joint_names[i], &starting_joint_values[i]);
    }
    rs.update();
    moveit::core::robotStateToRobotStateMsg(rs, req.start_state);
    // ---------------------------

    planning_pipeline_->generatePlan(*ls, req, res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "Failed to plan approach successfully");
      return false;
    }

    res.getMessage(pre_approach_plan);
    return true;
  }

  bool getIKSolution(const moveit::core::JointModelGroup* arm_jmg, const Eigen::Isometry3d& target_pose,
                     moveit::core::RobotState& solution, const std::string& link_name)
  {
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
        new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));

    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(), visual_tools_, _1, _2, _3);

    // seed IK call with current state
    solution = (*ls)->getCurrentState();

    // Solve IK problem for arm
    // disable explicit restarts to guarantee close solution if one exists
    const double timeout = 0.1;
    return solution.setFromIK(arm_jmg, target_pose, link_name, timeout, constraint_fn);
  }

  bool generateRandomCuboid(std::string& object_name, geometry_msgs::msg::Pose& object_pose, double& x_depth,
                            double& y_width, double& z_height)
  {
    // Generate random cuboid
    double xmin = 0.5;
    double xmax = 0.7;
    double ymin = -0.25;
    double ymax = 0.25;
    double zmin = 0.2;
    double zmax = 0.7;
    rviz_visual_tools::RandomPoseBounds pose_bounds(xmin, xmax, ymin, ymax, zmin, zmax);

    double cuboid_size_min = 0.01;
    double cuboid_size_max = 0.03;
    rviz_visual_tools::RandomCuboidBounds cuboid_bounds(cuboid_size_min, cuboid_size_max);

    object_name = "pick_target";
    visual_tools_->generateRandomCuboid(object_pose, x_depth, y_width, z_height, pose_bounds, cuboid_bounds);
    visual_tools_->publishCollisionCuboid(object_pose, x_depth, y_width, z_height, object_name, rviz_visual_tools::RED);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();

    bool success = true;
    double timeout = 5;  // seconds
    rclcpp::Time start_time = nh_->get_clock()->now();
    while (success && !planning_scene_monitor_->getPlanningScene()->knowsFrameTransform(object_name))
    {
      using namespace std::chrono_literals;
      rclcpp::sleep_for(10ms);
      success = (nh_->get_clock()->now() - start_time).seconds() < timeout;
    }
    return success;
  }

private:
  // A shared node handle
  rclcpp::Node::SharedPtr nh_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // MoveIt! Grasps
  moveit_grasps::TwoFingerGraspGeneratorPtr grasp_generator_;

  // Robot-specific data for generating grasps
  moveit_grasps::TwoFingerGraspDataPtr grasp_data_;

  // For planning approach and retreats
  moveit_grasps::GraspPlannerPtr grasp_planner_;

  // For selecting good grasps
  moveit_grasps::TwoFingerGraspFilterPtr grasp_filter_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Arm
  const moveit::core::JointModelGroup* arm_jmg_;

  // Robot
  moveit::core::RobotModelPtr robot_model_;

  // All the motion planning components
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // Choose which arm to use
  std::string ee_group_name_;
  std::string planning_group_name_;

};  // end of class

}  // namespace moveit_grasps_demo

int main(int argc, char* argv[])
{
  int num_tests = 1;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto grasp_generator_demo_node = rclcpp::Node::make_shared("grasp_generator_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(grasp_generator_demo_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Seed random
  srand(grasp_generator_demo_node->get_clock()->now().seconds());

  // Benchmark time
  rclcpp::Time start_time;
  start_time = grasp_generator_demo_node->get_clock()->now();

  // Run Tests
  moveit_grasps_demo::GraspPipelineDemo tester;
  tester.demoRandomGrasp();

  // Benchmark time
  double duration = (grasp_generator_demo_node->get_clock()->now() - start_time).seconds();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("grasp_filter_demo"), "Total time: " << duration << "\t" << num_tests);
  std::cout << "Total time: " << duration << "\t" << num_tests << std::endl;

  using namespace std::chrono_literals;
  rclcpp::sleep_for(1s);  // let rviz markers finish publishing

  return 0;
}
