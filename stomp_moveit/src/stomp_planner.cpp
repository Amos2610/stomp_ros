/**
 * @file stomp_planner.cpp
 * @brief This defines the stomp planner for MoveIt
 *
 * @author Jorge Nicho
 * @date April 4, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <stomp_moveit/stomp_planner.h>
#include <class_loader/class_loader.hpp>
#include <stomp/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <stomp_moveit/utils/kinematics.h>
#include <stomp_moveit/utils/polynomial.h>

static const std::string DEBUG_NS = "stomp_planner";
static const std::string DESCRIPTION = "STOMP";
static const double TIMEOUT_INTERVAL = 0.5;
static int const IK_ATTEMPTS = 4;
static int const IK_TIMEOUT = 0.005;
const static double MAX_START_DISTANCE_THRESH = 0.5;

/**
 * @brief Parses a XmlRpcValue and populates a StompComfiguration structure.
 * @param config        The XmlRpcValue of stomp configuration parameters
 * @param group         The moveit planning group
 * @param stomp_config  The stomp configuration structure
 * @return True if successfully parsed, otherwise false.
 */
bool parseConfig(XmlRpc::XmlRpcValue config,const moveit::core::JointModelGroup* group,stomp::StompConfiguration& stomp_config)
{
  using namespace XmlRpc;
  // Set default values for optional config parameters
  stomp_config.control_cost_weight = 0.0;
  stomp_config.initialization_method = 1; // LINEAR_INTERPOLATION
  stomp_config.num_timesteps = 40;
  stomp_config.delta_t = 1.0;
  stomp_config.num_iterations = 50;
  stomp_config.num_iterations_after_valid = 0;
  stomp_config.max_rollouts = 100;
  stomp_config.num_rollouts = 10;
  stomp_config.exponentiated_cost_sensitivity = 10.0;

  // Load optional config parameters if they exist
  if (config.hasMember("control_cost_weight"))
    stomp_config.control_cost_weight = static_cast<double>(config["control_cost_weight"]);

  if (config.hasMember("initialization_method"))
    stomp_config.initialization_method = static_cast<int>(config["initialization_method"]);

  if (config.hasMember("num_timesteps"))
    stomp_config.num_timesteps = static_cast<int>(config["num_timesteps"]);

  if (config.hasMember("delta_t"))
    stomp_config.delta_t = static_cast<double>(config["delta_t"]);

  if (config.hasMember("num_iterations"))
    stomp_config.num_iterations = static_cast<int>(config["num_iterations"]);

  if (config.hasMember("num_iterations_after_valid"))
    stomp_config.num_iterations_after_valid = static_cast<int>(config["num_iterations_after_valid"]);

  if (config.hasMember("max_rollouts"))
    stomp_config.max_rollouts = static_cast<int>(config["max_rollouts"]);

  if (config.hasMember("num_rollouts"))
    stomp_config.num_rollouts = static_cast<int>(config["num_rollouts"]);

  if (config.hasMember("exponentiated_cost_sensitivity"))
    stomp_config.exponentiated_cost_sensitivity = static_cast<int>(config["exponentiated_cost_sensitivity"]);

  // getting number of joints
  stomp_config.num_dimensions = group->getActiveJointModels().size();
  if(stomp_config.num_dimensions == 0)
  {
    ROS_ERROR("Planning Group %s has no active joints",group->getName().c_str());
    return false;
  }

  return true;
}

namespace stomp_moveit
{

StompPlanner::StompPlanner(const std::string& group,const XmlRpc::XmlRpcValue& config,
                           const moveit::core::RobotModelConstPtr& model):
    PlanningContext(DESCRIPTION,group),
    config_(config),
    robot_model_(model),
    ik_solver_(new utils::kinematics::IKSolver(model,group)),
    ph_(new ros::NodeHandle("~"))
{
  setup();
}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::setup()
{
  if(!getPlanningScene())
  {
    setPlanningScene(planning_scene::PlanningSceneConstPtr(new planning_scene::PlanningScene(robot_model_)));
  }

  // loading parameters
  try
  {
    // creating tasks
    XmlRpc::XmlRpcValue task_config;
    task_config = config_["task"];
    task_.reset(new StompOptimizationTask(robot_model_,group_,task_config));

    if(!robot_model_->hasJointModelGroup(group_))
    {
      std::string msg = "Stomp Planning Group '" + group_ + "' was not found";
      ROS_ERROR("%s",msg.c_str());
      throw std::logic_error(msg);
    }

    // parsing stomp parameters
    if(!config_.hasMember("optimization") || !parseConfig(config_["optimization" ],robot_model_->getJointModelGroup(group_),stomp_config_))
    {
      std::string msg = "Stomp 'optimization' parameter for group '" + group_ + "' failed to load";
      ROS_ERROR("%s", msg.c_str());
      throw std::logic_error(msg);
    }

    stomp_.reset(new stomp::Stomp(stomp_config_,task_));
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    throw std::logic_error("Stomp Planner failed to load configuration for group '" + group_+"'; " + e.getMessage());
  }

}

bool StompPlanner::solve(planning_interface::MotionPlanResponse &res)
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.back();
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = ros::Duration(wd.sec, wd.nsec).toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool StompPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  std::string userInput;
  std::cout << "Do you want to use the initial trajectory you set (y/n): ";
  std::getline(std::cin, userInput);
  using namespace stomp;

  // initializing response
  res.description_.resize(1,"plan");
  res.processing_time_.resize(1);
  res.trajectory_.resize(1);
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  ros::WallTime start_time = ros::WallTime::now();
  bool success = false;

  trajectory_msgs::JointTrajectory trajectory;
  Eigen::MatrixXd parameters;
  bool planning_success;

  // local stomp config copy
  auto config_copy = stomp_config_;

  // look for seed trajectory
  Eigen::MatrixXd initial_parameters;
  bool use_seed = getSeedParameters(initial_parameters);

  // create timeout timer
  ros::WallDuration allowed_time(request_.allowed_planning_time);
  ROS_WARN_COND(TIMEOUT_INTERVAL > request_.allowed_planning_time,
                "%s allowed planning time %f is less than the minimum planning time value of %f",
                getName().c_str(),request_.allowed_planning_time,TIMEOUT_INTERVAL);
  std::atomic<bool> terminating(false);
  ros::Timer timeout_timer = ph_->createTimer(ros::Duration(TIMEOUT_INTERVAL), [&](const ros::TimerEvent& evnt)
  {
    if(((ros::WallTime::now() - start_time) > allowed_time))
    {
      ROS_ERROR_COND(!terminating,"%s exceeded allowed time of %f , terminating",getName().c_str(),allowed_time.toSec());
      this->terminate();
      terminating = true;
    }

  },false);

  if (use_seed)
  {
    ROS_INFO("%s Seeding trajectory from MotionPlanRequest",getName().c_str());

    // updating time step in stomp configuraion
    config_copy.num_timesteps = initial_parameters.cols();
    
    // setting up up optimization task
    if(!task_->setMotionPlanRequest(planning_scene_, request_, config_copy, res.error_code_))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    stomp_->setConfig(config_copy);
    planning_success = stomp_->solve(initial_parameters, parameters);
  }

  else {

    if (userInput == "y" || userInput == "Y") {
      ROS_ERROR("Seeding trajectory from the initial trajectory you set!!!");
      // int seed = 1;
      // std::cout << "Enter the seed number: ";
      // std::cin >> seed;
      std::string userInput2;
      std::cout << "watanabe";
      std::getline(std::cin, userInput2);

      if (userInput2 == "1") {
          const int rows = 12;
          const int cols = 6;
          // printf("rows: %d, cols: %d\n", rows, cols);
          // initial_parameters.resize(rows, cols);
          // printf("rows: %d, cols: %d\n", initial_parameters.rows(), initial_parameters.cols());
          Eigen::MatrixXd initial_parameters(rows, cols);
            /* ここに行列の初期値を設定 */
          initial_parameters <<
                0.7760606615384615, -0.07116415999999996, -1.5177242676923077, 0.0011898815384615387, 1.589898823076923, 0.7756431069230769,
                0.8200456930769231, 0.028724220000000012, -1.5731555053846156, 0.001239543076923077, 1.545368796153846, 0.8195588838461538,
                0.8640307246153847, 0.1286126, -1.6285867430769232, 0.0012892046153846152, 1.5008387692307692, 0.8634746607692307,
                0.9080157561538462, 0.22850098000000008, -1.684017980769231, 0.0013388661538461539, 1.4563087423076921, 0.9073904376923078,
                0.9520007876923077, 0.32838936, -1.7394492184615384, 0.0013885276923076926, 1.4117787153846155, 0.9513062146153849,
                0.9959858192307692, 0.42827773999999996, -1.7948804561538463, 0.0014381892307692309, 1.3672486884615382, 0.9952219915384616,
                1.039970850769231, 0.5281661199999999, -1.850311693846154, 0.0014878507692307691, 1.3227186615384616, 1.0391377684615384,
                1.0839558823076925, 0.6280545000000001, -1.9057429315384617, 0.0015375123076923076, 1.2781886346153843, 1.0830535453846155,
                1.127940913846154, 0.7279428800000004, -1.9611741692307694, 0.0015871738461538465, 1.2336586076923077, 1.1269693223076926,
                1.1719259453846154, 0.82783126, -2.0166054069230768, 0.0016368353846153848, 1.1891285807692307, 1.1708850992307693,
                1.215910976923077, 0.9277196400000001, -2.072036644615385, 0.001686496923076923, 1.1445985538461536, 1.2148008761538462,
                1.2598960084615385, 1.02760802, -2.1274678823076925, 0.001736158461538462, 1.1000685269230763, 1.2587166530769232;
          
          std::cout << "initial_parameters:" << std::endl;
          std::cout << initial_parameters << std::endl;

          Eigen::MatrixXd initial_parameters_transpose = initial_parameters.transpose();
          std::cout << "initial_parameters_transpose:" << std::endl;
          std::cout << initial_parameters_transpose << std::endl;
          // updating time step in stomp configuraion
          config_copy.num_timesteps = initial_parameters_transpose.cols();

          // setting up up optimization task
          if(!task_->setMotionPlanRequest(planning_scene_, request_, config_copy, res.error_code_))
          {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
            return false;
          }
          stomp_->setConfig(config_copy);
          planning_success = stomp_->solve(initial_parameters_transpose, parameters);

      } 
      else if (userInput2 == "2") {
          const int rows = 12;
          const int cols = 6;
          initial_parameters.resize(rows, cols);
          Eigen::MatrixXd initial_parameters(rows, cols);
          initial_parameters <<
                1.2598960084615385, 1.02760802, -2.1274678823076925, 0.001736158461538462, 1.1000685269230763, 1.2587166530769232,
                1.215910976923077, 0.9277196400000001, -2.072036644615385, 0.001686496923076923, 1.1445985538461536, 1.2148008761538462,
                1.1719259453846154, 0.82783126, -2.0166054069230768, 0.0016368353846153848, 1.1891285807692307, 1.1708850992307693,
                1.127940913846154, 0.7279428800000004, -1.9611741692307694, 0.0015871738461538465, 1.2336586076923077, 1.1269693223076926,
                1.0839558823076925, 0.6280545000000001, -1.9057429315384617, 0.0015375123076923076, 1.2781886346153843, 1.0830535453846155,
                1.039970850769231, 0.5281661199999999, -1.850311693846154, 0.0014878507692307691, 1.3227186615384616, 1.0391377684615384,
                0.9959858192307692, 0.42827773999999996, -1.7948804561538463, 0.0014381892307692309, 1.3672486884615382, 0.9952219915384616,
                0.9520007876923077, 0.32838936, -1.7394492184615384, 0.0013885276923076926, 1.4117787153846155, 0.9513062146153849,
                0.9080157561538462, 0.22850098000000008, -1.684017980769231, 0.0013388661538461539, 1.4563087423076921, 0.9073904376923078,
                0.8640307246153847, 0.1286126, -1.6285867430769232, 0.0012892046153846152, 1.5008387692307692, 0.8634746607692307,
                0.8200456930769231, 0.028724220000000012, -1.5731555053846156, 0.001239543076923077, 1.545368796153846, 0.8195588838461538,
                0.7760606615384615, -0.07116415999999996, -1.5177242676923077, 0.0011898815384615387, 1.589898823076923, 0.7756431069230769;

          Eigen::MatrixXd initial_parameters_transpose = initial_parameters.transpose();
          std::cout << "initial_parameters_transpose:" << std::endl;
          std::cout << initial_parameters_transpose << std::endl;
          // updating time step in stomp configuraion
          config_copy.num_timesteps = initial_parameters_transpose.cols();

          // setting up up optimization task
          if(!task_->setMotionPlanRequest(planning_scene_, request_, config_copy, res.error_code_))
          {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
            return false;
          }
          stomp_->setConfig(config_copy);
          planning_success = stomp_->solve(initial_parameters_transpose, parameters);
      } 
      else {
          std::cout << "Invalid seed number" << std::endl;
          return 1;
      }

      // const int rows = 12;
      // const int cols = 6;
      // Eigen::MatrixXd initial_parameters(rows, cols);
      
      // // TODO: initial_parameters should be set by the user
      
      // initial_parameters <<
      //       0.7760606615384615, -0.07116415999999996, -1.5177242676923077, 0.0011898815384615387, 1.589898823076923, 0.7756431069230769,
      //       0.8200456930769231, 0.028724220000000012, -1.5731555053846156, 0.001239543076923077, 1.545368796153846, 0.8195588838461538,
      //       0.8640307246153847, 0.1286126, -1.6285867430769232, 0.0012892046153846152, 1.5008387692307692, 0.8634746607692307,
      //       0.9080157561538462, 0.22850098000000008, -1.684017980769231, 0.0013388661538461539, 1.4563087423076921, 0.9073904376923078,
      //       0.9520007876923077, 0.32838936, -1.7394492184615384, 0.0013885276923076926, 1.4117787153846155, 0.9513062146153849,
      //       0.9959858192307692, 0.42827773999999996, -1.7948804561538463, 0.0014381892307692309, 1.3672486884615382, 0.9952219915384616,
      //       1.039970850769231, 0.5281661199999999, -1.850311693846154, 0.0014878507692307691, 1.3227186615384616, 1.0391377684615384,
      //       1.0839558823076925, 0.6280545000000001, -1.9057429315384617, 0.0015375123076923076, 1.2781886346153843, 1.0830535453846155,
      //       1.127940913846154, 0.7279428800000004, -1.9611741692307694, 0.0015871738461538465, 1.2336586076923077, 1.1269693223076926,
      //       1.1719259453846154, 0.82783126, -2.0166054069230768, 0.0016368353846153848, 1.1891285807692307, 1.1708850992307693,
      //       1.215910976923077, 0.9277196400000001, -2.072036644615385, 0.001686496923076923, 1.1445985538461536, 1.2148008761538462,
      //       1.2598960084615385, 1.02760802, -2.1274678823076925, 0.001736158461538462, 1.1000685269230763, 1.2587166530769232;

      // std::cout << "initial_parameters:" << std::endl;
      // std::cout << initial_parameters << std::endl;
      
      // Eigen::MatrixXd initial_parameters_transpose = initial_parameters.transpose();
      // std::cout << "initial_parameters_transpose:" << std::endl;
      // std::cout << initial_parameters_transpose << std::endl;
      // // updating time step in stomp configuraion
      // config_copy.num_timesteps = initial_parameters_transpose.cols();

      // // setting up up optimization task
      // if(!task_->setMotionPlanRequest(planning_scene_, request_, config_copy, res.error_code_))
      // {
      //   res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      //   return false;
      // }
      // stomp_->setConfig(config_copy);
      // planning_success = stomp_->solve(initial_parameters_transpose, parameters);
    }
    if (userInput == "n" || userInput == "N") {
      // extracting start and goal
      Eigen::VectorXd start, goal;
      if(!getStartAndGoal(start,goal))
      {
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
        return false;
      }

      // setting up up optimization task
      if(!task_->setMotionPlanRequest(planning_scene_,request_, config_copy,res.error_code_))
      {
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
      }
      stomp_->setConfig(config_copy);
      planning_success = stomp_->solve(start,goal,parameters);
    }
  }

  // stopping timer
  timeout_timer.stop();

  // Handle results
  if(planning_success)
  {
    if(!parametersToJointTrajectory(parameters,trajectory))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    // creating request response
    moveit::core::RobotState robot_state = planning_scene_->getCurrentState();
    moveit::core::robotStateMsgToRobotState(request_.start_state,robot_state);
    res.trajectory_[0]= robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(
        robot_model_,group_));
    res.trajectory_.back()->setRobotTrajectoryMsg( robot_state,trajectory);
  }
  else
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // checking against planning scene
  if(planning_scene_ && !planning_scene_->isPathValid(*res.trajectory_.back(),group_,true))
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    success = false;
    ROS_ERROR_STREAM("STOMP Trajectory is in collision");
  }

  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.processing_time_[0] = ros::Duration(wd.sec, wd.nsec).toSec();
  ROS_INFO_STREAM("STOMP found a valid path after "<<res.processing_time_[0]<<" seconds");

  return true;
}

bool StompPlanner::getSeedParameters(Eigen::MatrixXd& parameters) const
{
  using namespace utils::kinematics;
  using namespace utils::polynomial;

  auto within_tolerance = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tol) -> bool
  {
    double dist = (a - b).cwiseAbs().sum();
    return dist <= tol;
  };

  trajectory_msgs::JointTrajectory traj;
  if(!extractSeedTrajectory(request_,traj))
  {
    ROS_DEBUG("%s Found no seed trajectory",getName().c_str());
    return false;
  }

  if(!jointTrajectorytoParameters(traj,parameters))
  {
    ROS_ERROR("%s Failed to created seed parameters from joint trajectory",getName().c_str());
    return false;
  }

  if(parameters.cols()<= 2)
  {
    ROS_ERROR("%s Found less than 3 points in seed trajectory",getName().c_str());
    return false;
  }

  /* ********************************************************************************
   * Validating seed trajectory by ensuring that it does obey the
   * motion plan request constraints
   */
  moveit::core::RobotState state = planning_scene_->getCurrentState();
  const auto* group = state.getJointModelGroup(group_);
  const auto& joint_names = group->getActiveJointModelNames();
  const auto& tool_link = group->getLinkModelNames().back();
  Eigen::VectorXd start, goal;

  // We check to see if the start state in the request and the seed state are 'close'
  if (moveit::core::robotStateMsgToRobotState(request_.start_state, state))
  {

    if(!state.satisfiesBounds(group))
    {
      ROS_ERROR("Start state is out of bounds");
      return false;
    }

    // copying start joint values
    start.resize(joint_names.size());
    for(auto j = 0u; j < joint_names.size(); j++)
    {
      start(j) = state.getVariablePosition(joint_names[j]);
    }

    if(within_tolerance(parameters.leftCols(1),start,MAX_START_DISTANCE_THRESH))
    {
      parameters.leftCols(1) = start;
    }
    else
    {
      ROS_ERROR("%s Start State is in discrepancy with the seed trajectory",getName().c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s Failed to get start state joints",getName().c_str());
    return false;
  }

  // We now extract the goal and make sure that the seed's goal obeys the goal constraints
  bool found_goal = false;
  goal = parameters.rightCols(1); // initializing goal;
  for(auto& gc : request_.goal_constraints)
  {
    if(!gc.joint_constraints.empty())
    {
      // copying goal values into state
      for(auto j = 0u; j < gc.joint_constraints.size() ; j++)
      {
        auto jc = gc.joint_constraints[j];
        state.setVariablePosition(jc.joint_name,jc.position);
      }

      // copying values into goal array
      if(!state.satisfiesBounds(group))
      {
        ROS_ERROR("%s Requested Goal joint pose is out of bounds",getName().c_str());
        continue;
      }

      for(auto j = 0u; j < joint_names.size(); j++)
      {
        goal(j) = state.getVariablePosition(joint_names[j]);
      }

      found_goal = true;
      break;
    }

    // now check Cartesian constraint
    state.updateLinkTransforms();
    Eigen::Affine3d start_tool_pose = state.getGlobalLinkTransform(tool_link);
    boost::optional<moveit_msgs::Constraints> tool_constraints = curateCartesianConstraints(gc,start_tool_pose);
    if(!tool_constraints.is_initialized())
    {
      ROS_WARN("Cartesian Goal could not be created from provided constraints");
      found_goal = true;
      break;
    }

    Eigen::VectorXd solution;
    ik_solver_->setKinematicState(state);
    if(ik_solver_->solve(goal,tool_constraints.get(),solution))
    {
      goal = solution;
      found_goal = true;
      break;
    }
    else
    {
      ROS_ERROR("A valid ik solution for the given Cartesian constraints was not found ");
      ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"IK failed with goal constraint \n"<<tool_constraints.get());
      ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"Reference Tool pose used was: \n"<<start_tool_pose.matrix());
    }
  }

  // forcing the goal into the seed trajectory
  if(found_goal)
  {
    if(within_tolerance(parameters.rightCols(1),goal,MAX_START_DISTANCE_THRESH))
    {
      parameters.rightCols(1) = goal;
    }
    else
    {
      ROS_ERROR("%s Goal in seed is too far away from requested goal constraints",getName().c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s requested goal constraint was invalid or unreachable, comparison with goal in seed isn't possible",getName().c_str());
    return false;
  }

  if(!applyPolynomialSmoothing(robot_model_,group_,parameters,5,1e-5))
  {
    return false;
  }

  return true;
}

bool StompPlanner::parametersToJointTrajectory(const Eigen::MatrixXd& parameters,
                                               trajectory_msgs::JointTrajectory& trajectory)
{
  // filling trajectory joint values
  trajectory.joint_names = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames();
  trajectory.points.clear();
  trajectory.points.resize(parameters.cols());
  std::vector<double> vals(parameters.rows());
  std::vector<double> zeros(parameters.rows(),0.0);
  for(auto t = 0u; t < parameters.cols() ; t++)
  {
    Eigen::VectorXd::Map(&vals[0],vals.size()) = parameters.col(t);
    trajectory.points[t].positions = vals;
    trajectory.points[t].velocities = zeros;
    trajectory.points[t].accelerations = zeros;
    trajectory.points[t].time_from_start = ros::Duration(0.0);
  }

  trajectory_processing::IterativeParabolicTimeParameterization time_generator;
  robot_trajectory::RobotTrajectory traj(robot_model_,group_);
  moveit::core::RobotState robot_state = planning_scene_->getCurrentState();
  if(!moveit::core::robotStateMsgToRobotState(request_.start_state,robot_state))
  {
    return false;
  }

  traj.setRobotTrajectoryMsg(robot_state,trajectory);

  // converting to msg
  moveit_msgs::RobotTrajectory robot_traj_msgs;
  if(time_generator.computeTimeStamps(traj,request_.max_velocity_scaling_factor))
  {
    traj.getRobotTrajectoryMsg(robot_traj_msgs);
    trajectory = robot_traj_msgs.joint_trajectory;
  }
  else
  {
    ROS_ERROR("%s Failed to generate timing data",getName().c_str());
    return false;
  }
  return true;
}

bool StompPlanner::jointTrajectorytoParameters(const trajectory_msgs::JointTrajectory& traj, Eigen::MatrixXd& parameters) const
{
  const auto dof = traj.joint_names.size();
  const auto timesteps = traj.points.size();

  Eigen::MatrixXd mat (dof, timesteps);

  for (size_t step = 0; step < timesteps; ++step)
  {
    for (size_t joint = 0; joint < dof; ++joint)
    {
      mat(joint, step) = traj.points[step].positions[joint];
    }
  }

  parameters = mat;
  return true;
}

bool StompPlanner::extractSeedTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const
{
  if (req.trajectory_constraints.constraints.empty())
    return false;

  const auto* joint_group = robot_model_->getJointModelGroup(group_);
  const auto& names = joint_group->getActiveJointModelNames();
  const auto dof = names.size();

  const auto& constraints = req.trajectory_constraints.constraints; // alias to keep names short
  // Test the first point to ensure that it has all of the joints required
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    auto n = constraints[i].joint_constraints.size();
    if (n != dof) // first test to ensure that dimensionality is correct
    {
      ROS_WARN("Seed trajectory index %lu does not have %lu constraints (has %lu instead).", i, dof, n);
      return false;
    }

    trajectory_msgs::JointTrajectoryPoint joint_pt;

    for (size_t j = 0; j < constraints[i].joint_constraints.size(); ++j)
    {
      const auto& c = constraints[i].joint_constraints[j];
      if (c.joint_name != names[j])
      {
        ROS_WARN("Seed trajectory (index %lu, joint %lu) joint name '%s' does not match expected name '%s'",
                 i, j, c.joint_name.c_str(), names[j].c_str());
        return false;
      }
      joint_pt.positions.push_back(c.position);
    }

    seed.points.push_back(joint_pt);
  }

  seed.joint_names = names;
  return true;
}

moveit_msgs::TrajectoryConstraints StompPlanner::encodeSeedTrajectory(const trajectory_msgs::JointTrajectory &seed)
{
  ROS_INFO_STREAM("encodeSeedTrajectory関数を使用します");

  moveit_msgs::TrajectoryConstraints res;

  const auto dof = seed.joint_names.size();

  for (size_t i = 0; i < seed.points.size(); ++i) // for each time step
  {
    moveit_msgs::Constraints c;

    if (seed.points[i].positions.size() != dof)
      throw std::runtime_error("All trajectory position fields must have same dimensions as joint_names");

    for (size_t j = 0; j < dof; ++j) // for each joint
    {
      moveit_msgs::JointConstraint jc;
      jc.joint_name = seed.joint_names[j];
      jc.position = seed.points[i].positions[j];

      c.joint_constraints.push_back(jc);
    }

    res.constraints.push_back(std::move(c));
  }

  return res;
}

bool StompPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  using namespace moveit::core;
  using namespace utils::kinematics;

  RobotStatePtr state(new RobotState(planning_scene_->getCurrentState()));
  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_);
  std::string tool_link = joint_group->getLinkModelNames().back();
  bool found_goal = false;

  try
  {
    // copying start state
    if(!robotStateMsgToRobotState(request_.start_state,*state))
    {
      ROS_ERROR("%s Failed to extract start state from MotionPlanRequest",getName().c_str());
      return false;
    }

    // copying start joint values
    const std::vector<std::string> joint_names= state->getJointModelGroup(group_)->getActiveJointModelNames();
    start.resize(joint_names.size());
    goal.resize(joint_names.size());

    if(!state->satisfiesBounds(state->getJointModelGroup(group_)))
    {
      ROS_ERROR("%s Start joint pose is out of bounds",getName().c_str());
      return false;
    }

    for(auto j = 0u; j < joint_names.size(); j++)
    {
      start(j) = state->getVariablePosition(joint_names[j]);
    }

    // check goal constraint
    if(request_.goal_constraints.empty())
    {
      ROS_ERROR("%s A goal constraint was not provided",getName().c_str());
      return false;
    }

    // extracting goal joint values
    for(const auto& gc : request_.goal_constraints)
    {

      // check joint constraints first
      if(!gc.joint_constraints.empty())
      {

        // copying goal values into state
        for(auto j = 0u; j < gc.joint_constraints.size() ; j++)
        {
          auto jc = gc.joint_constraints[j];
          state->setVariablePosition(jc.joint_name,jc.position);
        }


        if(!state->satisfiesBounds(state->getJointModelGroup(group_)))
        {
          ROS_ERROR("%s Requested Goal joint pose is out of bounds",getName().c_str());
          continue;
        }

        ROS_DEBUG("%s Found goal from joint constraints",getName().c_str());

        // copying values into goal array
        for(auto j = 0u; j < joint_names.size(); j++)
        {
          goal(j) = state->getVariablePosition(joint_names[j]);
        }

        found_goal = true;
        break;

      }

      // now check cartesian constraint
      state->updateLinkTransforms();
      Eigen::Affine3d start_tool_pose = state->getGlobalLinkTransform(tool_link);
      boost::optional<moveit_msgs::Constraints> tool_constraints = curateCartesianConstraints(gc,start_tool_pose);
      if(!tool_constraints.is_initialized())
      {
        ROS_WARN("Cartesian Goal could not be created from provided constraints");
        found_goal = true;
        break;
      }

      // now solve ik
      Eigen::VectorXd solution;
      Eigen::VectorXd seed = start;
      ik_solver_->setKinematicState(*state);
      if(ik_solver_->solve(seed,tool_constraints.get(),solution))
      {
        goal = solution;
        found_goal = true;
        break;
      }
      else
      {
        ROS_ERROR("A valid ik solution for the given Cartesian constraints was not found ");
        ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"IK failed with goal constraint \n"<<tool_constraints.get());
        ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"Reference Tool pose used was: \n"<<start_tool_pose.matrix());
      }
    }

    ROS_ERROR_COND(!found_goal,"%s was unable to retrieve the goal from the MotionPlanRequest",getName().c_str());

  }
  catch(moveit::Exception &e)
  {
    ROS_ERROR("Failure retrieving start or goal state joint values from request %s", e.what());
    return false;
  }

  return found_goal;
}


bool StompPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  // check group
  if(req.group_name != getGroupName())
  {
    ROS_ERROR("STOMP: Unsupported planning group '%s' requested", req.group_name.c_str());
    return false;
  }

  // check for single goal region
  if (req.goal_constraints.size() != 1)
  {
    ROS_ERROR("STOMP: Can only handle a single goal region.");
    return false;
  }

  // check that we have joint or cartesian constraints at the goal
  const auto& gc = req.goal_constraints[0];
  if ((gc.joint_constraints.size() == 0) &&
      !utils::kinematics::isCartesianConstraints(gc))
  {
    ROS_ERROR("STOMP couldn't find either a joint or cartesian goal.");
    return false;
  }

  return true;
}

bool StompPlanner::terminate()
{
  if(stomp_)
  {
    if(!stomp_->cancel())
    {
      ROS_ERROR_STREAM("Failed to interrupt Stomp");
      return false;
    }
  }
  return true;
}

void StompPlanner::clear()
{
  stomp_->clear();
}

bool StompPlanner::getConfigData(ros::NodeHandle &nh, std::map<std::string, XmlRpc::XmlRpcValue> &config, std::string param)
{
  // Create a stomp planner for each group
  XmlRpc::XmlRpcValue stomp_config;
  if(!nh.getParam(param, stomp_config))
  {
    ROS_ERROR("The 'stomp' configuration parameter was not found");
    return false;
  }

  // each element under 'stomp' should be a group name
  std::string group_name;
  try
  {
    for(XmlRpc::XmlRpcValue::iterator v = stomp_config.begin(); v != stomp_config.end(); v++)
    {
      group_name = static_cast<std::string>(v->second["group_name"]);
      config.insert(std::make_pair(group_name, v->second));
    }
    return true;
  }
  catch(XmlRpc::XmlRpcException& e )
  {
    ROS_ERROR("Unable to parse ROS parameter:\n %s",stomp_config.toXml().c_str());
    return false;
  }
}


} /* namespace stomp_moveit_interface */

