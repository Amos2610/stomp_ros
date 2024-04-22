/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Henning Kayser
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
 *   * Neither the name of Raghavender Sahdev nor the names of its
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

/* Author: Henning Kayser */

// ROS
#include <ros/ros.h>
#include <class_loader/class_loader.hpp>

// MoveIt
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/planning_interface/planning_interface.h>

// STOMP
#include <stomp_moveit/stomp_planner.h>

namespace stomp_moveit
{
class StompSmoothingAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter()
  {
    ros::NodeHandle nh("~");
    initialize(nh);
  }

  // todo[noetic] add override again
  virtual void initialize(const ros::NodeHandle& node_handle)
  {
    ros::NodeHandle nh(node_handle);
    if (!StompPlanner::getConfigData(nh, group_config_))
      ROS_ERROR("Unable to find valid group config for StompSmoothingAdapter");
  }

  virtual std::string getDescription() const override
  {
    return "Stomp Smoothing Adapter";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override
  {
    // Following call to planner() calls the OMPL planner and stores the trajectory inside the MotionPlanResponse res
    // variable which is then used by STOMP for optimization of the computed trajectory
    if (!planner(ps, req, res))
      return false;

    // STOMP reads the seed trajectory from trajectory constraints so we need to convert the waypoints first
    const size_t seed_waypoint_count = res.trajectory_->getWayPointCount();
    const std::vector<std::string> variable_names =
      res.trajectory_->getFirstWayPoint().getJointModelGroup(req.group_name)->getVariableNames();
    const size_t variable_count = variable_names.size();
    planning_interface::MotionPlanRequest seed_req = req;
    seed_req.trajectory_constraints.constraints.clear();
    seed_req.trajectory_constraints.constraints.resize(seed_waypoint_count);
    for (size_t i = 0; i < seed_waypoint_count; ++i)
    {
      seed_req.trajectory_constraints.constraints[i].joint_constraints.resize(variable_count);
      for (size_t j = 0; j < variable_count; ++j)
      {
        seed_req.trajectory_constraints.constraints[i].joint_constraints[j].joint_name = variable_names[j];
        seed_req.trajectory_constraints.constraints[i].joint_constraints[j].position =
          res.trajectory_->getWayPoint(i).getVariablePosition(variable_names[j]);
      }
    }

    // Get group config
    const auto& group_config_it = group_config_.find(req.group_name);
    if (group_config_it == group_config_.end())
    {
      ROS_ERROR_STREAM("STOMP is not configured for planning group " << req.group_name);
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    // Initialize STOMP Planner
    stomp_moveit::StompPlanner stomp_planner(req.group_name, group_config_it->second, ps->getRobotModel());
    if(!stomp_planner.canServiceRequest(seed_req))
    {
      ROS_ERROR("STOMP planner unable to service request");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    // Setup Planning Context
    stomp_planner.clear();
    stomp_planner.setPlanningScene(ps);
    stomp_planner.setMotionPlanRequest(seed_req);

    // Solve
    ROS_DEBUG("Smoothing result trajectory with STOMP");
    planning_interface::MotionPlanDetailedResponse stomp_res;
    bool success = stomp_planner.solve(stomp_res);
    if (success)
    {
      // Successful responses always contain one entry for trajectory_ and proccessing_time_
      res.trajectory_ = stomp_res.trajectory_.back();
      res.planning_time_ += stomp_res.processing_time_.back();
    }
    res.error_code_ = stomp_res.error_code_;
    return success;
  }

private:
  std::map<std::string, XmlRpc::XmlRpcValue> group_config_;
};
}  // namespace stomp_moveit

CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompSmoothingAdapter, planning_request_adapter::PlanningRequestAdapter);

// ------------------------------------------------------------------------------------------------------------------------------------------------------

// // ROS
// #include <ros/ros.h>
// #include <class_loader/class_loader.hpp>

// // MoveIt
// #include <moveit/planning_request_adapter/planning_request_adapter.h>
// #include <moveit/planning_interface/planning_interface.h>

// // STOMP
// #include <stomp_moveit/stomp_planner.h>

// namespace stomp_moveit
// {
// class StompSmoothingAdapter : public planning_request_adapter::PlanningRequestAdapter
// {
// public:
//   StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter()
//   {
//     ros::NodeHandle nh("~");
//     initialize(nh);
//   }

//   // todo[noetic] add override again
//   virtual void initialize(const ros::NodeHandle& node_handle)
//   {
//     ros::NodeHandle nh(node_handle);
//     if (!StompPlanner::getConfigData(nh, group_config_))
//       ROS_ERROR("Unable to find valid group config for StompSmoothingAdapter");
//   }

//   virtual std::string getDescription() const override
//   {
//     return "Stomp Smoothing Adapter";
//   }

//   bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
//                     const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
//                     std::vector<std::size_t>& added_path_index) const override
//   {
//     // Following call to planner() calls the OMPL planner and stores the trajectory inside the MotionPlanResponse res
//     // variable which is then used by STOMP for optimization of the computed trajectory
//     if (!planner(ps, req, res))
//       return false;

//     const size_t seed_waypoint_count = res.trajectory_->getWayPointCount(); // seed_waypoint_countにウェイポイントの数を格納
//     const std::vector<std::string> variable_names = res.trajectory_->getFirstWayPoint().getJointModelGroup(req.group_name)->getVariableNames(); // variable_namesに全ての関節名を格納
//     const size_t variable_count = variable_names.size(); // variable_namesの要素数(関節数)をvariable_countに格納
//     planning_interface::MotionPlanRequest seed_req = req;
//     seed_req.trajectory_constraints.constraints.clear(); // 制約をクリア

//     std::string userInput;
//     std::cout << "Do you want to use the SeedPath? (y/n): ";
//     std::getline(std::cin, userInput);

//     if (userInput == "n" || userInput == "N") {
//         std::cout << "Executing..." << std::endl;
//         seed_req.trajectory_constraints.constraints.resize(seed_waypoint_count); // 制約の数をウェイポイントの数に合わせる
//         for (size_t i = 0; i < seed_waypoint_count; ++i) // ウェイポイントの数だけループ
//         {
//           seed_req.trajectory_constraints.constraints[i].joint_constraints.resize(variable_count); // 各ウェイポイントに対する制約条件の数をvariable_countに合わせる
//           for (size_t j = 0; j < variable_count; ++j) // 関節数だけループ
//           {
//             seed_req.trajectory_constraints.constraints[i].joint_constraints[j].joint_name = variable_names[j]; // ウェイポイント[i]の関節[j]関節名を格納
//             seed_req.trajectory_constraints.constraints[i].joint_constraints[j].position = res.trajectory_->getWayPoint(i).getVariablePosition(variable_names[j]); // ウェイポイント[i]の関節[j]の目標位置を格納
//           }
//         }
//         //ROS_INFO_STREAM("seed_req: " << seed_req); // seed_reqの表示
//     }
//     if (userInput == "y" || userInput == "Y") {
//         std::cout << "Not executing..." << std::endl;
//         seed_req.trajectory_constraints.constraints.resize(36); // 制約の数をウェイポイントの数に合わせる
//         std::vector<std::vector<double>> waypoints = {
//         {0.0164890177507785, 0.02486466875942661, -0.08842499753476596, 0.05873248875014877, 0.0396862245362104, 0.08121969928560245},
//         {0.033020453023353345, 0.04970257678548351, -0.17683019858596483, 0.11748449260162712, 0.0793824205450373, 0.1624324725010244},
//         {0.04955188829592819, 0.07454048481154041, -0.2652353996371637, 0.1762364964531054, 0.11907861655386422, 0.24364524571644638},
//         {0.06608332356850304, 0.0993783928375973, -0.35364060068836256, 0.23498850030458382, 0.1587748125626911, 0.32485801893186833},
//         {0.08261475884107794, 0.12421630086365425, -0.44204580173956143, 0.2937405041560622, 0.19847100857151803, 0.40607079214729036},
//         {0.09914619411365272, 0.1490542088897111, -0.5304510027907603, 0.3524925080075404, 0.23816720458034493, 0.4872835653627123},
//         {0.11567762938622766, 0.17389211691576806, -0.6188562038419592, 0.4112445118590189, 0.27786340058917175, 0.5684963385781343},
//         {0.13220906465880242, 0.1987300249418249, -0.707261404893158, 0.4699965157104972, 0.3175595965979987, 0.6497091117935562},
//         {0.14874049993137733, 0.22356793296788185, -0.7956666059443569, 0.5287485195619754, 0.3572557926068257, 0.7309218850089781},
//         {0.16527193520395222, 0.24840584099393878, -0.8840718069955558, 0.5875005234134539, 0.39695198861565256, 0.8121346582244002},
//         {0.18180337047652706, 0.2732437490199957, -0.9724770080467549, 0.6462525272649322, 0.43664818462447946, 0.893347431439822},
//         {0.19833480574910178, 0.2980816570460525, -1.0608822090979535, 0.7050045311164104, 0.47634438063330636, 0.9745602046552441},
//         {0.21486624102167684, 0.32291956507210956, -1.1492874101491528, 0.7637565349678889, 0.5160405766421333, 1.0557729778706662},
//         {0.23139767629425168, 0.3477574730981664, -1.2376926112003512, 0.8225085388193674, 0.55573677265096, 1.1369857510860881},
//         {0.24792911156682645, 0.3725953811242233, -1.3260978122515503, 0.8812605426708456, 0.5954329686597871, 1.21819852430151},
//         {0.2644605468394012, 0.3974332891502801, -1.414503013302749, 0.940012546522324, 0.6351291646686139, 1.2994112975169319},
//         {0.2809919821119762, 0.42227119717633715, -1.502908214353948, 0.9987645503738021, 0.6748253606774409, 1.3806240707323538},
//         {0.297523417384551, 0.447109105202394, -1.5913134154051467, 1.0575165542252805, 0.7145215566862679, 1.4618368439477758},
//         {0.3140548526571256, 0.4719470132284509, -1.6797186164563456, 1.116268558076759, 0.7542177526950948, 1.5430496171631978},
//         {0.3305862879297008, 0.49678492125450785, -1.7681238175075444, 1.1750205619282375, 0.7939139487039216, 1.62426239037862},
//         {0.3471177232022754, 0.5216228292805646, -1.8565290185587433, 1.2337725657797158, 0.8336101447127484, 1.7054751635940417},
//         {0.4566480301733111, 0.5498715246366838, -1.8742376669650211, 1.2702324175219915, 0.9249757683063526, 1.7339134064978556},
//         {0.5661783371443472, 0.578120219992803, -1.8919463153712992, 1.3066922692642677, 1.016341391899957, 1.7623516494016698},
//         {0.6757086441153826, 0.6063689153489221, -1.9096549637775766, 1.3431521210065438, 1.1077070154935615, 1.7907898923054837},
//         {0.7852389510864182, 0.6346176107050411, -1.9273636121838544, 1.3796119727488192, 1.1990726390871658, 1.8192281352092978},
//         {0.8947692580574544, 0.6628663060611604, -1.9450722605901327, 1.4160718244910955, 1.29043826268077, 1.8476663781131117},
//         {1.0042995650284903, 0.6911150014172796, -1.9627809089964103, 1.452531676233371, 1.3818038862743744, 1.8761046210169259},
//         {1.036615721937181, 0.6955511194556292, -1.9507608052959378, 1.3072683908117395, 1.354338615696411, 1.8212400411675949},
//         {1.0689318788458726, 0.6999872374939793, -1.9387407015954667, 1.1620051053901077, 1.3268733451184491, 1.7663754613182643},
//         {1.1012480357545638, 0.704423355532329, -1.9267205978949948, 1.0167418199684757, 1.299408074540486, 1.7115108814689333},
//         {1.1335641926632554, 0.708859473570679, -1.9147004941945227, 0.8714785345468438, 1.2719428039625233, 1.6566463016196025},
//         {1.1658803495719465, 0.7132955916090288, -1.9026803904940512, 0.7262152491252118, 1.2444775333845608, 1.6017817217702721},
//         {1.1981965064806375, 0.7177317096473786, -1.890660286793579, 0.5809519637035802, 1.2170122628065978, 1.546917141920941},
//         {1.230512663389329, 0.7221678276857286, -1.8786401830931072, 0.4356886782819483, 1.1895469922286355, 1.4920525620716105},
//         {1.2628288202980198, 0.7266039457240783, -1.8666200793926353, 0.29042539286031616, 1.1620817216506727, 1.43718798222228},
//         {1.295144977206711, 0.731040063762428, -1.8545999756921632, 0.14516210743868446, 1.1346164510727101, 1.382323402372949}
//         };

//         // ウェイポイントの数だけループ
//         for (size_t i = 0; i < 36; ++i)
//         {
//             // 各ウェイポイントに対する制約条件の数をvariable_countに合わせる
//             seed_req.trajectory_constraints.constraints[i].joint_constraints.resize(6);

//             // 関節数だけループ
//             for (size_t j = 0; j < 6; ++j)
//             {
//                 // ウェイポイント[i]の関節[j]関節名を格納
//                 seed_req.trajectory_constraints.constraints[i].joint_constraints[j].joint_name = variable_names[j];

//                 // ウェイポイント[i]の関節[j]の目標位置を格納
//                 seed_req.trajectory_constraints.constraints[i].joint_constraints[j].position = waypoints[i][j];
//             }
//         }
//         //ROS_INFO_STREAM("seed_req: " << seed_req); // seed_reqの表示
//     }

//     // Get group config（関係なし）
//     const auto& group_config_it = group_config_.find(req.group_name);
//     if (group_config_it == group_config_.end())
//     {
//       ROS_ERROR_STREAM("STOMP is not configured for planning group " << req.group_name);
//       res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
//       return false;
//     }

//     // Initialize STOMP Planner（関係なし）
//     stomp_moveit::StompPlanner stomp_planner(req.group_name, group_config_it->second, ps->getRobotModel());
//     if(!stomp_planner.canServiceRequest(seed_req))
//     {
//       ROS_ERROR("STOMP planner unable to service request");
//       res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
//       return false;
//     }

//     // Setup Planning Context
//     stomp_planner.clear();
//     stomp_planner.setPlanningScene(ps);
//     //ROS_INFO_STREAM("seed_req: " << seed_req); // seed_reqの表示
//     stomp_planner.setMotionPlanRequest(seed_req);

//     // Solve
//     ROS_DEBUG("Smoothing result trajectory with STOMP");
//     planning_interface::MotionPlanDetailedResponse stomp_res;
//     bool success = stomp_planner.solve(stomp_res);
//     if (success)
//     {
//       // Successful responses always contain one entry for trajectory_ and proccessing_time_
//       res.trajectory_ = stomp_res.trajectory_.back();
//       res.planning_time_ += stomp_res.processing_time_.back();
//     }
//     res.error_code_ = stomp_res.error_code_;
//     return success;
//   }

// private:
//   std::map<std::string, XmlRpc::XmlRpcValue> group_config_;
// };
// }  // namespace stomp_moveit

// CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompSmoothingAdapter, planning_request_adapter::PlanningRequestAdapter);
