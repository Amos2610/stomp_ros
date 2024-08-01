/**
 * @file pathseed_callback.cpp
 * @brief This file contains the source code for the pathseed_callback node.
 *
 * The pathseed_callback node subscribes to the path_seed topic and processes incoming PathSeed messages.
 * The PathSeed message contains a matrix of initial parameters that can be used to initialize the STOMP planner.
 * The callback function converts the data vector to an Eigen::MatrixXd and prints it to the console.
 * You can use the initial parameters in your STOMP planner class methods to solve the planning problem.
 * 
 * Instructions:
 * - Adjust the callback function to fit your class structure and method calls.
 * - Compile the code using catkin build.
 * - Run the node using rosrun stomp_moveit pathseed_callback.
 * - Make sure the STOMP planner is running and publishing PathSeed messages.
 * - The pathseed_callback node will receive the PathSeed messages and process the initial parameters.
 */

#include <ros/ros.h>
#include <stomp_moveit/PathSeed.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "stomp_moveit/stomp_planner.h"  // 追加

// Define a callback function to process incoming PathSeed messages
void chatterCallback(const stomp_moveit::PathSeed::ConstPtr& msg)
{
  int rows = msg->rows;
  int cols = msg->cols;

  // Convert data vector to Eigen::MatrixXd
  Eigen::MatrixXd initial_parameters(rows, cols);
  int index = 0;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      initial_parameters(i, j) = msg->data[index++];
    }
  }

  std::cout << "Received matrix data:\n" << initial_parameters << std::endl;

  // シングルトンインスタンスにパラメータを設定
  stomp_moveit::StompPlanner::getInstance().setParameters(initial_parameters, rows, cols);

  // Now you can use initial_parameters, rows, and cols in your StompPlanner class methods
  // For example:
  // stomp_planner.solve(initial_parameters, rows, cols); // Adjust this according to your class structure
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathseed_callback");
  ros::NodeHandle nh;

  // Create a subscriber to receive PathSeed messages
  ros::Subscriber sub = nh.subscribe("path_seed", 1000, chatterCallback);

  ros::spin();

  return 0;
}
