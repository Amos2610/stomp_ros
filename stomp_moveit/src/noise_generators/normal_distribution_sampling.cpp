/**
 * @file normal_distribution_sampling.cpp
 * @brief This a normial distribution noisy trajectory update generator.
 *
 * @author Jorge Nicho
 * @date May 31, 2016
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
#include <stomp_moveit/noise_generators/normal_distribution_sampling.h>
#include <stomp_moveit/utils/multivariate_gaussian.h>
#include <XmlRpcException.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::noise_generators::NormalDistributionSampling,stomp_moveit::noise_generators::StompNoiseGenerator);

/*
 * These coefficients correspond to the five point stencil method
 */
static const std::vector<double> ACC_MATRIX_DIAGONAL_VALUES = {-1.0/12.0, 16.0/12.0, -30.0/12.0, 16.0/12.0, -1.0/12.0};
static const std::vector<int> ACC_MATRIX_DIAGONAL_INDICES = {-2, -1, 0 ,1, 2};

namespace stomp_moveit
{

namespace noise_generators
{

NormalDistributionSampling::NormalDistributionSampling():
    name_("NormalDistributionSampling")
{
  // TODO Auto-generated constructor stub

}

NormalDistributionSampling::~NormalDistributionSampling()

{
  // TODO Auto-generated destructor stub
}

bool NormalDistributionSampling::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  using namespace moveit::core;

  group_ = group_name;
  const JointModelGroup* joint_group = robot_model_ptr->getJointModelGroup(group_name);
  if(!joint_group)
  {
    ROS_ERROR("Invalid joint group %s",group_name.c_str());
    return false;
  }

  stddev_.resize(joint_group->getActiveJointModelNames().size());


  return configure(config);
}

bool NormalDistributionSampling::configure(const XmlRpc::XmlRpcValue& config)
{
  ros::NodeHandle nh("~");
  std::string param_path = "/move_group/stomp/xarm6/task/noise_generator";
  XmlRpc::XmlRpcValue noise_generator_config;

  // パラメータ全体を取得
  if (!nh.getParam(param_path, noise_generator_config))
  {
    ROS_ERROR("%s: Failed to load parameter at '%s'", getName().c_str(), param_path.c_str());
    return false;
  }

  // 取得した値をコンソールに出力
  ROS_INFO_STREAM("Parameter retrieved from '" << param_path << "': " << noise_generator_config);


  // noise_generatorがリスト形式であることを確認
  if (noise_generator_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("%s: Parameter at '%s' is not an array.", getName().c_str(), param_path.c_str());
    return false;
  }

  // noise_generatorの中からstddevを探す
  for (int i = 0; i < noise_generator_config.size(); i++)
  {
    if (noise_generator_config[i].hasMember("stddev"))
    {
      XmlRpc::XmlRpcValue stddev_param = noise_generator_config[i]["stddev"];

      // stddevがリスト形式であることを確認
      if (stddev_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("%s: 'stddev' is not a list.", getName().c_str());
        return false;
      }

      // サイズチェック
      if (stddev_param.size() < stddev_.size())
      {
        ROS_ERROR("%s: The 'stddev' parameter has fewer elements (%d) than the number of joints (%lu).",
                  getName().c_str(), stddev_param.size(), stddev_.size());
        return false;
      }

      // stddevの値を設定
      for (int j = 0; j < stddev_param.size(); j++)
      {
        stddev_[j] = static_cast<double>(stddev_param[j]);
        ROS_INFO("%s: stddev[%d] = %f", getName().c_str(), j, stddev_[j]);
      }

      return true;
    }
  }

  ROS_ERROR("%s: Parameter 'stddev' not found in '%s'", getName().c_str(), param_path.c_str());
  return false;
}


bool NormalDistributionSampling::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;

  auto fill_diagonal = [](Eigen::MatrixXd& m,double coeff,int diag_index)
  {
    std::size_t size = m.rows() - std::abs(diag_index);
    m.diagonal(diag_index) = VectorXd::Constant(size,coeff);
  };

  // creating finite difference acceleration matrix
  std::size_t num_timesteps = config.num_timesteps;
  Eigen::MatrixXd A = MatrixXd::Zero(num_timesteps,num_timesteps);
  int num_elements = (int((ACC_MATRIX_DIAGONAL_INDICES.size() -1)/2.0) + 1)* num_timesteps ;
  for(auto i = 0u; i < ACC_MATRIX_DIAGONAL_INDICES.size() ; i++)
  {
    fill_diagonal(A,ACC_MATRIX_DIAGONAL_VALUES[i],ACC_MATRIX_DIAGONAL_INDICES[i]);
  }

  // create and scale covariance matrix
  Eigen::MatrixXd covariance = MatrixXd::Identity(num_timesteps,num_timesteps);
  covariance = A.transpose() * A;
  covariance = covariance.fullPivLu().inverse();
  double max_val = covariance.array().abs().matrix().maxCoeff();
  covariance /= max_val;

  // create random generators
  rand_generators_.resize(stddev_.size());
  for(auto& r: rand_generators_)
  {
    r.reset(new utils::MultivariateGaussian(VectorXd::Zero(num_timesteps),covariance));
  }

  // preallocating noise data
  raw_noise_.resize(config.num_timesteps);
  raw_noise_.setZero();

  return true;
}


bool NormalDistributionSampling::generateNoise(const Eigen::MatrixXd& parameters,
                                     std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     int rollout_number,
                                     Eigen::MatrixXd& parameters_noise,
                                     Eigen::MatrixXd& noise)
{
  if(parameters.rows() != stddev_.size())
  {
    ROS_ERROR("Number of parameters %i differs from what was preallocated ",int(parameters.rows()));
    return false;
  }


  for(auto d = 0u; d < parameters.rows() ; d++)
  {
    rand_generators_[d]->sample(raw_noise_);
    raw_noise_.head(1).setZero();
    raw_noise_.tail(1).setZero(); // zeroing out the start and end noise values
    noise.row(d).transpose() = stddev_[d] * raw_noise_;
    parameters_noise.row(d) = parameters.row(d) + noise.row(d);
  }

  return true;
}

} /* namespace noise_generators */
} /* namespace stomp_moveit */