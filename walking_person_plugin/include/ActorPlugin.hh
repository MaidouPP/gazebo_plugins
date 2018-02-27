/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include <glog/logging.h>

#include <ros/ros.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include "json.hpp"

using nlohmann::json;

namespace gazebo
{
  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
  public:
      ActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
      virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
  private:
    void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the parent actor.
    physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    ignition::math::Vector3d velocity;

    /// \brief List of connections
    std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    double animationFactor = 1.0;

    /// \brief Time of the last update.
    common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    physics::TrajectoryInfoPtr trajectoryInfo;

    ros::NodeHandlePtr rosNode;

    std::vector<std::pair<double, double>> targets;
    std::string trajsFile;
    json trajs;

    int numOfTrajs;
    int idxOfCurrentTraj;
    int idx;
    double height;

    // std_msgs::Bool reachMsg;

    ros::Publisher posPublisher;
    ros::Publisher reachDestPublisher;
    ros::Publisher newStartPublisher;
  };
}
#endif
