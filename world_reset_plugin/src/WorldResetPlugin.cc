/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/transport/transport.hh"
#include <glog/logging.h>

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class WorldResetPlugin: public WorldPlugin
  {
  public:
    ros::NodeHandlePtr rosNode;
    ros::Subscriber subReset;
    physics::WorldPtr world;

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      this->world = _parent;
      this->rosNode.reset(new ros::NodeHandle("world_reset"));
      // ros::Subscriber reset_subscriber = rosNode.subscribe("/end_of_episode", 1000, callback);

      // // Create a new transport node
      // transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      // rosNode->Init(_parent->Name());

      this->subReset = this->rosNode->subscribe("/reach_dest", 10, &WorldResetPlugin::callback, this);

      // // Create a publisher on the ~/physics topic
      // transport::PublisherPtr physicsPub =
      //   node->Advertise<msgs::Physics>("~/physics");

      // msgs::Physics physicsMsg;
      // physicsMsg.set_type(msgs::Physics::ODE);

      // // Set the step time
      // physicsMsg.set_max_step_size(0.01);

      // // Change gravity
      // msgs::Set(physicsMsg.mutable_gravity(),
      //     ignition::math::Vector3d(0.01, 0, 0.1));
      // physicsPub->Publish(physicsMsg);
      // ros::spin();
    }

  public: void callback(const std_msgs::Bool::ConstPtr& msg) {
    // LOG(ERROR) << "here?";
    this->world->ResetEntities(gazebo::physics::Base::ACTOR);
  }
   
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldResetPlugin)
}
