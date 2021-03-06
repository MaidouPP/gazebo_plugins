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
#include <ignition/math.hh>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/transport/transport.hh"
#include <glog/logging.h>
#include <geometry_msgs/Point.h>

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
    ros::Subscriber subFail;
    physics::WorldPtr world;
    // ros::Publisher robotPose;

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      this->world = _parent;
      this->rosNode.reset(new ros::NodeHandle("world_reset"));
      // ros::Subscriber reset_subscriber = rosNode.subscribe("/end_of_episode", 1000, callback);

      // // Create a new transport node
      // transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      // rosNode->Init(_parent->Name());

      // this->robotPose = this->rosNode->advertise<geometry_msgs::Point>("/actor_pose", 10);
      this->subReset = this->rosNode->subscribe("/reach_dest", 10, &WorldResetPlugin::callbackReset, this);
      this->subFail = this->rosNode->subscribe("/bump", 10, &WorldResetPlugin::callbackFail, this);

      // ros::Rate loop_rate(20);

      // gazebo::physics::EntityPtr ptr  = this->world->EntityByName("actor1");
      // while (ros::ok()) {
      //   geometry_msgs::Point posOfActor;
      //   ignition::math::Pose3d pose = ptr->WorldPose();
      //   posOfActor.x = pose.Pos()[0];
      //   posOfActor.y = pose.Pos()[1];
      //   posOfActor.z = pose.Pos()[2];
      //   this->robotPose.publish(posOfActor);

      //   loop_rate.sleep();
      // }
    }

  public: void callbackReset(const std_msgs::Bool::ConstPtr& msg) {
    // LOG(ERROR) << "here?";
    this->world->ResetEntities(gazebo::physics::Base::ACTOR);
  }

  public: void callbackFail(const std_msgs::Bool::ConstPtr& msg) {
    LOG(ERROR) << "here?";
    this->world->ResetEntities(gazebo::physics::Base::ACTOR);
  }
   
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldResetPlugin)
}
