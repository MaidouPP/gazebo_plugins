/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
n * Licensed under the Apache License, Version 2.0 (the "License");
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

#include <functional>

#include <ignition/math.hh>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "gazebo/physics/physics.hh"
#include "ActorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  LOG(ERROR) << "Is this what reset calls?????";
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  ignition::math::Pose3d pose = this->actor->WorldPose();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else

    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  this->animationFactor = 5.0;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  this->height = pose.Pos()[2];

  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  this->posPublisher = this->rosNode->advertise<geometry_msgs::Point>("/actor_pos", 100);
  this->reachDestPublisher = this->rosNode->advertise<std_msgs::Bool>("/reach_dest", 100);
  this->newStartPublisher = this->rosNode->advertise<geometry_msgs::Point>("/new_start", 100);

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  this->rosNode->getParam("/trajs", trajsFile);
  std::ifstream in(trajsFile);
  in >> this->trajs;
  this->numOfTrajs = this->trajs.size();
  this->idx = 0;

  this->Reset();
  // pose.Pos().X(trajs["0"][0][0]);
  // pose.Pos().Y(trajs["0"][0][1]);
  // this->actor->SetWorldPose(pose);

  this->idxOfCurrentTraj = 1;
  // this->target = ignition::math::Vector3d(this->targets[this->idxOfCurrentTraj].first,
  //                                         this->targets[this->idxOfCurrentTraj].second,
  //                                         this->height);

  // this->reachDest = false;

}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  // this->velocity = 0.8;
  this->lastUpdate = 0;

  this->velocity = static_cast<float>(rand()) / static_cast<float>(RAND_MAX/0.4) + 0.4;

  // if (this->sdf && this->sdf->HasElement("target"))
  //   this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  // else
  //   this->target = ignition::math::Vector3d(0, -5, 1.2138);

  // ignition::math::Pose3d pose = this->actor->WorldPose();
  // std::string idx_ = std::to_string(this->idx);
  // pose.Pos().X(this->trajs[idx_][0][0]);
  // pose.Pos().Y(this->trajs[idx_][0][1]);
  // this->actor->SetWorldPose(pose, true, true);

  // this->ChooseNewTarget();
  if (this->idx == this->numOfTrajs-1)
    this->idx = 0;
  else
    this->idx ++;

  std::string traj = std::to_string(this->idx);
  ignition::math::Pose3d pose = this->actor->WorldPose();
  pose.Pos().X(this->trajs[traj][0][0]);
  pose.Pos().Y(this->trajs[traj][0][1]);
  this->actor->SetWorldPose(pose, true, true);
  this->idxOfCurrentTraj = 1;

  this->targets.clear();

  for (int j = 1 ; j < this->trajs[traj].size(); j++)
    this->targets.push_back(std::make_pair<double, double>
                            (trajs[traj][j][0], trajs[traj][j][1]));

  this->target = ignition::math::Vector3d(this->targets[this->idxOfCurrentTraj].first,
                                          this->targets[this->idxOfCurrentTraj].second,
                                          this->height);

  geometry_msgs::Point msg;
  msg.x =  this->target[0];
  msg.y = this->target[1];
  msg.z = this->target[2];
  this->newStartPublisher.publish(msg);
  LOG(ERROR) << "here";

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{
  if (this->idxOfCurrentTraj == this->targets.size()-1) {
    std_msgs::Bool msg;
    msg.data = true;
    this->reachDestPublisher.publish(msg);
    this->targets.clear();

    this->Reset();

    // LOG(ERROR) << ">>>>>>>>>>>" << this->actor->WorldPose();
    // this->actor->SetWorldPose(pose, true, true);

    // LOG(ERROR) << "<<<<<<<<<<<" << this->actor->WorldPose();
    this->idxOfCurrentTraj = 0;
  } else this->idxOfCurrentTraj ++;

  this->target = ignition::math::Vector3d(this->targets[this->idxOfCurrentTraj].first,
                                          this->targets[this->idxOfCurrentTraj].second,
                                          this->height);

  // if (this->idxOfCurrentTraj == 1) {
  //   std_msgs::Bool msg;
  //   msg.data = false;
  //   this->reachDestPublisher.publish(msg);

  //   // tell the robot new start point to make it prepare
  //   geometry_msgs::Point posOfStart;
  //   posOfStart.x = this->target[0];
  //   posOfStart.y = this->target[1];
  //   posOfStart.z = this->target[2];

  //   this->newStartPublisher.publish(posOfStart);
  // }

    // this->idxOfCurrentTraj = 0;

    // newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    // newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    // for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    // {
    //   double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
    //       - newTarget).Length();
    //   if (dist < 2.0)
    //   {
    //     newTarget = this->target;
    //     break;
    //   }
    // }
}

/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // static bool log_first_flag = true;
  // LOG_IF(ERROR, log_first_flag) << "The world pose in first OnUpdate is: " << this->actor->WorldPose();
  // log_first_flag = false;

  // LOG_EVERY_N(ERROR, 100) << "The world pose in OnUpdate is: " << this->actor->WorldPose();

  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

  geometry_msgs::Point posOfActor;
  posOfActor.x = pose.Pos()[0];
  posOfActor.y = pose.Pos()[1];
  posOfActor.z = pose.Pos()[2];

  this->posPublisher.publish(posOfActor);

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.2)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }


  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  // this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10)) {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
                                             yaw.Radian()*0.003);
  }
  else {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }
  // Make sure the actor stays within bounds
  // pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  // pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  // pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
