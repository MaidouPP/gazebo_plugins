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
#include "gazebo/physics/physics.hh"
#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
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
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  ignition::math::Pose3d pose = this->actor->WorldPose();

  // Read in the target weight
  // if (_sdf->HasElement("target_weight"))
  //   this->targetWeight = _sdf->Get<double>("target_weight");
  // else
  //   this->targetWeight = 1.15;

  // Read in the obstacle weight
  // if (_sdf->HasElement("obstacle_weight"))
  //   this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  // else
  //   this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  this->animationFactor = 6.0;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  this->height = pose.Pos()[2];

  this->targetRadius = 0.2;

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

  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  this->rosNode->getParam("/trajs", trajsFile);
  std::ifstream in(trajsFile);
  in >> this->trajs;
  this->numOfTrajs = this->trajs.size();

  this->Reset();
  pose.Pos().X(trajs["0"][0][0]);
  pose.Pos().Y(trajs["0"][0][1]);

  this->startPos = pose.Pos();

  this->actor->SetWorldPose(pose);

  for (int j = 1 ; j < this->trajs["0"].size(); j++) {
    this->targets.push_back(std::make_pair<double, double>
                            (trajs["0"][j][0], trajs["0"][j][1]));
  }

  this->idxOfCurrentTraj = 0;
  this->target = ignition::math::Vector3d(this->targets[this->idxOfCurrentTraj].first,
                                          this->targets[this->idxOfCurrentTraj].second,
                                          this->height);
  this->prevTarget =startPos;
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

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
    ignition::math::Pose3d pose = this->actor->WorldPose();
    pose.Pos().X(trajs["0"][0][0]);
    pose.Pos().Y(trajs["0"][0][1]);
    this->actor->SetWorldPose(pose);
    this->idxOfCurrentTraj = 0;
  } else this->idxOfCurrentTraj ++;

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
  this->prevTarget = this->target;
  this->target = ignition::math::Vector3d(this->targets[this->idxOfCurrentTraj].first,
                                          this->targets[this->idxOfCurrentTraj].second,
                                          this->height);
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

  ignition::math::Pose3d actorPose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - actorPose.Pos();
  ignition::math::Vector3d rpy = actorPose.Rot().Euler();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < this->targetRadius)
  {
    this->ChooseNewTarget();
    pos = this->target - actorPose.Pos();
  }


  // Normalize the direction vector, and apply the target weight
  // pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  // this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(30)) {
    if (!this->cornerAnimation) {
      auto prevDir = (this->target - this->prevTarget).Normalize() *
                                     this->targetRadius;

      // curve end point
      auto endPt = this->prevTarget + prevDir;

      // Total time to finish the curve, we try to keep about the same speed,
      // it will be a bit slower because we're doing an arch, not a straight
      // line
      auto curveDist = (endPt - actorPose.Pos()).Length();
      auto curveTime = curveDist / this->velocity;

      // Use pose animation for spline
      this->cornerAnimation = new common::PoseAnimation("anim", curveTime, false);

      // Start from actor's current pose
      auto start = this->cornerAnimation->CreateKeyFrame(0.0);
      start->Translation(actorPose.Pos());
      start->Rotation(actorPose.Rot());

      // End of curve
      auto endYaw = atan2(prevDir.Y(), prevDir.X()) + IGN_PI_2;
      auto end = this->cornerAnimation->CreateKeyFrame(curveTime);
      end->Translation(endPt);
      end->Rotation(ignition::math::Quaterniond(IGN_PI_2, 0, endYaw));

      this->firstCornerUpdate = _info.simTime;

      // actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
      //                                          yaw.Radian()*0.003);
    }

    // Get point in curve
    auto cornerDt = (_info.simTime - this->firstCornerUpdate).Double();
    common::PoseKeyFrame pose(cornerDt);
    this->cornerAnimation->SetTime(cornerDt);
    this->cornerAnimation->GetInterpolatedKeyFrame(pose);

    actorPose.Pos() = pose.Translation();
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, pose.Rotation().Yaw());
  }
  else {
    this->cornerAnimation = nullptr;
    actorPose.Pos() += pos * this->velocity * dt;
    actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }
  // Make sure the actor stays within bounds
  // pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  // pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  // pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(actorPose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
