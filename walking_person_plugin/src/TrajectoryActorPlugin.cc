/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <functional>

#include <glog/logging.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>

#include "TrajectoryActorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(TrajectoryActorPlugin)

using ignition::math::Pose3d;
using ignition::math::Vector3d;

struct TrajectoryActorPluginPrivate
{
  /// \brief Pointer to the actor.
  public: physics::ActorPtr actor{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief List of connections such as WorldUpdateBegin
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief List of targets
  public: std::vector<ignition::math::Vector3d> targets;

  /// \brief Current target index
  public: unsigned int currentTarget{0};

  /// \brief Radius in meters around target pose where we consider it was
  /// reached.
  public: double targetRadius{0.1};

  /// \brief Margin by which to increase an obstacle's bounding box on every
  /// direction (2x per axis).
  public: double obstacleMargin{0.1};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{5.1};

  /// \brief Time of the last update.
  public: common::Time lastUpdate;

  /// \brief Time when the corner starts
  public: common::Time firstCornerUpdate;

  /// \brief List of models to ignore when checking collisions.
  public: std::vector<std::string> ignoreModels;

  /// \brief Animation for corners
  public: common::PoseAnimation *cornerAnimation{nullptr};

public: double fixed_actor_height{1.2};
};

/////////////////////////////////////////////////
TrajectoryActorPlugin::TrajectoryActorPlugin()
    : data_ptr_(new TrajectoryActorPluginPrivate)
{
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  google::InstallFailureSignalHandler();
  LOG(ERROR) << "Load";

  data_ptr_->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  data_ptr_->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrajectoryActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Read in the velocity
  if (_sdf->HasElement("velocity"))
    data_ptr_->velocity = _sdf->Get<double>("velocity");

  // Read in the target poses
  // auto targetElem = _sdf->GetElement("target");
  // while (targetElem)
  // {
  //   data_ptr_->targets.push_back(targetElem->Get<ignition::math::Vector3d>());
  //   targetElem = targetElem->GetNextElement("target");
  // }

  // Read in the target mradius
  if (_sdf->HasElement("target_radius"))
    data_ptr_->targetRadius = _sdf->Get<double>("target_radius");

  // Read in the obstacle margin
  if (_sdf->HasElement("obstacle_margin"))
    data_ptr_->obstacleMargin = _sdf->Get<double>("obstacle_margin");

  // Read in the animation factor
  if (_sdf->HasElement("animation_factor"))
    data_ptr_->animationFactor = _sdf->Get<double>("animation_factor");

  // Add our own name to models we should ignore when avoiding obstacles.
  data_ptr_->ignoreModels.push_back(data_ptr_->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacle"))
  {
    auto ignoreElem = _sdf->GetElement("ignore_obstacle");
    while (ignoreElem)
    {
      auto name = ignoreElem->Get<std::string>();
      data_ptr_->ignoreModels.push_back(name);
      ignoreElem = ignoreElem->GetNextElement("ignore_obstacle");
    }
  }

  // Read in the animation name
  std::string animation{"animation"};
  if (_sdf->HasElement("animation"))
    animation = _sdf->Get<std::string>("animation");

  auto skelAnims = data_ptr_->actor->SkeletonAnimations();
  if (skelAnims.find(animation) == skelAnims.end())
  {
    gzerr << "Skeleton animation [" << animation << "] not found in Actor."
          << std::endl;
  }
  else
  {
    // Set custom trajectory
    gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
    trajectoryInfo->type = animation;
    trajectoryInfo->duration = 1.0;

    data_ptr_->actor->SetCustomTrajectory(trajectoryInfo);
  }

  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  this->rosNode->getParam("/trajs", trajsFile);
  std::ifstream in(trajsFile);
  in >> this->trajs;
  this->numOfTrajs = this->trajs.size();
  // LOG(ERROR) << " ======================== the number of trajs is: " << this->numOfTrajs << std::endl;
  / LOG(ERROR) << trajs;

  // LOG(ERROR) << "HERE";
  CHECK_GT(this->trajs["0"].size(), 0);
  // LOG(ERROR) << "HERE";

  for (int j = 0; j < this->trajs["0"].size(); j++) {
    // LOG(ERROR) << "HERE";
    ignition::math::Vector3d tmp(trajs["0"][j][0], trajs["0"][j][1], 1.2);
    // LOG(ERROR) << tmp;
    data_ptr_->targets.push_back(tmp);
  }

  for (const auto& element : data_ptr_->targets) {
      LOG(ERROR) << element;
  }


  // LOG(ERROR) << "HERE";
}

/////////////////////////////////////////////////
// void TrajectoryActorPlugin::Reset()
// {
//   LOG(ERROR) << "Reset =========================================";
//   data_ptr_->currentTarget = 0;
//   data_ptr_->cornerAnimation = nullptr;
//   data_ptr_->lastUpdate = common::Time::Zero;
// }

/////////////////////////////////////////////////
bool TrajectoryActorPlugin::ObstacleOnTheWay() const
{
  LOG(ERROR) << "ObstacleOnTheWay --------------------------------";
  CHECK(data_ptr_->actor);
  // LOG(ERROR) << data_ptr_->actor->WorldPose().Pos();
  ignition::math::Vector3d actorPose = data_ptr_->actor->WorldPose().Pos();
  // LOG(ERROR) << "HERE";
  auto world = data_ptr_->actor->GetWorld();
  // LOG(ERROR) << "HERE";

  // Iterate over all models in the world
  for (unsigned int i = 0; i < world->ModelCount(); ++i)
  {
    // LOG(ERROR) << "curreint i is " << i;
    // Skip models we're ignoring
    auto model = world->ModelByIndex(i);
    if (std::find(data_ptr_->ignoreModels.begin(),
                  data_ptr_->ignoreModels.end(), model->GetName()) !=
                  data_ptr_->ignoreModels.end())
    {
      continue;
    }

    // Obstacle's bounding box
    auto bb = model->BoundingBox();

    // Increase box by margin
    bb.Min() -= ignition::math::Vector3d::One * data_ptr_->obstacleMargin;
    bb.Max() += ignition::math::Vector3d::One * data_ptr_->obstacleMargin;

    // Increase vertically
    bb.Min().Z() -= 5;
    bb.Max().Z() += 5;

    // Check
    if (bb.Contains(actorPose))
    {
      // LOG(ERROR) << "HERE";
      return true;
    }

    // TODO: Improve obstacle avoidance. Some ideas: check contacts, ray-query
    // the path forward, check against bounding box of each collision shape...
  }
  // LOG(ERROR) << "HERE";
  return false;
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::UpdateTarget()
{
  // LOG(ERROR) << "UpdateTarget";
  // Current actor position
  Vector3d actorPos = data_ptr_->actor->WorldPose().Pos();

  // Current target
  Vector3d target = data_ptr_->targets[data_ptr_->currentTarget];

  // 2D distance to target
  Vector3d posDiff = target - actorPos;
  posDiff.Z(0);

  double distance = posDiff.Length();

  // Still far from target?
  if (distance > data_ptr_->targetRadius) {
    // LOG(ERROR) << "Not there yet.";
    return;
  }

  // Move on to next target
  LOG(ERROR) << "Moving to the next target.";
  data_ptr_->currentTarget++;
  if (data_ptr_->currentTarget > data_ptr_->targets.size() - 1)
    data_ptr_->currentTarget = 0;
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // LOG(ERROR) << "OnUpdate";
  // LOG(ERROR) << "HERE";
  // Time delta
  double dt = (_info.simTime - data_ptr_->lastUpdate).Double();
  CHECK_GT(dt, 0);
  // LOG(ERROR) << "HERE";

  // LOG(ERROR) << _info.simTime;
  data_ptr_->lastUpdate = _info.simTime;

  // Don't move if there's an obstacle on the way
  // if (this->ObstacleOnTheWay())
  //   return;

  // Target
  this->UpdateTarget();

  // Current pose - actor is oriented Y-up and Z-front
  ignition::math::Pose3d actorPose = data_ptr_->actor->WorldPose();
  // LOG(ERROR) << "CurWorldPose " << actorPose.Pos();

  // Current target
  ignition::math::Vector3d targetPose = data_ptr_->targets[data_ptr_->currentTarget];
  // LOG(ERROR) << "targetPose is " << targetPose;

  // Direction to target
  ignition::math::Vector3d dir = (targetPose - actorPose.Pos()).Normalize();
  // LOG(ERROR) << "direction to target is " << dir;

  // TODO: generalize for actors facing other directions
  double currentYaw = actorPose.Rot().Yaw();

  // Difference to target
  ignition::math::Angle yawDiff = atan2(dir.Y(), dir.X()) + IGN_PI_2 - currentYaw;
  yawDiff.Normalize();

  // LOG(ERROR) << "HERE";
  /*
  // Rotate if needed
  if (std::abs(yawDiff.Radian()) > IGN_DTOR(10))
  {
    // Not rotating yet
    // if (!data_ptr_->cornerAnimation)
    // {
      // Previous target (we assume we just reached it)
      int previousTarget = data_ptr_->currentTarget - 1;
      if (previousTarget < 0)
        previousTarget = data_ptr_->targets.size() - 1;

      // LOG(ERROR) << "HERE";
      auto prevTargetPos = data_ptr_->targets[previousTarget];

      // Direction from previous target to current target
      // LOG(ERROR) << "HERE";
      auto prevDir = (targetPose - prevTargetPos).Normalize() *
          data_ptr_->targetRadius;

      // Curve end point
      // LOG(ERROR) << "HERE";
      auto endPt = prevTargetPos + prevDir;

      // Total time to finish the curve, we try to keep about the same speed,
      // it will be a bit slower because we're doing an arch, not a straight
      // line
      // LOG(ERROR) << "HERE";
      auto curveDist = (endPt - actorPose.Pos()).Length();
      auto curveTime = curveDist / data_ptr_->velocity;

      // Use pose animation for spline
      // LOG(ERROR) << "HERE";
      data_ptr_->cornerAnimation = new common::PoseAnimation("anim", curveTime, false);

      // Start from actor's current pose
      // LOG(ERROR) << "HERE";
      auto start = data_ptr_->cornerAnimation->CreateKeyFrame(0.0);
      start->Translation(actorPose.Pos());
      start->Rotation(actorPose.Rot());

      // End of curve
      // LOG(ERROR) << "HERE";
      auto endYaw = atan2(prevDir.Y(), prevDir.X()) + IGN_PI_2;
      auto end = data_ptr_->cornerAnimation->CreateKeyFrame(curveTime);
      end->Translation(endPt);
      end->Rotation(ignition::math::Quaterniond(IGN_PI_2, 0, endYaw));

      // LOG(ERROR) << "HERE";
      data_ptr_->firstCornerUpdate = _info.simTime;
    // }

    // LOG(ERROR) << "HERE";

    // Get point in curve
    auto cornerDt = (_info.simTime - data_ptr_->firstCornerUpdate).Double();
    common::PoseKeyFrame pose(cornerDt);
    data_ptr_->cornerAnimation->SetTime(cornerDt);
    data_ptr_->cornerAnimation->GetInterpolatedKeyFrame(pose);

    actorPose.Pos() = pose.Translation();
    actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, pose.Rotation().Yaw());
    } */

  // {
    // LOG(ERROR) << "No YAW needed";
    data_ptr_->cornerAnimation = nullptr;

    CHECK_GT(data_ptr_->velocity, 0);
    actorPose.Pos() += dir * data_ptr_->velocity * dt;
    actorPose.Pos().Z(data_ptr_->fixed_actor_height);

    // TODO: remove hardcoded roll
    // actorPose.Rot() = ignition::math::Quaterniond(IGN_PI_2, 0, currentYaw + yawDiff.Radian());
  // }

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      data_ptr_->actor->WorldPose().Pos()).Length();
  // LOG(ERROR) << "distanceTraveled is -------------> " << distanceTraveled;

  // Update actor
  // LOG(ERROR) << "SetWorldPose " << actorPose.Pos();
  data_ptr_->actor->SetWorldPose(actorPose, false, false);
  data_ptr_->actor->SetScriptTime(data_ptr_->actor->ScriptTime() +
    (distanceTraveled * data_ptr_->animationFactor));
}
