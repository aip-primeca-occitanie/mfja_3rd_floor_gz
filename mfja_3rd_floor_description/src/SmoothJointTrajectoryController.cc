// Copyright 2026 MFJA contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

namespace mfja::sim::systems
{

namespace
{

constexpr double kLimitTolerance = 1e-9;
constexpr double kPositionTolerance = 1e-12;
constexpr double kMinimumSegmentDuration = 1e-6;
constexpr double kProgressPublishPeriod = 0.05;

template<typename TimeLike>
double MessageTimeSeconds(const TimeLike &_time)
{
  return static_cast<double>(_time.sec()) +
    static_cast<double>(_time.nsec()) * 1e-9;
}

double QuinticBlend(const double _u)
{
  const double u2 = _u * _u;
  const double u3 = u2 * _u;
  return u3 * (10.0 + _u * (-15.0 + 6.0 * _u));
}

double QuinticBlendDerivative(const double _u)
{
  const double oneMinusU = 1.0 - _u;
  return 30.0 * _u * _u * oneMinusU * oneMinusU;
}

}  // namespace

/// Deterministic trajectory execution for the MFJA visual robot models.
///
/// The industrial models intentionally have no collision geometry and the
/// existing grippers are already controlled kinematically. Applying a force
/// PID to those models introduced gravity droop and derivative kicks. This
/// system instead advances a bounded trajectory in simulation time and writes
/// authoritative joint position / velocity reset components on every step.
/// Consequently a completed command remains exact under gravity.
///
/// A one-point JointTrajectory is interpreted as a goal and is minimum-jerk
/// interpolated from the current state. A multi-point trajectory is expected
/// to be pre-interpolated (as required by Gazebo's native controller) and is
/// linearly interpolated between its timestamped samples. All segments are
/// retimed when necessary so the SDF joint velocity limits are respected.
class SmoothJointTrajectoryController final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemConfigurePriority,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemReset
{
  public: gz::sim::System::PriorityType ConfigurePriority() override
  {
    // WritePublicState in Update must run after physics so JointStatePublisher
    // cannot expose a one-step gravity residue.
    return 1000;
  }

  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    const gz::sim::Model model{_entity};
    if (!model.Valid(_ecm))
    {
      gzerr << "SmoothJointTrajectoryController must be attached to a model.\n";
      return;
    }

    this->modelName = model.Name(_ecm);
    this->defaultDuration =
      _sdf->Get<double>("default_duration_sec", 3.0).first;
    this->maxVelocityScale =
      _sdf->Get<double>("max_velocity_scale", 0.5).first;
    if (!std::isfinite(this->defaultDuration) || this->defaultDuration <= 0.0 ||
        !std::isfinite(this->maxVelocityScale) ||
        this->maxVelocityScale <= 0.0 || this->maxVelocityScale > 1.0)
    {
      gzerr << "SmoothJointTrajectoryController on model [" << this->modelName
            << "] requires default_duration_sec > 0 and "
               "0 < max_velocity_scale <= 1.\n";
      return;
    }

    const auto jointNames = ParseRepeated<std::string>(_sdf, "joint_name");
    const auto initialPositions = ParseRepeated<double>(
      _sdf, "initial_position");
    if (jointNames.empty() || initialPositions.size() != jointNames.size())
    {
      gzerr << "SmoothJointTrajectoryController on model [" << this->modelName
            << "] requires one <initial_position> for every <joint_name>.\n";
      return;
    }

    this->joints.reserve(jointNames.size());
    for (std::size_t index = 0; index < jointNames.size(); ++index)
    {
      if (this->jointIndex.count(jointNames[index]) != 0u)
      {
        gzerr << "SmoothJointTrajectoryController on model ["
              << this->modelName << "] has duplicate joint ["
              << jointNames[index] << "].\n";
        return;
      }

      const auto entity = model.JointByName(_ecm, jointNames[index]);
      if (entity == gz::sim::kNullEntity)
      {
        gzerr << "SmoothJointTrajectoryController on model ["
              << this->modelName << "] could not find joint ["
              << jointNames[index] << "].\n";
        return;
      }

      const auto *axisComponent =
        _ecm.Component<gz::sim::components::JointAxis>(entity);
      if (axisComponent == nullptr)
      {
        gzerr << "SmoothJointTrajectoryController on model ["
              << this->modelName << "] joint [" << jointNames[index]
              << "] has no primary JointAxis component.\n";
        return;
      }

      const auto &axis = axisComponent->Data();
      JointState joint;
      joint.name = jointNames[index];
      joint.entity = entity;
      joint.lower = axis.Lower();
      joint.upper = axis.Upper();
      const double sdfMaxVelocity = axis.MaxVelocity();
      joint.maxVelocity =
        std::isfinite(sdfMaxVelocity) && sdfMaxVelocity > 0.0 ?
        sdfMaxVelocity * this->maxVelocityScale : 1.0;

      if (!std::isfinite(initialPositions[index]) ||
          initialPositions[index] < joint.lower - kLimitTolerance ||
          initialPositions[index] > joint.upper + kLimitTolerance)
      {
        gzerr << "SmoothJointTrajectoryController on model ["
              << this->modelName << "] initial position ["
              << initialPositions[index] << "] for joint ["
              << joint.name << "] is outside [" << joint.lower << ", "
              << joint.upper << "].\n";
        return;
      }

      joint.initialPosition = std::clamp(
        initialPositions[index], joint.lower, joint.upper);
      joint.position = joint.initialPosition;
      joint.velocity = 0.0;
      this->jointIndex.emplace(joint.name, index);
      this->joints.push_back(std::move(joint));
    }

    std::string topic = _sdf->Get<std::string>("topic", "").first;
    if (topic.empty())
      topic = "/model/" + this->modelName + "/joint_trajectory";
    this->commandTopic = gz::transport::TopicUtils::AsValidTopic(topic);
    if (this->commandTopic.empty())
    {
      gzerr << "SmoothJointTrajectoryController on model [" << this->modelName
            << "] has invalid trajectory topic [" << topic << "].\n";
      return;
    }

    this->progressPublisher =
      this->node.Advertise<gz::msgs::Float>(this->commandTopic + "_progress");
    this->commandInbox = std::make_shared<CommandInbox>();
    const auto inbox = this->commandInbox;
    if (!this->node.Subscribe<gz::msgs::JointTrajectory>(
        this->commandTopic,
        [inbox](const gz::msgs::JointTrajectory &_message)
        {
          std::lock_guard<std::mutex> lock(inbox->mutex);
          inbox->latest = _message;
        }))
    {
      gzerr << "SmoothJointTrajectoryController on model [" << this->modelName
            << "] failed to subscribe to [" << this->commandTopic << "].\n";
      return;
    }

    for (const auto &joint : this->joints)
    {
      if (_ecm.Component<gz::sim::components::JointPosition>(joint.entity) ==
          nullptr)
      {
        _ecm.CreateComponent(
          joint.entity,
          gz::sim::components::JointPosition({joint.position}));
      }
      if (_ecm.Component<gz::sim::components::JointVelocity>(joint.entity) ==
          nullptr)
      {
        _ecm.CreateComponent(
          joint.entity,
          gz::sim::components::JointVelocity({0.0}));
      }
    }

    this->ApplyState(_ecm);
    this->WritePublicState(_ecm);
    this->configured = true;
    gzmsg << "SmoothJointTrajectoryController controlling ["
          << this->joints.size() << "] joints on model [" << this->modelName
          << "] via [" << this->commandTopic << "] with default duration ["
          << this->defaultDuration << "] s and velocity scale ["
          << this->maxVelocityScale << "].\n";
  }

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;

    if (_info.dt < std::chrono::steady_clock::duration::zero())
      this->ResetControllerState(_ecm);

    if (const auto command = this->TakeLatestCommand(); command.has_value())
      this->StartTrajectory(*command, _info.simTime);

    if (!_info.paused && this->trajectory.active)
      this->AdvanceTrajectory(_info.simTime);
    else if (!this->trajectory.active)
      this->SetAllVelocities(0.0);

    this->ApplyState(_ecm);
    this->MaybePublishProgress(_info.simTime);
  }

  public: void Update(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->configured)
      this->WritePublicState(_ecm);
  }

  public: void Reset(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->configured)
      this->ResetControllerState(_ecm);
  }

  private: template<typename T>
  static std::vector<T> ParseRepeated(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name)
  {
    std::vector<T> values;
    if (!_sdf->HasElement(_name))
      return values;

    auto element = _sdf->FindElement(_name);
    while (element != nullptr)
    {
      values.push_back(element->Get<T>());
      element = element->GetNextElement(_name);
    }
    return values;
  }

  private: struct JointState
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    double lower{-std::numeric_limits<double>::infinity()};
    double upper{std::numeric_limits<double>::infinity()};
    double maxVelocity{1.0};
    double initialPosition{0.0};
    double position{0.0};
    double velocity{0.0};
  };

  private: struct Waypoint
  {
    double time{0.0};
    std::vector<double> positions;
  };

  private: struct ActiveTrajectory
  {
    bool active{false};
    bool singleGoal{false};
    std::chrono::steady_clock::duration startTime{};
    std::vector<Waypoint> waypoints;
    double duration{0.0};
    double lastPublishedProgress{-1.0};
  };

  private: struct CommandInbox
  {
    std::mutex mutex;
    std::optional<gz::msgs::JointTrajectory> latest;
  };

  private: std::optional<gz::msgs::JointTrajectory> TakeLatestCommand()
  {
    std::lock_guard<std::mutex> lock(this->commandInbox->mutex);
    if (!this->commandInbox->latest.has_value())
      return std::nullopt;

    auto result = std::move(this->commandInbox->latest);
    this->commandInbox->latest.reset();
    return result;
  }

  private: void StartTrajectory(
    const gz::msgs::JointTrajectory &_message,
    const std::chrono::steady_clock::duration &_simTime)
  {
    if (_message.joint_names_size() == 0 || _message.points_size() == 0)
    {
      gzwarn << "SmoothJointTrajectoryController on model [" << this->modelName
             << "] ignored an empty trajectory.\n";
      return;
    }

    std::vector<std::size_t> messageJointIndices;
    messageJointIndices.reserve(_message.joint_names_size());
    std::unordered_map<std::string, bool> seenNames;
    for (const auto &name : _message.joint_names())
    {
      const auto found = this->jointIndex.find(name);
      if (found == this->jointIndex.end())
      {
        gzwarn << "SmoothJointTrajectoryController on model ["
               << this->modelName << "] ignored trajectory for unknown joint ["
               << name << "].\n";
        return;
      }
      if (seenNames.emplace(name, true).second == false)
      {
        gzwarn << "SmoothJointTrajectoryController on model ["
               << this->modelName << "] ignored duplicate joint [" << name
               << "] in a trajectory.\n";
        return;
      }
      messageJointIndices.push_back(found->second);
    }

    std::vector<Waypoint> requested;
    requested.reserve(_message.points_size());
    double previousRequestedTime = -1.0;
    std::vector<double> carriedPositions = this->CurrentPositions();
    for (int pointIndex = 0; pointIndex < _message.points_size(); ++pointIndex)
    {
      const auto &point = _message.points(pointIndex);
      if (point.positions_size() != _message.joint_names_size())
      {
        gzwarn << "SmoothJointTrajectoryController on model ["
               << this->modelName << "] ignored point [" << pointIndex
               << "] because its position count does not match joint_names.\n";
        return;
      }

      const double requestedTime = MessageTimeSeconds(point.time_from_start());
      if (!std::isfinite(requestedTime) || requestedTime < 0.0 ||
          requestedTime + kPositionTolerance < previousRequestedTime)
      {
        gzwarn << "SmoothJointTrajectoryController on model ["
               << this->modelName
               << "] ignored trajectory with invalid/non-monotonic time.\n";
        return;
      }

      for (int messageIndex = 0;
           messageIndex < point.positions_size(); ++messageIndex)
      {
        const auto controlledIndex = messageJointIndices[messageIndex];
        const double target = point.positions(messageIndex);
        const auto &joint = this->joints[controlledIndex];
        if (!std::isfinite(target) ||
            target < joint.lower - kLimitTolerance ||
            target > joint.upper + kLimitTolerance)
        {
          gzwarn << "SmoothJointTrajectoryController on model ["
                 << this->modelName << "] rejected target [" << target
                 << "] for joint [" << joint.name << "] outside ["
                 << joint.lower << ", " << joint.upper << "].\n";
          return;
        }
        carriedPositions[controlledIndex] = std::clamp(
          target, joint.lower, joint.upper);
      }

      requested.push_back(Waypoint{requestedTime, carriedPositions});
      previousRequestedTime = requestedTime;
    }

    ActiveTrajectory next;
    next.active = true;
    next.singleGoal = requested.size() == 1u;
    next.startTime = _simTime;
    next.lastPublishedProgress = -1.0;

    const auto current = this->CurrentPositions();
    next.waypoints.push_back(Waypoint{0.0, current});
    if (next.singleGoal)
    {
      const double requestedDuration = requested.front().time > 0.0 ?
        requested.front().time : this->defaultDuration;
      const double minimumDuration =
        1.875 * this->MinimumTravelTime(current, requested.front().positions);
      const double duration = std::max({
        requestedDuration, minimumDuration, kMinimumSegmentDuration});
      next.waypoints.push_back(
        Waypoint{duration, requested.front().positions});
      next.duration = duration;
    }
    else
    {
      double actualTime = 0.0;
      double previousMessageTime = 0.0;
      auto previousPositions = current;
      for (const auto &point : requested)
      {
        const double requestedSegment = std::max(
          0.0, point.time - previousMessageTime);
        const double minimumSegment =
          this->MinimumTravelTime(previousPositions, point.positions);
        double segmentDuration = std::max(requestedSegment, minimumSegment);

        if (segmentDuration <= kMinimumSegmentDuration &&
            PositionsEqual(previousPositions, point.positions))
        {
          previousMessageTime = point.time;
          continue;
        }

        segmentDuration = std::max(segmentDuration, kMinimumSegmentDuration);
        actualTime += segmentDuration;
        next.waypoints.push_back(Waypoint{actualTime, point.positions});
        previousMessageTime = point.time;
        previousPositions = point.positions;
      }

      if (next.waypoints.size() == 1u)
      {
        this->trajectory = ActiveTrajectory{};
        this->SetAllVelocities(0.0);
        this->PublishProgress(1.0);
        return;
      }
      next.duration = actualTime;
    }

    this->trajectory = std::move(next);
    this->PublishProgress(0.0);
    gzmsg << "SmoothJointTrajectoryController on model [" << this->modelName
          << "] accepted [" << this->trajectory.waypoints.size() - 1u
          << "] trajectory samples over [" << this->trajectory.duration
          << "] s.\n";
  }

  private: double MinimumTravelTime(
    const std::vector<double> &_from,
    const std::vector<double> &_to) const
  {
    double result = 0.0;
    for (std::size_t index = 0; index < this->joints.size(); ++index)
    {
      result = std::max(
        result,
        std::abs(_to[index] - _from[index]) /
          this->joints[index].maxVelocity);
    }
    return result;
  }

  private: static bool PositionsEqual(
    const std::vector<double> &_left,
    const std::vector<double> &_right)
  {
    for (std::size_t index = 0; index < _left.size(); ++index)
    {
      if (std::abs(_left[index] - _right[index]) > kPositionTolerance)
        return false;
    }
    return true;
  }

  private: void AdvanceTrajectory(
    const std::chrono::steady_clock::duration &_simTime)
  {
    const double elapsed = std::max(
      0.0,
      std::chrono::duration<double>(
        _simTime - this->trajectory.startTime).count());
    if (elapsed >= this->trajectory.duration)
    {
      this->SetPositions(this->trajectory.waypoints.back().positions);
      this->SetAllVelocities(0.0);
      this->trajectory.active = false;
      this->PublishProgress(1.0);
      return;
    }

    if (this->trajectory.singleGoal)
    {
      const auto &start = this->trajectory.waypoints.front().positions;
      const auto &goal = this->trajectory.waypoints.back().positions;
      const double u = std::clamp(
        elapsed / this->trajectory.duration, 0.0, 1.0);
      const double blend = QuinticBlend(u);
      const double blendRate =
        QuinticBlendDerivative(u) / this->trajectory.duration;
      for (std::size_t index = 0; index < this->joints.size(); ++index)
      {
        const double delta = goal[index] - start[index];
        this->joints[index].position = start[index] + delta * blend;
        this->joints[index].velocity = delta * blendRate;
      }
      return;
    }

    auto upper = std::upper_bound(
      this->trajectory.waypoints.begin(),
      this->trajectory.waypoints.end(),
      elapsed,
      [](const double _time, const Waypoint &_point)
      {
        return _time < _point.time;
      });
    if (upper == this->trajectory.waypoints.begin())
      ++upper;
    if (upper == this->trajectory.waypoints.end())
      upper = std::prev(this->trajectory.waypoints.end());

    const auto &to = *upper;
    const auto &from = *std::prev(upper);
    const double segmentDuration = std::max(
      to.time - from.time, kMinimumSegmentDuration);
    const double alpha = std::clamp(
      (elapsed - from.time) / segmentDuration, 0.0, 1.0);
    for (std::size_t index = 0; index < this->joints.size(); ++index)
    {
      const double delta = to.positions[index] - from.positions[index];
      this->joints[index].position = from.positions[index] + alpha * delta;
      this->joints[index].velocity = delta / segmentDuration;
    }
  }

  private: std::vector<double> CurrentPositions() const
  {
    std::vector<double> result;
    result.reserve(this->joints.size());
    for (const auto &joint : this->joints)
      result.push_back(joint.position);
    return result;
  }

  private: void SetPositions(const std::vector<double> &_positions)
  {
    for (std::size_t index = 0; index < this->joints.size(); ++index)
      this->joints[index].position = _positions[index];
  }

  private: void SetAllVelocities(const double _velocity)
  {
    for (auto &joint : this->joints)
      joint.velocity = _velocity;
  }

  private: void MaybePublishProgress(
    const std::chrono::steady_clock::duration &_simTime)
  {
    if (!this->trajectory.active)
      return;

    const double elapsed = std::max(
      0.0,
      std::chrono::duration<double>(
        _simTime - this->trajectory.startTime).count());
    const double progress = std::clamp(
      elapsed / this->trajectory.duration, 0.0, 1.0);
    if (this->trajectory.lastPublishedProgress < 0.0 ||
        (progress - this->trajectory.lastPublishedProgress) *
          this->trajectory.duration >= kProgressPublishPeriod)
    {
      this->PublishProgress(progress);
      this->trajectory.lastPublishedProgress = progress;
    }
  }

  private: void PublishProgress(const double _progress)
  {
    gz::msgs::Float message;
    message.set_data(static_cast<float>(std::clamp(_progress, 0.0, 1.0)));
    this->progressPublisher.Publish(message);
  }

  private: void ApplyState(
    gz::sim::EntityComponentManager &_ecm) const
  {
    for (const auto &joint : this->joints)
    {
      _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
        joint.entity, std::vector<double>{joint.position});
      _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(
        joint.entity, std::vector<double>{joint.velocity});
    }
  }

  private: void WritePublicState(
    gz::sim::EntityComponentManager &_ecm) const
  {
    for (const auto &joint : this->joints)
    {
      _ecm.SetComponentData<gz::sim::components::JointPosition>(
        joint.entity, std::vector<double>{joint.position});
      _ecm.SetComponentData<gz::sim::components::JointVelocity>(
        joint.entity, std::vector<double>{joint.velocity});
    }
  }

  private: void ResetControllerState(
    gz::sim::EntityComponentManager &_ecm)
  {
    this->trajectory = ActiveTrajectory{};
    for (auto &joint : this->joints)
    {
      joint.position = joint.initialPosition;
      joint.velocity = 0.0;
    }
    if (this->commandInbox != nullptr)
    {
      std::lock_guard<std::mutex> lock(this->commandInbox->mutex);
      this->commandInbox->latest.reset();
    }
    this->ApplyState(_ecm);
    this->WritePublicState(_ecm);
    this->PublishProgress(1.0);
  }

  private: bool configured{false};
  private: std::string modelName;
  private: std::string commandTopic;
  private: double defaultDuration{3.0};
  private: double maxVelocityScale{0.5};
  private: std::vector<JointState> joints;
  private: std::unordered_map<std::string, std::size_t> jointIndex;
  private: ActiveTrajectory trajectory;
  private: std::shared_ptr<CommandInbox> commandInbox;
  private: gz::transport::Node::Publisher progressPublisher;

  // Keep the transport node last so it disconnects before the shared inbox is
  // destroyed. In-flight callbacks retain their own shared inbox reference.
  private: gz::transport::Node node;
};

}  // namespace mfja::sim::systems

GZ_ADD_PLUGIN(
  mfja::sim::systems::SmoothJointTrajectoryController,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemConfigurePriority,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemUpdate,
  gz::sim::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(
  mfja::sim::systems::SmoothJointTrajectoryController,
  "mfja::sim::systems::SmoothJointTrajectoryController")
