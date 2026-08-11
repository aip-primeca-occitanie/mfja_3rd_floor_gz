// Copyright 2026 MFJA contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

namespace mfja::sim::systems
{

/// A deterministic, motion-only controller for a two-jaw parallel gripper.
///
/// Both joints always receive the same scalar position. The joints use
/// opposite axes in SDF, so one positive scalar opens both jaws symmetrically.
/// Position and velocity reset components intentionally bypass the physics
/// controller path: this is visual / kinematic motion, not physical grasping.
class SymmetricGripperController final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemConfigurePriority,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate,
  public gz::sim::ISystemReset
{
  public: gz::sim::System::PriorityType ConfigurePriority() override
  {
    // Run Update after the default physics system so the public joint state
    // exactly reflects the deterministic kinematic command for this step.
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
      gzerr << "SymmetricGripperController must be attached to a model.\n";
      return;
    }

    if (!_sdf->HasElement("left_joint_name") ||
        !_sdf->HasElement("right_joint_name") ||
        !_sdf->HasElement("max_position") ||
        !_sdf->HasElement("max_velocity"))
    {
      gzerr << "SymmetricGripperController requires <left_joint_name>, "
            << "<right_joint_name>, <max_position>, and <max_velocity>.\n";
      return;
    }

    this->leftJointName = _sdf->Get<std::string>("left_joint_name");
    this->rightJointName = _sdf->Get<std::string>("right_joint_name");
    this->leftJoint = model.JointByName(_ecm, this->leftJointName);
    this->rightJoint = model.JointByName(_ecm, this->rightJointName);
    if (gz::sim::kNullEntity == this->leftJoint ||
        gz::sim::kNullEntity == this->rightJoint)
    {
      gzerr << "SymmetricGripperController on model [" << model.Name(_ecm)
            << "] could not find joints [" << this->leftJointName << "] and ["
            << this->rightJointName << "].\n";
      return;
    }

    this->minPosition =
      _sdf->Get<double>("min_position", 0.0).first;
    this->maxPosition = _sdf->Get<double>("max_position");
    this->maxVelocity = _sdf->Get<double>("max_velocity");
    const double requestedInitial =
      _sdf->Get<double>("initial_position", 0.0).first;

    if (!std::isfinite(this->minPosition) ||
        !std::isfinite(this->maxPosition) ||
        !std::isfinite(this->maxVelocity) ||
        !std::isfinite(requestedInitial) ||
        this->maxPosition < this->minPosition ||
        this->maxVelocity <= 0.0)
    {
      gzerr << "SymmetricGripperController on model [" << model.Name(_ecm)
            << "] has invalid limits, velocity, or initial position.\n";
      return;
    }

    this->initialPosition = std::clamp(
      requestedInitial, this->minPosition, this->maxPosition);
    this->currentPosition = this->initialPosition;

    std::string subTopic = _sdf->Get<std::string>(
      "sub_topic", "gripper/position_command").first;
    while (!subTopic.empty() && subTopic.front() == '/')
      subTopic.erase(subTopic.begin());

    this->commandTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + model.Name(_ecm) + "/" + subTopic);
    this->commandState = std::make_shared<CommandState>(
      this->minPosition,
      this->maxPosition,
      this->initialPosition,
      this->commandTopic);
    const auto commandState = this->commandState;
    if (this->commandTopic.empty() ||
        !this->node.Subscribe<gz::msgs::Double>(
          this->commandTopic,
          [commandState](const gz::msgs::Double &_msg)
          {
            if (!std::isfinite(_msg.data()))
            {
              gzwarn << "SymmetricGripperController ignored a non-finite "
                     << "command on [" << commandState->commandTopic
                     << "].\n";
              return;
            }

            commandState->targetPosition.store(std::clamp(
              _msg.data(),
              commandState->minPosition,
              commandState->maxPosition));
          }))
    {
      gzerr << "SymmetricGripperController failed to subscribe to ["
            << this->commandTopic << "].\n";
      return;
    }

    this->ApplyState(_ecm);
    this->configured = true;
    gzmsg << "SymmetricGripperController controlling ["
          << this->leftJointName << ", " << this->rightJointName << "] on ["
          << this->commandTopic << "] with range [" << this->minPosition
          << ", " << this->maxPosition << "] and maximum velocity ["
          << this->maxVelocity << "] m/s.\n";
  }

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;

    // Commands received while paused are retained, but simulation-time motion
    // only advances after unpausing. Reapply the current state while paused so
    // no dynamics can desynchronise the jaws.
    this->reportedVelocity = 0.0;
    if (!_info.paused && _info.dt > std::chrono::steady_clock::duration::zero())
    {
      const double previousPosition = this->currentPosition;
      const double target = this->commandState->targetPosition.load();
      const double delta = target - this->currentPosition;
      const double dtSeconds = std::chrono::duration<double>(_info.dt).count();
      const double maxStep = this->maxVelocity * dtSeconds;

      if (std::abs(delta) <= maxStep)
        this->currentPosition = target;
      else
        this->currentPosition += std::copysign(maxStep, delta);

      this->currentPosition = std::clamp(
        this->currentPosition, this->minPosition, this->maxPosition);
      this->reportedVelocity =
        (this->currentPosition - previousPosition) / dtSeconds;
    }

    this->ApplyState(_ecm);
  }

  public: void Update(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;

    // Physics consumes the reset commands during Update. Publishing the same
    // authoritative state after physics prevents a one-step gravity / solver
    // residue from leaking into JointStatePublisher.
    this->WritePublicState(_ecm);
  }

  public: void Reset(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;

    this->currentPosition = this->initialPosition;
    this->reportedVelocity = 0.0;
    this->commandState->targetPosition.store(this->initialPosition);
    this->ApplyState(_ecm);
    this->WritePublicState(_ecm);
  }

  private: struct CommandState
  {
    CommandState(
      double _minPosition,
      double _maxPosition,
      double _initialPosition,
      std::string _commandTopic)
    : minPosition(_minPosition),
      maxPosition(_maxPosition),
      commandTopic(std::move(_commandTopic)),
      targetPosition(_initialPosition)
    {
    }

    const double minPosition;
    const double maxPosition;
    const std::string commandTopic;
    std::atomic<double> targetPosition;
  };

  private: void ApplyState(gz::sim::EntityComponentManager &_ecm) const
  {
    const std::vector<double> position{this->currentPosition};
    const std::vector<double> zeroVelocity{0.0};
    _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
      this->leftJoint, position);
    _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
      this->rightJoint, position);
    _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(
      this->leftJoint, zeroVelocity);
    _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(
      this->rightJoint, zeroVelocity);
  }

  private: void WritePublicState(
    gz::sim::EntityComponentManager &_ecm) const
  {
    const std::vector<double> position{this->currentPosition};
    const std::vector<double> velocity{this->reportedVelocity};
    _ecm.SetComponentData<gz::sim::components::JointPosition>(
      this->leftJoint, position);
    _ecm.SetComponentData<gz::sim::components::JointPosition>(
      this->rightJoint, position);
    _ecm.SetComponentData<gz::sim::components::JointVelocity>(
      this->leftJoint, velocity);
    _ecm.SetComponentData<gz::sim::components::JointVelocity>(
      this->rightJoint, velocity);
  }

  private: bool configured{false};
  private: gz::sim::Entity leftJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightJoint{gz::sim::kNullEntity};
  private: std::string leftJointName;
  private: std::string rightJointName;
  private: std::string commandTopic;
  private: double minPosition{0.0};
  private: double maxPosition{0.0};
  private: double maxVelocity{0.0};
  private: double initialPosition{0.0};
  private: double currentPosition{0.0};
  private: double reportedVelocity{0.0};
  private: std::shared_ptr<CommandState> commandState;

  // Keep the transport node last so it disconnects before other members are
  // destroyed. Any in-flight callback owns its shared command state, so it can
  // also finish safely after unsubscription begins.
  private: gz::transport::Node node;
};

}  // namespace mfja::sim::systems

GZ_ADD_PLUGIN(
  mfja::sim::systems::SymmetricGripperController,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemConfigurePriority,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemUpdate,
  gz::sim::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(
  mfja::sim::systems::SymmetricGripperController,
  "mfja::sim::systems::SymmetricGripperController")
