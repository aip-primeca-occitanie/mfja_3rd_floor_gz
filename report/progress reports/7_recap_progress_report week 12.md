# Latest Weekly Progress Report

Period covered: May 25, 2026 to May 26, 2026  
Project: MFJA 3rd Floor ROS 2 / Gazebo Simulation  
Prepared on: May 26, 2026

## Page 1


### What We Changed

1. Stopper-linked sensors were simplified

We changed the sensor model so there is no separate `approach_sensors` section anymore. Instead, the sensors before stoppers are regular `position_sensors`.

Example:

```yaml
- name: A1_STOPPER_SENSOR
  stopper: A1
  before_stopper_m: 0.1
  radius_m: 0.08
```

This means the sensor position is derived from the matching stopper position. If the stopper moves, the linked sensor moves with it. The sensor still publishes normal `SensorFeedback`, but it is no longer treated as a special device type.

2. Removed the `APPROACH` naming

We removed names such as `A1_APPROACH`, because they did not match the real system and were confusing. The new names are:

- `A1_STOPPER_SENSOR`
- `A2_STOPPER_SENSOR`
- `A3_STOPPER_SENSOR`
- `A4_STOPPER_SENSOR`

These names are clearer: they are position sensors related to stoppers, not a separate approach sensor category.

3. Stopper stopping logic now uses the linked sensor point

Before, the shuttle stopped based on a general distance before the stopper. Now, when a stopper is active, the current Room 315 YAML makes the shuttle stop at the linked stopper sensor point. This better matches the real setup, where the sensor before the stopper detects the shuttle before it reaches the physical stopper.

4. DA4R was fixed on the right rail

`DA4R` was only defined on `A4E`. That meant if the shuttle passed through `A4I`, the visual marker could look close to the shuttle, but the sensor would not become active because the code requires the shuttle to be on the same segment.

We changed `DA4R` to have two points:

```yaml
- name: DA4R
  switch: A4
  radius_m: 0.08
  points:
  - segment: A4E
    s_ratio: 0.969477997
  - segment: A4I
    s_ratio: 0.969477997
```

Now `DA4R` can become active whether the shuttle comes through the exterior or interior A4 connector.

5. Switch naming was standardized

We replaced the old `G` and `S` naming with `E` and `I` everywhere in the Room 315 rail logic and config. The meaning is:

- `E` = `EXTERIOR`
- `I` = `INTERIOR`

This was applied to rail network YAML, rail device YAML, raw segment file names, switch states, launch text, and documentation.

## Page 2

### Robot and Runbook Updates

6. TIAGo base-only variant was added

We added support for a TIAGo variant without arm and head:

```yaml
name: tiago_base1
model: tiago_base
```

This model has mobile base motion through:

```text
/tiago_base1/cmd_vel
```

It also has:

```text
/tiago_base1/joint_trajectory
```

but only for the existing torso joint. It does not have arm or head joints.

7. Visual Feedback Improvements

We also improved the visual feedback of the Room 315 rail simulation to make the system state easier to understand during runtime. Position sensors now change color when a shuttle passes over them, which makes sensor activation visible directly in Gazebo. The shuttle also changes color to red when it enters falling mode, making invalid routing or off-rail situations immediately noticeable. In addition, the switches now change color depending on their current state, distinguishing between the INTERIOR and EXTERIOR positions. These visual updates make debugging easier and help users understand the behavior of sensors, shuttles, and switches without relying only on ROS topics or terminal output.

8. Nix Development Environment

I also added a Nix file to improve the development environment setup. The Nix configuration does not install ROS 2 or Gazebo directly. Instead, it provides a hybrid development shell that works alongside the system-installed ROS 2 Jazzy and Gazebo Harmonic packages. It helps load additional tools and runtime dependencies more consistently, making the workspace easier to prepare and reducing setup issues while still relying on the ROS 2 and Gazebo installations already available on the system.

9. Documentation and runbook were updated

We updated the README, Room 315 kinematics README, and runbook to match the new reality:

- No separate `approach_sensors` concept.
- Stopper-linked sensors are normal `position_sensors`.
- New `*_STOPPER_SENSOR` names.
- Correct TIAGo base-only control topics.
- Correct workspace path.
- Clearer terminal tool behavior for moving sensors and stoppers.



