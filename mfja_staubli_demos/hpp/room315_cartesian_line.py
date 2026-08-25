"""Reusable HPP computation for a Room 315 Staubli Cartesian line."""

import operator
from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from pinocchio import StdVec_Bool as Mask
from pyhpp.constraints import (
    ComparisonType,
    ComparisonTypes,
    Implicit,
    Orientation,
    Position,
)
from pyhpp.core import ConfigProjector, ConstraintSet, Dichotomy, Problem, Straight
from pyhpp.pinocchio import Device, urdf

from staubli_trajectory_export import joint_trajectory_payload


JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
FRAME = "staubli/tool0"
# Room 315 staging configuration supplied in degrees as [0, 50, 70, 0, 55, 0].
DEFAULT_Q_START = np.array(
    [0.0, 0.8726646259971648, 1.2217304763960306, 0.0, 0.9599310885968813, 0.0]
)
DEFAULT_LINE = np.array([0.0, 0.0, 0.4])
ROOM315_ROBOT_POSE = (-15.251, -6.0, 1.0, 0.0, 0.0, 0.0)

ROBOT_URDF = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
ROBOT_SRDF = "package://mfja_staubli_demos/hpp/staubli_tx2_60l.srdf"
CELL_URDF = "package://mfja_3rd_floor_description/urdf/room315_cell.urdf"
CELL_SRDF = "package://mfja_3rd_floor_description/urdf/room315_cell.srdf"

START_HOLD = 1.0


@dataclass
class CartesianLinePlan:
    configurations: list
    start_position: np.ndarray
    end_position: np.ndarray
    max_deviation: float


def build_problem():
    robot = Device("staubli")
    urdf.loadModel(
        robot, 0, "staubli", "anchor", ROBOT_URDF, ROBOT_SRDF, pin.SE3.Identity()
    )

    x, y, z, roll, pitch, yaw = ROOM315_ROBOT_POSE
    robot_world = pin.SE3(
        pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z])
    )
    # Cell link origins are world poses; placing the cell at the inverse of
    # the robot world pose expresses everything in the robot base frame.
    urdf.loadModel(
        robot, 0, "room315", "anchor", CELL_URDF, CELL_SRDF, robot_world.inverse()
    )

    problem = Problem(robot)
    problem.addConfigValidation("CollisionValidation")
    problem.addConfigValidation("JointBoundValidation")
    problem.steeringMethod(Straight(problem))
    problem.pathValidation(Dichotomy(robot, 0.0))
    return robot, problem


def sample_path(path, samples):
    length = path.length()
    configurations = []
    for index in range(samples):
        configuration, success = path(index / (samples - 1) * length)
        if not success:
            raise RuntimeError(f"path evaluation failed at sample {index}")
        configurations.append(np.asarray(configuration).flatten())
    return configurations


def _inputs(q_start, line, samples):
    if q_start is None:
        q_start = DEFAULT_Q_START.copy()
    else:
        q_start = np.asarray(q_start, dtype=float)
    if q_start.shape != (len(JOINT_NAMES),):
        raise ValueError(f"q_start must contain {len(JOINT_NAMES)} joint values")
    if not np.all(np.isfinite(q_start)):
        raise ValueError("q_start values must be finite")

    if line is None:
        line = DEFAULT_LINE.copy()
    else:
        line = np.asarray(line, dtype=float)
    if line.shape != (3,):
        raise ValueError("line must contain three Cartesian values")
    if not np.all(np.isfinite(line)) or np.linalg.norm(line) <= 0.0:
        raise ValueError("line must be finite and non-zero")
    try:
        samples = operator.index(samples)
    except TypeError as error:
        raise ValueError("samples must be an integer") from error
    if samples < 2:
        raise ValueError("samples must be at least 2")
    return q_start, line, samples


def plan_cartesian_line(*, q_start=None, line=None, samples=80):
    """Compute a collision-checked Cartesian line."""
    q_start, line, samples = _inputs(q_start, line, samples)
    robot, problem = build_problem()
    valid, report = problem.isConfigValid(q_start)
    if not valid:
        raise RuntimeError(f"start configuration is invalid: {report}")

    model = robot.model()
    frame_id = model.getFrameId(FRAME)
    frame = model.frames[frame_id]
    joint_id = frame.parentJoint
    tool_in_joint = frame.placement
    data = model.createData()

    pin.forwardKinematics(model, data, q_start)
    pin.updateFramePlacements(model, data)
    start_pose = data.oMf[frame_id].copy()
    goal_pose = pin.SE3(start_pose.rotation, start_pose.translation + line)

    xyz = Mask()
    xyz[:] = (True, True, True)
    line_xy = Mask()
    line_xy[:] = (True, True, False)
    active_xy = Mask()
    active_xy[:] = (True, True)
    equal3 = ComparisonTypes()
    equal3[:] = (ComparisonType.EqualToZero,) * 3
    equal2 = ComparisonTypes()
    equal2[:] = (ComparisonType.EqualToZero,) * 2

    goal_projector = ConfigProjector(robot, "goal_projector", 1e-4, 200)
    goal_projector.add(
        Implicit(
            Position(
                "tool0_goal_position",
                robot,
                joint_id,
                tool_in_joint,
                goal_pose,
                xyz,
            ),
            equal3,
            xyz,
        ),
        0,
    )
    goal_projector.add(
        Implicit(
            Orientation(
                "tool0_goal_orientation",
                robot,
                joint_id,
                pin.SE3(tool_in_joint.rotation, np.zeros(3)),
                pin.SE3(goal_pose.rotation, np.zeros(3)),
                xyz,
            ),
            equal3,
            xyz,
        ),
        0,
    )
    goal_constraints = ConstraintSet(robot, "goal")
    goal_constraints.addConstraint(goal_projector)
    problem.setConstraints(goal_constraints)
    success, q_goal, residual = problem.applyConstraints(q_start)
    if not success:
        raise RuntimeError(
            f"line end {np.round(goal_pose.translation, 3)} is not reachable "
            f"from the supplied configuration (HPP projection residual "
            f"{residual:.3g})"
        )
    q_goal = np.asarray(q_goal).flatten()
    valid, report = problem.isConfigValid(q_goal)
    if not valid:
        raise RuntimeError(f"goal configuration is invalid: {report}")

    direction = line / np.linalg.norm(line)
    z = direction
    x = np.cross([0.0, 0.0, 1.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross([0.0, 1.0, 0.0], z)
    x /= np.linalg.norm(x)
    line_frame = pin.SE3(
        np.column_stack([x, np.cross(z, x), z]), start_pose.translation
    )

    projector = ConfigProjector(robot, "line_projector", 1e-4, 40)
    projector.add(
        Implicit(
            Position(
                "tool0_on_line",
                robot,
                joint_id,
                tool_in_joint,
                line_frame,
                line_xy,
            ),
            equal2,
            active_xy,
        ),
        0,
    )
    projector.add(
        Implicit(
            Orientation(
                "tool0_orientation",
                robot,
                joint_id,
                pin.SE3(tool_in_joint.rotation, np.zeros(3)),
                pin.SE3(start_pose.rotation, np.zeros(3)),
                xyz,
            ),
            equal3,
            xyz,
        ),
        0,
    )
    constraint_set = ConstraintSet(robot, "line")
    constraint_set.addConstraint(projector)
    problem.setConstraints(constraint_set)

    success, path, report = problem.directPath(q_start, q_goal, True)
    if not success:
        raise RuntimeError(f"HPP could not find a collision-free line: {report}")

    configurations = sample_path(path, samples)
    positions = []
    for configuration in configurations:
        pin.forwardKinematics(model, data, configuration)
        pin.updateFramePlacements(model, data)
        positions.append(data.oMf[frame_id].translation.copy())
    positions = np.array(positions)
    offsets = positions - start_pose.translation
    closest = np.outer(offsets @ direction, direction)
    max_deviation = float(np.max(np.linalg.norm(offsets - closest, axis=1)))
    return CartesianLinePlan(
        configurations=configurations,
        start_position=start_pose.translation.copy(),
        end_position=goal_pose.translation.copy(),
        max_deviation=max_deviation,
    )


def compute_cartesian_line_trajectory(
    *, q_start=None, line=None, duration=5.0, samples=80
):
    """Return a plain JointTrajectory dictionary for a computed HPP line."""
    duration = float(duration)
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    plan = plan_cartesian_line(q_start=q_start, line=line, samples=samples)
    times = [0.0] + np.linspace(
        START_HOLD, START_HOLD + duration, len(plan.configurations)
    ).tolist()
    configurations = [plan.configurations[0], *plan.configurations]
    return joint_trajectory_payload(configurations, times, JOINT_NAMES)
