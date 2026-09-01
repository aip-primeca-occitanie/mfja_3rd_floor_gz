"""HPP model and problem construction for the Room 315 demo."""

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose
from pyhpp.manipulation import Device, Graph, Problem, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins

from room315_config import load_config

config = load_config()


def se3_from_pose(pose):
    x, y, z, roll, pitch, yaw = pose
    return pin.SE3(pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z]))


def world_pose_in_robot_frame(world_pose):
    robot_world_pose = config["scene"]["robot_world_pose"]
    return se3_from_pose(robot_world_pose).inverse() * se3_from_pose(world_pose)


def pose_msg_from_se3(placement):
    quat = pin.Quaternion(placement.rotation).coeffs()
    pose = Pose()
    pose.position.x = float(placement.translation[0])
    pose.position.y = float(placement.translation[1])
    pose.position.z = float(placement.translation[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def build_problem():
    scene = config["scene"]
    models = config["models"]
    graph_config = config["graph"]
    robot = Device("room315_staubli_manipulation")

    urdf.loadModel(
        robot,
        0,
        "staubli",
        "anchor",
        models["robot"]["urdf"],
        models["robot"]["srdf"],
        pin.SE3.Identity(),
    )
    urdf.loadModel(
        robot,
        0,
        "room315",
        "anchor",
        models["cell"]["urdf"],
        models["cell"]["srdf"],
        se3_from_pose(scene["robot_world_pose"]).inverse(),
    )
    urdf.loadModel(
        robot,
        0,
        "staubli_table",
        "anchor",
        models["table"]["urdf"],
        models["table"]["srdf"],
        world_pose_in_robot_frame(scene["table_drop_zone_pose"]),
    )
    urdf.loadModel(
        robot,
        0,
        "box",
        "freeflyer",
        models["payload"]["urdf"],
        models["payload"]["srdf"],
        pin.SE3.Identity(),
    )
    robot.setJointBounds(
        "box/root_joint",
        [
            -1.2,
            1.2,
            -1.0,
            1.2,
            -0.4,
            0.8,
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
        ],
    )

    problem = Problem(robot)
    problem.addConfigValidation("CollisionValidation")
    problem.addConfigValidation("JointBoundValidation")

    graph = Graph(graph_config["name"], robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(1e-5)

    factory = ConstraintGraphFactory(graph)
    factory.setGrippers([graph_config["gripper"]])
    factory.setObjects(
        ["box"],
        [[graph_config["payload_handle"]]],
        [[graph_config["payload_contact"]]],
    )
    factory.environmentContacts(["staubli_table/drop_zone"])
    factory.generate()

    margins = SecurityMargins(
        problem,
        factory,
        ["staubli", "box", "room315", "staubli_table"],
        robot,
    )
    margins.setSecurityMarginBetween(
        "box",
        "room315",
        graph_config["payload_to_room_margin"],
    )
    margins.setSecurityMarginBetween(
        "staubli",
        "room315",
        graph_config["robot_to_room_margin"],
    )
    margins.apply()

    graph.initialize()
    return robot, problem, graph


def box_rank(robot):
    return robot.rankInConfiguration["box/root_joint"]


def box_world_pose(robot, q):
    rank = box_rank(robot)
    quat = pin.Quaternion(np.asarray(q[rank + 3 : rank + 7]))
    box_in_robot = pin.SE3(quat.matrix(), np.asarray(q[rank : rank + 3]))
    return se3_from_pose(config["scene"]["robot_world_pose"]) * box_in_robot


def box_world_pose_msg(robot, q):
    return pose_msg_from_se3(box_world_pose(robot, q))


def box_configuration_from_world_pose(q_arm, world_pose):
    box_pose = world_pose_in_robot_frame(world_pose)
    return np.r_[q_arm, box_pose.translation, pin.Quaternion(box_pose.rotation).coeffs()]


def table_box_world_pose(x_offset=0.0, y_offset=0.0):
    scene = config["scene"]
    x, y, z, roll, pitch, yaw = scene["table_drop_zone_pose"]
    return (
        x + x_offset,
        y + y_offset,
        z + 0.5 * scene["payload_size"][2],
        roll,
        pitch,
        yaw,
    )


def project_free_configuration(problem, graph, q, label):
    ok, q_projected, error = graph.applyStateConstraints(graph.getState("free"), q)
    if not ok:
        raise RuntimeError(f"failed to project {label} on free state: {error:.3g}")
    q_projected = np.asarray(q_projected).flatten()
    valid, report = problem.isConfigValid(q_projected)
    if not valid:
        raise RuntimeError(f"{label} configuration is invalid: {report}")
    return q_projected


def normalize_box_quaternion(robot, q):
    q = np.asarray(q).copy()
    rank = box_rank(robot)
    quat = q[rank + 3 : rank + 7]
    norm = np.linalg.norm(quat)
    if norm > 1e-12:
        q[rank + 3 : rank + 7] = quat / norm
    return q
