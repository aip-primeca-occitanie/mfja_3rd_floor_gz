"""HPP model constants and problem construction for the Room 315 demo."""

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose
from pyhpp.manipulation import Device, Graph, Problem, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins

ROOM315_ROBOT_POSE = (-15.1622, -6.0, 1.0, 0.0, 0.0, 1.57)
DEFAULT_SHUTTLE_SLOT3_POSE = (-15.240, -5.536, 0.839, 0.0, 0.0, 0.0)
DEFAULT_SHUTTLE_SLOT4_POSE = (-14.770, -5.536, 0.839346, 0.0, 0.0, -0.0014)
TABLE_DROP_ZONE_POSE = (-14.65, -5.84, 1.003, 0.0, 0.0, 0.0)
GRAPH_NAME = "room315_staubli_shuttle_box"

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
DEFAULT_Q_START = np.array(
    [-1.56136443, 0.47307870, 2.04964315, -0.00130315, -0.32991444, 0.00524110]
)
BOX_SIZE = (0.07, 0.05, 0.06)
BOX_HEIGHT = BOX_SIZE[2]
SHUTTLE_CONTACT_Z = 0.085
BOX_ENTITY_NAME = "room315_payload_box"
WORLD_NAME = "room_315_only"
BOX_ROOM315_MARGIN = 0.03
STAUBLI_ROOM315_MARGIN = 0.02

ROBOT_URDF = (
    "package://mfja_staubli_manipulation_demos/urdf/"
    "staubli_tx2_60l_gripper.urdf"
)
ROBOT_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/"
    "staubli_tx2_60l_manipulation.srdf"
)
CELL_URDF = "package://mfja_staubli_manipulation_demos/hpp/room315_cell.urdf"
CELL_SRDF = "package://mfja_staubli_manipulation_demos/hpp/room315_cell.srdf"
BOX_URDF = "package://mfja_staubli_manipulation_demos/hpp/room315_payload_box.urdf"
BOX_SRDF = "package://mfja_staubli_manipulation_demos/hpp/room315_payload_box.srdf"
SHUTTLE_URDF = (
    "package://mfja_staubli_manipulation_demos/hpp/room315_shuttle_deck.urdf"
)
SHUTTLE_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/room315_shuttle_deck.srdf"
)
TABLE_URDF = (
    "package://mfja_staubli_manipulation_demos/hpp/"
    "room315_staubli_table_drop_zone.urdf"
)
TABLE_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/"
    "room315_staubli_table_drop_zone.srdf"
)

GRIPPER_NAME = "staubli/tool0_gripper"
BOX_HANDLE = "box/top_handle"
BOX_CONTACT = "box/bottom_surface"
GAZEBO_GRIPPER_JOINTS = [
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]
GAZEBO_GRIPPER_OPEN_POSITIONS = [0.028, 0.028]
GAZEBO_GRIPPER_CLOSE_POSITIONS = [0.0255, 0.0255]
GRASP_NAME = f"{GRIPPER_NAME} > {BOX_HANDLE}"
RELEASE_NAME = f"{GRIPPER_NAME} < {BOX_HANDLE}"
PICK_TRANSITIONS = [f"{GRASP_NAME} | f_{step}" for step in ("01", "12", "23", "34")]
TRANSFER_TRANSITION = "Loop | 0-0"
RELEASE_TRANSITIONS = [
    f"{RELEASE_NAME} | 0-0_{step}" for step in ("43", "32", "21", "10")
]
GRASP_TRANSITION = f"{GRASP_NAME} | f_23"
RELEASE_TRANSITION = f"{RELEASE_NAME} | 0-0_21"

PAYLOAD_BOX_SDF = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{BOX_ENTITY_NAME}">
    <static>true</static>
    <link name="base_link">
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.0002</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0002</iyy>
          <iyz>0</iyz>
          <izz>0.0002</izz>
        </inertia>
      </inertial>
      <visual name="payload_visual">
        <geometry>
          <box>
            <size>{BOX_SIZE[0]} {BOX_SIZE[1]} {BOX_SIZE[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.35 0.95 1</ambient>
          <diffuse>0.05 0.35 0.95 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def se3_from_pose(pose):
    x, y, z, roll, pitch, yaw = pose
    return pin.SE3(pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z]))


def world_pose_in_robot_frame(world_pose):
    return se3_from_pose(ROOM315_ROBOT_POSE).inverse() * se3_from_pose(world_pose)


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


def build_problem(shuttle_pose, destination_shuttle_pose=None):
    robot = Device("room315_staubli_manipulation")

    urdf.loadModel(
        robot, 0, "staubli", "anchor", ROBOT_URDF, ROBOT_SRDF, pin.SE3.Identity()
    )
    urdf.loadModel(
        robot,
        0,
        "room315",
        "anchor",
        CELL_URDF,
        CELL_SRDF,
        se3_from_pose(ROOM315_ROBOT_POSE).inverse(),
    )
    urdf.loadModel(
        robot,
        0,
        "shuttle",
        "anchor",
        SHUTTLE_URDF,
        SHUTTLE_SRDF,
        world_pose_in_robot_frame(shuttle_pose),
    )
    environment_contacts = ["shuttle/top_surface", "staubli_table/drop_zone"]
    security_margin_names = ["staubli", "box", "room315", "shuttle", "staubli_table"]

    if destination_shuttle_pose is not None:
        urdf.loadModel(
            robot,
            0,
            "drop_shuttle",
            "anchor",
            SHUTTLE_URDF,
            SHUTTLE_SRDF,
            world_pose_in_robot_frame(destination_shuttle_pose),
        )
        environment_contacts.append("drop_shuttle/top_surface")
        security_margin_names.append("drop_shuttle")

    urdf.loadModel(
        robot,
        0,
        "staubli_table",
        "anchor",
        TABLE_URDF,
        TABLE_SRDF,
        world_pose_in_robot_frame(TABLE_DROP_ZONE_POSE),
    )
    urdf.loadModel(
        robot, 0, "box", "freeflyer", BOX_URDF, BOX_SRDF, pin.SE3.Identity()
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

    graph = Graph(GRAPH_NAME, robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(1e-5)

    factory = ConstraintGraphFactory(graph)
    factory.setGrippers([GRIPPER_NAME])
    factory.setObjects(
        ["box"],
        [[BOX_HANDLE]],
        [[BOX_CONTACT]],
    )
    factory.environmentContacts(environment_contacts)
    factory.generate()

    margins = SecurityMargins(
        problem,
        factory,
        security_margin_names,
        robot,
    )
    margins.setSecurityMarginBetween("box", "room315", BOX_ROOM315_MARGIN)
    margins.setSecurityMarginBetween("staubli", "room315", STAUBLI_ROOM315_MARGIN)
    margins.apply()

    graph.initialize()
    return robot, problem, graph


def mapping_names(mapping):
    if hasattr(mapping, "keys"):
        return sorted(mapping.keys())
    return sorted(entry.key() for entry in mapping)


def box_rank(robot):
    return robot.rankInConfiguration["box/root_joint"]


def box_world_pose(robot, q):
    rank = box_rank(robot)
    quat = pin.Quaternion(np.asarray(q[rank + 3 : rank + 7]))
    box_in_robot = pin.SE3(quat.matrix(), np.asarray(q[rank : rank + 3]))
    return se3_from_pose(ROOM315_ROBOT_POSE) * box_in_robot


def box_world_pose_msg(robot, q):
    return pose_msg_from_se3(box_world_pose(robot, q))


def box_configuration_from_world_pose(q_arm, world_pose):
    box_pose = world_pose_in_robot_frame(world_pose)
    return np.r_[q_arm, box_pose.translation, pin.Quaternion(box_pose.rotation).coeffs()]


def shuttle_box_world_pose(shuttle_pose):
    x, y, z, roll, pitch, yaw = shuttle_pose
    return (x, y, z + SHUTTLE_CONTACT_Z + 0.5 * BOX_HEIGHT, roll, pitch, yaw)


def table_box_world_pose():
    x, y, z, roll, pitch, yaw = TABLE_DROP_ZONE_POSE
    return (x, y, z + 0.5 * BOX_HEIGHT, roll, pitch, yaw)


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
