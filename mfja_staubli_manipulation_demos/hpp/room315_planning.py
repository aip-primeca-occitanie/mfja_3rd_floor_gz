"""HPP target selection, transition planning, and phase sampling."""

from dataclasses import dataclass

import numpy as np
from pyhpp.manipulation import TransitionPlanner

from room315_problem import (
    GRASP_TRANSITION,
    PICK_TRANSITIONS,
    RELEASE_TRANSITION,
    RELEASE_TRANSITIONS,
    TRANSFER_TRANSITION,
    box_rank,
    normalize_box_quaternion,
)


@dataclass
class PlannedSegment:
    transition_name: str
    path: object
    q_start: np.ndarray
    q_goal: np.ndarray


@dataclass
class ExecutionPhase:
    name: str
    planned_segments: list[PlannedSegment]
    payload_mode: str
    configs: list[np.ndarray]
    payload_configs: list[np.ndarray]
    times: list[float]


def make_goal_matrix(robot, q_goal):
    q_goals = np.zeros((1, robot.configSize()), order="F")
    q_goals[0, :] = q_goal
    return q_goals


def validate_transition_config(transition, q, label):
    valid, report = transition.pathValidation().validateConfiguration(q)
    if not valid:
        raise RuntimeError(f"{label} target is invalid: {report}")


def seeded_target(shooter, q_free, rank, attempt, preferred=None):
    q_seed = np.asarray(shooter.shoot()).flatten()
    q_seed[rank : rank + 7] = q_free[rank : rank + 7]
    if preferred is not None and attempt % 3 == 0:
        q_seed[:6] = preferred[:6]
    elif attempt % 3 == 1:
        q_seed[:6] = q_free[:6]
    return q_seed


def score_pick_chain(q_free, chain, preferred=None):
    reference = q_free if preferred is None else preferred
    configs = [q_free] + chain
    motion = sum(
        float(np.max(np.abs(current[:6] - previous[:6])))
        for previous, current in zip(configs[:-1], configs[1:])
    )
    posture = float(np.max(np.abs(chain[-1][:6] - reference[:6])))
    wrist_wrap = float(np.sum(np.maximum(0.0, np.abs(chain[-1][:6]) - np.pi)))
    return motion + 0.5 * posture + 0.5 * wrist_wrap


def generate_pick_chain(robot, problem, graph, q_free, attempts, label, preferred=None):
    shooter = problem.configurationShooter()
    rank = box_rank(robot)
    best_chain = None
    best_attempt = 0
    best_score = float("inf")

    for attempt in range(attempts):
        seed = seeded_target(shooter, q_free, rank, attempt, preferred)
        source = q_free
        chain = []

        for index, transition_name in enumerate(PICK_TRANSITIONS):
            transition = graph.getTransition(transition_name)
            initializer = seed if index == 0 else source
            ok, q_next, error = graph.generateTargetConfig(
                transition, source, initializer
            )
            if not ok:
                break

            q_next = np.asarray(q_next).flatten()
            try:
                validate_transition_config(
                    transition, q_next, f"{label} {transition_name}"
                )
            except RuntimeError:
                break

            chain.append(q_next)
            source = q_next

        if len(chain) != len(PICK_TRANSITIONS):
            continue

        score = score_pick_chain(q_free, chain, preferred)
        if score < best_score:
            best_chain = chain
            best_attempt = attempt + 1
            best_score = score

    if best_chain is not None:
        print(
            f"{label} pick chain selected from {attempts} attempt(s) "
            f"(best attempt {best_attempt}, score {best_score:.3f})",
            flush=True,
        )
        return best_chain

    raise RuntimeError(f"failed to generate {label} pick chain after {attempts} attempts")


def plan_transition(robot, planner, graph, transition_name, q_start, q_goal):
    transition = graph.getTransition(transition_name)
    validate_transition_config(transition, q_goal, transition_name)
    planner.setEdge(transition)
    success, path, report = planner.directPath(q_start, q_goal, True)
    if success:
        return PlannedSegment(transition_name, path, q_start, q_goal)

    try:
        path = planner.planPath(q_start, make_goal_matrix(robot, q_goal), True)
    except Exception as exc:
        raise RuntimeError(
            f"failed to plan transition {transition_name}: {report}"
        ) from exc
    return PlannedSegment(transition_name, path, q_start, q_goal)


def plan_manipulation(
    robot,
    problem,
    graph,
    q_source,
    q_destination,
    *,
    source_label,
    destination_label,
    target_attempts,
    transition_iterations,
    transition_timeout,
):
    problem.constraintGraph(graph)
    planner = TransitionPlanner(problem)
    planner.maxIterations(transition_iterations)
    planner.timeOut(transition_timeout)

    source_pick = generate_pick_chain(
        robot, problem, graph, q_source, target_attempts, source_label
    )
    destination_pick = generate_pick_chain(
        robot,
        problem,
        graph,
        q_destination,
        target_attempts,
        destination_label,
        preferred=source_pick[-1],
    )

    segments = []
    current = q_source
    for transition_name, target in zip(PICK_TRANSITIONS, source_pick):
        segment = plan_transition(robot, planner, graph, transition_name, current, target)
        segments.append(segment)
        current = target

    segment = plan_transition(
        robot, planner, graph, TRANSFER_TRANSITION, current, destination_pick[-1]
    )
    segments.append(segment)
    current = destination_pick[-1]

    release_targets = [
        destination_pick[2],
        destination_pick[1],
        destination_pick[0],
        q_destination,
    ]
    for transition_name, target in zip(RELEASE_TRANSITIONS, release_targets):
        segment = plan_transition(robot, planner, graph, transition_name, current, target)
        segments.append(segment)
        current = target

    return segments


def direction_endpoints(direction, q_shuttle, q_table, q_drop_shuttle):
    if direction == "shuttle-to-table":
        return q_shuttle, q_table, "shuttle", "table"
    if direction == "table-to-shuttle":
        return q_table, q_shuttle, "table", "shuttle"
    if direction == "shuttle-to-shuttle":
        if q_drop_shuttle is None:
            raise RuntimeError("shuttle-to-shuttle requires --destination-shuttle-pose")
        return q_shuttle, q_drop_shuttle, "pickup-shuttle", "drop-shuttle"
    raise ValueError(f"unsupported manipulation direction: {direction}")


def sample_path(path, samples):
    length = float(path.length())
    if samples < 2:
        samples = 2
    if length <= 1e-9:
        q, ok = path(0.0)
        if not ok:
            raise RuntimeError("HPP failed to evaluate a zero-length path")
        config = np.asarray(q).flatten()
        return [config, config.copy()]

    configs = []
    for index in range(samples):
        q, ok = path(index / (samples - 1) * length)
        if not ok:
            raise RuntimeError(f"HPP failed to evaluate path sample {index}")
        configs.append(np.asarray(q).flatten())
    return configs


def format_plan(segments):
    rows = []
    total = 0.0
    for index, segment in enumerate(segments):
        length = float(segment.path.length())
        total += length
        rows.append((index, segment.transition_name, length))

    print("planned manipulation transitions:")
    for index, name, length in rows:
        print(f"  {index:02d}  {length:8.3f}  {name}")
    print(f"total HPP path parameter length: {total:.3f}")


def path_sample_count(path, samples_per_path_unit, min_segment_samples):
    return max(min_segment_samples, int(float(path.length()) * samples_per_path_unit) + 1)


def retime_joint_configs(configs, *, max_joint_speed, min_sample_dt, initial_hold):
    times = [0.0]
    if len(configs) > 1:
        times.append(initial_hold)

    for previous, current in zip(configs[1:-1], configs[2:]):
        delta = float(np.max(np.abs(current[:6] - previous[:6])))
        times.append(times[-1] + max(min_sample_dt, delta / max_joint_speed))
    return times


def execution_config(robot, arm_config, payload_config):
    q = np.asarray(arm_config).copy()
    rank = box_rank(robot)
    q[rank : rank + 7] = payload_config[rank : rank + 7]
    return normalize_box_quaternion(robot, q)


def append_execution_sample(robot, arm_configs, payload_configs, arm_config, payload_config):
    rank = box_rank(robot)
    arm_config = np.asarray(arm_config).flatten()
    payload_config = np.asarray(payload_config).flatten()
    same_arm = (
        arm_configs
        and np.max(np.abs(arm_config[:6] - arm_configs[-1][:6])) < 1e-8
    )
    same_payload = (
        payload_configs
        and np.max(
            np.abs(
                payload_config[rank : rank + 7]
                - payload_configs[-1][rank : rank + 7]
            )
        )
        < 1e-8
    )
    if same_arm and same_payload:
        arm_configs[-1] = arm_config
        payload_configs[-1] = payload_config
    else:
        arm_configs.append(arm_config)
        payload_configs.append(payload_config)


def build_execution_phase(
    robot,
    graph,
    name,
    planned_segments,
    payload_mode,
    fixed_payload,
    args,
):
    configs = []
    payload_configs = []
    transition_names = []

    for segment_index, segment in enumerate(planned_segments):
        transition = graph.getTransition(segment.transition_name)
        transition_names.append(segment.transition_name)
        samples = path_sample_count(
            segment.path, args.samples_per_path_unit, args.min_segment_samples
        )
        segment_configs = sample_path(segment.path, samples)

        if payload_mode == "follow":
            segment_payload = segment_configs
        else:
            segment_payload = [fixed_payload.copy() for _ in segment_configs]

        for sample_index, (arm_config, payload_config) in enumerate(
            zip(segment_configs, segment_payload)
        ):
            arm_config = np.asarray(arm_config).flatten()
            payload_config = np.asarray(payload_config).flatten()
            q = execution_config(robot, arm_config, payload_config)
            valid, report = transition.pathValidation().validateConfiguration(q)
            if not valid:
                raise RuntimeError(
                    f"execution phase {name} segment {segment_index} "
                    f"sample {sample_index} is invalid: {report}"
                )
            append_execution_sample(
                robot, configs, payload_configs, arm_config, payload_config
            )

    if configs:
        configs.insert(1, configs[0].copy())
        payload_configs.insert(1, payload_configs[0].copy())

    times = retime_joint_configs(
        configs,
        max_joint_speed=args.max_joint_speed,
        min_sample_dt=args.min_sample_dt,
        initial_hold=args.phase_start_hold,
    )
    validate_sampled_configs(robot, configs, payload_configs, times)
    print(
        f"execution phase {name}: {payload_mode}, "
        f"{len(configs)} points, {times[-1]:.1f} s",
        flush=True,
    )
    for transition_name in transition_names:
        print(f"  {transition_name}", flush=True)
    return ExecutionPhase(
        name,
        planned_segments,
        payload_mode,
        configs,
        payload_configs,
        times,
    )


def build_execution_phases(
    robot,
    graph,
    segments,
    q_source,
    q_destination,
    source_label,
    destination_label,
    args,
):
    grasp_index = next(
        index
        for index, segment in enumerate(segments)
        if segment.transition_name == GRASP_TRANSITION
    )
    release_index = next(
        index
        for index, segment in enumerate(segments)
        if segment.transition_name == RELEASE_TRANSITION
    )

    phases = [
        build_execution_phase(
            robot,
            graph,
            f"approach-{source_label}-pregrasp",
            segments[:grasp_index],
            f"{source_label}-fixed",
            q_source,
            args,
        ),
        build_execution_phase(
            robot,
            graph,
            "grasp-transfer",
            segments[grasp_index:release_index],
            "follow",
            q_source,
            args,
        ),
        build_execution_phase(
            robot,
            graph,
            f"release-{destination_label}-retreat",
            segments[release_index:],
            f"{destination_label}-fixed",
            q_destination,
            args,
        ),
    ]
    total_points = sum(len(phase.configs) for phase in phases)
    total_duration = sum(phase.times[-1] for phase in phases)
    print(
        f"execution preview: {len(phases)} phases, "
        f"{total_points} points, {total_duration:.1f} s",
        flush=True,
    )
    return phases


def validate_sampled_configs(robot, arm_configs, payload_configs, times):
    if not (len(arm_configs) == len(payload_configs) == len(times)):
        raise RuntimeError(
            "internal execution sampling error: arm, payload, and time lengths differ"
        )
    if len(arm_configs) < 2:
        raise RuntimeError("internal execution sampling error: empty trajectory")
    for index, (arm, payload) in enumerate(zip(arm_configs, payload_configs)):
        if arm.shape[0] != robot.configSize() or payload.shape[0] != robot.configSize():
            raise RuntimeError(f"internal execution sample {index} has wrong size")
