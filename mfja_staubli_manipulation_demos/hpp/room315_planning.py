"""HPP target selection, transition planning, and execution segment sampling."""

from dataclasses import dataclass

import numpy as np
from hpp_exec import Segment
from pyhpp.manipulation import TransitionPlanner

from room315_problem import box_rank, config, normalize_box_quaternion


@dataclass
class PlannedSegment:
    transition_name: str
    path: object


@dataclass
class ExecutionPlan:
    configs: list[np.ndarray]
    payload_configs: list[np.ndarray]
    times: list[float]
    segments: list[Segment]
    segment_names: list[str]
    payload_modes: list[str]


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


def generate_pick_chains(robot, problem, graph, q_free, attempts, label, preferred=None):
    pick_transitions = config["graph"]["transitions"]["pick"]
    shooter = problem.configurationShooter()
    rank = box_rank(robot)
    candidates = []

    for attempt in range(attempts):
        seed = seeded_target(shooter, q_free, rank, attempt, preferred)
        source = q_free
        chain = []

        for index, transition_name in enumerate(pick_transitions):
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

        if len(chain) != len(pick_transitions):
            continue

        score = score_pick_chain(q_free, chain, preferred)
        if any(
            np.max(np.abs(np.asarray(chain) - np.asarray(previous))) < 1e-5
            for _, _, previous in candidates
        ):
            continue
        candidates.append((score, attempt + 1, chain))

    if candidates:
        candidates.sort(key=lambda candidate: candidate[0])
        best_score, best_attempt, _ = candidates[0]
        print(
            f"{label}: generated {len(candidates)} valid pick chain(s) from "
            f"{attempts} attempt(s) (best attempt {best_attempt}, "
            f"score {best_score:.3f})",
            flush=True,
        )
        return [chain for _, _, chain in candidates]

    raise RuntimeError(f"failed to generate {label} pick chain after {attempts} attempts")


def plan_transition(robot, planner, graph, transition_name, q_start, q_goal):
    transition = graph.getTransition(transition_name)
    validate_transition_config(transition, q_goal, transition_name)
    planner.setTransition(transition)
    success, path, report = planner.directPath(q_start, q_goal, True)
    if success:
        return PlannedSegment(transition_name, path)

    try:
        q_goals = np.zeros((1, robot.configSize()), order="F")
        q_goals[0, :] = q_goal
        path = planner.planPath(q_start, q_goals, True)
    except Exception as exc:
        raise RuntimeError(
            f"failed to plan transition {transition_name}: {report}"
        ) from exc
    return PlannedSegment(transition_name, path)


def validate_simple_plan(segments):
    planning_config = config["planning"]
    first_length = float(segments[0].path.length())
    total_length = sum(float(segment.path.length()) for segment in segments)
    if first_length > planning_config["max_first_approach_path_length"]:
        raise RuntimeError(
            f"first approach path length {first_length:.3f} exceeds the simple-demo "
            f"limit {planning_config['max_first_approach_path_length']:.3f}"
        )
    if total_length > planning_config["max_manipulation_path_length"]:
        raise RuntimeError(
            f"manipulation path length {total_length:.3f} exceeds the simple-demo "
            f"limit {planning_config['max_manipulation_path_length']:.3f}"
        )
    return first_length, total_length


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
    target_pair_attempts,
    transition_iterations,
    transition_timeout,
):
    transitions = config["graph"]["transitions"]
    problem.constraintGraph(graph)
    planner = TransitionPlanner(problem)
    planner.maxIterations(transition_iterations)
    planner.timeOut(transition_timeout)

    source_picks = generate_pick_chains(
        robot, problem, graph, q_source, target_attempts, source_label
    )
    destination_picks = generate_pick_chains(
        robot,
        problem,
        graph,
        q_destination,
        target_attempts,
        destination_label,
        preferred=source_picks[0][-1],
    )

    target_pairs = [
        (source_index, destination_index)
        for source_index in range(len(source_picks))
        for destination_index in range(len(destination_picks))
    ]
    target_pairs.sort(key=lambda pair: (sum(pair), pair[0]))

    last_error = None
    attempted_pairs = 0
    for source_index, destination_index in target_pairs[:target_pair_attempts]:
        source_pick = source_picks[source_index]
        destination_pick = destination_picks[destination_index]
        attempted_pairs += 1
        try:
            segments = []
            current = q_source
            for transition_name, target in zip(transitions["pick"], source_pick):
                segments.append(
                    plan_transition(
                        robot, planner, graph, transition_name, current, target
                    )
                )
                current = target

            segments.append(
                plan_transition(
                    robot,
                    planner,
                    graph,
                    transitions["transfer"],
                    current,
                    destination_pick[-1],
                )
            )
            current = destination_pick[-1]

            release_targets = list(reversed(destination_pick[:-1])) + [q_destination]
            for transition_name, target in zip(
                transitions["release"], release_targets
            ):
                segments.append(
                    plan_transition(
                        robot, planner, graph, transition_name, current, target
                    )
                )
                current = target
            validate_simple_plan(segments)
        except RuntimeError as exc:
            last_error = exc
            print(
                f"target pair {source_index + 1}/{destination_index + 1} "
                f"failed: {exc}",
                flush=True,
            )
            continue

        print(
            f"planned target pair {source_index + 1}/{destination_index + 1}",
            flush=True,
        )
        return segments

    raise RuntimeError(
        f"failed to plan {attempted_pairs} target pair(s): {last_error}"
    ) from last_error


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
    total = 0.0
    print("planned manipulation transitions:")
    for index, segment in enumerate(segments):
        length = float(segment.path.length())
        total += length
        print(f"  {index:02d}  {length:8.3f}  {segment.transition_name}")
    print(f"total HPP path parameter length: {total:.3f}")


def retime_joint_configs(configs, *, max_joint_speed, min_sample_dt, initial_hold):
    times = [0.0]
    if len(configs) > 1:
        times.append(initial_hold)

    for previous, current in zip(configs[1:-1], configs[2:]):
        delta = float(np.max(np.abs(current[:6] - previous[:6])))
        times.append(times[-1] + max(min_sample_dt, delta / max_joint_speed))
    return times


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


def sample_execution_segment(
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
    rank = box_rank(robot)

    for segment_index, segment in enumerate(planned_segments):
        transition = graph.getTransition(segment.transition_name)
        transition_names.append(segment.transition_name)
        samples = max(
            args.min_segment_samples,
            int(float(segment.path.length()) * args.samples_per_path_unit) + 1,
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
            q = arm_config.copy()
            q[rank : rank + 7] = payload_config[rank : rank + 7]
            q = normalize_box_quaternion(robot, q)
            valid, report = transition.pathValidation().validateConfiguration(q)
            if not valid:
                raise RuntimeError(
                    f"execution segment {name} transition {segment_index} "
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
        initial_hold=args.segment_start_hold,
    )
    print(
        f"execution segment {name}: {payload_mode}, "
        f"{len(configs)} points, {times[-1]:.1f} s",
        flush=True,
    )
    for transition_name in transition_names:
        print(f"  {transition_name}", flush=True)
    return configs, payload_configs, times


def build_execution_plan(
    robot,
    graph,
    segments,
    q_source,
    q_destination,
    source_label,
    destination_label,
    args,
):
    transitions = config["graph"]["transitions"]
    grasp_index = next(
        index
        for index, segment in enumerate(segments)
        if segment.transition_name == transitions["grasp"]
    )
    release_index = next(
        index
        for index, segment in enumerate(segments)
        if segment.transition_name == transitions["detach"]
    )

    segment_specs = [
        (
            f"approach-{source_label}-pregrasp",
            segments[:grasp_index],
            f"{source_label}-fixed",
            q_source,
            segments[0].transition_name,
        ),
        (
            "grasp-transfer",
            segments[grasp_index:release_index],
            "follow",
            q_source,
            transitions["grasp"],
        ),
        (
            f"release-{destination_label}-retreat",
            segments[release_index:],
            f"{destination_label}-fixed",
            q_destination,
            transitions["detach"],
        ),
    ]

    configs = []
    payload_configs = []
    times = []
    execution_segments = []
    segment_names = []
    payload_modes = []
    time_offset = 0.0

    for name, planned, payload_mode, fixed_payload, transition_name in segment_specs:
        segment_configs, segment_payload_configs, segment_times = (
            sample_execution_segment(
                robot,
                graph,
                name,
                planned,
                payload_mode,
                fixed_payload,
                args,
            )
        )
        start_index = len(configs)
        configs.extend(segment_configs)
        payload_configs.extend(segment_payload_configs)
        times.extend(time_offset + value for value in segment_times)
        end_index = len(configs)
        end_time = time_offset + segment_times[-1]
        execution_segments.append(
            Segment(
                start_index,
                end_index,
                start_time=time_offset,
                end_time=end_time,
                transition_name=transition_name,
            )
        )
        segment_names.append(name)
        payload_modes.append(payload_mode)
        time_offset = end_time

    print(
        f"execution preview: {len(execution_segments)} segments, "
        f"{len(configs)} points, {time_offset:.1f} s",
        flush=True,
    )
    return ExecutionPlan(
        configs,
        payload_configs,
        times,
        execution_segments,
        segment_names,
        payload_modes,
    )
