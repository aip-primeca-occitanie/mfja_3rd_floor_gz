#!/usr/bin/env python3
"""Generate deterministic LaTeX source catalogues for the handover manual."""

from __future__ import annotations

import ast
import re
import subprocess
from collections import Counter
from pathlib import Path


DOC_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = DOC_ROOT.parents[1]
GENERATED_DIR = DOC_ROOT / "generated"


FALLBACK_DESCRIPTIONS = {
    "conveyor_loop_mode_controller.py": (
        "Controls the visual Room 315 switch assemblies and mirrors their "
        "accepted loop state in Gazebo."
    ),
    "presentation_left_slot3_kuka_slot2.py": (
        "Runs the bounded presentation scenario with a left shuttle and KUKA."
    ),
    "robot_joint_command.py": (
        "Validates robot selectors and joint targets, then publishes a namespaced JointTrajectory."
    ),
    "room_315_device_position_tool.py": (
        "Converts between Gazebo coordinates and rail segment/arc positions for device calibration."
    ),
    "room_315_kinematic_shuttle.py": (
        "ROS-independent directed-rail geometry, routing, interpolation, and one-shuttle motion core."
    ),
    "room_315_kinematic_shuttle_node.py": (
        "ROS 2/Gazebo adapter for multi-shuttle lifecycle, rail devices, markers, payloads, and pose updates."
    ),
    "room_315_vla_dataset_recorder.py": (
        "Records synchronized camera, task, action, and privileged evaluation data into guarded episodes."
    ),
    "room_315_vla_event_extractor.py": (
        "Extracts event-level VLA examples from recorded Room 315 episodes."
    ),
    "room_315_vla_split_dataset.py": (
        "Creates deterministic, leakage-aware train/validation dataset partitions."
    ),
    "room_315_vla_supervisor.py": (
        "Deterministic safety supervisor that validates primitives and alone publishes typed rail commands."
    ),
    "room_315_vla_to_lerobot.py": (
        "Exports the repository episode representation to the optional LeRobot format."
    ),
}


MODEL_ROLES = {
    "mfja_3rd_floor": "Complete third-floor environment asset",
    "room_315": "Detailed Room 315 room and rail-cell asset",
    "room315_vla_overhead_devices": "Paired rail-focused RGB-D camera assembly",
    "room315_shuttle": "Generic shuttle template",
    "room315_vla_removable_obstacle_marker": "Removable VLA obstacle marker",
    "kuka_kr6r900sixx": "KUKA KR6 R900 Sixx visual robot model",
    "staubli_tx2_60l": "Stäubli TX2-60L visual robot model",
    "yaskawa_hc10": "Yaskawa HC10 visual robot model",
    "yaskawa_hc10dt": "Yaskawa HC10DT visual robot model",
    "tiago": "TIAGo mobile manipulator model",
    "tiago_base": "TIAGo mobile-base variant",
    "rail_switch_3pos_droit": "Animated three-position rail-switch model",
    "rail_switch_3pos_droit_static_black": "Neutral static rail-switch model",
}


CONFIG_ROLES = {
    "robots.yaml": "Full-floor robot registry, spawn pose, model, and enable flags",
    "robots_room_315_only.yaml": "Room-only robot registry and defaults",
    "gripper_command_defaults.yaml": "Per-robot gripper endpoints and percentage defaults",
    "gazebo_params.yaml": "Installed legacy Gazebo parameter file; no current runtime consumer",
    "rail_network_right.yaml": "Right-rail directed topology and routing",
    "rail_network_left.yaml": "Left-rail directed topology and public switch mapping",
    "rail_devices_right.yaml": "Right slots, binary sensors, and stoppers",
    "rail_devices_left.yaml": "Left slots, binary sensors, and stoppers",
    "domain_room315.pddl": "Expert/data-generation symbolic domain",
    "domain_room315_runtime.pddl": "Runtime PlanSys2 symbolic domain",
    "shuttle_identity.yaml": "Canonical identity and visual marker registry",
    "task_execution_runtime.yaml": "Default fail-closed task-execution parameters",
    "task_execution_runtime_v4.yaml": "Versioned V4 qualification task-execution snapshot",
    "task_goal_understanding.yaml": "English TaskGoal parser and local-model policy",
    "task_goal_english_benchmark.yaml": "Offline TaskGoal benchmark corpus",
    "visual_state_runtime.yaml": "Active/shadow visual runtime and artifact guard defaults",
    "visual_state_training_v4.json": "V4 training, split, loss, and evaluation contract",
    "visual_state_final_test_v4.json": "Sealed V4 final-test preparation contract",
    "visual_observed_state_calibration.yaml": "Visual observation calibration and validation policy",
    "vla_supervisor.yaml": "Primitive supervisor defaults",
}

SCRIPT_CATEGORY_ORDER = {
    "Robot and demonstration control": 0,
    "Rail runtime and fleet foundations": 1,
    "Visual datasets, models, training, and evaluation": 2,
    "Language and TaskGoal understanding": 3,
    "Contracts, planning, and execution": 4,
    "VLA supervision, recording, and operations": 5,
    "Qualification and immutable runtime packaging": 6,
}


def run_git(*args: str) -> str:
    completed = subprocess.run(
        ["git", *args],
        cwd=REPO_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return completed.stdout.strip()


def tracked(pattern: str | None = None) -> list[Path]:
    output = run_git("ls-files", pattern) if pattern else run_git("ls-files")
    return [REPO_ROOT / line for line in output.splitlines() if line.strip()]


def ensure_clean_package_snapshot() -> None:
    """Refuse to stamp mutable package content as the documented HEAD snapshot."""
    package_roots = [
        "mfja_rail_interfaces",
        "mfja_3rd_floor_description",
        "mfja_robot_control_config",
        "mfja_3rd_floor_bringup",
    ]
    dirty = run_git(
        "status",
        "--porcelain=v1",
        "--untracked-files=all",
        "--",
        *package_roots,
    )
    if dirty:
        details = "\n".join(f"  {line}" for line in dirty.splitlines())
        raise SystemExit(
            "Refusing to generate a HEAD-labelled catalogue from dirty ROS package "
            f"content. Commit or intentionally restore these paths first:\n{details}"
        )


def tracked_modes(pattern: str) -> dict[str, str]:
    """Return index modes keyed by repository-relative path."""
    output = run_git("ls-files", "--stage", pattern)
    modes: dict[str, str] = {}
    for line in output.splitlines():
        metadata, relative = line.split("\t", maxsplit=1)
        modes[relative] = metadata.split(maxsplit=1)[0]
    return modes


def auxiliary_code_role(relative: str) -> str:
    """Classify executable/source-like files outside the four ROS packages."""
    if relative == "flake.nix":
        return "Auxiliary Nix development shell; it is not the complete ROS environment."
    if "/source_snapshot/" in relative:
        return "Frozen evidence source snapshot; never edit or import it as live runtime code."
    if relative.startswith("report/tools/test_"):
        return "Offline regression test for a report campaign runner."
    if relative.startswith("report/tools/"):
        return "Offline, hash-bound campaign or evaluation runner used by the report."
    if relative.startswith("report/scripts/"):
        return "Report figure-generation utility; outside the ROS runtime."
    if relative.startswith("report/evidence/"):
        return "Frozen evidence reproduction, verification, evaluation, or figure helper."
    return "Supplemental offline automation outside the maintained ROS runtime."


def tex_escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(char, char) for char in value)


def tex_label_slug(value: str) -> str:
    """Return a stable, label-safe identifier for generated table labels."""
    return re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")


def order_top_level_sections(lines: list[str], ordered_titles: list[str]) -> list[str]:
    """Place reader-facing source sections in the requested maintenance order."""
    prefix: list[str] = []
    blocks: dict[str, list[str]] = {}
    current_title: str | None = None
    for line in lines:
        match = re.fullmatch(r"\\section\{([^{}]+)\}", line)
        if match:
            current_title = match.group(1)
            blocks[current_title] = [line]
        elif current_title is None:
            prefix.append(line)
        else:
            blocks[current_title].append(line)

    missing = [title for title in ordered_titles if title not in blocks]
    if missing:
        raise RuntimeError(f"Missing repository-index sections: {missing}")
    remaining = [title for title in blocks if title not in ordered_titles]
    return prefix + [
        line
        for title in [*ordered_titles, *remaining]
        for line in blocks[title]
    ]


def first_sentence(value: str, limit: int = 230) -> str:
    compact = " ".join(value.strip().split())
    if not compact:
        return ""
    match = re.search(r"(?<=[.!?])\s", compact)
    sentence = compact[: match.start() + 1] if match else compact
    if len(sentence) > limit:
        sentence = sentence[: limit - 1].rstrip() + "\u2026"
    return sentence


def module_description(path: Path) -> str:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        description = first_sentence(ast.get_docstring(tree) or "")
    except (OSError, SyntaxError):
        description = ""
    return description or FALLBACK_DESCRIPTIONS.get(
        path.name,
        "Specialized Room 315 utility; inspect --help and its tests before use.",
    )


def script_category(name: str) -> str:
    if name.startswith(("robot_", "conveyor_", "presentation_")):
        return "Robot and demonstration control"
    if any(token in name for token in (
        "kinematic_shuttle", "rail_defaults", "rail_devices", "shuttle_geometry",
        "device_position", "multi_shuttle",
    )):
        return "Rail runtime and fleet foundations"
    if "task_goal" in name or name.startswith("setup_room315_intent"):
        return "Language and TaskGoal understanding"
    if any(token in name for token in (
        "pddl_", "task_execution", "closed_loop_executive", "contracts",
        "observed_state_provider", "presence_provider",
    )):
        return "Contracts, planning, and execution"
    if any(token in name for token in (
        "runtime_acceptance", "authorize_runtime", "promote_runtime",
        "build_runtime_candidate", "closed_loop_fault_campaign",
    )):
        return "Qualification and immutable runtime packaging"
    if any(token in name for token in (
        "vla_supervisor", "vla_dataset", "vla_event", "vla_split", "vla_to_lerobot",
        "vla_benchmark", "vla_obstacle", "vla_shuttle_identity", "payload_case",
    )):
        return "VLA supervision, recording, and operations"
    return "Visual datasets, models, training, and evaluation"


def test_family(name: str) -> str:
    rules = (
        (("robot", "gripper", "tiago", "mobile"), "Robot and model control"),
        (("rail", "shuttle", "switch", "stopper", "headway", "block_reservation"), "Rail and fleet safety"),
        (("task_goal", "task_execution", "closed_loop"), "Task understanding and execution"),
        (("pddl", "route_topology"), "PDDL and topology planning"),
        (("dataset", "capture", "split", "label", "scenario"), "Dataset generation and integrity"),
        (("visual", "vla"), "Visual/VLA model and runtime"),
        (("contract", "runtime_candidate", "acceptance", "authorization", "promote"), "Contracts and deployment gates"),
        (("launch", "singleton", "full_package"), "Launch and package integration"),
    )
    for tokens, family in rules:
        if any(token in name for token in tokens):
            return family
    return "General regression"


def humanize_test(name: str) -> str:
    stem = Path(name).stem
    stem = re.sub(r"^test_", "", stem)
    stem = re.sub(r"^room315_", "", stem)
    return "Regression coverage for " + stem.replace("_", " ") + "."


def config_role(path: Path) -> str:
    if path.name in CONFIG_ROLES:
        return CONFIG_ROLES[path.name]
    suffix = path.suffix.lower()
    if suffix == ".csv":
        return "Calibrated directed rail-segment samples (index, x, y, z)"
    if suffix in {".gui", ".config"} or ".gui." in path.name:
        return "Gazebo GUI layout/profile"
    if "scenario" in path.name or "cases" in path.name:
        return "Scenario or benchmark manifest"
    if "experiment" in path.name:
        return "Versioned experiment/dataset contract"
    if suffix in {".yaml", ".json"}:
        return "Versioned runtime, dataset, or evaluation configuration"
    if suffix == ".md":
        return "Package-local configuration guidance"
    if suffix == ".html":
        return "Supplemental explanatory material; not a runtime input"
    return "Configuration support file"


def model_role(name: str) -> str:
    if name in MODEL_ROLES:
        return MODEL_ROLES[name]
    if name.startswith("room315_shuttle_"):
        return "Identity-specific preloaded shuttle visual asset"
    if name.startswith("room315_vla_payload_"):
        return "VLA payload/occlusion visual asset"
    if name.startswith("room"):
        return "Reusable room furniture or environment asset"
    if name.endswith("table") or "table" in name:
        return "Robot/laboratory table asset"
    if "cell" in name or "carter" in name or "path" in name:
        return "Room 315 rail-cell CAD/visual asset"
    return "Reusable Gazebo environment model"


def internal_import_counts(script_paths: list[Path]) -> Counter[str]:
    names = {path.stem for path in script_paths}
    edges: set[tuple[str, str]] = set()
    for path in script_paths:
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            dependencies: list[str] = []
            if isinstance(node, ast.Import):
                dependencies = [alias.name.split(".")[0] for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                dependencies = [node.module.split(".")[0]]
            for dependency in dependencies:
                if dependency in names and dependency != path.stem:
                    edges.add((path.stem, dependency))
    return Counter(dependency for _, dependency in edges)


def write_macros(
    *,
    all_tracked: list[Path],
    scripts: list[Path],
    installed_scripts: set[str],
    executable_scripts: set[str],
    control_tests: list[Path],
    description_tests: list[Path],
    models: list[Path],
    launch_files: list[Path],
    config_files: list[Path],
) -> None:
    commit = run_git("rev-parse", "HEAD")
    branch = run_git("branch", "--show-current") or "detached"
    commit_date = run_git("log", "-1", "--format=%ad", "--date=iso-strict")
    message_count = len(tracked("mfja_rail_interfaces/msg/*.msg"))
    service_count = len(tracked("mfja_rail_interfaces/srv/*.srv"))
    macros = f"""% Repository baseline maintained by the documentation build. Do not edit.
\\newcommand{{\\RepositoryCommit}}{{\\texttt{{{tex_escape(commit)}}}}}
\\newcommand{{\\RepositoryCommitShort}}{{\\texttt{{{tex_escape(commit[:12])}}}}}
\\newcommand{{\\RepositoryBranch}}{{\\texttt{{{tex_escape(branch)}}}}}
\\newcommand{{\\RepositoryCommitDate}}{{{tex_escape(commit_date)}}}
\\newcommand{{\\RepositoryTrackedFileCount}}{{{len(all_tracked)}}}
\\newcommand{{\\RepositoryScriptCount}}{{{len(scripts)}}}
\\newcommand{{\\RepositoryInstalledScriptCount}}{{{len(installed_scripts)}}}
\\newcommand{{\\RepositoryExecutableSourceScriptCount}}{{{len(executable_scripts)}}}
\\newcommand{{\\RepositorySourceOnlyScriptCount}}{{{len(scripts) - len(installed_scripts)}}}
\\newcommand{{\\RepositoryControlTestCount}}{{{len(control_tests)}}}
\\newcommand{{\\RepositoryDescriptionTestCount}}{{{len(description_tests)}}}
\\newcommand{{\\RepositoryModelCount}}{{{len(models)}}}
\\newcommand{{\\RepositoryLaunchCount}}{{{len(launch_files)}}}
\\newcommand{{\\RepositoryConfigCount}}{{{len(config_files)}}}
\\newcommand{{\\RepositoryMessageCount}}{{{message_count}}}
\\newcommand{{\\RepositoryServiceCount}}{{{service_count}}}
"""
    (GENERATED_DIR / "repository_baseline.tex").write_text(macros, encoding="utf-8")


def write_catalog(
    *,
    all_tracked: list[Path],
    scripts: list[Path],
    installed_scripts: set[str],
    executable_scripts: set[str],
    control_tests: list[Path],
    description_tests: list[Path],
    models: list[Path],
    launch_files: list[Path],
    config_files: list[Path],
) -> None:
    package_roots = {
        "mfja_rail_interfaces",
        "mfja_3rd_floor_description",
        "mfja_robot_control_config",
        "mfja_3rd_floor_bringup",
    }
    top_level_counts: Counter[str] = Counter()
    for path in all_tracked:
        parts = path.relative_to(REPO_ROOT).parts
        top_level_counts[parts[0] if len(parts) > 1 else "(repository root)"] += 1

    top_level_roles = {
        "mfja_rail_interfaces": "Typed rail messages and the AddShuttle lifecycle service.",
        "mfja_3rd_floor_description": "Worlds, Gazebo models, simulation plugins, and structural tests.",
        "mfja_robot_control_config": "Runtime control, rail, planning, perception, dataset, configuration, and tests.",
        "mfja_3rd_floor_bringup": "Stable user-facing launch entry points and shared composition.",
        "docs": "Maintained engineering notes, operational guidance, design records, and handover material.",
        "report": "Academic report, immutable evidence, and offline reproduction utilities.",
        "tutorial_videos": "Training material and recorded operator walkthroughs.",
        "(repository root)": "Repository metadata, overview material, auxiliary Nix shell, and standalone guides.",
    }

    auxiliary_code: list[tuple[str, str]] = []
    additional_package_code: list[tuple[str, str]] = []
    plugin_source_roles = {
        "SmoothJointTrajectoryController.cc": (
            "Gazebo system implementing validated, retimed, deterministic joint-trajectory motion."
        ),
        "SymmetricGripperController.cc": (
            "Gazebo system implementing bounded, symmetric, motion-only two-jaw gripper animation."
        ),
    }
    for path in all_tracked:
        relative = path.relative_to(REPO_ROOT).as_posix()
        if relative.startswith("mfja_robot_control_config/experiment_a_v3r1/"):
            additional_package_code.append((relative, module_description(path)))
        elif relative.startswith("mfja_3rd_floor_description/src/") and path.suffix == ".cc":
            additional_package_code.append(
                (relative, plugin_source_roles.get(path.name, "Gazebo system plugin source."))
            )
        if Path(relative).parts[0] in package_roots:
            continue
        if path.suffix in {".py", ".sh"} or relative == "flake.nix":
            auxiliary_code.append((relative, auxiliary_code_role(relative)))

    lines = [
        "% Repository source index maintained by the documentation build. Do not edit.",
        r"\section{Repository overview}",
        (
            r"Table~\ref{tab:catalogue-top-level} classifies the repository's top-level areas. "
            r"The four ROS 2 packages are the maintained runtime surface; the other areas remain "
            r"important as design context, evidence, reproduction support, or training material."
        ),
        r"\begin{longtable}{L{0.24\textwidth} r L{0.58\textwidth}}",
        r"\caption{Repository top-level areas and maintenance classification}\label{tab:catalogue-top-level}\\",
        r"\toprule",
        r"\tableheader Top-level area & Files & Maintenance classification \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Top-level area & Files & Maintenance classification \\",
        r"\midrule",
        r"\endhead",
    ]
    preferred_order = [
        "mfja_rail_interfaces",
        "mfja_3rd_floor_description",
        "mfja_robot_control_config",
        "mfja_3rd_floor_bringup",
        "docs",
        "report",
        "tutorial_videos",
        "(repository root)",
    ]
    ordered_areas = [area for area in preferred_order if area in top_level_counts]
    ordered_areas.extend(sorted(set(top_level_counts) - set(ordered_areas)))
    for area in ordered_areas:
        role = top_level_roles.get(
            area,
            "Supplemental repository content; inspect its local README before changing it.",
        )
        display_area = r"\texttt{(repository root)}" if area == "(repository root)" else rf"\path{{{area}}}"
        lines.append(
            rf"{display_area} & {top_level_counts[area]} & {tex_escape(role)} \\ \rowseparator"
        )
    lines.extend([r"\bottomrule", r"\end{longtable}"])

    lines.extend([
        r"\section{Non-package automation and frozen code}",
        (
            r"The following source-like files are tracked outside the ROS packages. They are not "
            r"installed as ROS executables. Treat evidence snapshots as immutable and keep offline "
            r"report tooling separated from the live control path."
        ),
        r"\scriptsize",
        r"\begin{longtable}{L{0.59\textwidth} L{0.34\textwidth}}",
        r"\caption{Source-like automation outside the ROS package runtime}\label{tab:catalogue-non-package-code}\\",
        r"\toprule",
        r"\tableheader Tracked path & Classification \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Tracked path & Classification \\",
        r"\midrule",
        r"\endhead",
    ])
    for relative, role in sorted(auxiliary_code):
        lines.append(rf"\path{{{relative}}} & {tex_escape(role)} \\ \rowseparator")
    lines.extend([r"\bottomrule", r"\end{longtable}", r"\normalsize"])

    lines.extend([
        r"\section{Additional package source modules}",
        (
            r"This closes the source-code inventory outside the main control-script, launch, and "
            r"test references: two compiled Gazebo plugins and eight isolated Experiment-A helpers. "
            r"The experiment helpers are not installed as ROS executables."
        ),
        r"\footnotesize",
        r"\begin{longtable}{L{0.49\textwidth} L{0.44\textwidth}}",
        r"\caption{Compiled plugins and isolated Experiment-A source modules}\label{tab:catalogue-additional-source}\\",
        r"\toprule",
        r"\tableheader Tracked source & Primary responsibility \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Tracked source & Primary responsibility \\",
        r"\midrule",
        r"\endhead",
    ])
    for relative, role in sorted(additional_package_code):
        lines.append(rf"\path{{{relative}}} & {tex_escape(role)} \\ \rowseparator")
    lines.extend([r"\bottomrule", r"\end{longtable}", r"\normalsize"])

    lines.extend([
        r"\section{Python executables and support modules}",
        (
            r"These tables cover every tracked Python file directly under "
            r"\file{mfja_robot_control_config/scripts}. ``CMake-listed'' means that "
            r"the file appears in the package's explicit \code{install(PROGRAMS)} list. "
            r"``Executable bit'' records the tracked source mode; it matters for symlink installs and "
            r"\code{ros2 pkg executables}. A support module may be CMake-listed without being a public CLI."
        ),
    ])
    current_category = ""
    for path in sorted(
        scripts,
        key=lambda item: (
            SCRIPT_CATEGORY_ORDER[script_category(item.name)],
            item.name,
        ),
    ):
        category = script_category(path.name)
        if category != current_category:
            if current_category:
                lines.extend([r"\bottomrule", r"\end{longtable}"])
            lines.extend([
                rf"\subsection{{{tex_escape(category)}}}",
                r"\footnotesize",
                r"\begin{longtable}{L{0.28\textwidth} L{0.10\textwidth} L{0.10\textwidth} L{0.43\textwidth}}",
                rf"\caption{{Python modules: {tex_escape(category)}}}"
                rf"\label{{tab:catalogue-python-{tex_label_slug(category)}}}\\",
                r"\toprule",
                r"\tableheader File & CMake-listed & Executable bit & Primary responsibility \\",
                r"\midrule",
                r"\endfirsthead",
                r"\toprule",
                r"\tableheader File & CMake-listed & Executable bit & Primary responsibility \\",
                r"\midrule",
                r"\endhead",
            ])
            current_category = category
        installed = "Yes" if path.name in installed_scripts else "Source only"
        executable = "Yes" if path.name in executable_scripts else "No"
        relative = path.relative_to(REPO_ROOT).as_posix()
        lines.append(
            rf"\path{{{relative}}} & {installed} & {executable} & {tex_escape(module_description(path))} \\ \rowseparator"
        )
    if current_category:
        lines.extend([r"\bottomrule", r"\end{longtable}", r"\normalsize"])

    counts = internal_import_counts(scripts)
    lines.extend([
        r"\section{Internal Python dependency hot spots}",
        (
            r"The following modules have the largest number of distinct in-repository importers. "
            r"Treat them as high-impact foundations: a contract change here normally requires a broad regression run."
        ),
        r"\begin{longtable}{L{0.55\textwidth} r L{0.27\textwidth}}",
        r"\caption{Internal Python dependency hot spots}\label{tab:catalogue-python-hotspots}\\",
        r"\toprule",
        r"\tableheader Module & Importers & Maintenance significance \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Module & Importers & Maintenance significance \\",
        r"\midrule",
        r"\endhead",
    ])
    for module, count in counts.most_common(18):
        lines.append(
            rf"\path{{mfja_robot_control_config/scripts/{module}.py}} & {count} & Shared contract/data/runtime foundation; run consumer tests. \\ \rowseparator"
        )
    lines.extend([r"\bottomrule", r"\end{longtable}"])

    lines.extend([
        r"\section{Launch files}",
        r"\begin{longtable}{L{0.46\textwidth} L{0.46\textwidth}}",
        r"\caption{Launch files and responsibilities}\label{tab:catalogue-launch-files}\\",
        r"\toprule",
        r"\tableheader Launch source & Role \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Launch source & Role \\",
        r"\midrule",
        r"\endhead",
    ])
    launch_roles = {
        "full_floor.launch.py": "Stable full-floor wrapper around the shared floor composition",
        "room_315_only.launch.py": "Stable Room 315-only wrapper around the shared floor composition",
        "room_315_floor_common.py": "Top-level profile defaults, singleton lock, argument forwarding, and delayed includes",
        "single_industrial_robot.launch.py": "Stable isolated industrial-robot wrapper",
        "multi_robot_sim.launch.py": "Gazebo startup, robot selection/spawn, per-instance bridges, and state publishers",
        "isolated_industrial_robot.launch.py": "Lower-level one-industrial-robot Gazebo composition",
        "room_315_dual_kinematic_shuttles.launch.py": "Right/left rail node instantiation and type-safe parameter forwarding",
        "room_315_vla_supervisor.launch.py": "Camera bridge, primitive supervisor, and optional recorder",
        "room_315_visual_state_runtime.launch.py": "V4 visual inference and optional RGB bridge",
        "room_315_task_execution.launch.py": "PlanSys2 lifecycle and fail-closed task gateway",
        "room_315_runtime_acceptance.launch.py": "Guarded runtime-acceptance campaign composition",
        "gripper_range_config.py": "Launch-time gripper config validation and temporary SDF/URDF materialization",
        "room_315_launch_utils.py": "Shared world-name parsing helper",
    }
    for path in launch_files:
        relative = path.relative_to(REPO_ROOT).as_posix()
        role = launch_roles.get(path.name, "Launch support module")
        lines.append(rf"\path{{{relative}}} & {tex_escape(role)} \\ \rowseparator")
    lines.extend([r"\bottomrule", r"\end{longtable}"])

    lines.extend([
        r"\section{Configuration and data contracts}",
        r"\scriptsize",
        r"\begin{longtable}{L{0.50\textwidth} L{0.43\textwidth}}",
        r"\caption{Configuration files and data contracts}\label{tab:catalogue-config-files}\\",
        r"\toprule",
        r"\tableheader Source file & Role \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Source file & Role \\",
        r"\midrule",
        r"\endhead",
    ])
    for path in config_files:
        relative = path.relative_to(REPO_ROOT).as_posix()
        lines.append(rf"\path{{{relative}}} & {tex_escape(config_role(path))} \\ \rowseparator")
    lines.extend([r"\bottomrule", r"\end{longtable}", r"\normalsize"])

    lines.extend([
        r"\section{Gazebo models}",
        r"\footnotesize",
        r"\begin{longtable}{L{0.31\textwidth} L{0.46\textwidth} r}",
        r"\caption{Gazebo model directories}\label{tab:catalogue-models}\\",
        r"\toprule",
        r"\tableheader Model directory & Role & Tracked files \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"\tableheader Model directory & Role & Tracked files \\",
        r"\midrule",
        r"\endhead",
    ])
    for model_dir in models:
        tracked_files = [
            path for path in tracked(f"mfja_3rd_floor_description/models/{model_dir.name}/**")
            if path.is_file()
        ]
        relative = model_dir.relative_to(REPO_ROOT).as_posix()
        lines.append(
            rf"\path{{{relative}}} & {tex_escape(model_role(model_dir.name))} & {len(tracked_files)} \\ \rowseparator"
        )
    lines.extend([r"\bottomrule", r"\end{longtable}", r"\normalsize"])

    lines.extend([
        r"\section{Automated tests}",
        (
            r"This reference lists source tests, not only CMake-registered ament tests. "
            r"Use direct pytest for the complete source surface and colcon test for the installed package surface."
        ),
    ])
    all_tests = [("mfja_3rd_floor_description", path) for path in description_tests]
    all_tests += [("mfja_robot_control_config", path) for path in control_tests]
    current_package = ""
    for package, path in sorted(all_tests, key=lambda item: (item[0], test_family(item[1].name), item[1].name)):
        if package != current_package:
            if current_package:
                lines.extend([r"\bottomrule", r"\end{longtable}"])
            lines.extend([
                rf"\subsection{{{tex_escape(package)}}}",
                r"\scriptsize",
                r"\begin{longtable}{L{0.44\textwidth} L{0.22\textwidth} L{0.27\textwidth}}",
                rf"\caption{{Test modules: {tex_escape(package)}}}"
                rf"\label{{tab:catalogue-tests-{tex_label_slug(package)}}}\\",
                r"\toprule",
                r"\tableheader Test file & Family & Intent \\",
                r"\midrule",
                r"\endfirsthead",
                r"\toprule",
                r"\tableheader Test file & Family & Intent \\",
                r"\midrule",
                r"\endhead",
            ])
            current_package = package
        relative = path.relative_to(REPO_ROOT).as_posix()
        lines.append(
            rf"\path{{{relative}}} & {tex_escape(test_family(path.name))} & {tex_escape(humanize_test(path.name))} \\ \rowseparator"
        )
    if current_package:
        lines.extend([r"\bottomrule", r"\end{longtable}", r"\normalsize"])

    lines = order_top_level_sections(
        lines,
        [
            "Repository overview",
            "Python executables and support modules",
            "Internal Python dependency hot spots",
            "Additional package source modules",
            "Launch files",
            "Configuration and data contracts",
            "Gazebo models",
            "Automated tests",
            "Non-package automation and frozen code",
        ],
    )
    for index, line in enumerate(lines):
        if line == r"\bottomrule" and index:
            lines[index - 1] = lines[index - 1].removesuffix(r" \rowseparator")
    (GENERATED_DIR / "repository_source_index.tex").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def main() -> int:
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    ensure_clean_package_snapshot()
    all_tracked = tracked()
    scripts = tracked("mfja_robot_control_config/scripts/*.py")
    control_tests = tracked("mfja_robot_control_config/test/test_*.py")
    description_tests = tracked("mfja_3rd_floor_description/test/test_*.py")
    launch_files = sorted(
        tracked("mfja_3rd_floor_bringup/launch/*.py")
        + tracked("mfja_robot_control_config/launch/*.py")
    )
    config_files = tracked("mfja_robot_control_config/config/**")
    model_configs = tracked("mfja_3rd_floor_description/models/*/model.config")
    models = sorted({path.parent for path in model_configs})

    cmake = (REPO_ROOT / "mfja_robot_control_config/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    installed_scripts = set(re.findall(r"scripts/([A-Za-z0-9_]+\.py)", cmake))
    script_modes = tracked_modes("mfja_robot_control_config/scripts/*.py")
    executable_scripts = {
        Path(relative).name
        for relative, mode in script_modes.items()
        if mode == "100755"
    }

    write_macros(
        all_tracked=all_tracked,
        scripts=scripts,
        installed_scripts=installed_scripts,
        executable_scripts=executable_scripts,
        control_tests=control_tests,
        description_tests=description_tests,
        models=models,
        launch_files=launch_files,
        config_files=config_files,
    )
    write_catalog(
        all_tracked=all_tracked,
        scripts=scripts,
        installed_scripts=installed_scripts,
        executable_scripts=executable_scripts,
        control_tests=control_tests,
        description_tests=description_tests,
        models=models,
        launch_files=launch_files,
        config_files=config_files,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
