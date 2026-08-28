#!/usr/bin/env python3
"""Fail when the handover manual drifts from its text-and-table contract."""

from __future__ import annotations

import re
import sys
from collections import Counter
from pathlib import Path


DOC_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = DOC_ROOT.parents[1]
LONGTABLE_RE = re.compile(
    r"\\begin\{longtable\}.*?\\end\{longtable\}", re.DOTALL
)
ROW_END_RE = re.compile(r"(?<!\\)\\\\(?!\\)")
FORBIDDEN = {
    r"\begin{figure": "figure environment",
    r"\includegraphics": "embedded image",
    r"\begin{tikzpicture": "TikZ diagram",
    r"\listoffigures": "List of Figures",
    r"\graphicspath": "image search path",
    r"\usepackage{graphicx}": "explicit graphics package",
    r"\usepackage{tikz}": "explicit TikZ package",
    r"\usetikzlibrary": "TikZ library declaration",
}
REPOSITORY_PATH_PREFIXES = (
    "mfja_",
    "docs/",
    "report/",
    "tutorial_videos/",
    ".gitignore",
    "README.md",
    "flake.nix",
)
EXPECTED_INPUT_ORDER = (
    "chapters/01_project_overview",
    "chapters/02_repository_and_package_ownership",
    "chapters/03_system_architecture_and_data_flow",
    "chapters/04_installation_and_build",
    "chapters/05_configuration_and_deployment",
    "chapters/06_launch_and_runtime",
    "chapters/07_simulation_and_robot_control",
    "chapters/08_rail_transport",
    "chapters/09_visual_state_and_planning",
    "chapters/10_operator_runbook",
    "chapters/11_verification_and_evidence",
    "chapters/12_maintenance_and_troubleshooting",
    "chapters/13_safety_risks_and_handover",
    "appendices/a_glossary_and_naming",
    "appendices/b_ros_interface_reference",
    "appendices/c_launch_argument_reference",
    "appendices/d_repository_source_index",
)
READER_FACING_INTERNAL_REFERENCES = (
    "docs/project_documentation/generated",
    "docs/project_documentation/scripts/generate_catalog.py",
    "generated catalogue",
    "generated catalog",
)


def source_files() -> list[Path]:
    files = [DOC_ROOT / "main.tex", DOC_ROOT / "preamble.tex"]
    files.extend(sorted((DOC_ROOT / "chapters").glob("*.tex")))
    files.extend(sorted((DOC_ROOT / "appendices").glob("*.tex")))
    files.append(DOC_ROOT / "generated" / "repository_source_index.tex")
    return files


def line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def main() -> int:
    errors: list[str] = []
    labels: list[tuple[str, Path, int]] = []

    main_text = (DOC_ROOT / "main.tex").read_text(encoding="utf-8")
    if r"\listoftables" not in main_text:
        errors.append("main.tex: missing \\listoftables")

    actual_inputs = tuple(
        value
        for value in re.findall(r"\\input\{([^}]+)\}", main_text)
        if value.startswith(("chapters/", "appendices/"))
        and value != "chapters/00_frontmatter"
    )
    if actual_inputs != EXPECTED_INPUT_ORDER:
        errors.append(
            "main.tex: chapter/appendix inputs do not follow the maintained "
            "logical and filename order"
        )
    if len(re.findall(r"\\part\{", main_text)) != 5:
        errors.append("main.tex: expected four main parts and one appendix part")

    numbered_chapters = sorted((DOC_ROOT / "chapters").glob("[0-9][0-9]_*.tex"))
    numbered_chapters = [
        path for path in numbered_chapters if path.name != "00_frontmatter.tex"
    ]
    expected_prefixes = [f"{number:02d}_" for number in range(1, 14)]
    actual_prefixes = [path.name[:3] for path in numbered_chapters]
    if actual_prefixes != expected_prefixes:
        errors.append(
            "chapters/: expected exactly one sequentially named chapter from "
            "01 through 13"
        )
    for path in numbered_chapters:
        text = path.read_text(encoding="utf-8")
        if len(re.findall(r"^\\chapter\{", text, re.MULTILINE)) != 1:
            errors.append(f"{path.relative_to(DOC_ROOT)}: expected one chapter title")
        if len(re.findall(r"^\\chapteroverview", text, re.MULTILINE)) != 1:
            errors.append(
                f"{path.relative_to(DOC_ROOT)}: expected one chapter guide"
            )

    for path in source_files():
        if not path.is_file():
            errors.append(f"{path.relative_to(DOC_ROOT)}: required source is missing")
            continue
        text = path.read_text(encoding="utf-8")
        relative = path.relative_to(DOC_ROOT)

        if relative.parts[0] in {"chapters", "appendices"} or relative == Path(
            "generated/repository_source_index.tex"
        ):
            lowered = text.lower()
            for token in READER_FACING_INTERNAL_REFERENCES:
                if token.lower() in lowered:
                    errors.append(
                        f"{relative}: reader-facing build-internal reference: {token}"
                    )

        arabic = re.search(r"[\u0600-\u06ff\u0750-\u077f\u08a0-\u08ff]", text)
        if arabic:
            errors.append(
                f"{relative}:{line_number(text, arabic.start())}: non-English "
                "Arabic-script text"
            )

        for token, description in FORBIDDEN.items():
            for match in re.finditer(re.escape(token), text):
                errors.append(
                    f"{relative}:{line_number(text, match.start())}: "
                    f"forbidden {description} markup"
                )

        ordinary_tables = len(
            re.findall(r"\\begin\{(?:tabularx|tabular\*?|table)\}", text)
        )
        if relative == Path("chapters/00_frontmatter.tex"):
            if ordinary_tables != 1:
                errors.append(
                    f"{relative}: expected only the single unnumbered title-page "
                    f"metadata table, found {ordinary_tables}"
                )
        elif ordinary_tables:
            errors.append(
                f"{relative}: found {ordinary_tables} unnumbered/non-longtable table(s); "
                "use a captioned longtable"
            )

        for match in LONGTABLE_RE.finditer(text):
            block = match.group()
            table_line = line_number(text, match.start())
            captions = re.findall(r"\\caption\{", block)
            table_labels = re.findall(r"\\label\{(tab:[^}]+)\}", block)
            if len(captions) != 1:
                errors.append(
                    f"{relative}:{table_line}: longtable has {len(captions)} captions; "
                    "expected exactly one"
                )
            if block.count(r"\endfirsthead") != 1 or block.count(r"\endhead") != 1:
                errors.append(
                    f"{relative}:{table_line}: longtable must define one first-page "
                    "header and one repeated continuation header"
                )
            if re.search(r"\\\\ \\rowseparator\s*\\bottomrule", block):
                errors.append(
                    f"{relative}:{table_line}: final table row has a redundant "
                    "separator before the bottom rule"
                )
            if len(table_labels) != 1:
                errors.append(
                    f"{relative}:{table_line}: longtable has {len(table_labels)} "
                    "table labels; expected exactly one"
                )
            elif table_labels:
                label_offset = match.start() + block.find(r"\label")
                labels.append((table_labels[0], relative, line_number(text, label_offset)))

            pieces = ROW_END_RE.split(block)
            data_rows = [
                piece
                for piece in pieces
                if "&" in piece and r"\tableheader" not in piece
            ]
            required_separators = max(0, len(data_rows) - 1)
            actual_separators = block.count(r"\rowseparator")
            if actual_separators < required_separators:
                errors.append(
                    f"{relative}:{table_line}: longtable has {len(data_rows)} data "
                    f"rows but only {actual_separators} row separators; expected at "
                    f"least {required_separators}"
                )

        for reference in re.finditer(r"\\(?:file|path)\{([^{}]+)\}", text):
            value = reference.group(1).strip()
            if not value.startswith(REPOSITORY_PATH_PREFIXES):
                continue
            if any(token in value for token in ("*", "<", ">", "...", "\\")):
                continue
            candidate = REPO_ROOT / value.rstrip("/")
            if not candidate.exists():
                errors.append(
                    f"{relative}:{line_number(text, reference.start())}: "
                    f"repository path does not exist: {value}"
                )

    counts = Counter(label for label, _, _ in labels)
    for label, count in sorted(counts.items()):
        if count > 1:
            locations = ", ".join(
                f"{path}:{line}" for item, path, line in labels if item == label
            )
            errors.append(f"duplicate table label {label}: {locations}")

    if errors:
        print("Manual structure check failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(
        f"Manual structure check passed: {len(labels)} captioned, labeled "
        "text-only tables with row separators."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
