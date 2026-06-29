#!/usr/bin/env python3
"""Deterministic language generation for Room 315 PDDL scenarios.

The generated text is task language intended for model_input.language. PDDL
goals, planner traces, and symbolic plans are returned as metadata outside
model_input so they remain expert/data-generation context, not learned-model
input features.
"""

import argparse
import json
import random
import re
import shlex
from copy import deepcopy
from dataclasses import dataclass
from dataclasses import replace
from typing import Any


SIDES = {'right', 'left'}
STATION_LABELS = {
    'yaskawa': 'Yaskawa',
    'staubli': 'Staubli',
    'kuka': 'KUKA',
}
DEFAULT_SOURCE_BY_SIDE_AND_TARGET = {
    ('right', 'staubli'): 'yaskawa',
    ('right', 'yaskawa'): 'staubli',
    ('left', 'kuka'): 'yaskawa',
    ('left', 'yaskawa'): 'kuka',
}
ROUTE_TEMPLATES = {
    'move_from_to': 'move the {side} shuttle from {from_station} to {to_station}',
    'send_to_station': 'send the {side} shuttle to the {to_station} station',
    'route_between_stations': (
        'route the {side} shuttle from {from_station} station to {to_station} station'
    ),
    'bring_to_station': 'bring the {side} shuttle to {to_station}',
}
IDENTITY_ROUTE_TEMPLATES = {
    'explicit_id_to_station': 'move {shuttle_label} to the {to_station} station',
    'send_id_to_station': 'send {shuttle_label} to {to_station}',
    'labeled_id_to_station': 'move the shuttle labeled {shuttle_label} to {to_station}',
    'rail_id_to_station': 'route {shuttle_label} on the {side} rail to {to_station}',
    'loaded_id_to_station': 'move {shuttle_label} to {to_station} even though it is carrying a part',
}
RELATIONAL_ROUTE_TEMPLATES = {
    'front_shuttle_to_station': 'move the front {side} shuttle to {to_station}',
    'rear_shuttle_to_station': 'move the rear shuttle on the {side} rail to {to_station}',
}
PAYLOAD_ROUTE_TEMPLATES = {
    'loaded_shuttle_to_station': 'move the loaded {side} shuttle to {to_station}',
    'empty_shuttle_to_station': 'move the empty {side} shuttle to {to_station}',
    'carrying_part_id_to_station': 'move {shuttle_label} carrying a part to {to_station}',
}
SLOT_SEQUENCE_TEMPLATES = {
    'send_slot_sequence': 'send the {side} shuttle to slot {first_slot} and then to slot {last_slot}',
    'move_slot_sequence': 'move the {side} shuttle from slot {first_slot} to slot {last_slot}',
}
SLOT_TARGET_TEMPLATES = {
    'send_to_slot': 'send the {side} shuttle to slot {slot}',
    'move_to_slot': 'move the {side} shuttle to slot {slot}',
}
PAYLOAD_SLOT_TEMPLATES = {
    'loaded_shuttle_to_slot': 'move the loaded {side} shuttle to slot {slot}',
    'empty_shuttle_to_slot': 'move the empty {side} shuttle to slot {slot}',
    'carrying_part_id_to_slot': 'move {shuttle_label} carrying a part to slot {slot}',
}


@dataclass(frozen=True)
class LanguageGoal:
    side: str
    shuttle: str
    target_station: str = ''
    source_station: str = ''
    slot_sequence: tuple[str, ...] = ()
    payload_condition: str = ''
    raw_goal: str = ''


@dataclass(frozen=True)
class GeneratedLanguage:
    language: str
    template_id: str
    metadata: dict[str, Any]


def generate_language(
    *,
    pddl_goal: str = '',
    pddl_problem: str = '',
    symbolic_plan: list[str] | tuple[str, ...] | None = None,
    action_sequence: str = '',
    template_id: str = '',
    seed: int | None = None,
    include_raw_pddl: bool = False,
) -> GeneratedLanguage:
    """Generate one deterministic task-language command."""

    plan = [str(step) for step in (symbolic_plan or [])]
    goal = _language_goal_from_inputs(
        pddl_goal=pddl_goal,
        symbolic_plan=plan,
        action_sequence=action_sequence,
    )
    chosen_template_id = template_id or _choose_template_id(goal, seed=seed)
    language = _render_template(goal, chosen_template_id)
    if include_raw_pddl and pddl_goal:
        language = f'{language} [{pddl_goal}]'
    metadata = {
        'pddl_goal': pddl_goal,
        'pddl_problem': pddl_problem,
        'generated_language_template_id': chosen_template_id,
        'symbolic_plan': plan,
    }
    if goal.payload_condition:
        metadata['payload_condition'] = goal.payload_condition
    if goal.slot_sequence:
        metadata['target_slot'] = goal.slot_sequence[-1]
    return GeneratedLanguage(
        language=language,
        template_id=chosen_template_id,
        metadata=metadata,
    )


def generate_language_variants(
    *,
    pddl_goal: str = '',
    pddl_problem: str = '',
    symbolic_plan: list[str] | tuple[str, ...] | None = None,
    action_sequence: str = '',
) -> list[GeneratedLanguage]:
    """Return every deterministic paraphrase available for the inferred goal."""

    plan = [str(step) for step in (symbolic_plan or [])]
    goal = _language_goal_from_inputs(
        pddl_goal=pddl_goal,
        symbolic_plan=plan,
        action_sequence=action_sequence,
    )
    template_ids = _template_ids_for_goal(goal)
    return [
        generate_language(
            pddl_goal=pddl_goal,
            pddl_problem=pddl_problem,
            symbolic_plan=plan,
            action_sequence=action_sequence,
            template_id=template_id,
        )
        for template_id in template_ids
    ]


def attach_language_to_scenario_metadata(
    scenario: dict[str, Any],
    generated: GeneratedLanguage,
) -> dict[str, Any]:
    """Attach generated language and planner metadata to a scenario record.

    Planner metadata is copied to top-level fields. Only the generated natural
    language string is placed in model_input.language.
    """

    updated = deepcopy(scenario)
    model_input = dict(updated.get('model_input') or {})
    model_input['language'] = generated.language
    updated['model_input'] = model_input
    for key, value in generated.metadata.items():
        updated[key] = deepcopy(value)
    return updated


def _language_goal_from_inputs(
    *,
    pddl_goal: str,
    symbolic_plan: list[str],
    action_sequence: str,
) -> LanguageGoal:
    parsed_goal = _parse_pddl_goal(pddl_goal)
    payload_hint = parsed_goal.payload_condition if parsed_goal is not None else ''
    for parser_input in (action_sequence, *symbolic_plan):
        if not str(parser_input or '').strip():
            continue
        parsed = _parse_action_sequence(parser_input)
        if parsed is not None:
            if payload_hint and not parsed.payload_condition:
                return replace(parsed, payload_condition=payload_hint)
            return parsed
    if parsed_goal is not None:
        return parsed_goal
    raise ValueError('could not infer a Room 315 language goal')


def _parse_action_sequence(text: str) -> LanguageGoal | None:
    normalized = _normalize_text(text)
    payload_condition = _payload_condition_from_text(normalized)
    identity_to_slot = re.search(
        r'\b(?:move|send|route|bring)\s+(?:the\s+shuttle\s+labeled\s+)?'
        r'(?P<shuttle>[rl][1-4]|(?:right|left)_shuttle_[1-4])\b.*?'
        r'\b(?:to|back\s+to)\s+slot\s*(?P<slot>[1-4])\b',
        normalized,
    )
    if identity_to_slot:
        shuttle = _clean_symbol(identity_to_slot.group('shuttle'))
        side = _infer_side(shuttle)
        return LanguageGoal(
            side=side,
            shuttle=shuttle,
            slot_sequence=(identity_to_slot.group('slot'),),
            payload_condition=payload_condition,
            raw_goal=text,
        )

    identity_to_station = re.search(
        r'\b(?:move|send|route|bring)\s+(?:the\s+shuttle\s+labeled\s+)?'
        r'(?P<shuttle>[rl][1-4]|(?:right|left)_shuttle_[1-4])\b.*?\b(?:to|back\s+to)\s+'
        r'(?P<target>[a-z0-9_]+)\b',
        normalized,
    )
    if identity_to_station:
        shuttle = _clean_symbol(identity_to_station.group('shuttle'))
        side = _infer_side(shuttle)
        target = identity_to_station.group('target')
        return _route_goal(
            side=side,
            shuttle=shuttle,
            source=_infer_source(side, target),
            target=target,
            payload_condition=payload_condition,
            raw_goal=text,
        )

    slot_match = re.search(
        r'\b(?P<side>right|left)\s+shuttle\b.*?\bslot\s*(?P<first>[1-4])\b.*?'
        r'\b(?:then\s+to|to)\s+slot\s*(?P<last>[1-4])\b',
        normalized,
    )
    if slot_match:
        side = slot_match.group('side')
        return LanguageGoal(
            side=side,
            shuttle=f'{side}_shuttle',
            slot_sequence=(slot_match.group('first'), slot_match.group('last')),
            raw_goal=text,
        )

    single_slot_match = re.search(
        r'\b(?:move|send|route|bring)\s+(?:the\s+)?'
        r'(?:(?:loaded|empty|unloaded)\s+)?(?:(?P<side>right|left)\s+)?'
        r'shuttle\b.*?\b(?:to|back\s+to)\s+slot\s*(?P<slot>[1-4])\b',
        normalized,
    )
    if single_slot_match:
        side = single_slot_match.group('side') or 'right'
        return LanguageGoal(
            side=side,
            shuttle=f'{side}_shuttle',
            slot_sequence=(single_slot_match.group('slot'),),
            payload_condition=payload_condition,
            raw_goal=text,
        )

    route_match = re.search(
        r'\b(?P<side>right|left)\s+shuttle\b.*?\bfrom\s+'
        r'(?P<source>[a-z0-9_]+)\s+to\s+(?P<target>[a-z0-9_]+)\b',
        normalized,
    )
    if route_match:
        side = route_match.group('side')
        return _route_goal(
            side=side,
            shuttle=f'{side}_shuttle',
            source=route_match.group('source'),
            target=route_match.group('target'),
            payload_condition=payload_condition,
            raw_goal=text,
        )

    to_station_match = re.search(
        r'\b(?:move|send|route|bring)\s+(?:the\s+)?'
        r'(?:(?:loaded|empty|unloaded)\s+)?(?:(?P<side>right|left)\s+)?shuttle\b.*?'
        r'\b(?:to|back\s+to)\s+(?P<target>[a-z0-9_]+)\b',
        normalized,
    )
    if to_station_match and payload_condition:
        target = to_station_match.group('target')
        side = to_station_match.group('side') or _infer_side_from_target(target)
        return _route_goal(
            side=side,
            shuttle=f'{side}_shuttle',
            source=_infer_source(side, target),
            target=target,
            payload_condition=payload_condition,
            raw_goal=text,
        )

    tokens = _plan_tokens(text)
    if not tokens:
        return None
    action = tokens[0]
    if action == 'move_shuttle':
        side, shuttle, source, target = _move_step_parts(tokens[1:])
        return _route_goal(
            side=side,
            shuttle=shuttle,
            source=source,
            target=target,
            payload_condition=payload_condition,
            raw_goal=text,
        )
    if action in {'go_to_slot', 'move_to_slot', 'visit_slot'}:
        side, shuttle, slot = _slot_step_parts(tokens[1:])
        return LanguageGoal(
            side=side,
            shuttle=shuttle,
            slot_sequence=(slot,),
            raw_goal=text,
        )
    return None


def _parse_pddl_goal(text: str) -> LanguageGoal | None:
    if not str(text or '').strip():
        return None
    normalized = _normalize_text(text)

    task_done = re.search(r'\btask_done\s+(?P<shuttle>\S+)\s+(?P<station>\S+)', normalized)
    if task_done:
        shuttle = _clean_symbol(task_done.group('shuttle'))
        target = _station_symbol(task_done.group('station'))
        side = _infer_side(shuttle, task_done.group('station'))
        source = _infer_source(side, target)
        return _route_goal(
            side=side,
            shuttle=shuttle,
            source=source,
            target=target,
            payload_condition=_payload_condition_for_shuttle(normalized, shuttle),
            raw_goal=text,
        )

    at_goal = re.search(
        r'\b(?P<shuttle>(?:right|left)_shuttle(?:_[1-4])?|[rl][1-4])\s+at\s+'
        r'(?P<station>\S+)',
        normalized,
    )
    if at_goal:
        shuttle = _clean_symbol(at_goal.group('shuttle'))
        target = _station_symbol(at_goal.group('station'))
        side = _infer_side(shuttle, at_goal.group('station'))
        source = _infer_source(side, target)
        return _route_goal(
            side=side,
            shuttle=shuttle,
            source=source,
            target=target,
            payload_condition=_payload_condition_for_shuttle(normalized, shuttle),
            raw_goal=text,
        )
    return None


def _route_goal(
    *,
    side: str,
    shuttle: str,
    source: str,
    target: str,
    raw_goal: str,
    payload_condition: str = '',
) -> LanguageGoal:
    normalized_side = _normalize_side(side)
    target_station = _station_symbol(target)
    source_station = _station_symbol(source) or _infer_source(normalized_side, target_station)
    if not target_station:
        raise ValueError(f'could not infer target station from {target!r}')
    return LanguageGoal(
        side=normalized_side,
        shuttle=_clean_symbol(shuttle) or f'{normalized_side}_shuttle',
        source_station=source_station,
        target_station=target_station,
        payload_condition=_normalize_payload_condition(payload_condition),
        raw_goal=raw_goal,
    )


def _choose_template_id(goal: LanguageGoal, *, seed: int | None) -> str:
    template_ids = _template_ids_for_goal(goal)
    rng = random.Random(0 if seed is None else seed)
    return rng.choice(template_ids)


def _template_ids_for_goal(goal: LanguageGoal) -> list[str]:
    if len(goal.slot_sequence) >= 2:
        return list(SLOT_SEQUENCE_TEMPLATES)
    if len(goal.slot_sequence) == 1:
        template_ids = list(SLOT_TARGET_TEMPLATES)
        if goal.payload_condition == 'loaded':
            template_ids.append('loaded_shuttle_to_slot')
            if _shuttle_label(goal.shuttle):
                template_ids.append('carrying_part_id_to_slot')
        elif goal.payload_condition == 'empty':
            template_ids.append('empty_shuttle_to_slot')
        return template_ids
    template_ids = list(ROUTE_TEMPLATES)
    if _shuttle_label(goal.shuttle):
        template_ids.extend(IDENTITY_ROUTE_TEMPLATES)
    template_ids.extend(RELATIONAL_ROUTE_TEMPLATES)
    if goal.payload_condition == 'loaded':
        template_ids.append('loaded_shuttle_to_station')
        if _shuttle_label(goal.shuttle):
            template_ids.append('carrying_part_id_to_station')
    elif goal.payload_condition == 'empty':
        template_ids.append('empty_shuttle_to_station')
    return template_ids


def _render_template(goal: LanguageGoal, template_id: str) -> str:
    if template_id in SLOT_SEQUENCE_TEMPLATES:
        if len(goal.slot_sequence) < 2:
            raise ValueError(f'template {template_id!r} needs a slot sequence')
        template = SLOT_SEQUENCE_TEMPLATES[template_id]
        return template.format(
            side=goal.side,
            first_slot=goal.slot_sequence[0],
            last_slot=goal.slot_sequence[-1],
        )
    slot_templates = {
        **SLOT_TARGET_TEMPLATES,
        **PAYLOAD_SLOT_TEMPLATES,
    }
    if template_id in slot_templates:
        if len(goal.slot_sequence) < 1:
            raise ValueError(f'template {template_id!r} needs a target slot')
        shuttle_label = _shuttle_label(goal.shuttle)
        if template_id == 'carrying_part_id_to_slot' and not shuttle_label:
            raise ValueError(f'template {template_id!r} needs a specific shuttle identity')
        return slot_templates[template_id].format(
            side=goal.side,
            shuttle_label=shuttle_label or f'{goal.side} shuttle',
            slot=goal.slot_sequence[-1],
        )
    station_language_templates = {
        **ROUTE_TEMPLATES,
        **IDENTITY_ROUTE_TEMPLATES,
        **RELATIONAL_ROUTE_TEMPLATES,
        **PAYLOAD_ROUTE_TEMPLATES,
    }
    if template_id not in station_language_templates:
        allowed = ', '.join([
            *station_language_templates,
            *SLOT_SEQUENCE_TEMPLATES,
            *slot_templates,
        ])
        raise ValueError(f'unknown language template {template_id!r}; allowed: {allowed}')
    shuttle_label = _shuttle_label(goal.shuttle)
    if template_id in IDENTITY_ROUTE_TEMPLATES and not shuttle_label:
        raise ValueError(f'template {template_id!r} needs a specific shuttle identity')
    if template_id == 'carrying_part_id_to_station' and not shuttle_label:
        raise ValueError(f'template {template_id!r} needs a specific shuttle identity')
    template = station_language_templates[template_id]
    return template.format(
        side=goal.side,
        shuttle_label=shuttle_label or f'{goal.side} shuttle',
        from_station=_station_label(goal.source_station),
        to_station=_station_label(goal.target_station),
    )


def _move_step_parts(args: list[str]) -> tuple[str, str, str, str]:
    if len(args) < 4:
        raise ValueError('move_shuttle needs side/shuttle/from/to arguments')
    if _is_side(args[0]):
        side = _normalize_side(args[0])
        shuttle = args[1]
        source = args[2]
        target = args[3]
    else:
        shuttle = args[0]
        side = _normalize_side(args[1])
        source = args[2]
        target = args[3]
    return side, shuttle, source, target


def _slot_step_parts(args: list[str]) -> tuple[str, str, str]:
    if len(args) < 3:
        raise ValueError('slot step needs side/shuttle/slot arguments')
    if _is_side(args[0]):
        return _normalize_side(args[0]), args[1], _slot_symbol(args[2])
    return _normalize_side(args[1]), args[0], _slot_symbol(args[2])


def _plan_tokens(text: str) -> list[str]:
    cleaned = str(text or '').split(';', 1)[0].strip()
    cleaned = re.sub(r'^\s*\d+(?:\.\d+)?\s*:\s*', '', cleaned)
    cleaned = re.sub(r'\s*\[[^\]]*\]\s*$', '', cleaned).strip()
    if cleaned.startswith('(') and cleaned.endswith(')'):
        cleaned = cleaned[1:-1].strip()
    if not cleaned:
        return []
    return [_clean_symbol(token).lower() for token in shlex.split(cleaned) if '=' not in token]


def _normalize_text(text: str) -> str:
    cleaned = str(text or '').casefold().replace('-', '_')
    cleaned = cleaned.replace('(', ' ').replace(')', ' ')
    cleaned = cleaned.replace(':goal', ' ').replace('and', ' ')
    return ' '.join(cleaned.split())


def _station_symbol(value: str) -> str:
    symbol = _clean_symbol(value).lower()
    if symbol.startswith('right_') or symbol.startswith('left_'):
        symbol = symbol.split('_', 1)[1]
    return symbol if symbol in STATION_LABELS else ''


def _station_label(value: str) -> str:
    station = _station_symbol(value)
    if not station:
        return str(value).strip()
    return STATION_LABELS[station]


def _slot_symbol(value: str) -> str:
    match = re.search(r'[1-4]', str(value or ''))
    if not match:
        raise ValueError(f'invalid slot value {value!r}')
    return match.group(0)


def _infer_source(side: str, target: str) -> str:
    return DEFAULT_SOURCE_BY_SIDE_AND_TARGET.get((_normalize_side(side), _station_symbol(target)), '')


def _infer_side_from_target(target: str) -> str:
    station = _station_symbol(target)
    if station == 'kuka':
        return 'left'
    if station == 'staubli':
        return 'right'
    return 'right'


def _normalize_payload_condition(raw: str) -> str:
    text = str(raw or '').strip().casefold().replace('-', '_')
    if text in {'loaded', 'load', 'with_payload', 'carrying', 'part', 'box'}:
        return 'loaded'
    if text in {'empty', 'unloaded', 'without_payload', 'no_payload', 'none'}:
        return 'empty'
    return ''


def _payload_condition_from_text(text: str) -> str:
    normalized = str(text or '').casefold().replace('-', '_')
    if any(token in normalized for token in ('loaded', 'carrying', 'with a part', 'with payload')):
        return 'loaded'
    if any(token in normalized for token in ('empty', 'unloaded', 'without payload', 'no payload')):
        return 'empty'
    return ''


def _payload_condition_for_shuttle(normalized_text: str, shuttle: str) -> str:
    aliases = _payload_shuttle_aliases(shuttle)
    for alias in aliases:
        if re.search(rf'\bloaded\s+{re.escape(alias)}\b', normalized_text):
            return 'loaded'
        if re.search(rf'\bempty\s+{re.escape(alias)}\b', normalized_text):
            return 'empty'
    return _payload_condition_from_text(normalized_text)


def _payload_shuttle_aliases(shuttle: str) -> set[str]:
    text = _clean_symbol(shuttle).lower()
    aliases = {text} if text else set()
    label = _shuttle_label(text)
    if label:
        aliases.add(label.casefold())
        side = 'right' if label.startswith('R') else 'left'
        aliases.add(f'{side}_shuttle_{label[1:]}')
    return aliases


def _infer_side(*values: str) -> str:
    for value in values:
        text = str(value or '').casefold()
        if re.fullmatch(r'r[1-4]', text):
            return 'right'
        if re.fullmatch(r'l[1-4]', text):
            return 'left'
        if 'right' in text:
            return 'right'
        if 'left' in text:
            return 'left'
    raise ValueError(f'could not infer rail side from {values!r}')


def _normalize_side(value: str) -> str:
    text = str(value or '').strip().casefold()
    if text in {'right', 'r'}:
        return 'right'
    if text in {'left', 'l'}:
        return 'left'
    raise ValueError(f'invalid side {value!r}; expected right or left')


def _is_side(value: str) -> bool:
    return str(value or '').strip().casefold() in SIDES


def _clean_symbol(value: str) -> str:
    return str(value or '').strip().strip('()[]{}:,').replace('-', '_')


def _shuttle_label(value: str) -> str:
    text = _clean_symbol(value).upper()
    match = re.fullmatch(r'([RL])([1-4])', text)
    if match:
        return f'{match.group(1)}{match.group(2)}'
    match = re.fullmatch(r'(RIGHT|LEFT)_SHUTTLE_([1-4])', text)
    if match:
        return f'{"R" if match.group(1) == "RIGHT" else "L"}{match.group(2)}'
    return ''


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate deterministic Room 315 task language from PDDL goals or plans.'
    )
    parser.add_argument('--pddl-goal', default='', help='PDDL goal or simple goal text.')
    parser.add_argument('--problem', default='', help='Optional PDDL problem identifier.')
    parser.add_argument('--plan-step', action='append', default=[], help='Symbolic plan step.')
    parser.add_argument('--action-sequence', default='', help='Plain text action sequence.')
    parser.add_argument('--template-id', default='', help='Explicit language template id.')
    parser.add_argument('--seed', type=int, default=None, help='Deterministic random seed.')
    parser.add_argument(
        '--include-raw-pddl',
        action='store_true',
        help='Append the raw PDDL goal to the generated language for debugging.',
    )
    args = parser.parse_args()
    generated = generate_language(
        pddl_goal=args.pddl_goal,
        pddl_problem=args.problem,
        symbolic_plan=args.plan_step,
        action_sequence=args.action_sequence,
        template_id=args.template_id,
        seed=args.seed,
        include_raw_pddl=args.include_raw_pddl,
    )
    print(_json_dumps({
        'language': generated.language,
        **generated.metadata,
    }))


if __name__ == '__main__':
    main()
