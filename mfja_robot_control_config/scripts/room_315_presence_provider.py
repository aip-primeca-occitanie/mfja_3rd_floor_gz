#!/usr/bin/env python3
"""Deterministic Room 315 shuttle-presence providers.

The core is ROS-independent.  A transport adapter supplies only an entity
name, the message header timestamp, the receive timestamp, and the topic side.
No kinematic ShuttleState field is accepted by this API.
"""

from __future__ import annotations

import math
import sys
from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES


PRESENCE_PRESENT = 'present'
PRESENCE_ABSENT = 'absent'
PRESENCE_UNKNOWN = 'unknown'
PRESENCE_STATES = frozenset({
    PRESENCE_PRESENT,
    PRESENCE_ABSENT,
    PRESENCE_UNKNOWN,
})
PRESENCE_SIDES = ('left', 'right')


class PresenceError(ValueError):
    """Raised when a deterministic presence source violates its contract."""


@dataclass(frozen=True)
class PresenceEntry:
    identity: str
    side: str
    state: str
    entity_name: str = ''
    source_stamp_s: float | None = None
    receive_time_s: float | None = None
    age_s: float | None = None
    reason: str = ''

    def __post_init__(self) -> None:
        if self.state not in PRESENCE_STATES:
            raise PresenceError(f'unsupported presence state: {self.state!r}')


@dataclass(frozen=True)
class PresenceSnapshot:
    timestamp_s: float
    ready: bool
    entries: tuple[PresenceEntry, ...]
    reasons: tuple[str, ...]
    initialized_sides: tuple[str, ...]
    stale_sides: tuple[str, ...]
    source: str

    def by_identity(self) -> dict[str, PresenceEntry]:
        return {entry.identity: entry for entry in self.entries}


class PresenceProvider(ABC):
    """Replaceable deterministic source for the eight fixed identity slots."""

    @abstractmethod
    def snapshot(self, *, now_s: float) -> PresenceSnapshot:
        """Return present/absent/unknown for every authoritative identity."""

    @abstractmethod
    def reset(self) -> None:
        """Reset startup and freshness state."""


@dataclass
class _PresenceRecord:
    identity: str
    side: str
    entity_name: str
    source_stamp_s: float
    receive_time_s: float


@dataclass
class _SourceState:
    first_receive_time_s: float | None = None
    last_receive_time_s: float | None = None
    last_source_stamp_s: float | None = None


@dataclass
class _Fault:
    reason: str
    receive_time_s: float


class ShuttleStatePresenceProvider(PresenceProvider):
    """Fresh-name registry backed by the two controller ShuttleState streams.

    Repeated messages with the same canonical entity name are expected.
    The kinematic controller publishes a blank-name message as the explicit
    heartbeat for a rail whose active shuttle inventory is empty.  That
    heartbeat initializes and refreshes the side without inventing an entity;
    every identity on the side is therefore absent once the complete registry
    is ready.  Unknown *non-empty* names remain fail-closed faults.
    Different fresh names resolving to the same fixed identity are rejected as
    duplicate reports.  A missing identity becomes explicitly absent only
    while both side sources are initialized and fresh.
    """

    SOURCE_NAME = 'gazebo_controller_shuttle_state_name_and_timestamp_only'

    def __init__(
        self,
        *,
        timeout_s: float = 1.0,
        warmup_s: float = 0.5,
        future_stamp_tolerance_s: float = 0.25,
    ) -> None:
        self.timeout_s = _positive_finite(timeout_s, 'timeout_s')
        self.warmup_s = _nonnegative_finite(warmup_s, 'warmup_s')
        self.future_stamp_tolerance_s = _nonnegative_finite(
            future_stamp_tolerance_s,
            'future_stamp_tolerance_s',
        )
        self._specs = tuple(all_shuttle_specs())
        self._identity_order = tuple(FIXED_VISUAL_SHUTTLE_IDENTITIES)
        self._spec_by_identity = {spec.short_id: spec for spec in self._specs}
        self.reset()

    @property
    def identity_order(self) -> tuple[str, ...]:
        return self._identity_order

    def reset(self) -> None:
        self._records: dict[str, _PresenceRecord] = {}
        self._sources = {side: _SourceState() for side in PRESENCE_SIDES}
        self._faults: list[_Fault] = []

    def observe(
        self,
        *,
        topic_side: str,
        entity_name: Any,
        source_stamp_s: Any,
        receive_time_s: Any,
    ) -> None:
        """Ingest only fields authorized for presence gating."""

        side = str(topic_side or '').strip().lower()
        receive = _nonnegative_finite(receive_time_s, 'receive_time_s')
        stamp = _nonnegative_finite(source_stamp_s, 'source_stamp_s')
        if side not in PRESENCE_SIDES:
            self._record_fault(f'unsupported_presence_topic_side:{side or "missing"}', receive)
            return

        source = self._sources[side]
        if source.first_receive_time_s is None:
            source.first_receive_time_s = receive
        source.last_receive_time_s = receive
        if (
            source.last_source_stamp_s is None
            or stamp > source.last_source_stamp_s
        ):
            source.last_source_stamp_s = stamp

        raw_name = str(entity_name or '').strip()
        # Producer contract: Room315KinematicShuttleNode emits one blank-name
        # ShuttleState every publish cycle when ``self.shuttles`` is empty.
        # The timestamped message is an empty-inventory heartbeat, not an
        # unknown entity.  Returning here deliberately preserves the source
        # freshness update above while adding no presence record.
        if not raw_name:
            return
        spec = normalize_shuttle_ref(raw_name)
        if spec is None:
            self._record_fault(f'unknown_presence_entity:{raw_name}', receive)
            return
        if spec.side != side:
            self._record_fault(
                f'presence_identity_side_conflict:{raw_name}:{side}!={spec.side}',
                receive,
            )
            return

        identity = spec.short_id
        existing = self._records.get(identity)
        if (
            existing is not None
            and existing.entity_name != raw_name
            and receive - existing.receive_time_s <= self.timeout_s
        ):
            self._record_fault(
                f'duplicate_presence_identity:{identity}:'
                f'{existing.entity_name},{raw_name}',
                receive,
            )
            return

        self._records[identity] = _PresenceRecord(
            identity=identity,
            side=side,
            entity_name=raw_name,
            source_stamp_s=stamp,
            receive_time_s=receive,
        )

    def snapshot(self, *, now_s: float) -> PresenceSnapshot:
        now = _nonnegative_finite(now_s, 'now_s')
        self._expire_faults(now)

        initialized: list[str] = []
        stale_sides: list[str] = []
        reasons: list[str] = []
        for side in PRESENCE_SIDES:
            source = self._sources[side]
            if source.first_receive_time_s is None:
                reasons.append(f'presence_topic_not_initialized:{side}')
                continue
            warmup_age = now - source.first_receive_time_s
            if warmup_age < self.warmup_s:
                reasons.append(f'presence_topic_warming_up:{side}')
                continue
            initialized.append(side)
            receive_stale = (
                source.last_receive_time_s is None
                or now - source.last_receive_time_s > self.timeout_s
            )
            stamp_age = (
                math.inf
                if source.last_source_stamp_s is None
                else now - source.last_source_stamp_s
            )
            header_stale = (
                stamp_age > self.timeout_s
                or stamp_age < -self.future_stamp_tolerance_s
            )
            if receive_stale or header_stale:
                stale_sides.append(side)
                reasons.append(f'presence_source_stale:{side}')

        reasons.extend(fault.reason for fault in self._faults)
        ready = (
            len(initialized) == len(PRESENCE_SIDES)
            and not stale_sides
            and not self._faults
        )

        entries: list[PresenceEntry] = []
        for identity in self._identity_order:
            spec = self._spec_by_identity[identity]
            record = self._records.get(identity)
            if not ready:
                entries.append(PresenceEntry(
                    identity=identity,
                    side=spec.side,
                    state=PRESENCE_UNKNOWN,
                    reason='complete_presence_registry_not_ready',
                ))
                continue

            if record is None:
                entries.append(PresenceEntry(
                    identity=identity,
                    side=spec.side,
                    state=PRESENCE_ABSENT,
                    reason='not_reported_by_fresh_initialized_source',
                ))
                continue

            receive_age = now - record.receive_time_s
            stamp_age = now - record.source_stamp_s
            stamp_is_fresh = (
                stamp_age <= self.timeout_s
                and stamp_age >= -self.future_stamp_tolerance_s
            )
            if receive_age <= self.timeout_s and stamp_is_fresh:
                entries.append(PresenceEntry(
                    identity=identity,
                    side=spec.side,
                    state=PRESENCE_PRESENT,
                    entity_name=record.entity_name,
                    source_stamp_s=record.source_stamp_s,
                    receive_time_s=record.receive_time_s,
                    age_s=max(receive_age, stamp_age, 0.0),
                ))
            else:
                # A side source that continues publishing proves that this
                # identity has disappeared; this is explicit absence, not a
                # stale complete registry.
                entries.append(PresenceEntry(
                    identity=identity,
                    side=spec.side,
                    state=PRESENCE_ABSENT,
                    entity_name=record.entity_name,
                    source_stamp_s=record.source_stamp_s,
                    receive_time_s=record.receive_time_s,
                    age_s=max(receive_age, stamp_age, 0.0),
                    reason='identity_state_timed_out_while_source_remained_fresh',
                ))

        return PresenceSnapshot(
            timestamp_s=now,
            ready=ready,
            entries=tuple(entries),
            reasons=tuple(_dedupe(reasons)),
            initialized_sides=tuple(initialized),
            stale_sides=tuple(stale_sides),
            source=self.SOURCE_NAME,
        )

    def _record_fault(self, reason: str, receive_time_s: float) -> None:
        self._faults.append(_Fault(reason=reason, receive_time_s=receive_time_s))

    def _expire_faults(self, now_s: float) -> None:
        self._faults = [
            fault
            for fault in self._faults
            if now_s - fault.receive_time_s <= self.timeout_s
        ]


def _dedupe(values: list[str]) -> list[str]:
    return list(dict.fromkeys(values))


def _nonnegative_finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PresenceError(f'{name} must be numeric') from exc
    if not math.isfinite(result) or result < 0.0:
        raise PresenceError(f'{name} must be finite and non-negative')
    return result


def _positive_finite(value: Any, name: str) -> float:
    result = _nonnegative_finite(value, name)
    if result <= 0.0:
        raise PresenceError(f'{name} must be positive')
    return result
