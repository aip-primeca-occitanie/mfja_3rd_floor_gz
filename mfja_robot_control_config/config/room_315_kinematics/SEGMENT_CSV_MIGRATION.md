# Room 315 Segment CSV Filename Migration

## Scope

This migration changes physical CSV filenames and explicit YAML references
only. CSV bytes, coordinate order, segment direction, declared nodes, switch
labels, Fixed/Select/Guard rules, `FALLING`, interpolation, planner behavior,
controller commands, action vectors, and safety behavior remain unchanged.

The pre-migration evidence is stored in
`test/fixtures/room315_segment_csv_pre_normalization.json`. It records all 14
public segments, all 14 source CSV associations, 276 parsed coordinate rows,
per-segment SHA-256/size/first/last samples, right/left successor rules, the
complete topology edges, and topology hashes.

## Collision-safe physical rename

The filenames form swap cycles, so the migration was performed through unique
temporary names before installing the final names. No destination was
overwritten. The exact physical movement was:

| Legacy physical CSV | Normalized physical CSV |
|---|---|
| `A34E.csv` | `A12E.csv` |
| `A34I.csv` | `A12I.csv` |
| `A23.csv` | `A14.csv` |
| `A3E.csv` | `A1E.csv` |
| `A3I.csv` | `A1I.csv` |
| `A14.csv` | `A23.csv` |
| `A4E.csv` | `A2E.csv` |
| `A4I.csv` | `A2I.csv` |
| `A12E.csv` | `A34E.csv` |
| `A12I.csv` | `A34I.csv` |
| `A1E.csv` | `A3E.csv` |
| `A1I.csv` | `A3I.csv` |
| `A2E.csv` | `A4E.csv` |
| `A2I.csv` | `A4I.csv` |

The final public-segment mapping is therefore explicit and identity-named:

`A12E`, `A12I`, `A14`, `A1E`, `A1I`, `A23`, `A2E`, `A2I`, `A34E`, `A34I`,
`A3E`, `A3I`, `A4E`, and `A4I` each reference
`raw_segments/<public-segment>.csv`.

## Authority and compatibility

`rail_network_right.yaml` and `rail_network_left.yaml` remain authoritative.
Every segment retains an explicit `csv` field; the loader does not derive the
filename from the public segment key. The
`csv_reference_schema: public_segment_filename_v1` marker identifies the live
normalized layout.

Immutable historical schema snapshots were found with the exact legacy
references. The loader therefore accepts only the 14 known
`(public segment, legacy reference)` pairs when the normalized schema marker is
absent, emits a `DeprecationWarning`, and resolves their normalized geometry.
Live YAML and new outputs use normalized references only. No duplicate legacy
CSV files are retained.

## Rollback

Rollback must also be collision-safe:

1. Preserve the current 14 CSVs in unique temporary files.
2. Apply the inverse of the physical-movement table above from those temporary
   files; never rename one live CSV directly onto another.
3. Restore the two topology YAML files to the legacy explicit references and
   remove the normalized schema marker.
4. Restore the loader, CMake registration, regression test, fixture, and
   documentation changes from the same pre-migration revision.
5. Recompute every per-public-segment SHA-256, size, coordinate count and
   first/last sample against the evidence fixture, then rerun the focused and
   repository-wide test suites.

Do not normalize line endings or rewrite CSV text during either migration or
rollback; byte identity is part of the integrity contract.
