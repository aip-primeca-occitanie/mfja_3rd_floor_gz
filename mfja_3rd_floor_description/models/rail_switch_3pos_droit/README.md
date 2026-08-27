# rail_switch_3pos_droit

This model provides the rotating switch blade.

- Mesh: `aiguillage3.stl`
- Pivot: mesh origin from SolidWorks
- Control mode: kinematic pose update through `/world/<world_name>/set_pose`

Notes:

- `model.sdf` places the pivot at the mesh origin
- A uniform mesh scale is applied because the SolidWorks export is smaller than the previous in-scene blade by a constant factor
- The fixed rail base remains a separate model: `cell_static_droit_final`
- Default world pivot position used by the worlds and control script: `-15.7470 -4.42 0.73`

Allowed command angles:

- position `0` -> `2.6`
- position `1` -> `-1.59`
- position `2` -> `0.50666`
