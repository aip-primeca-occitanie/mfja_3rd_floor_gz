# rail_switch_3pos_left

This model contains the rotating switch blade only.

- Mesh: `aiguillage3.stl`
- Pivot: mesh origin from SolidWorks
- Control mode: kinematic pose update through `/world/<world_name>/set_pose`

Notes:

- No local pose offset is applied in `model.sdf`
- A uniform mesh scale is applied because the SolidWorks export is smaller than the previous in-scene blade by a constant factor
- The fixed rail base remains a separate model: `cell_static_gauche_final`
- Default world pivot position used by the worlds and control script: `-16.73 -4.42 0.73`

Allowed command angles:

- position `0` -> `2.6`
- position `1` -> `-1.59`
- position `2` -> `0.50666`
