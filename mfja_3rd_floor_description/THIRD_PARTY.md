# Third-Party Asset Attribution

This package contains a mix of MFJA-specific simulation assets and robot description
assets that were imported from, or derived from, third-party upstream projects.
The original license terms of those upstream projects continue to apply to the
corresponding files in this package.

This document records the best-known provenance of the redistributed robot assets
currently shipped in `mfja_3rd_floor_description/models/` and
`mfja_3rd_floor_description/urdf/`.

Scope of this file:

- robot `URDF` files under `urdf/`
- robot meshes and Gazebo model wrappers under `models/`

Excluded from third-party attribution because they are MFJA-local work:

- non-robot environment meshes created by the MFJA project team
- room and building geometry created by the MFJA project team
- furniture and other scene assets used by `worlds/` when they were created by the MFJA project team

## Current Status

- `kuka_kr6r900sixx` uses assets confirmed by the local project contributor as coming from `ros-industrial/kuka_experimental`
- `staubli_tx2_60l` uses assets confirmed by the local project contributor as coming from `ros-industrial/staubli_experimental`
- `yaskawa_hc10` and `yaskawa_hc10dt` use assets confirmed by the local project contributor as coming from `ros-industrial/motoman`
- `tiago` uses assets confirmed by the local project contributor as coming from the `Tiago-Harmonic` repository family
- `model.sdf`, `model.config`, launch integration, and package-local path rewrites are MFJA-local integration work
- non-robot environment meshes, room geometry, building geometry, furniture, and other scene assets used by `worlds/` are MFJA-local work unless explicitly documented otherwise

## Asset Map

### KUKA Asset Family Used For Local KR6 Integration

Local files:

- `urdf/kuka_kr6r900sixx.urdf`
- `models/kuka_kr6r900sixx/`

Confirmed upstream source:

- ROS-Industrial KUKA support assets from `ros-industrial/kuka_experimental`
- Repository: `ros-industrial/kuka_experimental`
- Repository URL: <https://github.com/ros-industrial/kuka_experimental>
- Package index URL: <https://index.ros.org/p/kuka_kr6_support/>

Contributor-provided clarification:

- the imported KUKA source model used by the project was the KR6 R700 family
- the local MFJA package currently exposes the robot under the local file and model name `kuka_kr6r900sixx`
- the local mesh tree contains both `kr6r700sixx` and `kr6r900sixx` subdirectories, so this package should be treated as a locally adapted derivative of ROS-Industrial KUKA support assets rather than a byte-for-byte copy of a single upstream package

Observed upstream licensing metadata:

- ROS Index lists package `kuka_kr6_support` under `BSD`
- the GitHub repository root is labeled `Apache-2.0`

Local modifications in this repository:

- upstream package paths were rewritten to local package paths
- Gazebo `model.sdf` and `model.config` wrappers were added
- local visual meshes under `models/kuka_kr6r900sixx/meshes/converted_visual/` were converted for local use

### Staeubli TX2-60L

Local files:

- `urdf/staubli_tx2_60l.urdf`
- `models/staubli_tx2_60l/`

Confirmed upstream source:

- ROS-Industrial Staeubli support assets from `ros-industrial/staubli_experimental`
- Package family: `staubli_tx2_60_support`
- Repository URL: <https://github.com/ros-industrial/staubli_experimental>
- Package index URL: <https://index.ros.org/p/staubli_tx2_60_support/>

Contributor-provided clarification:

- the project source model was the TX2 60L
- local naming and mesh layout match the ROS-Industrial TX2 support family

Observed upstream licensing metadata:

- ROS Index lists `staubli_experimental` packages under `Apache2.0`

Local modifications in this repository:

- support-package mesh paths were rewritten to local package paths
- a package-local URDF was materialized for MFJA use
- Gazebo `model.sdf` and `model.config` wrappers were added

### Yaskawa HC10 and HC10DT

Local files:

- `urdf/yaskawa_hc10.urdf`
- `urdf/yaskawa_hc10dt.urdf`
- `models/yaskawa_hc10/`
- `models/yaskawa_hc10dt/`

Confirmed upstream source:

- ROS-Industrial Motoman support assets from `ros-industrial/motoman`
- Package family: `motoman_hc10_support`
- Repository URL: <https://github.com/ros-industrial/motoman>
- Package index URL: <https://index.ros.org/p/motoman_hc10_support/>

Contributor-provided clarification:

- the left Yaskawa robot in the project is HC10DT
- the right Yaskawa robot in the project is HC10
- local link naming and mesh layout match the ROS-Industrial HC10 support family

Observed upstream licensing metadata:

- ROS Index lists `motoman_hc10_support` under `BSD`
- the GitHub repository root shows both Apache-2.0 and BSD-3-Clause badges

Local modifications in this repository:

- upstream package paths were rewritten to local package paths
- package-local URDF files were materialized for MFJA use
- Gazebo `model.sdf` and `model.config` wrappers were added

### TIAGo

Local files:

- `urdf/tiago.urdf`
- `models/tiago/`

Confirmed upstream source:

- Tiago Harmonic repository family
- Repository URL: <https://github.com/Tiago-Harmonic>

Contributor-provided clarification:

- the TIAGo assets used in this project were taken from the `Tiago-Harmonic` GitHub organization or one of its repositories
- the local model and mesh naming matches the TIAGo robot description stack, including arm, torso, and sensor mesh families

Observed upstream licensing metadata:

- licensing for the exact imported `Tiago-Harmonic` source repository still needs to be confirmed and copied into this repository before external redistribution

Local modifications in this repository:

- a simplified MFJA-local `urdf/tiago.urdf` was created instead of shipping the full upstream xacro stack
- package paths were rewritten to local package paths
- Gazebo `model.sdf` and `model.config` wrappers were added

## Redistribution Notes

- This package should be treated as containing both Apache-2.0 and BSD-licensed upstream material.
- Attribution to the upstream repositories above should be preserved in any redistribution of the corresponding robot assets.
- If this repository will be published outside the lab or submitted externally, the safest next step is to vendor the exact upstream license texts into a local `LICENSES/` directory and pin the specific upstream commit SHAs used for each imported robot family.

## Sources Used To Prepare This Record

- ROS package manifest format specification: <https://ros.org/reps/rep-0149.html>
- direct source clarification from the MFJA project contributor for KUKA, Staeubli, and Yaskawa assets
- ROS-Industrial KUKA repository: <https://github.com/ros-industrial/kuka_experimental>
- ROS Index entry for `kuka_kr6_support`: <https://index.ros.org/p/kuka_kr6_support/>
- Public upstream-generated `kr6r900sixx.urdf` reference: <https://gist.github.com/gavanderhoorn/cfea4a8238e39a0c3b0a5c56d979c4d4>
- ROS-Industrial Staeubli repository family: <https://github.com/ros-industrial/staubli_experimental>
- ROS Index entry for `staubli_tx2_60_support`: <https://index.ros.org/p/staubli_tx2_60_support/>
- ROS-Industrial Motoman repository: <https://github.com/ros-industrial/motoman>
- ROS Index entry for `motoman_hc10_support`: <https://index.ros.org/p/motoman_hc10_support/>
- direct source clarification from the MFJA project contributor for TIAGo assets: <https://github.com/Tiago-Harmonic>
