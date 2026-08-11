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
- the KUKA gripper extension uses user-provided SCHUNK KGG 140-60 CAD/STL assets from CADENAS
- `staubli_tx2_60l` uses assets confirmed by the local project contributor as coming from `ros-industrial/staubli_experimental`
- the Staubli gripper extension uses user-provided SCHUNK PGN-plus-P 40 CAD/STL assets
- `yaskawa_hc10` and `yaskawa_hc10dt` use assets confirmed by the local project contributor as coming from `ros-industrial/motoman`
- the Yaskawa HC10 gripper extension uses user-provided Zimmer Group GEH6040IL CAD/STL assets from CADENAS
- the Yaskawa HC10DT gripper extension uses user-provided Zimmer Group LWR50L CAD/STL assets from CADENAS
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

### KUKA SCHUNK KGG 140-60 Gripper Extension

Local files:

- `models/kuka_kr6r900sixx/meshes/gripper/schunk_kgg_140_60_011l5_mss_22_01.stl`
- `models/kuka_kr6r900sixx/meshes/gripper/jaw_kuka.stl`
- `models/kuka_kr6r900sixx/cad/SCHUNK-0303070_KGG_140-60_011L5_MSS_22_01.stp`
- `models/kuka_kr6r900sixx/cad/JAW_KUKA.stp`
- `models/kuka_kr6r900sixx/cad/license_schunk.txt`
- `models/kuka_kr6r900sixx/cad/readme-and-terms-of-use-3d-cad-models.txt`

Known source metadata:

- user-provided CAD/STL files for the KUKA gripper integration
- STEP header name: `SCHUNK-0303070 KGG 140-60 _011L5_MSS_22_01`
- STEP header author field: `License CC BY-ND 4.0`
- STEP header organization: `CADENAS`
- STEP header originating system: `PARTsolutions`

Local modifications in this repository:

- source filenames were normalized for package-local mesh paths
- the original monolithic CC BY-ND STL remains unmodified in the package, but is
  no longer used as the runtime visual because it contains fixed jaws
- `jaw_kuka.stl` also remains unmodified and is instantiated twice at runtime at
  unit scale, with one instance rotated 180 degrees to provide the opposite jaw
- the runtime gripper body uses MFJA-local primitive geometry and is fixed to
  `tool0`; the two mesh-based, visual-only jaws use symmetric prismatic joints
  and have no collision geometry

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

### Staubli SCHUNK Gripper Extension

Local files:

- `models/staubli_tx2_60l/meshes/gripper/schunk_edited.stl`
- `models/staubli_tx2_60l/cad/SCHUNK-edited.stp`

Known source metadata:

- user-provided CAD/STL files for the Staubli robot gripper integration
- STEP header name: `SCHUNK-edited.stp`
- STEP header author field: `LEGION`
- STEP header originating system: `Autodesk Inventor 2021`
- STEP product metadata includes `SCHUNK-0318448 PGN-plus-P 40, 000`

Local modifications in this repository:

- source filenames were normalized for package-local mesh paths
- the monolithic source STL remains unmodified in the package, but is no longer
  used as the runtime visual because it contains fixed jaws
- the runtime gripper body and two jaws use MFJA-local primitive geometry; the
  body is fixed to `tool0`, while the visual-only jaws use symmetric prismatic
  joints and have no collision geometry

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

### Yaskawa HC10 Zimmer GEH6040IL Gripper Extension

Local files:

- `models/yaskawa_hc10/meshes/gripper/geh6040il_03_b01geh6000il.stl`
- `models/yaskawa_hc10/cad/GEH6040IL-03-B01GEH6000IL.stp`
- `models/yaskawa_hc10/cad/readme-and-terms-of-use-3d-cad-models.txt`

Known source metadata:

- user-provided CAD/STL files for the Yaskawa HC10 gripper integration
- STEP header name: `GEH6040IL-03-B01GEH6000IL`
- STEP header author field: `License CC BY-ND 4.0`
- STEP header organization: `CADENAS`
- STEP header originating system: `PARTsolutions`

Local modifications in this repository:

- source filenames were normalized for package-local mesh paths
- the monolithic CC BY-ND source STL remains unmodified in the package, but is no
  longer used as the runtime visual because it contains fixed jaw-base components
- the runtime gripper body and two jaws use MFJA-local primitive geometry; the
  body is fixed to `tool0`, while the visual-only jaws use symmetric prismatic
  joints and have no collision geometry
- the configured per-jaw travel is `40 mm` and maximum velocity is `60 mm/s`,
  matching the Zimmer Group technical data for `GEH6040IL-03-B`

### Yaskawa HC10DT Zimmer LWR50L Gripper Extension

Local files:

- `models/yaskawa_hc10dt/meshes/gripper/lwr50l_03_00001_a_000.stl`
- `models/yaskawa_hc10dt/cad/LWR50L-03-00001-A_000.stp`
- `models/yaskawa_hc10dt/cad/readme-and-terms-of-use-3d-cad-models.txt`

Known source metadata:

- user-provided CAD/STL files for the Yaskawa HC10DT gripper integration
- STEP header name: `LWR50L-03-00001-A(000)`
- STEP header author field: `License CC BY-ND 4.0`
- STEP header organization: `CADENAS`
- STEP header originating system: `PARTsolutions`

Local modifications in this repository:

- source filenames were normalized for package-local mesh paths
- the monolithic source STL remains unmodified in the package, but is no longer used
  as the runtime visual because it contains fixed fingers
- the runtime gripper body and two jaws use MFJA-local primitive geometry; the body
  is fixed to `tool0`, while the visual-only jaws use symmetric prismatic joints
  and have no collision geometry
- the configured per-jaw travel is `10 mm`, matching the Zimmer Group technical
  data for `LWR50L-03-00001-A`

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

- This package contains Apache-2.0, BSD, and CC BY-ND 4.0 upstream material as
  documented by asset family above; the MFJA-local primitive gripper visuals are
  separate integration work.
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
- Zimmer Group `GEH6040IL-03-B` technical data: <https://www.zimmer-group.com/en-us/products/components/handling-technology/2-jaw-parallel-grippers/series-geh6000il/products/geh6040il-03-b>
- Zimmer Group `LWR50L-03-00001-A` datasheet: <https://www.zimmer-group.com/fileadmin/pim/MER/GD/PG/MER_GD_PG_LWR50L-03-00001-A__SEN__APD__V1.pdf>
- direct source clarification from the MFJA project contributor for TIAGo assets: <https://github.com/Tiago-Harmonic>
