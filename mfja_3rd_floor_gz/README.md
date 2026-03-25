# mfja_3rd_floor_gz

Umbrella and compatibility ROS 2 package for the MFJA 3rd floor Gazebo
meta-repository.

This package exists to give the repository a subpackage with the same name as
the Git repository and to provide compatibility launch entry points that forward
to the dedicated bringup packages.

Preferred launch packages:

- `mfja_3rd_floor_bringup`
- `mfja_room_315_bringup`

Compatibility launch examples:

- `ros2 launch mfja_3rd_floor_gz full_floor.launch.py`
- `ros2 launch mfja_3rd_floor_gz room_315_only.launch.py`
