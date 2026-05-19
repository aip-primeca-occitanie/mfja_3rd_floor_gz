{
  description = "Hybrid development shell for the MFJA 3rd floor ROS 2 Jazzy / Gazebo Harmonic simulation";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
  };

  outputs = { nixpkgs, ... }:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;
          };

          pythonEnv = pkgs.python312.withPackages (ps: with ps; [
            catkin-pkg
            empy
            lark
            pyyaml
            setuptools
          ]);

          colconWrapper = pkgs.writeShellScriptBin "colcon" ''
            if [ -x /usr/bin/colcon ]; then
              exec /usr/bin/colcon "$@"
            fi

            echo "colcon is not installed on the host." >&2
            echo "Install it with: sudo apt install python3-colcon-common-extensions" >&2
            exit 127
          '';
        in
        {
          default = pkgs.mkShell {
            name = "mfja-hybrid-ros2-jazzy-gz-harmonic";

            packages = [
              pkgs.bashInteractive
              pkgs.cmake
              colconWrapper
              pkgs.gcc
              pkgs.git
              pkgs.gnumake
              pkgs.ninja
              pkgs.pkg-config
              pythonEnv
            ];

            shellHook = ''
              export ROS_DISTRO=jazzy
              export MFJA_NIX_MODE=hybrid

              if [ -f /opt/ros/jazzy/setup.bash ]; then
                source /opt/ros/jazzy/setup.bash
                echo "Entered MFJA hybrid Nix shell."
                echo "ROS 2 Jazzy was sourced from /opt/ros/jazzy."
              else
                echo "Entered MFJA hybrid Nix shell."
                echo "WARNING: /opt/ros/jazzy/setup.bash was not found."
                echo "Install ROS 2 Jazzy and the ROS-Gazebo packages on the host before building."
              fi

              echo "Nix provides build tools only; ROS 2 and Gazebo come from the host apt installation."
              echo "Build from the colcon workspace root, for example:"
              echo "  cd ../.. && colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz"
            '';
          };
        });
    };
}
