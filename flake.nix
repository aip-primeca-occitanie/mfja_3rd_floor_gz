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
            numpy
            pyyaml
            setuptools
          ]);

          spdlogRos = (pkgs.spdlog.override {
            fmt = pkgs.fmt_9;
          }).overrideAttrs (finalAttrs: _oldAttrs: {
            version = "1.12.0";

            src = pkgs.fetchFromGitHub {
              owner = "gabime";
              repo = "spdlog";
              rev = "v${finalAttrs.version}";
              hash = "sha256-cxTaOuLXHRU8xMz9gluYz0a93O0ez2xOxbloyc1m1ns=";
            };

            patches = [ ];
            doCheck = false;
          });

          tinyxml2Ros = pkgs.tinyxml-2.overrideAttrs (finalAttrs: _oldAttrs: {
            version = "10.0.0";

            src = pkgs.fetchFromGitHub {
              owner = "leethomason";
              repo = "tinyxml2";
              rev = finalAttrs.version;
              hash = "sha256-9xrpPFMxkAecg3hMHzzThuy0iDt970Iqhxs57Od+g2g=";
            };
          });

          rosRuntimeLibs = pkgs.lib.makeLibraryPath [
            pkgs.fmt_9
            pkgs.lttng-ust.out
            pkgs.openssl
            spdlogRos
            pkgs.stdenv.cc.cc.lib
            tinyxml2Ros
          ];

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
              pkgs.fmt_9
              pkgs.git
              pkgs.gnumake
              pkgs.lttng-ust.out
              pkgs.ninja
              pkgs.openssl
              pkgs.pkg-config
              spdlogRos
              pkgs.stdenv.cc.cc.lib
              tinyxml2Ros
              pythonEnv
            ];

            shellHook = ''
              export ROS_DISTRO=jazzy
              export MFJA_NIX_MODE=hybrid
              export RMW_IMPLEMENTATION="''${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
              export LD_LIBRARY_PATH="${rosRuntimeLibs}''${LD_LIBRARY_PATH:+:}''${LD_LIBRARY_PATH:-}"

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
