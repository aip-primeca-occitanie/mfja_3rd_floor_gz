{
  description = "Pinned auxiliary build-tool shell for the Ubuntu MFJA ROS 2 Jazzy / Gazebo Harmonic simulation";

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

          nixTools = [
            pkgs.bashInteractive
            pkgs.git
            pkgs.gnumake
            pkgs.ninja
          ];

          nixToolPath = pkgs.lib.makeBinPath nixTools;
        in
        {
          default = pkgs.mkShellNoCC {
            name = "mfja-hybrid-ros2-jazzy-gz-harmonic";

            packages = nixTools;

            shellHook = ''
              # Remove overlays and compiler flags inherited from interactive
              # Bash startup files before loading the supported Ubuntu stack.
              unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
              unset CMAKE_INCLUDE_PATH CMAKE_LIBRARY_PATH
              unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
              unset PKG_CONFIG_PATH PKG_CONFIG_LIBDIR PKG_CONFIG_SYSROOT_DIR
              unset PKG_CONFIG_ALLOW_SYSTEM_CFLAGS PKG_CONFIG_ALLOW_SYSTEM_LIBS
              unset NIXPKGS_CMAKE_PREFIX_PATH
              unset NIX_CFLAGS_COMPILE NIX_CFLAGS_COMPILE_FOR_BUILD
              unset NIX_CFLAGS_LINK NIX_LDFLAGS NIX_LDFLAGS_FOR_BUILD
              unset NIX_CXXSTDLIB_COMPILE
              unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS
              unset COMPILER_PATH GCC_EXEC_PREFIX
              unset CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH LIBRARY_PATH
              unset PYTHONHOME VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV

              # Keep the pinned ABI-neutral Nix tools, then use only Ubuntu
              # executables for the compiler/runtime side of the ROS stack.
              export PATH="${nixToolPath}:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/sbin:/usr/local/bin"

              export ROS_DISTRO=jazzy
              export MFJA_NIX_MODE=hybrid
              export RMW_IMPLEMENTATION="''${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

              # ROS/Gazebo apt packages must be compiled and loaded with the
              # matching Ubuntu compiler, Python, pkg-config, and binutils.
              export CC=/usr/bin/gcc
              export CXX=/usr/bin/g++
              export AR=/usr/bin/ar
              export AS=/usr/bin/as
              export LD=/usr/bin/ld
              export NM=/usr/bin/nm
              export RANLIB=/usr/bin/ranlib
              export STRIP=/usr/bin/strip
              export PKG_CONFIG=/usr/bin/pkg-config
              export PYTHON=/usr/bin/python3

              for hostTool in \
                /usr/bin/cmake \
                /usr/bin/colcon \
                /usr/bin/gcc \
                /usr/bin/g++ \
                /usr/bin/pkg-config \
                /usr/bin/python3; do
                if [ ! -x "$hostTool" ]; then
                  echo "WARNING: required Ubuntu tool is missing: $hostTool" >&2
                fi
              done
              unset hostTool

              if [ -f /opt/ros/jazzy/setup.bash ]; then
                source /opt/ros/jazzy/setup.bash
                echo "Entered MFJA hybrid Nix shell."
                echo "ROS 2 Jazzy was sourced from /opt/ros/jazzy."
              else
                echo "Entered MFJA hybrid Nix shell."
                echo "WARNING: /opt/ros/jazzy/setup.bash was not found."
                echo "Install ROS 2 Jazzy and the ROS-Gazebo packages on the host before building."
              fi

              echo "Nix provides Bash, Ninja, Make, and Git."
              echo "Ubuntu provides CMake, GCC/G++, Python, pkg-config, colcon, ROS 2, and Gazebo."
              echo "Build from the colcon workspace root, for example:"
              echo "  cd ../.. && colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz --cmake-args -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DPython3_EXECUTABLE=/usr/bin/python3"
            '';
          };
        });
    };
}
