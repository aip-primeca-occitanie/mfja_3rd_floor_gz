from glob import glob

from setuptools import setup

package_name = "mfja_staubli_manipulation_demos"
room315_executables = [
    "scripts/room315_check_setup.sh",
    "scripts/room315_pick_place.sh",
]
room315_support_scripts = ["scripts/room315_env.sh"]
room315_launch_files = [
    "launch/room_315_staubli_hardware.launch.py",
    "launch/room_315_staubli_pick_place_sim.launch.py",
]
room315_docs = ["docs/room315_pick_place_walkthrough.md"]
room315_hpp_files = [
    "hpp/room315_config.py",
    "hpp/room315_execution.py",
    "hpp/room315_execution_profiles.py",
    "hpp/room315_payload_box.srdf",
    "hpp/room315_payload_box.urdf",
    "hpp/room315_pick_place.py",
    "hpp/room315_planning.py",
    "hpp/room315_problem.py",
    "hpp/room315_staubli_table_drop_zone.srdf",
    "hpp/room315_staubli_table_drop_zone.urdf",
    "hpp/staubli_tx2_60l_manipulation.srdf",
]

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/docs", room315_docs),
        (f"share/{package_name}/launch", room315_launch_files),
        (f"share/{package_name}/hpp", room315_hpp_files),
        (
            f"share/{package_name}/scripts",
            room315_executables + room315_support_scripts,
        ),
        (
            f"lib/{package_name}",
            room315_executables + ["scripts/room315_env.sh"],
        ),
        (
            f"share/{package_name}/models/staubli_tx2_60l_gripper",
            glob("models/staubli_tx2_60l_gripper/*"),
        ),
        (f"share/{package_name}/models", glob("models/*.sdf")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Paul Sardin",
    maintainer_email="paulsardin123@gmail.com",
    description="HPP table pick-and-place for the Room 315 Staubli.",
    license="Apache License 2.0",
)
