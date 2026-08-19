from glob import glob
from os.path import isfile
from setuptools import setup

package_name = "mfja_staubli_manipulation_demos"
room315_executables = [
    "scripts/room315_check_setup.sh",
    "scripts/room315_demo.sh",
    "scripts/room315_hpp_manipulation.sh",
    "scripts/room315_manipulation_demo.sh",
    "scripts/room315_moving_shuttle_demo.sh",
]
room315_support_scripts = [
    "scripts/room315_env.sh",
    "scripts/room315_manipulation_sequence.py",
    "scripts/room315_moving_shuttle_sequence.py",
]

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/docs", glob("docs/*.md")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/hpp", [path for path in glob("hpp/*") if isfile(path)]),
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
    description=(
        "HPP manipulation demos for the Room 315 Staubli with the kinematic "
        "shuttle system."
    ),
    license="Apache License 2.0",
)
