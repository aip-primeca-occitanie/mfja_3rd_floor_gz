from glob import glob
from os.path import isfile

from setuptools import setup

package_name = "mfja_staubli_demos"
room315_scripts = [
    "scripts/room315_check_environment.py",
    "scripts/room315_demo.sh",
    "scripts/room315_env.sh",
    "scripts/room315_export_staubli_line.py",
    "scripts/room315_export_staubli_line.sh",
    "scripts/room315_hpp_line.sh",
    "scripts/room315_read_configuration.py",
]

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (
            f"share/{package_name}/launch",
            ["launch/room_315_staubli_cartesian_demo.launch.py"],
        ),
        (f"share/{package_name}/hpp", [path for path in glob("hpp/*") if isfile(path)]),
        (f"share/{package_name}/scripts", room315_scripts),
        (f"lib/{package_name}", room315_scripts),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Paul Sardin",
    maintainer_email="paulsardin123@gmail.com",
    description=(
        "HPP-planned Cartesian line demo for the Room 315 Staubli TX2-60L."
    ),
    license="Apache License 2.0",
)
