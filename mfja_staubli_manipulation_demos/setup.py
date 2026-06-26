from glob import glob
from os.path import isfile
from setuptools import setup

package_name = "mfja_staubli_manipulation_demos"

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
            [path for path in glob("scripts/*") if isfile(path)],
        ),
        (
            f"share/{package_name}/models/staubli_tx2_60l_gripper",
            glob("models/staubli_tx2_60l_gripper/*"),
        ),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Paul Sardin",
    maintainer_email="paulsardin123@gmail.com",
    description=(
        "HPP manipulation demos for the Room 315 Staubli with the kinematic "
        "shuttle system."
    ),
    license="Apache License 2.0",
)
