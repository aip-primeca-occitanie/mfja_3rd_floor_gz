from setuptools import setup

package_name = "mfja_staubli_demos"

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            ["launch/room_315_staubli_cartesian_demo.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Paul Sardin",
    maintainer_email="paulsardin123@gmail.com",
    description="HPP-planned Cartesian line demo for the Room 315 Staubli TX2-60L.",
    license="Apache License 2.0",
)
