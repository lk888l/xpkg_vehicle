from glob import glob
from setuptools import find_packages, setup

setup(
    name="vehicle_gui", version="1.0.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/vehicle_gui"]),
        ("share/vehicle_gui", ["package.xml"]),
        ("share/vehicle_gui/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], zip_safe=False,
    maintainer="Luka", maintainer_email="207119930@qq.com", license="Apache-2.0",
    description="PyQt5 ROS 2 vehicle control console",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["vehicle_gui = vehicle_gui.main:main"]},
)
