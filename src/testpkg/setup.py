from setuptools import setup
from glob import glob
import os

package_name = "testpkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/objects.json"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="davood dorostkar",
    maintainer_email="davood.dorostkar.ut@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "custom_manual_control=testpkg.custom_manual_control:main"
        ],  # "simulator = testpkg.simulator:main"
    },
    # package_dir={"": "src"},
    # package_data={"": ["CARLA_VERSION"]},
    # include_package_data=True,
)
