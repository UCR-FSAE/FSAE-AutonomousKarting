from setuptools import setup
import os
from glob import glob

package_name = 'gokart_manual_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="michael",
    maintainer_email="wuxiaohua1011@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "manual_controller_node = gokart_manual_controller.manual_controller_node:main"
        ],
    },
)
