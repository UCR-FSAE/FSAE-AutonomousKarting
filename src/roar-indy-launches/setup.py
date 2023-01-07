# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

from setuptools import setup
import os
from glob import glob


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(".", path, filename))
    return paths


package_name = "roar-indy-launches"
setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join(os.path.join("share", package_name), "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(os.path.join("share", package_name), "launch"),
            glob("launch/livox/*.launch.py"),
        ),
        (
            os.path.join(os.path.join("share", package_name), "config"),
            package_files("config"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="roar",
    maintainer_email="wuxiaohua1011@berkeley.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
