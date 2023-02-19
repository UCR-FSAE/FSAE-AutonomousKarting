<!--
 Copyright 2023 michael. All rights reserved.
 Use of this source code is governed by a BSD-style
 license that can be found in the LICENSE file.
-->

1. record waypoint using

`ros2 launch global_planner_manager waypoint_recorder interval:=10.0 output_path:=./data/my_path.txt`


1. waypoint following server
`ros2 launch global_planner_manager global_planner_manager input_file_path:=/data/my_path.txt`