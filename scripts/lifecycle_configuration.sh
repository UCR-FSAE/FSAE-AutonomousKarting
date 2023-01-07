# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

echo "Starting to configure"
echo -e "--- Configuring costmap_node_manager ---"
ros2 lifecycle set /costmap_node_manager configure 
echo -e "--- Done --- \n"

echo -e "--- Configuring global_planner_manager ---"
ros2 lifecycle set /global_planner_manager configure 
echo -e "--- Done --- \n"

echo -e "--- Configuring simple local planner ---"
ros2 lifecycle set /simple_local_planner configure 
echo -e "--- Done --- \n"


echo -e "\n\nAll configure done\n\n"


echo "Starting to Activate"
echo -e "--- Activating costmap_node_manager ---"
ros2 lifecycle set /costmap_node_manager activate 
echo -e "--- Done --- \n"

echo -e "--- Activating global_planner_manager ---"
ros2 lifecycle set /global_planner_manager activate 
echo -e "--- Done --- \n"

echo -e "--- Activating simple local planner ---"
ros2 lifecycle set /simple_local_planner activate 
echo -e "--- Done --- \n"

