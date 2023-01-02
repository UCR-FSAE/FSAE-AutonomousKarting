echo -e "--- Configuring costmap_node_manager ---"
ros2 lifecycle set /costmap_node_manager configure 
echo -e "--- Done --- \n"

echo -e "--- Configuring global_planner_manager ---"
ros2 lifecycle set /global_planner_manager configure 
echo -e "--- Done --- \n"

echo -e "--- Activating costmap_node_manager ---"
ros2 lifecycle set /costmap_node_manager activate 
echo -e "--- Done --- \n"

echo -e "--- Activating global_planner_manager ---"
ros2 lifecycle set /global_planner_manager activate 
echo -e "--- Done --- \n"
