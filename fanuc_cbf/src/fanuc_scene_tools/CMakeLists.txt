cmake_minimum_required(VERSION 3.8)
project(fanuc_scene_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  fanuc_scene_tools/collision_scene_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collision_scene_publisher
)
install(PROGRAMS
  fanuc_scene_tools/workspace_boundary_sweep.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME workspace_boundary_sweep
)
install(PROGRAMS
  fanuc_scene_tools/render_workspace_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME render_workspace_map
)
install(PROGRAMS
  fanuc_scene_tools/orientation_access_volume.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME orientation_access_volume
)
install(FILES
  fanuc_scene_tools/orientation_access_volume.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  fanuc_scene_tools/task_reachability_atlas.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME task_reachability_atlas
)
install(PROGRAMS
  fanuc_scene_tools/cartesian_goal_panel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cartesian_goal_panel
)
install(PROGRAMS
  fanuc_scene_tools/cartesian_jog_panel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cartesian_jog_panel
)
install(PROGRAMS
  fanuc_scene_tools/cbf_velocity_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cbf_velocity_filter
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  fanuc_scene_tools/reachability_sweep.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reachability_sweep
)
install(PROGRAMS
  fanuc_scene_tools/placement_optimizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME placement_optimizer
)
install(FILES
  launch/crx3ia_collision_scene.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})

ament_package()