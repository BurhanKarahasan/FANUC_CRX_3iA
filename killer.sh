pkill -INT -f 'ros2 launch fanuc_scene_tools crx3ia_collision_scene.launch.py'
  
pkill -INT -f '/opt/ros/jazzy/lib/moveit_ros_move_group/move_group'
pkill -INT -f '/opt/ros/jazzy/lib/controller_manager/ros2_control_node'
pkill -INT -f 'cartesian_goal_panel'

ps -ef | grep -E '[r]os2 launch|[m]ove_group|[r]os2_control_node|[c]artesian_goal_panel'
