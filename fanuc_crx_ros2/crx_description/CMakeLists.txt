cmake_minimum_required(VERSION 3.8)
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
  robot_model:=crx3ia \
  use_mock:=true \
  launch_rviz:=falseproject(crx_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
