cmake_minimum_required(VERSION 3.5)
project(moveit_grasps)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)
find_package(OpenMP REQUIRED)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")


# Load ament and all dependencies required for this package
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  # message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  rclcpp
  rosparam_shortcuts
  rcpputils
  std_msgs
  tf2
  tf2_eigen
  trajectory_msgs
  Eigen3
  Boost
  OpenMP
)

# Ament
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)

ament_package()

###########
## Build ##
###########

include_directories(
  include
)

# # Grasp Library
# add_library(${PROJECT_NAME}
#   src/grasp_candidate.cpp
#   src/grasp_data.cpp
#   src/grasp_generator.cpp
#   src/grasp_scorer.cpp
#   src/suction_grasp_candidate.cpp
#   src/suction_grasp_data.cpp
#   src/suction_grasp_generator.cpp
#   src/suction_grasp_scorer.cpp
#   src/two_finger_grasp_data.cpp
#   src/two_finger_grasp_generator.cpp
#   src/two_finger_grasp_scorer.cpp
# )
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

# # Grasp Filter Library
# add_library(${PROJECT_NAME}_filter
#   src/grasp_filter.cpp
#   src/suction_grasp_filter.cpp
#   src/two_finger_grasp_filter.cpp
#   src/grasp_planner.cpp
# )
# target_link_libraries(${PROJECT_NAME}_filter
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# set_target_properties(${PROJECT_NAME}_filter PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # for threading
# set_target_properties(${PROJECT_NAME}_filter PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

# # Demo filter executable
# add_executable(${PROJECT_NAME}_grasp_filter_demo src/demo/grasp_filter_demo.cpp)
# ament_target_dependencies(${PROJECT_NAME}_grasp_filter_demo
#     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Demo grasp executable
add_executable(${PROJECT_NAME}_grasp_generator_demo
  # include/moveit_grasps/two_finger_grasp_data.h
  # include/moveit_grasps/two_finger_grasp_generator.h
  # include/moveit_grasps/two_finger_grasp_scorer.h
  # include/moveit_grasps/grasp_data.h
  # include/moveit_grasps/grasp_candidate.h
  # include/moveit_grasps/grasp_generator.h
  # include/moveit_grasps/grasp_scorer.h
  # include/moveit_grasps/suction_voxel_matrix.h
  src/grasp_data.cpp
  src/grasp_candidate.cpp
  src/grasp_scorer.cpp
  src/grasp_generator.cpp
  src/grasp_filter.cpp
  src/two_finger_grasp_data.cpp
  src/two_finger_grasp_generator.cpp
  src/two_finger_grasp_scorer.cpp
  src/demo/grasp_generator_demo.cpp
)
ament_target_dependencies(${PROJECT_NAME}_grasp_generator_demo
${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# # Demo grasp data settings
# add_executable(${PROJECT_NAME}_grasp_poses_visualizer_demo src/demo/grasp_poses_visualizer_demo.cpp)
# target_link_libraries(${PROJECT_NAME}_grasp_poses_visualizer_demo
#   ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

# # Demo grasp pipeline
# add_executable(${PROJECT_NAME}_grasp_pipeline_demo src/demo/grasp_pipeline_demo.cpp)
# target_link_libraries(${PROJECT_NAME}_grasp_pipeline_demo
#   ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

# # Demo suction grasp pipeline
# add_executable(${PROJECT_NAME}_suction_grasp_pipeline_demo src/demo/suction_grasp_pipeline_demo.cpp)
# target_link_libraries(${PROJECT_NAME}_suction_grasp_pipeline_demo
#   ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

#############
## INSTALL ##
#############

# # Install libraries
# install(TARGETS
#   ${PROJECT_NAME}
#   ${PROJECT_NAME}_filter
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# Install shared resources
install(DIRECTORY config config_robot launch resources
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}_grasp_generator_demo
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   add_rostest_gtest(two_finger_grasp_data_test test/two_finger_grasp_data_test.test test/two_finger_grasp_data_test.cpp)
#   target_link_libraries(two_finger_grasp_data_test
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

#   add_rostest_gtest(two_finger_grasp_generator_test test/two_finger_grasp_generator_test.test test/two_finger_grasp_generator_test.cpp)
#   target_link_libraries(two_finger_grasp_generator_test
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

#   add_rostest_gtest(two_finger_grasp_filter_test test/two_finger_grasp_filter_test.test test/two_finger_grasp_filter_test.cpp)
#   target_link_libraries(two_finger_grasp_filter_test
#     ${PROJECT_NAME}
#     ${PROJECT_NAME}_filter
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

#   add_rostest_gtest(suction_grasp_pipeline_test test/suction_grasp_pipeline_test.test test/suction_grasp_pipeline_test.cpp)
#   target_link_libraries(suction_grasp_pipeline_test
#     ${PROJECT_NAME}
#     ${PROJECT_NAME}_filter
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

#   add_rostest_gtest(suction_grasp_unit_tests test/suction_grasp_unit_tests.test test/suction_grasp_unit_tests.cpp)
#   target_link_libraries(suction_grasp_unit_tests
#     ${PROJECT_NAME}
#     ${PROJECT_NAME}_filter
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

#   ## Test for correct C++ source code
#   roslint_cpp()
# endif()
