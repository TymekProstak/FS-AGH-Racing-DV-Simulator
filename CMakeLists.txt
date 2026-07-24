cmake_minimum_required(VERSION 3.7)
project(lem_simulator)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  message_generation
)

find_package(dv_interfaces QUIET)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

set(DV_INTERFACES_EXPORTED_TARGETS)
set(DV_INTERFACES_INCLUDE_DIRS)

if(dv_interfaces_FOUND)
  message(STATUS "LEM Simulator: using workspace dv_interfaces")
  set(DV_INTERFACES_EXPORTED_TARGETS ${dv_interfaces_EXPORTED_TARGETS})
  set(DV_INTERFACES_INCLUDE_DIRS ${dv_interfaces_INCLUDE_DIRS})
else()
  message(STATUS "LEM Simulator: generating bundled minimal dv_interfaces")

  set(LEM_SIMULATOR_PROJECT_NAME ${PROJECT_NAME})
  set(PROJECT_NAME dv_interfaces)
  set(dv_interfaces_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

  add_message_files(
    NOINSTALL
    DIRECTORY interfaces/dv_interfaces/msg
    FILES
      Pos.msg
      Imu.msg
      Cone.msg
      Cones.msg
      Control.msg
      DV_board.msg
      full_state.msg
  )

  generate_messages(
    DEPENDENCIES
      std_msgs
  )

  set(DV_INTERFACES_EXPORTED_TARGETS
      ${dv_interfaces_EXPORTED_TARGETS})
  set(PROJECT_NAME ${LEM_SIMULATOR_PROJECT_NAME})
  catkin_destinations()
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DV_INTERFACES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(simSRC
  src_helpers/cone_detector.cpp
  src_helpers/cone_track.cpp
  src_helpers/centerline_metrics.cpp
  src_helpers/double_track.cpp
  src_helpers/imu_simulator.cpp
  src_helpers/steering_system.cpp
  src_helpers/tire_model.cpp
  src_helpers/torque_allocation.cpp
  src_helpers/utilities.cpp
  src_helpers/visualization_helpers.cpp
  src_helpers/suspension.cpp
)

add_executable(sim_loop_node
  sim_loop.cpp
  main.cpp
  ${simSRC}
)

add_dependencies(sim_loop_node
  ${catkin_EXPORTED_TARGETS}
  ${DV_INTERFACES_EXPORTED_TARGETS}
)

target_link_libraries(sim_loop_node
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  m
)

add_executable(false_dv_path_planning_node
  false_dv_path_planning_node.cpp
)

add_dependencies(false_dv_path_planning_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(false_dv_path_planning_node
  ${catkin_LIBRARIES}
)

install(TARGETS
  sim_loop_node
  false_dv_path_planning_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config tracks logs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY interfaces/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/interfaces
)
