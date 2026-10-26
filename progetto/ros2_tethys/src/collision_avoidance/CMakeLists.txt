cmake_minimum_required(VERSION 3.8)
project(collision_avoidance)

# Genera build/collision_avoidance/compile_commands.json, a cui puntano il symlink
# alla root del workspace e il .clangd. Senza, clangd resta privo di include path e
# segnala errori inesistenti (array non trovato, std e M_PI non dichiarati).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# CAMBIATO: Ora l'eseguibile si chiama collision_avoidance_node
add_executable(collision_avoidance_node
  src/main.cpp
  src/collision_avoidance_node.cpp
  src/sensors.cpp
  src/navigation.cpp
  src/motion_control.cpp
  src/metrics.cpp
  src/vfh_planner.cpp
)

target_include_directories(collision_avoidance_node PRIVATE
  include
)

# Collega le librerie
ament_target_dependencies(collision_avoidance_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2_msgs
  ros_gz_interfaces
)

# Installa l'eseguibile
if(BUILD_TESTING)
  enable_testing()
  add_executable(vfh_planner_test test/vfh_planner_test.cpp src/vfh_planner.cpp)
  target_include_directories(vfh_planner_test PRIVATE include)
  add_test(NAME vfh_planner_regression COMMAND vfh_planner_test)
endif()

install(TARGETS
  collision_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
