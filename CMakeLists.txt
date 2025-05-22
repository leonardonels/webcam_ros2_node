cmake_minimum_required(VERSION 3.8)
project(webcam_publisher)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Includi la cartella include/
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Libreria (opzionale, usata solo se vuoi separare)
add_library(webcam_publisher_lib
  src/webcam_publisher.cpp
)
ament_target_dependencies(webcam_publisher_lib
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# Nodo eseguibile
add_executable(webcam_node
  src/webcam_node.cpp
  src/webcam_publisher.cpp
)

ament_target_dependencies(webcam_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(webcam_node
  ${OpenCV_LIBS}
)

# Installazione eseguibili
install(TARGETS
  webcam_node
  DESTINATION lib/${PROJECT_NAME}
)

# (Opzionale) installazione dei file di configurazione/launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
