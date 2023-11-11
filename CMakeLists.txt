cmake_minimum_required(VERSION 3.8)
project(TF_Tool)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

include_directories(
    src
)

add_executable(${PROJECT_NAME}
    src/main.cpp

    src/node/MonitorNode.cpp
    src/ui/models/FrameTreeModel.cpp
    
    src/ui/MainWindow.cpp
    src/ui/MainWindow.ui
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    tf2_msgs
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
)

ament_package()
