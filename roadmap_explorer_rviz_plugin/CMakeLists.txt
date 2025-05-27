## BEGIN_TUTORIAL
## This CMakeLists.txt file for roadmap_explorer_rviz_plugin builds the
## TeleopPanel, ImuDisplay, and PlantFlagTool tutorials.

cmake_minimum_required(VERSION 3.8)
project(roadmap_explorer_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roadmap_explorer_msgs REQUIRED)

set(dependencies
ament_cmake
geometry_msgs
pluginlib
rclcpp
rviz_common
rviz_rendering
sensor_msgs
roadmap_explorer_msgs
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/panel.cpp
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "roadmap_explorer_rviz_plugin", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``. We also add the needed dependencies.
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS})

## Here we export the plugins and meshes so they can be found by RViz at runtime.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
# register_rviz_ogre_media_exports(DIRECTORIES "roadmap_explorer_rviz_plugin/media")

## END_TUTORIAL

ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY roadmap_explorer_rviz_plugin/media/
#   DESTINATION share/${PROJECT_NAME}/media)


ament_package()
