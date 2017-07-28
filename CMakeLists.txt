cmake_minimum_required(VERSION 2.8.3)
project(giskard_sim)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CMakeToolsHelpers OPTIONAL)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-std=c++11 -g -Wreturn-type)

set(QT_FORMS ${PROJECT_SOURCE_DIR}/ui/simcontrols.ui
             ${PROJECT_SOURCE_DIR}/ui/scenario_info.ui
             ${PROJECT_SOURCE_DIR}/ui/giskardsimpanel.ui
             ${PROJECT_SOURCE_DIR}/ui/urdf.ui
             ${PROJECT_SOURCE_DIR}/ui/poses_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/pose_add_joint_item.ui
             ${PROJECT_SOURCE_DIR}/ui/pose_joint_item.ui
             ${PROJECT_SOURCE_DIR}/ui/controller_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/vector_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/object_info.ui
             ${PROJECT_SOURCE_DIR}/ui/scene_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/create_object_dialog.ui
             ${PROJECT_SOURCE_DIR}/ui/input_scalar_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/input_vector_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/input_rotation_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/input_frame_widget.ui
             ${PROJECT_SOURCE_DIR}/ui/ros_pkg_file_selector.ui)
set(QT_RESS ${PROJECT_SOURCE_DIR}/resources/giskard_sim.qrc)

qt4_wrap_cpp(MOC_FILES
    include/${PROJECT_NAME}/simcontrols.h
    include/${PROJECT_NAME}/scenario_info.h
    include/${PROJECT_NAME}/urdf_info.h
    include/${PROJECT_NAME}/giskardsimpanel.h
    include/${PROJECT_NAME}/pose_tree_widget.h
    include/${PROJECT_NAME}/controller_widget.h
    include/${PROJECT_NAME}/robot_model_display.h
    include/${PROJECT_NAME}/vector_widget.h
    include/${PROJECT_NAME}/object_info_widget.h
    include/${PROJECT_NAME}/color_editor.h
    include/${PROJECT_NAME}/scene_widget.h
    include/${PROJECT_NAME}/line_edit.h
    include/${PROJECT_NAME}/create_object_dialog.h
    include/${PROJECT_NAME}/input_scalar_widget.h
    include/${PROJECT_NAME}/input_vector_widget.h
    include/${PROJECT_NAME}/input_rotation_widget.h
    include/${PROJECT_NAME}/input_frame_widget.h
    include/${PROJECT_NAME}/tf_combo_box.h
    include/${PROJECT_NAME}/pkg_file_combo_box.h
    include/${PROJECT_NAME}/pkg_combo_box.h
    include/${PROJECT_NAME}/ros_pkg_file_selector.h
)

qt4_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt4_add_resources(QT_RESOURCES ${QT_RESS})


set(SRC_FILES
    src/simcontrols.cpp
    src/scenario_info.cpp
    src/urdf_info.cpp
    src/giskardsimpanel.cpp
    src/pose_tree_widget.cpp
    src/controller_widget.cpp
    src/vector_widget.cpp
    src/object_info_widget.cpp
    src/scene_widget.cpp
    src/create_object_dialog.cpp
    src/input_scalar_widget.cpp
    src/input_vector_widget.cpp
    src/input_rotation_widget.cpp
    src/input_frame_widget.cpp
    src/tf_combo_box.cpp
    src/pkg_file_combo_box.cpp
    src/pkg_combo_box.cpp
    src/ros_pkg_file_selector.cpp
    ${MOC_FILES}
)

# everything else
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  giskard_core
  tf
  tf_conversions
  eigen_conversions
  interactive_markers
  iai_naive_kinematics_sim
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #${KDL_LIBRARY}
  CATKIN_DEPENDS geometry_msgs roscpp rviz
  DEPENDS system_lib #orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
#find_package(orocos_kdl REQUIRED)
#link_directories(${orocos_kdl_LIBRARY_DIRS})
#find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Actual simulation components
add_library(${PROJECT_NAME} src/scenario_instance.cpp src/utils.cpp src/controller_runner.cpp src/input_assignments.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# GUI Library
add_library(${PROJECT_NAME}_UI ${SRC_FILES} ${QT_FORMS_HPP} ${QT_RESOURCES})
target_link_libraries(${PROJECT_NAME}_UI ${PROJECT_NAME} ${catkin_LIBRARIES}) #${orocos_kdl_LIBRARIES}
add_dependencies(${PROJECT_NAME}_UI ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(interactionTest src/interactive_test.cpp) # ${SRC_FILES}
target_link_libraries(interactionTest ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(interactionTest ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS 
  ${PROJECT_NAME}_UI
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
