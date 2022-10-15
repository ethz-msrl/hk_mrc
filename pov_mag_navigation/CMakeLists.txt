cmake_minimum_required(VERSION 3.0.2)
project(pov_mag_navigation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(ps_navi_controller src/ps_navi_controller.cpp)


file(GLOB python_scripts scripts/*.py)
catkin_install_python(PROGRAMS
    ${python_scripts} DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()

cs_export()

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
