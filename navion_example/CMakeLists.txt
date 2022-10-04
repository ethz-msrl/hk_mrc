cmake_minimum_required(VERSION 3.0.2)
project(navion_example)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_install()  

cs_export()

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
