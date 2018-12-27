cmake_minimum_required(VERSION 2.8.3)
project(medlab_common)

add_compile_options(-std=c++14)

find_package(catkin
  REQUIRED
  COMPONENTS
    roscpp
)

find_package(Eigen3
  REQUIRED
  NO_MODULE
)

set(${PROJECT_NAME}_SRCS
    src/ctr3_robot.cpp
    src/robotics_math.cpp
    include/${PROJECT_NAME}/medlab_types.h # header only (no .cpp) so needs to be explicitly included
)

if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    include
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  /home/remireaa/Documents/LIBRARIES/MathTools
  /home/remireaa/Documents/LIBRARIES/CannulaKinematics
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
