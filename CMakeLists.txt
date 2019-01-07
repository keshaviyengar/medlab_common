cmake_minimum_required(VERSION 3.0)
project(medlab_common)

#add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED NO_MODULE) # sets the ${EIGEN3_INCLUDE_DIR} cmake variable
find_package(Boost REQUIRED) # needed to build cannula_kinematics library

# path to Cannula Kinematics library
set(CannulaKinematics_DIR
  /home/remireaa/Documents/LIBRARIES/CannulaKinematics_shared
)


if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    include
    ${CannulaKinematics_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES
    robotics_math
    ctr3_robot
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## robotics_math library
add_library(robotics_math src/robotics_math.cpp)

## cannula_kinematic library
#add_library(cannula_kinematics
#  ${CannulaKinematics_DIR}/src/DynamicMemoryBlock.cpp
#  ${CannulaKinematics_DIR}/src/ReferenceCannulaKinematics.cpp
#  ${CannulaKinematics_DIR}/src/Rk8_Coeffs.cpp
#  ${CannulaKinematics_DIR}/src/TaggedInterval.cpp
#)

## ctr3_robot library
#link_directories(${CannulaKinematics_DIR}/build)
add_library(ctr3_robot src/ctr3_robot.cpp)
target_link_libraries(ctr3_robot ${CannulaKinematics_DIR}/build/libcannula_kinematics.so) # link against cannula_kinematics


#############
## Install ##
#############

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
