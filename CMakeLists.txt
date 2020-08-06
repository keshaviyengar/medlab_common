cmake_minimum_required(VERSION 3.0)
project(medlab_common)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED NO_MODULE) # sets the ${EIGEN3_INCLUDE_DIR} cmake variable



# path to Cannula Kinematics library
set(KINEMATICS_DIR
  /home/keshav/catkin_ws/src/vanderbiltmedlab/CannulaKinematics
)

set(KINEMATICS_INCLUDE_DIRS
  ${KINEMATICS_DIR}/include
)

find_library(
  KINEMATICS_LIBRARIES
  NAMES
    cannula_kinematics
  PATHS
    ${KINEMATICS_DIR}/build
)

#if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
#  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
#endif()

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    include
    ${KINEMATICS_INCLUDE_DIRS}
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
    ctr3_robot_viz
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

## uncomment if building cannula_kinematics library
#find_package(Boost REQUIRED)
#add_library(cannula_kinematics
#  ${CannulaKinematics_DIR}/src/DynamicMemoryBlock.cpp
#  ${CannulaKinematics_DIR}/src/ReferenceCannulaKinematics.cpp
#  ${CannulaKinematics_DIR}/src/Rk8_Coeffs.cpp
#  ${CannulaKinematics_DIR}/src/TaggedInterval.cpp
#)

## ctr3_robot library
link_directories(${CannulaKinematics_DIR}/build)

add_library(ctr3_robot src/ctr3_robot.cpp)
target_link_libraries(ctr3_robot robotics_math ${KINEMATICS_LIBRARIES}) # link against cannula_kinematics

add_library(ctr3_robot_viz src/ctr3_robot_viz.cpp)
target_link_libraries(ctr3_robot_viz ${KINEMATICS_LIBRARIES}) # link against cannula_kinematics

add_executable(main src/main.cpp)
target_link_libraries(main robotics_math ctr3_robot ctr3_robot_viz ${catkin_LIBRARIES} ${KINEMATICS_LIBRARIES})


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
