cmake_minimum_required(VERSION 2.8.3)
project(rail_cal_simulation)

add_compile_options(-Wall -Wextra -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Ceres REQUIRED)
#find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${CERES_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    CERES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/rail_cal_simulation/camera_model.cpp
  src/rail_cal_simulation/dot_grid.cpp
  src/rail_cal_simulation/random.cpp
  src/observation_creator.cpp
  src/rail_cal_simulation/pnp.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME rail_cal_sim PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
