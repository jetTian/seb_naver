cmake_minimum_required(VERSION 3.5.1)

set(ENABLE_CUDA true)

if(ENABLE_CUDA)
  # set(CUDA_TOOLKIT_ROOT_DIR "/home/xulong/cuda/cuda-11.7")
  project(terrain_analyzer CXX CUDA)
  add_definitions(-DENABLE_CUDA)
else(ENABLE_CUDA)
  project(terrain_analyzer)
endif(ENABLE_CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(
  CATKIN_PACKAGE_DEPENDENCIES
    eigen_conversions
    se2_grid_core
    se2_grid_ros
    se2_grid_msgs
    message_filters
    pcl_ros
    roscpp
    cv_bridge
    sensor_msgs
    std_msgs
    geometry_msgs
    tf
    tf_conversions
    pluginlib
    message_runtime
)

find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    OpenCV
)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

if(ENABLE_CUDA)

  set(CUDA_SEPARABLE_COMPILATION ON)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math;)
  set(CUDA_NVCC_FLAGS
        # Jetson Orin NX
        -gencode arch=compute_87,code=sm_87;
        # RTX 2060
        # -gencode arch=compute_75,code=sm_75;
  )
  set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

  # find_package(CUDA REQUIRED)

  add_library(${PROJECT_NAME}_library
    src/TerrainAnalyzer.cpp
  )

  add_library(cuda_computer src/TerrainBank.cu)
  target_link_libraries(cuda_computer 
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
  )

  target_link_libraries(${PROJECT_NAME}_library
    ${catkin_LIBRARIES}
    cuda_computer
  )

else(ENABLE_CUDA)

  add_library(${PROJECT_NAME}_library
    src/TerrainAnalyzer.cpp
  )

  target_link_libraries(${PROJECT_NAME}_library
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
  )

endif(ENABLE_CUDA)

##############
# Executable #
##############

add_executable(${PROJECT_NAME}
  src/terrain_analyzer_node.cpp
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

# plugin
add_library(post_processor_plugin
  src/post_processors.cpp
)

target_include_directories(post_processor_plugin
  SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(post_processor_plugin
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# install
install(
  TARGETS 
    ${PROJECT_NAME}
    ${PROJECT_NAME}_library
    post_processor_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION 
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES post_processors.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY 
    params
    rviz
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
