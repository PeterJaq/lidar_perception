cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "-std=c++14 -O3 -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
            sensor_msgs
            std_msgs
            roscpp
            rosbag
            lidar_perception_msgs
            visualization_msgs
            tf)

find_package(PCL REQUIRED)

# generate_messages(
#    DEPENDENCIES
#    std_msgs
# )

set(ALL_TARGET_LIBRARIES "")
set(TENSORRT_TARGET_LIBRARIES /usr/lib/x86_64-linux-gnu/libnvinfer.so /usr/lib/x86_64-linux-gnu/libnvonnxparser.so)

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/geographic.cmake)
include(cmake/protobuf.cmake)
include(cmake/PCL.cmake)
# include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
include(cmake/pointpillarsRT.cmake)

catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE CUDA_SRCS "*.cu")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ${ALL_SRCS} ${NODE_SRCS})

cuda_add_executable(lidar_objects_detection_node src/apps/lidar_objects_detection_node.cpp ${ALL_SRCS} ${CUDA_SRCS})
target_link_libraries(lidar_objects_detection_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} ${TENSORRT_TARGET_LIBRARIES})

install(TARGETS 
    lidar_objects_detection_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)
