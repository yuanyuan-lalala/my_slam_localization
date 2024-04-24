cmake_minimum_required(VERSION 3.5)
project(my_slam_in_auto_driving)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


add_definitions("-DCATKIN_ENABLE_TESTING=0")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/kd_tree)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/run_imu_integration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/utils)

find_package(glog REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})



file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/app/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
# list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
message(WARNING ${ALL_SRCS})

list(REMOVE_ITEM ALL_SRCS ${REMOVE_CMAKE})
add_executable(run_imu_integration_node src/app/run_imu_integration_node.cpp ${ALL_SRCS})

target_link_libraries(run_imu_integration_node glog::glog ${PCL_LIBRARIES} )   

add_executable(kd_tree_test_node src/app/kd_tree_test_node.cpp ${ALL_SRCS})
target_link_libraries(kd_tree_test_node glog::glog ${PCL_LIBRARIES} )   

# message(WARNING "This is a warning message ${GLOG_LIBRARIES}" ) 