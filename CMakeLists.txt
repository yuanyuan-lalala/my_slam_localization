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

include_directories(${CMAKE_SOURCE_DIR}/include) 

find_package(glog REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})



list( APPEND CMAKE_MODULE_PATH /home/yuanyuan/my_slam_localization/third_party/g2o/cmake_modules ) 
set(G2O_ROOT /usr/local/include/g2o) 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})


file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/app/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

# message(WARNING ${ALL_SRCS})

list(REMOVE_ITEM ALL_SRCS ${REMOVE_CMAKE})

add_executable(run_imu_integration_node src/app/run_imu_integration_node.cpp ${ALL_SRCS})
target_link_libraries(run_imu_integration_node glog::glog ${PCL_LIBRARIES} ${OpenCV_LIBS})   

add_executable(kd_tree_test_node src/app/kd_tree_test_node.cpp ${ALL_SRCS})
target_link_libraries(kd_tree_test_node glog::glog ${PCL_LIBRARIES} ${G2O_LIBRARIES} ${OpenCV_LIBS} )   
target_include_directories(kd_tree_test_node PUBLIC /home/yuanyuan/my_slam_localization/third_party/oneTBB-2019_U8/include/tbb
)
target_link_directories(kd_tree_test_node  PUBLIC /home/yuanyuan/my_slam_localization/third_party/oneTBB-2019_U8/build/linux_intel64_gcc_cc11.4.0_libc2.31_kernel5.15.0_release
)


# message(WARNING "This is a warning message ${GLOG_LIBRARIES}" ) 