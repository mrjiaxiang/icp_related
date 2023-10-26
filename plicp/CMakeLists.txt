cmake_minimum_required(VERSION 2.8.3)
project(plicp)

set(CMAKE_CXX_STANDARD 11)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(include)


add_executable(plicp src/main.cpp src/brute_force_cpu.cpp src/kdtree_cpu.cpp src/iterative_closest_point.cpp src/nabo.cpp src/plicp.cpp
        src/plicp_debug.cpp src/pose2d.cpp src/Solve.cpp)
target_link_libraries(plicp ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} glog gflags)