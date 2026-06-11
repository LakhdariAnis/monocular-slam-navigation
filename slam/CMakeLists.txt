cmake_minimum_required(VERSION 3.10)
project(slam_reader)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS
    "-Wno-error -Wall -O3 -march=native -DCOMPILEDWITHC11 -DHAVE_EIGEN -DHAVE_EPOXY -DHAVE_PYTHON -DPANGO_DEFAULT_WIN_URI=\\\"wayland\\\" -D_LINUX_ -DEIGEN_DONT_VECTORIZE -Wno-error=ignored-qualifiers -Wno-error=sign-compare -Wno-error=deprecated-copy -Wno-error=missing-field-initializers -Wno-error=pessimizing-move"
)

set(ORB_SLAM3_ROOT "$ENV{HOME}/autonomous_car/ORB_SLAM3")

find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)

set(EIGEN3_INCLUDE_DIR "/usr/local/eigen34/include/eigen3")

option(WITH_ZMQ   "Publish pose over ZMQ port 5557" OFF)
option(WITH_ARUCO "Detect ArUco markers" ON)

include_directories(
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
    ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2
    ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/DBoW2
    ${ORB_SLAM3_ROOT}/Thirdparty/g2o
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(slam_reader slam_reader.cc)

target_link_libraries(slam_reader
    ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    -lpthread
    -lrt
)

if(WITH_ZMQ)
    target_compile_definitions(slam_reader PRIVATE WITH_ZMQ)
    target_link_libraries(slam_reader zmq)
endif()

if(WITH_ARUCO)
    target_compile_definitions(slam_reader PRIVATE WITH_ARUCO)
endif()
