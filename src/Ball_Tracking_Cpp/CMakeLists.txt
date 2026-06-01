cmake_minimum_required(VERSION 3.10)
project(ball_tracking_cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Using ccache")
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dv-processing REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(TBB REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

pkg_check_modules(RAYLIB REQUIRED raylib)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(DBSCAN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/DBSCAN)

add_executable(talker
  src/publisher_member_function.cpp
  src/EventWriter.cpp
  src/Gui.cpp
  src/Camera.cpp
  src/BallTracker.cpp
)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  dv-processing
  OpenCV
)

target_include_directories(talker PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Ball_Tracking_Cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${DBSCAN_DIR}
  ${RAYLIB_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(talker
  ${OpenCV_LIBS}
  Eigen3::Eigen
  fmt::fmt
  ${RAYLIB_LINK_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  ${HDF5_LIBRARIES}
  TBB::tbb
  GL
  m
  pthread
  dl
  rt
  X11
)

target_compile_options(talker PRIVATE
  -w
  -O3
  -march=native
)

add_executable(ball_tracker_h5_benchmark
  src/ball_tracker_h5_benchmark.cpp
  src/TraceBenchmark.cpp
  src/EventWriter.cpp
)

ament_target_dependencies(ball_tracker_h5_benchmark
  dv-processing
  OpenCV
)

target_include_directories(ball_tracker_h5_benchmark PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Ball_Tracking_Cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../.deps/prefix/include
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/eigen3
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(ball_tracker_h5_benchmark
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${HDF5_LIBRARIES}
)

target_compile_options(ball_tracker_h5_benchmark PRIVATE
  -O3
)

set_target_properties(ball_tracker_h5_benchmark PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/..
)

install(TARGETS
  talker
  ball_tracker_h5_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
