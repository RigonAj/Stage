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
find_package(ur3e_catch_msgs REQUIRED)
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
  src/SequenceDataset.cpp
  src/TraceAnalysis.cpp
  src/TraceRuntime.cpp
  src/Camera.cpp
  src/BallTracker.cpp
)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  ur3e_catch_msgs
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

# Link-time optimization: the 1 kHz timer path crosses translation units
# (publisher -> Camera -> Gui) on every tick; LTO lets the compiler inline
# across them.
include(CheckIPOSupported)
check_ipo_supported(RESULT LTO_SUPPORTED OUTPUT LTO_CHECK_OUTPUT)
if(LTO_SUPPORTED)
  set_target_properties(talker PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message(STATUS "LTO not supported: ${LTO_CHECK_OUTPUT}")
endif()

# Headless offline comparison of the two 3D estimators against the simulated
# ground truth. No rclcpp, no Gui: it drives TraceRuntime and BallTracker
# directly. raylib is still linked because TraceAnalysis uses its Vector2/
# Vector3/Color types, but no window is ever created.
add_executable(method_comparison_benchmark
  src/method_comparison_benchmark.cpp
  src/BallTracker.cpp
  src/Camera.cpp
  src/EventWriter.cpp
  src/SequenceDataset.cpp
  src/TraceAnalysis.cpp
  src/TraceRuntime.cpp
)

ament_target_dependencies(method_comparison_benchmark
  dv-processing
  OpenCV
)

target_include_directories(method_comparison_benchmark PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Ball_Tracking_Cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${DBSCAN_DIR}
  ${RAYLIB_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(method_comparison_benchmark
  ${OpenCV_LIBS}
  Eigen3::Eigen
  fmt::fmt
  ${RAYLIB_LINK_LIBRARIES}
  ${HDF5_LIBRARIES}
  TBB::tbb
  m
  pthread
)

target_compile_options(method_comparison_benchmark PRIVATE
  -w
  -O3
)

install(TARGETS
  talker
  method_comparison_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
