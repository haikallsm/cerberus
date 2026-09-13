# cmake_minimum_required(VERSION 3.16)
# project(hawkeyev2)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(mavros_msgs REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(PkgConfig REQUIRED)
# find_package(nlohmann_json REQUIRED)

# # --- GStreamer (webrtcbin untuk WebRTC) -------------------------------------
# # Semua modul di bawah ini didapat dari paket apt resmi Ubuntu 24.04:
# #   sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
# #                     libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \
# #                     gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
# #                     gstreamer1.0-plugins-ugly gstreamer1.0-nice gstreamer1.0-tools
# pkg_check_modules(GST REQUIRED IMPORTED_TARGET
#   gstreamer-1.0
#   gstreamer-app-1.0
#   gstreamer-video-1.0
#   gstreamer-sdp-1.0
#   gstreamer-webrtc-1.0
# )

# # --- libmicrohttpd (HTTP signaling server, padanan aiohttp) ----------------
# #   sudo apt install libmicrohttpd-dev
# pkg_check_modules(MHD REQUIRED IMPORTED_TARGET libmicrohttpd)

# # --- ONNX Runtime ------------------------------------------------------------
# # TIDAK tersedia via apt di Ubuntu -> pakai binary prebuilt resmi (linux-aarch64),
# # tidak perlu build dari source sama sekali:
# #   wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-aarch64-1.19.2.tgz
# #   tar xf onnxruntime-linux-aarch64-1.19.2.tgz
# #   sudo mv onnxruntime-linux-aarch64-1.19.2 /opt/onnxruntime
# set(ONNXRUNTIME_ROOT "/opt/onnxruntime")
# target_include_directories(hawkeyev2 PRIVATE
#     ${ONNXRUNTIME_ROOT}/include
# )

# target_link_directories(hawkeyev2 PRIVATE
#     ${ONNXRUNTIME_ROOT}/lib
# )

# target_link_libraries(hawkeyev2
#     onnxruntime
# )
# set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "Lokasi extract onnxruntime-linux-aarch64-*.tgz")
# find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
#   PATHS ${ONNXRUNTIME_ROOT}/include)
# find_library(ONNXRUNTIME_LIB onnxruntime
#   PATHS ${ONNXRUNTIME_ROOT}/lib)
# if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
#   message(FATAL_ERROR
#     "ONNX Runtime tidak ditemukan di ONNXRUNTIME_ROOT='${ONNXRUNTIME_ROOT}'.\n"
#     "Download dulu (lihat komentar di CMakeLists.txt ini), lalu jalankan ulang:\n"
#     "  colcon build --cmake-args -DONNXRUNTIME_ROOT=/opt/onnxruntime")
# endif()

# add_executable(hawkeyev2 src/vision.cpp)

# target_include_directories(hawkeyev2 PRIVATE
#   ${ONNXRUNTIME_INCLUDE_DIR}
# )

# ament_target_dependencies(hawkeyev2
#   rclcpp
#   sensor_msgs
#   mavros_msgs
# )

# target_link_libraries(hawkeyev2
#   ${OpenCV_LIBS}
#   ${ONNXRUNTIME_LIB}
#   PkgConfig::GST
#   PkgConfig::MHD
#   nlohmann_json::nlohmann_json
# )

# install(TARGETS hawkeyev2
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch models
#   DESTINATION share/${PROJECT_NAME}
#   OPTIONAL)

# ament_package()

cmake_minimum_required(VERSION 3.16)
project(hawkeyev2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json REQUIRED)

# ============================================================
# GStreamer
# ============================================================

pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-sdp-1.0
  gstreamer-webrtc-1.0
)

# ============================================================
# libmicrohttpd
# ============================================================

pkg_check_modules(MHD REQUIRED IMPORTED_TARGET
  libmicrohttpd
)

# ============================================================
# ONNX Runtime
# ============================================================

set(ONNXRUNTIME_ROOT
    "/opt/onnxruntime"
    CACHE PATH
    "Lokasi ONNX Runtime"
)

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS
    ${ONNXRUNTIME_ROOT}/include
  NO_DEFAULT_PATH
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS
    ${ONNXRUNTIME_ROOT}/lib
  NO_DEFAULT_PATH
)

if(NOT ONNXRUNTIME_INCLUDE_DIR)
  message(FATAL_ERROR
    "onnxruntime_cxx_api.h tidak ditemukan di "
    "${ONNXRUNTIME_ROOT}/include"
  )
endif()

if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR
    "libonnxruntime.so tidak ditemukan di "
    "${ONNXRUNTIME_ROOT}/lib"
  )
endif()

message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime library: ${ONNXRUNTIME_LIB}")

# ============================================================
# Executable
# ============================================================

add_executable(hawkeyev2
  src/vision.cpp
)

# ============================================================
# Include directories
# ============================================================

target_include_directories(hawkeyev2 PRIVATE
  ${ONNXRUNTIME_INCLUDE_DIR}
)

# ============================================================
# ROS 2 dependencies
# ============================================================

ament_target_dependencies(hawkeyev2
  rclcpp
  sensor_msgs
  mavros_msgs
)

# ============================================================
# Libraries
# ============================================================

target_link_libraries(hawkeyev2
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIB}
  PkgConfig::GST
  PkgConfig::MHD
  nlohmann_json::nlohmann_json
)

# ============================================================
# Runtime library path
# ============================================================

set_target_properties(hawkeyev2 PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

# ============================================================
# Install semua Python script dari src/
# ============================================================

file(GLOB PYTHON_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.py"
)

install(
  PROGRAMS
    ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install
# ============================================================

install(TARGETS hawkeyev2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()