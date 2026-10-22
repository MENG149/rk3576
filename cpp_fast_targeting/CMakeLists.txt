cmake_minimum_required(VERSION 3.16)

project(rkfast_targeting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RKFAST_USE_RGA "Enable RGA hardware acceleration" ON)

find_package(Threads REQUIRED)

set(RKFAST_OPENCV_TARGET "")
set(RKFAST_OPENCV_INCLUDE_DIRS "")
set(RKFAST_OPENCV_LIBS "")

find_package(OpenCV QUIET COMPONENTS core imgproc video videoio highgui)
if(OpenCV_FOUND)
  set(RKFAST_OPENCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
  set(RKFAST_OPENCV_LIBS ${OpenCV_LIBS})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OPENCV4 REQUIRED IMPORTED_TARGET opencv4)
  set(RKFAST_OPENCV_TARGET PkgConfig::OPENCV4)
  set(RKFAST_OPENCV_INCLUDE_DIRS ${OPENCV4_INCLUDE_DIRS})
  set(RKFAST_OPENCV_LIBS ${OPENCV4_LIBRARIES})
endif()

find_path(
  RKNN_API_INCLUDE_DIR
  NAMES rknn_api.h
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn/include
    /usr/include
    /usr/local/include
    /usr/include/rknn
    /usr/include/librknn_api
    /usr/include/rknpu2
)

find_library(
  RKNN_API_LIB
  NAMES rknnrt librknnrt
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn/lib
    /usr/lib
    /usr/local/lib
    /usr/lib64
    /usr/local/lib64
    /usr/lib/aarch64-linux-gnu
)

if(NOT RKNN_API_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find rknn_api.h. Set RKNN_API_INCLUDE_DIR.")
endif()

if(NOT RKNN_API_LIB)
  message(FATAL_ERROR "Could not find librknnrt. Set RKNN_API_LIB.")
endif()

if(RKFAST_USE_RGA)
  find_path(
    RGA_INCLUDE_DIR
    NAMES rga/RgaApi.h rga/im2d.h
    PATHS
      /usr/include
      /usr/local/include
  )
  find_library(
    RGA_LIB
    NAMES rga librga
    PATHS
      /usr/lib
      /usr/local/lib
      /usr/lib/aarch64-linux-gnu
  )
  if(RGA_INCLUDE_DIR AND RGA_LIB)
    message(STATUS "RGA found: ${RGA_LIB}")
    set(RKFAST_HAS_RGA TRUE)
  else()
    message(WARNING "RGA requested but not found, disabling RGA support")
    set(RKFAST_HAS_RGA FALSE)
  endif()
else()
  set(RKFAST_HAS_RGA FALSE)
endif()

include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES ${RKNN_API_INCLUDE_DIR})
set(CMAKE_REQUIRED_LIBRARIES ${RKNN_API_LIB})
check_cxx_source_compiles(
  "
  #include <rknn_api.h>
  int main() {
    rknn_context ctx = 0;
    return static_cast<int>(rknn_set_core_mask(ctx, RKNN_NPU_CORE_0_1));
  }
  "
  RKFAST_RKNN_HAS_CORE_MASK
)
unset(CMAKE_REQUIRED_LIBRARIES)
unset(CMAKE_REQUIRED_INCLUDES)
if(RKFAST_RKNN_HAS_CORE_MASK)
  message(STATUS "RKNN core mask API available — --core-mask will work at runtime.")
else()
  message(STATUS "RKNN core mask API not found in header — --core-mask will be ignored at runtime.")
endif()

set(RKFAST_SOURCES
  src/aim_writer.cpp
  src/camera_source.cpp
  src/config.cpp
  src/main.cpp
  src/mjpeg_server.cpp
  src/rect_refiner.cpp
  src/rga_preprocess.cpp
  src/rknn_detector.cpp
  src/target_plane.cpp
  src/thread_affinity.cpp
  src/tracker.cpp
)

add_executable(rkfast_target ${RKFAST_SOURCES})

target_include_directories(
  rkfast_target
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RKFAST_OPENCV_INCLUDE_DIRS}
    ${RKNN_API_INCLUDE_DIR}
    $<$<BOOL:${RKFAST_HAS_RGA}>:${RGA_INCLUDE_DIR}>
)

target_link_libraries(
  rkfast_target
  PRIVATE
    ${RKNN_API_LIB}
    Threads::Threads
    $<$<BOOL:${RKFAST_HAS_RGA}>:${RGA_LIB}>
)

if(RKFAST_OPENCV_TARGET)
  target_link_libraries(rkfast_target PRIVATE ${RKFAST_OPENCV_TARGET})
else()
  target_link_libraries(rkfast_target PRIVATE ${RKFAST_OPENCV_LIBS})
endif()

target_compile_definitions(
  rkfast_target
  PRIVATE
    $<$<BOOL:${RKFAST_HAS_RGA}>:RKFAST_USE_RGA=1>
    $<$<BOOL:${RKFAST_RKNN_HAS_CORE_MASK}>:RKFAST_RKNN_HAS_CORE_MASK=1>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(
    rkfast_target
    PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      $<$<CONFIG:Release>:-O3>
      # NOTE: -ffast-math is intentionally NOT enabled. It implies
      # -ffinite-math-only, which lets the compiler assume NaN/Inf never occur
      # and optimize away the std::isfinite() guards that the decode, tracker
      # and geometry paths rely on to fail closed on bad model/camera output.
      # The two flags below are safe: they only affect errno/trap behavior,
      # not the value semantics of NaN/Inf, so isfinite() still works.
      $<$<CONFIG:Release>:-fno-math-errno>
      $<$<CONFIG:Release>:-fno-trapping-math>
  )
endif()

# Diagnostic tool: run the real detector+refiner pipeline against a single
# static image (no camera loop needed). Not part of the runtime product.
add_executable(image_probe
  tools/image_probe.cpp
  src/config.cpp
  src/rect_refiner.cpp
  src/rga_preprocess.cpp
  src/rknn_detector.cpp
  src/thread_affinity.cpp
)
target_include_directories(
  image_probe
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RKFAST_OPENCV_INCLUDE_DIRS}
    ${RKNN_API_INCLUDE_DIR}
)
target_link_libraries(
  image_probe
  PRIVATE
    ${RKNN_API_LIB}
    Threads::Threads
)
if(RKFAST_OPENCV_TARGET)
  target_link_libraries(image_probe PRIVATE ${RKFAST_OPENCV_TARGET})
else()
  target_link_libraries(image_probe PRIVATE ${RKFAST_OPENCV_LIBS})
endif()
target_compile_definitions(
  image_probe
  PRIVATE
    $<$<BOOL:${RKFAST_RKNN_HAS_CORE_MASK}>:RKFAST_RKNN_HAS_CORE_MASK=1>
)

add_executable(input_mode_compare
  tools/input_mode_compare.cpp
  src/config.cpp
  src/rga_preprocess.cpp
  src/rknn_detector.cpp
  src/thread_affinity.cpp
)
target_include_directories(
  input_mode_compare
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RKFAST_OPENCV_INCLUDE_DIRS}
    ${RKNN_API_INCLUDE_DIR}
)
target_link_libraries(
  input_mode_compare
  PRIVATE
    ${RKNN_API_LIB}
    Threads::Threads
)
if(RKFAST_OPENCV_TARGET)
  target_link_libraries(input_mode_compare PRIVATE ${RKFAST_OPENCV_TARGET})
else()
  target_link_libraries(input_mode_compare PRIVATE ${RKFAST_OPENCV_LIBS})
endif()
target_compile_definitions(
  input_mode_compare
  PRIVATE
    $<$<BOOL:${RKFAST_RKNN_HAS_CORE_MASK}>:RKFAST_RKNN_HAS_CORE_MASK=1>
)
