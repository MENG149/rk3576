# Host-side tests for NPU-independent logic (coords, tracker, refiner).
# Build standalone (does NOT require RKNN/RGA):
#   cmake -S cpp_fast_targeting/tests -B build-tests
#   cmake --build build-tests && ./build-tests/rkfast_tests
cmake_minimum_required(VERSION 3.16)
project(rkfast_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(RKFAST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(OpenCV QUIET COMPONENTS core imgproc)
if(NOT OpenCV_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OPENCV4 REQUIRED IMPORTED_TARGET opencv4)
endif()

add_executable(rkfast_tests
  rkfast_tests.cpp
  ${RKFAST_ROOT}/src/config.cpp
  ${RKFAST_ROOT}/src/rect_refiner.cpp
  ${RKFAST_ROOT}/src/tracker.cpp
)

target_include_directories(rkfast_tests PRIVATE ${RKFAST_ROOT}/include)

if(OpenCV_FOUND)
  target_include_directories(rkfast_tests PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(rkfast_tests PRIVATE ${OpenCV_LIBS})
else()
  target_link_libraries(rkfast_tests PRIVATE PkgConfig::OPENCV4)
endif()

enable_testing()
add_test(NAME rkfast_tests COMMAND rkfast_tests)
