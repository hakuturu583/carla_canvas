cmake_minimum_required(VERSION 3.20)
project(carla_canvas LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# typesafe_carla's C ABI header (typesafe_carla/ffi.h, ABI 5.3 or later).
# The plugin does not link against the library (it gets the functions through
# get_proc at load time), it only needs the declarations. Found, in order:
# TYPESAFE_CARLA_INCLUDE_DIR, the installed typesafe_carla Python package
# (typesafe_carla.paths.include_dir()), a sibling checkout ../typesafe_carla.
# ---------------------------------------------------------------------------
set(TYPESAFE_CARLA_INCLUDE_DIR "$ENV{TYPESAFE_CARLA_INCLUDE_DIR}" CACHE PATH
    "Directory containing typesafe_carla/ffi.h")
if(NOT TYPESAFE_CARLA_INCLUDE_DIR)
  find_package(Python COMPONENTS Interpreter QUIET)
  if(Python_EXECUTABLE)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -c
              "from typesafe_carla import paths; print(paths.include_dir())"
      OUTPUT_VARIABLE _tsc_include OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET
      RESULT_VARIABLE _tsc_result)
    if(_tsc_result EQUAL 0 AND EXISTS "${_tsc_include}/typesafe_carla/ffi.h")
      set(TYPESAFE_CARLA_INCLUDE_DIR "${_tsc_include}")
    endif()
  endif()
endif()
if(NOT TYPESAFE_CARLA_INCLUDE_DIR AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../typesafe_carla/native/include/typesafe_carla/ffi.h")
  set(TYPESAFE_CARLA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../typesafe_carla/native/include")
endif()
if(NOT EXISTS "${TYPESAFE_CARLA_INCLUDE_DIR}/typesafe_carla/ffi.h")
  message(FATAL_ERROR "typesafe_carla/ffi.h not found: install typesafe-carla in the build "
                      "environment or set TYPESAFE_CARLA_INCLUDE_DIR")
endif()
message(STATUS "carla_canvas: typesafe_carla header from ${TYPESAFE_CARLA_INCLUDE_DIR}")

# ---------------------------------------------------------------------------
# gRPC / protobuf / yaml-cpp (Ubuntu: libgrpc++-dev protobuf-compiler-grpc
# libprotobuf-dev protobuf-compiler libyaml-cpp-dev)
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)

set(_proto_dir "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(_proto "${_proto_dir}/carla_canvas/v1/renderer.proto")
set(_gen "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(_gen_srcs
  "${_gen}/carla_canvas/v1/renderer.pb.cc"
  "${_gen}/carla_canvas/v1/renderer.grpc.pb.cc")
add_custom_command(
  OUTPUT ${_gen_srcs}
         "${_gen}/carla_canvas/v1/renderer.pb.h"
         "${_gen}/carla_canvas/v1/renderer.grpc.pb.h"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${_gen}"
  COMMAND protobuf::protoc
          --proto_path "${_proto_dir}"
          --cpp_out "${_gen}"
          --grpc_out "${_gen}"
          --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
          "${_proto}"
  DEPENDS "${_proto}"
  COMMENT "Generating the renderer protocol (C++)")

# ---------------------------------------------------------------------------
# The sensor plugin (loaded by typesafe_carla: TYPESAFE_CARLA_SENSOR_PLUGIN)
# ---------------------------------------------------------------------------
add_library(carla_canvas_plugin MODULE
  native/src/plugin.cpp
  native/src/config.cpp
  ${_gen_srcs})
target_include_directories(carla_canvas_plugin PRIVATE
  "${TYPESAFE_CARLA_INCLUDE_DIR}" "${_gen}" native/src)
target_link_libraries(carla_canvas_plugin PRIVATE
  gRPC::grpc++ protobuf::libprotobuf yaml-cpp Threads::Threads)
set_target_properties(carla_canvas_plugin PROPERTIES
  OUTPUT_NAME carla_canvas_plugin
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(carla_canvas_plugin PRIVATE -Wall -Wextra)
  # Export only tsc_sensor_plugin_init.
  target_link_options(carla_canvas_plugin PRIVATE
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/native/src/exports.map")
endif()

install(TARGETS carla_canvas_plugin LIBRARY DESTINATION carla_canvas/_native
        COMPONENT carla_canvas)

option(CARLA_CANVAS_BUILD_TESTS "Build native tests" ON)
if(CARLA_CANVAS_BUILD_TESTS)
  enable_testing()
  add_executable(test_config native/tests/test_config.cpp native/src/config.cpp)
  target_include_directories(test_config PRIVATE native/src)
  target_link_libraries(test_config PRIVATE protobuf::libprotobuf yaml-cpp)
  add_test(NAME test_config COMMAND test_config "${CMAKE_CURRENT_SOURCE_DIR}/native/tests")
endif()
