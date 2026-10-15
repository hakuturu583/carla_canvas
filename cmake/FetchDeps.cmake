# Builds the sensor plugin's C++ dependencies from source (CARLA_CANVAS_FETCH_DEPS):
# what the published wheels do, and what a source build without system packages
# (apt) uses. Everything is static and position independent, linked into the
# plugin, whose only exported symbol is tsc_sensor_plugin_init: it cannot clash
# with another gRPC or protobuf in the program's process.
#
#   carla_canvas_fetch_grpc()       gRPC (with its protobuf, abseil, BoringSSL,
#                                   c-ares, RE2, zlib); targets grpc++,
#                                   libprotobuf, protoc, grpc_cpp_plugin
#   carla_canvas_fetch_yaml_cpp()   yaml-cpp; target yaml-cpp
#   carla_canvas_fetch_tsc_header(<out>)
#                                   typesafe_carla's C ABI header, pinned; <out>
#                                   is the include directory

include_guard(GLOBAL)
include(FetchContent)

set(CARLA_CANVAS_GRPC_TAG "v1.84.0" CACHE STRING "gRPC release built by CARLA_CANVAS_FETCH_DEPS")
set(CARLA_CANVAS_YAML_CPP_TAG "yaml-cpp-0.9.0" CACHE STRING
    "yaml-cpp release built by CARLA_CANVAS_FETCH_DEPS")
# typesafe_carla's ffi.h with the sensor source C ABI 5.3 (hakuturu583/typesafe_carla#119).
# The plugin resolves the functions at load time (get_proc) and checks the ABI,
# so it works with any later 5.x library.
set(_tsc_header_commit "e978acdc099519e56e0d01d49da3097564d58476")
set(_tsc_header_sha256 "882b172c9481428d045a207ee5658cdcc2504714b096df24719f818c81c15ca7")

macro(_carla_canvas_require_cmake_328)
  if(CMAKE_VERSION VERSION_LESS 3.28)
    message(FATAL_ERROR "CARLA_CANVAS_FETCH_DEPS needs CMake 3.28 or later "
                        "(pip install cmake), found ${CMAKE_VERSION}")
  endif()
endmacro()

macro(carla_canvas_fetch_grpc)
  _carla_canvas_require_cmake_328()
  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_TESTING OFF)
  set(ABSL_PROPAGATE_CXX_STD ON)
  set(ABSL_ENABLE_INSTALL OFF)
  set(protobuf_BUILD_TESTS OFF)
  set(protobuf_INSTALL OFF)
  set(utf8_range_ENABLE_INSTALL OFF)
  set(gRPC_BUILD_TESTS OFF)
  set(gRPC_INSTALL OFF)
  set(gRPC_BUILD_CSHARP_EXT OFF)
  foreach(_lang CSHARP NODE OBJECTIVE_C PHP PYTHON RUBY)
    set(gRPC_BUILD_GRPC_${_lang}_PLUGIN OFF)
  endforeach()
  set(RE2_BUILD_TESTING OFF)
  set(CARES_BUILD_TOOLS OFF)
  set(CARES_SHARED OFF)
  set(CARES_STATIC ON)
  # Some bundled projects still declare minimum versions CMake 4 refuses.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  message(STATUS "carla_canvas: building gRPC ${CARLA_CANVAS_GRPC_TAG} from source")
  FetchContent_Declare(grpc
    GIT_REPOSITORY https://github.com/grpc/grpc.git
    GIT_TAG ${CARLA_CANVAS_GRPC_TAG}
    GIT_SHALLOW TRUE
    GIT_SUBMODULES
      third_party/abseil-cpp third_party/boringssl-with-bazel third_party/cares/cares
      third_party/grpc-proto third_party/protobuf third_party/re2 third_party/zlib
    GIT_SUBMODULES_RECURSE FALSE
    EXCLUDE_FROM_ALL
    SYSTEM)
  FetchContent_MakeAvailable(grpc)
endmacro()

macro(carla_canvas_fetch_yaml_cpp)
  _carla_canvas_require_cmake_328()
  set(YAML_CPP_BUILD_TESTS OFF)
  set(YAML_CPP_BUILD_TOOLS OFF)
  set(YAML_CPP_BUILD_CONTRIB OFF)
  set(YAML_CPP_INSTALL OFF)
  set(YAML_BUILD_SHARED_LIBS OFF)
  message(STATUS "carla_canvas: building yaml-cpp ${CARLA_CANVAS_YAML_CPP_TAG} from source")
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG ${CARLA_CANVAS_YAML_CPP_TAG}
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL
    SYSTEM)
  FetchContent_MakeAvailable(yaml-cpp)
endmacro()

function(carla_canvas_fetch_tsc_header out)
  set(_dir "${CMAKE_BINARY_DIR}/_deps/typesafe_carla-include")
  set(_file "${_dir}/typesafe_carla/ffi.h")
  if(NOT EXISTS "${_file}")
    message(STATUS "carla_canvas: downloading typesafe_carla's ffi.h (${_tsc_header_commit})")
    file(DOWNLOAD
      "https://raw.githubusercontent.com/hakuturu583/typesafe_carla/${_tsc_header_commit}/native/include/typesafe_carla/ffi.h"
      "${_file}"
      EXPECTED_HASH SHA256=${_tsc_header_sha256}
      TLS_VERIFY ON
      STATUS _status)
    list(GET _status 0 _code)
    if(NOT _code EQUAL 0)
      file(REMOVE "${_file}")
      message(FATAL_ERROR "carla_canvas: cannot download typesafe_carla's ffi.h: ${_status}")
    endif()
  endif()
  set(${out} "${_dir}" PARENT_SCOPE)
endfunction()
