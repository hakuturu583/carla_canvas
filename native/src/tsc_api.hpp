// The typesafe_carla C ABI functions the plugin uses, resolved by name through
// the get_proc callback the library passes to tsc_sensor_plugin_init: the
// plugin does not link against libtypesafe_carla_ffi, so one build serves
// every CARLA ref's library of an installation.
#pragma once

#include <typesafe_carla/ffi.h>

#include <stdexcept>
#include <string>

namespace carla_canvas {

#define CARLA_CANVAS_TSC_FUNCTIONS(X) \
  X(tsc_last_error_message)           \
  X(tsc_handle_release)               \
  X(tsc_string_free)                  \
  X(tsc_string_list_free)             \
  X(tsc_actor_get_id)                 \
  X(tsc_actor_get_type_id)            \
  X(tsc_actor_get_attributes)         \
  X(tsc_actor_get_parent)             \
  X(tsc_actor_get_bounding_box)       \
  X(tsc_actor_get_world)              \
  X(tsc_world_get_id)                 \
  X(tsc_world_get_map)                \
  X(tsc_map_get_name)                 \
  X(tsc_map_to_opendrive)             \
  X(tsc_world_get_actor)              \
  X(tsc_world_get_snapshot)           \
  X(tsc_world_wait_for_tick)          \
  X(tsc_world_snapshot_get_id)        \
  X(tsc_world_snapshot_get_timestamp) \
  X(tsc_world_snapshot_size)          \
  X(tsc_world_snapshot_get)           \
  X(tsc_sensor_source_install)        \
  X(tsc_sensor_source_push)           \
  X(tsc_sensor_data_new_image)        \
  X(tsc_sensor_data_new_lidar)        \
  X(tsc_sensor_data_new_semantic_lidar)

struct Api {
#define CARLA_CANVAS_DECLARE(name) decltype(&::name) name = nullptr;
  CARLA_CANVAS_TSC_FUNCTIONS(CARLA_CANVAS_DECLARE)
#undef CARLA_CANVAS_DECLARE

  // Throws std::runtime_error naming the first function get_proc lacks.
  static Api load(void *(*get_proc)(const char *)) {
    Api api;
#define CARLA_CANVAS_LOAD(name)                                                  \
  {                                                                             \
    void *proc = get_proc(#name);                                               \
    if (proc == nullptr) throw std::runtime_error("typesafe_carla lacks " #name); \
    api.name = reinterpret_cast<decltype(&::name)>(proc);                       \
  }
    CARLA_CANVAS_TSC_FUNCTIONS(CARLA_CANVAS_LOAD)
#undef CARLA_CANVAS_LOAD
    return api;
  }

  // Throws with the library's message unless status is TSC_OK.
  void check(tsc_status_t status, const char *what) const {
    if (status != TSC_OK) {
      throw std::runtime_error(std::string(what) + ": " + tsc_last_error_message());
    }
  }
};

// Releases a handle at the end of a scope.
template <typename T>
class Handle {
 public:
  Handle(const Api &api, T *h = nullptr) : api_(&api), h_(h) {}
  ~Handle() { reset(); }
  Handle(const Handle &) = delete;
  Handle &operator=(const Handle &) = delete;
  T *get() const { return h_; }
  T **out() {
    reset();
    return &h_;
  }
  void reset() {
    if (h_ != nullptr) api_->tsc_handle_release(reinterpret_cast<tsc_handle_t *>(h_));
    h_ = nullptr;
  }
  explicit operator bool() const { return h_ != nullptr; }

 private:
  const Api *api_;
  T *h_;
};

}  // namespace carla_canvas
