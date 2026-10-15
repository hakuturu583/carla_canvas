// The carla_canvas configuration file (YAML), named by CARLA_CANVAS_CONFIG.
// The plugin reads the client side (which sensors to take over, where the
// renderer is); the renderer service reads the same file (docs/configuration.md).
#pragma once

#include <google/protobuf/struct.pb.h>

#include <string>
#include <vector>

namespace carla_canvas {

// A rule of `sensors:`: a sensor is taken over when its blueprint id matches
// `type` and, if given, its role_name matches `role_name` (glob patterns).
struct SensorRule {
  std::string type;
  std::string role_name;  // empty: any
};

struct Config {
  std::string path;  // the file read
  std::string endpoint = "localhost:50051";
  // Run when nothing answers at `endpoint` ("{config}" is replaced by `path`).
  std::vector<std::string> command;
  double startup_timeout = 60.0;  // seconds to wait for a started renderer
  double timeout = 30.0;  // per renderer call, seconds
  bool async_render = true;  // also render ticks the program did not make (asynchronous mode)
  std::vector<SensorRule> sensors;
  google::protobuf::Struct renderer;  // the whole `renderer:` section, sent with LoadWorld
};

// Parses the file; throws std::runtime_error with the path and the problem.
Config load_config(const std::string &path);

// fnmatch-style glob ('*', '?', '[...]').
bool glob_match(const std::string &pattern, const std::string &text);

// Whether the rules take over a sensor of this blueprint and role name.
bool claims_sensor(const Config &config, const std::string &type_id, const std::string &role_name);

}  // namespace carla_canvas
