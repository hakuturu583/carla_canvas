#include "config.hpp"

#include <fnmatch.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace carla_canvas {
namespace {

google::protobuf::Value to_value(const YAML::Node &node);

google::protobuf::Struct to_struct(const YAML::Node &node) {
  google::protobuf::Struct s;
  for (const auto &item : node) {
    (*s.mutable_fields())[item.first.as<std::string>()] = to_value(item.second);
  }
  return s;
}

// YAML scalars keep YAML's types: bool, number, null, else string.
google::protobuf::Value to_value(const YAML::Node &node) {
  google::protobuf::Value v;
  switch (node.Type()) {
    case YAML::NodeType::Map:
      *v.mutable_struct_value() = to_struct(node);
      break;
    case YAML::NodeType::Sequence:
      for (const auto &item : node) *v.mutable_list_value()->add_values() = to_value(item);
      break;
    case YAML::NodeType::Scalar: {
      const std::string text = node.Scalar();
      bool b;
      double d;
      if (node.Tag() == "!") {  // quoted
        v.set_string_value(text);
      } else if (YAML::convert<bool>::decode(node, b)) {
        v.set_bool_value(b);
      } else if (YAML::convert<double>::decode(node, d)) {
        v.set_number_value(d);
      } else {
        v.set_string_value(text);
      }
      break;
    }
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
      v.set_null_value(google::protobuf::NULL_VALUE);
      break;
  }
  return v;
}

[[noreturn]] void bad(const std::string &path, const std::string &what) {
  throw std::runtime_error("carla_canvas config '" + path + "': " + what);
}

}  // namespace

Config load_config(const std::string &path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception &e) {
    bad(path, e.what());
  }
  if (!root.IsMap()) bad(path, "the top level must be a mapping");
  Config config;
  config.path = path;
  try {
    if (const auto renderer = root["renderer"]) {
      if (!renderer.IsMap()) bad(path, "`renderer` must be a mapping");
      config.renderer = to_struct(renderer);
      if (renderer["endpoint"]) config.endpoint = renderer["endpoint"].as<std::string>();
      if (const auto command = renderer["command"]) {
        if (command.IsSequence()) {
          for (const auto &arg : command) config.command.push_back(arg.as<std::string>());
        } else {
          bad(path, "`renderer.command` must be a list of arguments");
        }
      }
      if (renderer["startup_timeout"]) config.startup_timeout = renderer["startup_timeout"].as<double>();
      if (renderer["timeout"]) config.timeout = renderer["timeout"].as<double>();
      if (renderer["async"]) config.async_render = renderer["async"].as<bool>();
    }
    const auto sensors = root["sensors"];
    if (!sensors) {
      // Default: the cameras and LiDARs a renderer can draw.
      config.sensors = {{"sensor.camera.*", ""}, {"sensor.lidar.*", ""}};
    } else if (!sensors.IsSequence()) {
      bad(path, "`sensors` must be a list of {type, role_name} rules");
    } else {
      for (const auto &rule : sensors) {
        SensorRule r;
        if (rule.IsScalar()) {
          r.type = rule.as<std::string>();
        } else if (rule.IsMap() && rule["type"]) {
          r.type = rule["type"].as<std::string>();
          if (rule["role_name"]) r.role_name = rule["role_name"].as<std::string>();
        } else {
          bad(path, "each `sensors` rule needs a `type`");
        }
        config.sensors.push_back(std::move(r));
      }
    }
  } catch (const YAML::Exception &e) {
    bad(path, e.what());
  }
  if (!(config.timeout > 0.0) || !(config.startup_timeout > 0.0)) {
    bad(path, "timeouts must be positive");
  }
  return config;
}

bool glob_match(const std::string &pattern, const std::string &text) {
  return fnmatch(pattern.c_str(), text.c_str(), 0) == 0;
}

bool claims_sensor(const Config &config, const std::string &type_id, const std::string &role_name) {
  for (const auto &rule : config.sensors) {
    if (!glob_match(rule.type, type_id)) continue;
    if (!rule.role_name.empty() && !glob_match(rule.role_name, role_name)) continue;
    return true;
  }
  return false;
}

}  // namespace carla_canvas
