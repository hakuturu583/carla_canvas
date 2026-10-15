// The configuration loader of the plugin (native/src/config.cpp).
#include "config.hpp"

#include <cstdio>
#include <fstream>
#include <string>

static int g_failures = 0;

#define CHECK(cond)                                                               \
  do {                                                                            \
    if (!(cond)) {                                                                \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                               \
    }                                                                             \
  } while (0)

static std::string write(const std::string &name, const std::string &text) {
  const std::string path = "/tmp/carla_canvas_test_" + name + ".yaml";
  std::ofstream(path) << text;
  return path;
}

int main() {
  using namespace carla_canvas;
  const auto full = load_config(write("full", R"(
renderer:
  endpoint: "127.0.0.1:6000"
  command: [carla-canvas, serve, --config, "{config}"]
  timeout: 2.5
  async: false
  backend: gaussian_splatting
  options: {sh_degree: 3, background: [0, 0, 0], name: "1"}
sensors:
  - type: sensor.camera.rgb
    role_name: front*
  - sensor.lidar.ray_cast
)"));
  CHECK(full.endpoint == "127.0.0.1:6000");
  CHECK(full.command.size() == 4 && full.command[3] == "{config}");
  CHECK(full.timeout == 2.5 && !full.async_render);
  CHECK(full.renderer.fields().at("backend").string_value() == "gaussian_splatting");
  const auto &options = full.renderer.fields().at("options").struct_value().fields();
  CHECK(options.at("sh_degree").number_value() == 3);
  CHECK(options.at("background").list_value().values_size() == 3);
  CHECK(options.at("name").string_value() == "1");  // quoted: stays a string
  CHECK(claims_sensor(full, "sensor.camera.rgb", "front_left"));
  CHECK(!claims_sensor(full, "sensor.camera.rgb", "rear"));
  CHECK(claims_sensor(full, "sensor.lidar.ray_cast", ""));
  CHECK(!claims_sensor(full, "sensor.lidar.ray_cast_semantic", ""));

  const auto defaults = load_config(write("defaults", "renderer: {}\n"));
  CHECK(defaults.endpoint == "localhost:50051" && defaults.async_render);
  CHECK(claims_sensor(defaults, "sensor.camera.depth", ""));
  CHECK(claims_sensor(defaults, "sensor.lidar.ray_cast_semantic", ""));
  CHECK(!claims_sensor(defaults, "sensor.other.gnss", ""));

  bool threw = false;
  try {
    load_config(write("bad", "sensors: {type: x}\n"));
  } catch (const std::exception &e) {
    threw = std::string(e.what()).find("sensors") != std::string::npos;
  }
  CHECK(threw);
  threw = false;
  try {
    load_config("/nonexistent/canvas.yaml");
  } catch (const std::exception &e) {
    threw = std::string(e.what()).find("/nonexistent/canvas.yaml") != std::string::npos;
  }
  CHECK(threw);
  if (g_failures) return 1;
  std::puts("test_config: all checks passed");
  return 0;
}
