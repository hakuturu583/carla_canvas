// The carla_canvas sensor plugin: a typesafe_carla sensor source (ABI 5.3)
// that hands the sensors the configuration names to a renderer service over
// gRPC (proto/carla_canvas/v1/renderer.proto) and queues its measurements as
// LibCarla's own, so the program's sensor code does not change.
//
// typesafe_carla loads it from TYPESAFE_CARLA_SENSOR_PLUGIN (the launcher sets
// that when CARLA_CANVAS_CONFIG is set); it is inactive without
// CARLA_CANVAS_CONFIG.
#include "config.hpp"
#include "tsc_api.hpp"

#include "carla_canvas/v1/renderer.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace rpc = carla_canvas::v1;  // not "pb": protobuf 30+ has a ::pb namespace

namespace carla_canvas {
namespace {

constexpr uint32_t kProtocolVersion = 2;

thread_local std::string tl_error;

void log(const std::string &message) {
  std::fprintf(stderr, "[carla_canvas] %s\n", message.c_str());
}

// --- Conversions ---------------------------------------------------------------

void to_proto(const tsc_vector3d_t &v, rpc::Vector3D *out) {
  out->set_x(v.x);
  out->set_y(v.y);
  out->set_z(v.z);
}

void to_proto(const tsc_rotation_t &r, rpc::Rotation *out) {
  out->set_pitch(r.pitch);
  out->set_yaw(r.yaw);
  out->set_roll(r.roll);
}

void to_proto(const tsc_transform_t &t, rpc::Transform *out) {
  to_proto(t.location, out->mutable_location());
  to_proto(t.rotation, out->mutable_rotation());
}

tsc_transform_t from_proto(const rpc::Transform &t) {
  tsc_transform_t out{};
  out.location = {t.location().x(), t.location().y(), t.location().z()};
  out.rotation = {t.rotation().pitch(), t.rotation().yaw(), t.rotation().roll()};
  return out;
}

std::string take(const Api &api, tsc_string_t &s) {
  std::string value(s.data != nullptr ? s.data : "", s.size);
  api.tsc_string_free(&s);
  return value;
}

// The actor's attributes (as spawned), in LibCarla's order.
std::vector<std::pair<std::string, std::string>> attributes(const Api &api, tsc_actor_t *actor) {
  std::vector<std::pair<std::string, std::string>> out;
  tsc_string_list_t ids{}, values{};
  if (api.tsc_actor_get_attributes(actor, &ids, &values) != TSC_OK) return out;
  for (size_t i = 0; i < ids.size && i < values.size; ++i) {
    out.emplace_back(std::string(ids.items[i].data, ids.items[i].size),
                     std::string(values.items[i].data, values.items[i].size));
  }
  api.tsc_string_list_free(&ids);
  api.tsc_string_list_free(&values);
  return out;
}

// carla.Actor of a handle (its attributes are the blueprint's).
rpc::Actor actor_info(const Api &api, tsc_actor_t *actor) {
  rpc::Actor info;
  uint32_t id = 0;
  api.check(api.tsc_actor_get_id(actor, &id), "Actor.id");
  info.set_id(id);
  tsc_string_t type_id{};
  api.check(api.tsc_actor_get_type_id(actor, &type_id), "Actor.type_id");
  info.set_type_id(take(api, type_id));
  for (auto &[id, value] : attributes(api, actor)) {
    auto *a = info.add_attributes();
    a->set_id(std::move(id));
    a->set_value(std::move(value));
  }
  Handle<tsc_actor_t> parent(api);
  if (api.tsc_actor_get_parent(actor, parent.out()) == TSC_OK && parent) {
    uint32_t parent_id = 0;
    if (api.tsc_actor_get_id(parent.get(), &parent_id) == TSC_OK) info.set_parent_id(parent_id);
  }
  tsc_bounding_box_t box{};
  if (api.tsc_actor_get_bounding_box(actor, &box) == TSC_OK) {
    auto *b = info.mutable_bounding_box();
    to_proto(box.location, b->mutable_location());
    to_proto(box.extent, b->mutable_extent());
    to_proto(box.rotation, b->mutable_rotation());
  }
  return info;
}

std::string attribute(const rpc::Actor &actor, const std::string &id) {
  for (const auto &a : actor.attributes()) {
    if (a.id() == id) return a.value();
  }
  return {};
}

// --- The renderer process ---------------------------------------------------------

class RendererProcess {
 public:
  ~RendererProcess() { stop(); }

  // Starts `command` ("{config}" replaced by config_path) in its own process
  // group, with CARLA_CANVAS_PARENT_PID set: `carla-canvas serve` exits when
  // this process is gone (e.g. killed), and stop() ends it at exit. (Not
  // PR_SET_PDEATHSIG: that follows the forking thread, which may end first.)
  void start(const std::vector<std::string> &command, const std::string &config_path) {
    std::vector<std::string> args;
    for (auto arg : command) {
      for (size_t at = 0; (at = arg.find("{config}", at)) != std::string::npos;
           at += config_path.size()) {
        arg.replace(at, 8, config_path);
      }
      args.push_back(std::move(arg));
    }
    std::vector<char *> argv;
    for (auto &a : args) argv.push_back(a.data());
    argv.push_back(nullptr);
    // Everything the child needs is built before fork(): after it, in a
    // multithreaded process, only async-signal-safe calls are allowed.
    std::vector<std::string> env_strings;
    for (char **e = environ; *e != nullptr; ++e) {
      if (std::strncmp(*e, "CARLA_CANVAS_PARENT_PID=", 24) != 0) env_strings.emplace_back(*e);
    }
    env_strings.push_back("CARLA_CANVAS_PARENT_PID=" + std::to_string(getpid()));
    std::vector<char *> envp;
    for (auto &e : env_strings) envp.push_back(e.data());
    envp.push_back(nullptr);
    const pid_t pid = fork();
    if (pid < 0) throw std::runtime_error("cannot start the renderer: fork failed");
    if (pid == 0) {
      setpgid(0, 0);
      execvpe(argv[0], argv.data(), envp.data());
      std::fprintf(stderr, "[carla_canvas] cannot run '%s': %s\n", argv[0], std::strerror(errno));
      _exit(127);
    }
    pid_ = pid;
    log("started the renderer (pid " + std::to_string(pid) + "): " + args[0]);
  }

  // Whether it is still running (false once it exited).
  bool running() {
    if (pid_ <= 0) return false;
    int status = 0;
    if (waitpid(pid_, &status, WNOHANG) == pid_) {
      pid_ = -1;
      return false;
    }
    return true;
  }

  void stop() {
    if (pid_ <= 0) return;
    kill(-pid_, SIGTERM);
    for (int i = 0; i < 50 && running(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (pid_ > 0) {
      kill(-pid_, SIGKILL);
      waitpid(pid_, nullptr, 0);
      pid_ = -1;
    }
  }

 private:
  pid_t pid_ = -1;
};

// --- The source -------------------------------------------------------------------

struct SensorState {
  rpc::Actor info;
  double sensor_tick = 0.0;  // seconds between measurements, 0: every frame
  double last_time = -1.0;  // elapsed_seconds of the last measurement
  bool listening = false;  // the program has a stream
  bool ros = false;  // enable_for_ros()
  bool spawned = false;  // SpawnSensor done in the current episode
  bool seen = false;  // in a snapshot already
};

class Plugin {
 public:
  Plugin(Api api, Config config) : api_(api), config_(std::move(config)) {}

  // At exit: no new callbacks, the worker's call cancelled and joined, the
  // callbacks in progress finished; then the renderer it started is stopped.
  ~Plugin() {
    api_.tsc_sensor_source_install(nullptr);
    stopping_ = true;
    {
      std::lock_guard<std::mutex> lock(call_mutex_);
      if (active_call_ != nullptr) active_call_->TryCancel();
    }
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
  }

  bool claims(tsc_actor_t *actor, uint32_t id, const std::string &type_id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (auto it = claims_.find(id); it != claims_.end()) return it->second;
    std::string role_name;
    for (const auto &[key, value] : attributes(api_, actor)) {
      if (key == "role_name") role_name = value;
    }
    const bool claimed = claims_sensor(config_, type_id, role_name);
    claims_[id] = claimed;
    return claimed;
  }

  void start(tsc_actor_t *actor, uint32_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto &sensor = track(actor, id);
    spawn(sensor);  // a sensor the renderer refuses never becomes due
    sensor.listening = true;
  }

  void stop(uint32_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (auto it = sensors_.find(id); it != sensors_.end()) it->second.listening = false;
  }

  void set_ros(tsc_actor_t *actor, uint32_t id, bool enabled) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto &sensor = track(actor, id);
    spawn(sensor);
    send_ros_enabled(id, enabled);
    sensor.ros = enabled;
  }

  bool is_ros(uint32_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = sensors_.find(id);
    return it != sensors_.end() && it->second.ros;
  }

  void destroyed(uint32_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    claims_.erase(id);
    auto it = sensors_.find(id);
    if (it == sensors_.end()) return;
    const bool spawned = it->second.spawned;
    sensors_.erase(it);
    if (spawned && stub_) {
      rpc::DestroySensorRequest request;
      request.set_sensor_id(id);
      rpc::DestroySensorResponse response;
      try {
        call("DestroySensor", [&](grpc::ClientContext &c) {
          return stub_->DestroySensor(&c, request, &response);
        });
      } catch (const std::exception &e) {
        log(e.what());  // destroy() itself succeeded
      }
    }
  }

  // World.tick() returned `frame`: render it before the program's dispatch
  // (unless the worker already did; a frame whose render failed is retried
  // here, so the failure reaches tick()).
  void ticked(tsc_world_t *world, uint64_t frame) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!active()) return;
    uint64_t episode = 0;
    api_.check(api_.tsc_world_get_id(world, &episode), "World.id");
    if (rendered(episode, frame)) return;
    Handle<tsc_world_snapshot_t> snapshot(api_);
    api_.check(api_.tsc_world_get_snapshot(world, snapshot.out()), "World.get_snapshot");
    render(world, snapshot.get());
  }

 private:
  // Frame numbers restart with a new episode (load_world).
  bool rendered(uint64_t episode, uint64_t frame) const {
    return episode_ && *episode_ == episode && frame <= last_frame_;
  }

  bool active() const {
    for (const auto &[id, s] : sensors_) {
      if (s.listening || s.ros) return true;
    }
    return false;
  }

  SensorState &track(tsc_actor_t *actor, uint32_t id) {
    auto it = sensors_.find(id);
    if (it == sensors_.end()) {
      SensorState state;
      state.info = actor_info(api_, actor);
      const std::string tick = attribute(state.info, "sensor_tick");
      if (!tick.empty()) state.sensor_tick = std::max(0.0, std::atof(tick.c_str()));
      it = sensors_.emplace(id, std::move(state)).first;
    }
    Handle<tsc_world_t> world(api_);
    api_.check(api_.tsc_actor_get_world(actor, world.out()), "Actor.get_world");
    uint64_t episode = 0;
    api_.check(api_.tsc_world_get_id(world.get(), &episode), "World.id");
    if (config_.async_render && worker_episode_ != episode) {
      // The worker waits on this episode's World (an old one expires with
      // load_world): it picks it up at its next wait.
      api_.check(api_.tsc_actor_get_world(actor, next_world_.out()), "Actor.get_world");
      worker_episode_ = episode;
      if (!worker_.joinable()) worker_ = std::thread([this] { work(); });
    }
    connect();
    load_world(world.get(), episode);
    return it->second;
  }

  // --- gRPC ---------------------------------------------------------------------

  static void set_timeout(grpc::ClientContext &context, double seconds) {
    context.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(static_cast<int64_t>(1000.0 * seconds)));
  }

  template <typename F>
  void call(const char *what, F &&f, double timeout = -1.0) {
    grpc::ClientContext context;
    set_timeout(context, timeout > 0 ? timeout : config_.timeout);
    {
      std::lock_guard<std::mutex> lock(call_mutex_);
      if (stopping_) throw std::runtime_error("carla_canvas is shutting down");
      active_call_ = &context;  // ~Plugin cancels it
    }
    const grpc::Status status = f(context);
    {
      std::lock_guard<std::mutex> lock(call_mutex_);
      active_call_ = nullptr;
    }
    if (!status.ok()) {
      throw std::runtime_error(std::string("renderer ") + what + " at " + config_.endpoint +
                               " failed: " + status.error_message() + " (gRPC code " +
                               std::to_string(static_cast<int>(status.error_code())) + ")");
    }
  }

  bool get_info(rpc::RendererInfo &info, double timeout) {
    grpc::ClientContext context;
    set_timeout(context, timeout);
    context.set_wait_for_ready(true);
    return stub_->GetInfo(&context, rpc::GetInfoRequest{}, &info).ok();
  }

  void connect() {
    if (stub_) return;
    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(-1);  // a frame's images easily exceed gRPC's 4 MB default
    args.SetMaxSendMessageSize(-1);
    auto channel =
        grpc::CreateCustomChannel(config_.endpoint, grpc::InsecureChannelCredentials(), args);
    stub_ = rpc::Renderer::NewStub(channel);
    rpc::RendererInfo info;
    bool ok = get_info(info, config_.command.empty() ? config_.timeout : 1.0);
    bool started = false;
    // On any failure below: no half-connected stub, no renderer left running
    // (the next connect() would start a second one).
    auto fail = [&](const std::string &message) {
      stub_.reset();
      if (started) process_.stop();
      throw std::runtime_error(message);
    };
    if (!ok && !config_.command.empty()) {
      try {
        process_.start(config_.command, config_.path);
      } catch (const std::exception &e) {
        fail(e.what());
      }
      started = true;
      const auto until = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(static_cast<int64_t>(1000 * config_.startup_timeout));
      while (!ok && std::chrono::steady_clock::now() < until) {
        if (!process_.running()) {
          fail("the renderer command exited before it answered at " + config_.endpoint);
        }
        ok = get_info(info, 1.0);
      }
    }
    if (!ok) {
      fail("no renderer answers at " + config_.endpoint +
           " (start one with `carla-canvas serve`, or set renderer.command)");
    }
    if (info.protocol_version() != kProtocolVersion) {
      fail("the renderer at " + config_.endpoint + " speaks protocol " +
           std::to_string(info.protocol_version()) + ", not " + std::to_string(kProtocolVersion));
    }
    log("rendering with '" + info.name() + "' " + info.version() + " at " + config_.endpoint);
  }

  // LoadWorld once per episode; a new episode (load_world) starts over. The
  // episode comes from a snapshot or a fresh World: a World handle keeps the
  // id of the episode it was made in, though it follows the client's
  // current episode otherwise (the worker's does).
  void load_world(tsc_world_t *world, uint64_t episode) {
    if (episode_ && *episode_ == episode) return;
    Handle<tsc_map_t> map(api_);
    api_.check(api_.tsc_world_get_map(world, map.out()), "World.get_map");
    tsc_string_t name{};
    api_.check(api_.tsc_map_get_name(map.get(), &name), "Map.name");
    tsc_string_t opendrive{};
    api_.check(api_.tsc_map_to_opendrive(map.get(), &opendrive), "Map.to_opendrive");
    rpc::LoadWorldRequest request;
    request.set_map_name(take(api_, name));
    request.set_opendrive(take(api_, opendrive));
    request.set_episode_id(episode);
    *request.mutable_renderer_config() = config_.renderer;
    rpc::LoadWorldResponse response;
    call("LoadWorld", [&](grpc::ClientContext &c) { return stub_->LoadWorld(&c, request, &response); },
         std::max(config_.timeout, config_.startup_timeout));
    episode_ = episode;
    last_frame_ = 0;
    known_actors_.clear();
    for (auto &[id, s] : sensors_) s.spawned = false;
  }

  void spawn(SensorState &sensor) {
    if (sensor.spawned) return;
    rpc::SpawnSensorRequest request;
    *request.mutable_sensor() = sensor.info;
    rpc::SpawnSensorResponse response;
    call("SpawnSensor", [&](grpc::ClientContext &c) { return stub_->SpawnSensor(&c, request, &response); });
    sensor.spawned = true;
    if (sensor.ros) send_ros_enabled(sensor.info.id(), true);  // again, after a new episode
  }

  void send_ros_enabled(uint32_t id, bool enabled) {
    rpc::EnableForRosRequest request;
    request.set_sensor_id(id);
    request.set_enabled(enabled);
    rpc::EnableForRosResponse response;
    call("EnableForRos", [&](grpc::ClientContext &c) {
      return stub_->EnableForRos(&c, request, &response);
    });
  }

  // --- Rendering ------------------------------------------------------------------

  void render(tsc_world_t *world, const tsc_world_snapshot_t *snapshot) {
    tsc_timestamp_t ts{};
    api_.check(api_.tsc_world_snapshot_get_timestamp(snapshot, &ts), "WorldSnapshot.timestamp");
    uint64_t snapshot_id = 0;
    api_.check(api_.tsc_world_snapshot_get_id(snapshot, &snapshot_id), "WorldSnapshot.id");
    if (rendered(snapshot_id, ts.frame)) return;
    load_world(world, snapshot_id);

    const size_t n = api_.tsc_world_snapshot_size(snapshot);
    std::vector<tsc_actor_snapshot_t> actors(n);
    std::set<uint32_t> present;
    for (size_t i = 0; i < n; ++i) {
      api_.check(api_.tsc_world_snapshot_get(snapshot, i, &actors[i]), "WorldSnapshot[i]");
      present.insert(actors[i].id);
    }
    // A sensor gone from the world without Actor.destroy() (a batch
    // DestroyActor, another client): forget it, as destroy() would.
    // Only once it was in an earlier snapshot: an asynchronous snapshot may
    // predate the sensor's spawn.
    std::vector<uint32_t> vanished;
    for (auto &[id, sensor] : sensors_) {
      if (present.count(id) != 0) {
        sensor.seen = true;
      } else if (sensor.seen) {
        vanished.push_back(id);
      }
    }
    for (uint32_t id : vanished) destroyed(id);

    // Nothing below is committed (frame, sensor times, known actors) until the
    // renderer has answered: a failed frame is sent again, whole.
    rpc::RenderRequest request;
    std::vector<SensorState *> due;
    for (auto &[id, sensor] : sensors_) {
      if ((!sensor.listening && !sensor.ros) || !sensor.seen) continue;
      if (sensor.last_time >= 0.0 && sensor.sensor_tick > 0.0 &&
          ts.elapsed_seconds - sensor.last_time + 1e-9 < sensor.sensor_tick) {
        continue;
      }
      spawn(sensor);
      auto *r = request.add_sensors();
      r->set_sensor_id(id);
      r->set_deliver(sensor.listening);
      due.push_back(&sensor);
    }
    if (request.sensors_size() == 0) {
      last_frame_ = ts.frame;
      return;
    }

    auto *snap = request.mutable_snapshot();
    snap->set_id(snapshot_id);
    auto *t = snap->mutable_timestamp();
    t->set_frame(ts.frame);
    t->set_elapsed_seconds(ts.elapsed_seconds);
    t->set_delta_seconds(ts.delta_seconds);
    t->set_platform_timestamp(ts.platform_timestamp);
    for (const auto &a : actors) {
      auto *out = snap->add_actors();
      out->set_id(a.id);
      to_proto(a.transform, out->mutable_transform());
      to_proto(a.velocity, out->mutable_velocity());
      to_proto(a.angular_velocity, out->mutable_angular_velocity());
      to_proto(a.acceleration, out->mutable_acceleration());
      if (known_actors_.count(a.id) == 0) {
        Handle<tsc_actor_t> actor(api_);
        if (api_.tsc_world_get_actor(world, a.id, actor.out()) == TSC_OK && actor) {
          *request.add_new_actors() = actor_info(api_, actor.get());
        }
      }
    }
    for (uint32_t id : known_actors_) {
      if (present.count(id) == 0) request.add_removed_actors(id);
    }

    rpc::RenderResponse response;
    call("Render", [&](grpc::ClientContext &c) { return stub_->Render(&c, request, &response); });
    last_frame_ = ts.frame;
    for (auto *sensor : due) sensor->last_time = ts.elapsed_seconds;
    for (const auto &a : request.new_actors()) known_actors_.insert(a.id());
    for (uint32_t id : request.removed_actors()) known_actors_.erase(id);
    for (const auto &m : response.measurements()) deliver(m);
  }

  void deliver(const rpc::SensorData &m) {
    auto it = sensors_.find(m.sensor_id());
    if (it == sensors_.end() || !it->second.listening) return;
    const tsc_transform_t t = from_proto(m.transform());
    Handle<tsc_sensor_data_t> data(api_);
    switch (m.data_case()) {
      case rpc::SensorData::kImage: {
        const auto &image = m.image();
        api_.check(api_.tsc_sensor_data_new_image(
                       m.frame(), m.timestamp(), &t, image.width(), image.height(), image.fov(),
                       reinterpret_cast<const uint8_t *>(image.raw_data().data()),
                       image.raw_data().size(), data.out()),
                   "renderer image");
        break;
      }
      case rpc::SensorData::kLidar:
        new_lidar(m, m.lidar(), api_.tsc_sensor_data_new_lidar, 4, &t, data);
        break;
      case rpc::SensorData::kSemanticLidar:
        new_lidar(m, m.semantic_lidar(), api_.tsc_sensor_data_new_semantic_lidar, 1, &t, data);
        break;
      default:
        throw std::runtime_error("the renderer returned an empty measurement for sensor " +
                                 std::to_string(m.sensor_id()));
    }
    size_t delivered = 0;
    api_.check(api_.tsc_sensor_source_push(m.sensor_id(), data.get(), &delivered), "push");
    if (delivered == 0) it->second.listening = false;  // its handles are gone
  }

  // A (semantic) LiDAR measurement of `lidar` (raw_data: `New`'s point type
  // in groups of `per_point`, read in place: protobuf's buffers are aligned).
  template <typename Message, typename Point>
  void new_lidar(const rpc::SensorData &m, const Message &lidar,
                 tsc_status_t (*New)(uint64_t, double, const tsc_transform_t *, float,
                                     const uint32_t *, uint32_t, const Point *, size_t,
                                     tsc_sensor_data_t **),
                 size_t per_point, const tsc_transform_t *t, Handle<tsc_sensor_data_t> &out) {
    const auto &raw = lidar.raw_data();
    const std::vector<uint32_t> counts(lidar.points_per_channel().begin(),
                                       lidar.points_per_channel().end());
    api_.check(New(m.frame(), m.timestamp(), t, lidar.horizontal_angle(), counts.data(),
                   static_cast<uint32_t>(counts.size()), reinterpret_cast<const Point *>(raw.data()),
                   raw.size() / sizeof(Point) / per_point, out.out()),
               "renderer LiDAR measurement");
  }

  // Renders the ticks of an asynchronous simulation (no World.tick() from the
  // program). In synchronous mode it sees the same frames as ticked(), which
  // renders each frame once.
  void work() {
    std::string last_error;
    Handle<tsc_world_t> world(api_);
    while (!stopping_) {
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (next_world_) world.swap(next_world_);
        next_world_.reset();
      }
      Handle<tsc_world_snapshot_t> snapshot(api_);
      if (api_.tsc_world_wait_for_tick(world.get(), 0.2, snapshot.out()) != TSC_OK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // an expired episode fails at once
        continue;
      }
      try {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!active()) continue;
        render(world.get(), snapshot.get());
        last_error.clear();
      } catch (const std::exception &e) {
        if (e.what() != last_error) log(e.what());
        last_error = e.what();
      }
    }
  }

  Api api_;
  Config config_;
  std::recursive_mutex mutex_;
  std::map<uint32_t, bool> claims_;
  std::map<uint32_t, SensorState> sensors_;
  std::set<uint32_t> known_actors_;
  std::optional<uint64_t> episode_;
  uint64_t last_frame_ = 0;
  Handle<tsc_world_t> next_world_{api_};  // handed to the worker
  std::optional<uint64_t> worker_episode_;  // of the last World handed over
  std::unique_ptr<rpc::Renderer::Stub> stub_;
  RendererProcess process_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  std::mutex call_mutex_;
  grpc::ClientContext *active_call_ = nullptr;  // the call in progress
};

// Destroyed at exit (before typesafe_carla's statics: the plugin was loaded
// after it), which joins the worker and stops a renderer it started.
std::unique_ptr<Plugin> g_plugin;

template <typename F>
int32_t guarded(F &&f) {
  try {
    f();
    return TSC_OK;
  } catch (const std::exception &e) {
    tl_error = e.what();
  } catch (...) {
    tl_error = "unknown error";
  }
  return TSC_ERROR;
}

// The callbacks after ~Plugin (a call that began during exit) do nothing.
int32_t on_claims(void *, tsc_actor_t *sensor, uint32_t id, const char *type_id, size_t len) {
  if (!g_plugin) return 0;
  bool claimed = false;
  if (guarded([&] { claimed = g_plugin->claims(sensor, id, std::string(type_id, len)); }) != TSC_OK) {
    log("cannot decide whether to render sensor " + std::to_string(id) + ": " + tl_error);
  }
  return claimed ? 1 : 0;
}

int32_t on_start(void *, tsc_actor_t *sensor, uint32_t id) {
  if (!g_plugin) return TSC_OK;
  return guarded([&] { g_plugin->start(sensor, id); });
}

void on_stop(void *, uint32_t id) {
  if (g_plugin) guarded([&] { g_plugin->stop(id); });
}

int32_t on_set_ros(void *, tsc_actor_t *sensor, uint32_t id, int32_t enabled) {
  if (!g_plugin) return TSC_OK;
  return guarded([&] { g_plugin->set_ros(sensor, id, enabled != 0); });
}

int32_t on_is_ros(void *, uint32_t id) { return g_plugin && g_plugin->is_ros(id) ? 1 : 0; }

void on_destroyed(void *, uint32_t id) {
  if (g_plugin) guarded([&] { g_plugin->destroyed(id); });
}

int32_t on_ticked(void *, tsc_world_t *world, uint64_t frame) {
  if (!g_plugin) return TSC_OK;
  return guarded([&] { g_plugin->ticked(world, frame); });
}

const char *on_last_error(void *) { return tl_error.c_str(); }

}  // namespace
}  // namespace carla_canvas

extern "C" __attribute__((visibility("default"))) int32_t tsc_sensor_plugin_init(
    uint32_t abi_version, void *(*get_proc)(const char *), const char **out_error) {
  using namespace carla_canvas;
  static std::string error;
  if ((abi_version >> 16) != TSC_ABI_VERSION_MAJOR || (abi_version & 0xffff) < 3) {
    error = "carla_canvas needs typesafe_carla's C ABI " + std::to_string(TSC_ABI_VERSION_MAJOR) +
            ".3 or later (the library has " + std::to_string(abi_version >> 16) + "." +
            std::to_string(abi_version & 0xffff) + ")";
    *out_error = error.c_str();
    return TSC_VERSION_ERROR;
  }
  const char *path = std::getenv("CARLA_CANVAS_CONFIG");
  if (path == nullptr || *path == '\0') return TSC_OK;  // not configured: CARLA renders
  try {
    Api api = Api::load(get_proc);
    g_plugin = std::make_unique<Plugin>(api, load_config(path));
    tsc_sensor_source_t source{};
    source.abi_major = TSC_ABI_VERSION_MAJOR;
    source.struct_size = sizeof(source);
    source.claims = on_claims;
    source.start = on_start;
    source.stop = on_stop;
    source.set_ros_enabled = on_set_ros;
    source.is_ros_enabled = on_is_ros;
    source.actor_destroyed = on_destroyed;
    source.world_ticked = on_ticked;
    source.last_error = on_last_error;
    api.check(api.tsc_sensor_source_install(&source), "tsc_sensor_source_install");
  } catch (const std::exception &e) {
    g_plugin.reset();
    error = e.what();
    *out_error = error.c_str();
    return TSC_ERROR;
  }
  return TSC_OK;
}
