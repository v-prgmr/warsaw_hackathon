#include "g1_loco_cmdvel/velocity_packet.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/idl/hg/BmsState_.hpp>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

std::int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Options
{
  std::string network_interface{"lo"};
  std::string socket_path{"/tmp/g1_loco_cmdvel.sock"};
  bool enabled{false};
  bool explicit_actuation_ack{false};
  double timeout_sec{0.30};
};

Options parse_options(int argc, char **argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    const auto equals = arg.find('=');
    const auto key = arg.substr(0, equals);
    const auto value = equals == std::string::npos ? std::string{} : arg.substr(equals + 1);
    if (key == "--network-interface") options.network_interface = value;
    if (key == "--socket") options.socket_path = value;
    if (key == "--enabled") options.enabled = value == "true" || value == "1";
    if (key == "--i-accept-high-level-actuation") {
      options.explicit_actuation_ack = value == "true" || value == "1";
    }
    if (key == "--timeout-sec") options.timeout_sec = std::stod(value);
  }
  return options;
}
}  // namespace

int main(int argc, char **argv)
{
  const auto options = parse_options(argc, argv);
  if (options.enabled && !options.explicit_actuation_ack) {
    std::cerr << "ERROR: --enabled requires --i-accept-high-level-actuation=true.\n";
    return 2;
  }
  if (options.enabled && (options.network_interface.empty() || options.network_interface == "lo")) {
    std::cerr << "ERROR: enabled Loco client needs the verified wired robot interface.\n";
    return 2;
  }
  if (!std::isfinite(options.timeout_sec) || options.timeout_sec <= 0.0 ||
      options.timeout_sec > 0.30) {
    std::cerr << "ERROR: watchdog timeout must be in (0, 0.30] seconds.\n";
    return 2;
  }
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  // A socket in /tmp must not be writable by other local users.
  const auto previous_umask = umask(0077);
  int socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
  umask(previous_umask);
  if (socket_fd < 0) {
    std::cerr << "socket: " << std::strerror(errno) << '\n';
    return 1;
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (options.socket_path.size() >= sizeof(address.sun_path)) {
    std::cerr << "socket path is too long\n";
    return 1;
  }
  std::strncpy(address.sun_path, options.socket_path.c_str(), sizeof(address.sun_path) - 1);
  // Do not remove an arbitrary existing file or a live client's socket.
  struct stat existing{};
  if (lstat(address.sun_path, &existing) == 0) {
    if (!S_ISSOCK(existing.st_mode) || existing.st_uid != geteuid()) {
      std::cerr << "Refusing to replace a socket path not owned by this user\n";
      close(socket_fd);
      return 1;
    }
    int probe = socket(AF_UNIX, SOCK_DGRAM, 0);
    const int connected = connect(probe, reinterpret_cast<const sockaddr *>(&address),
      sizeof(address));
    const int error = errno;
    close(probe);
    if (connected == 0 || error != ECONNREFUSED) {
      std::cerr << "Refusing to replace an active or unverified socket\n";
      close(socket_fd);
      return 1;
    }
    unlink(address.sun_path);
  }
  // bind() applies this umask when creating the socket file.
  const auto bind_umask = umask(0077);
  if (bind(socket_fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
    umask(bind_umask);
    std::cerr << "bind: " << std::strerror(errno) << '\n';
    close(socket_fd);
    return 1;
  }
  umask(bind_umask);

  std::atomic<int> battery_soc{-1};
  std::atomic<std::int64_t> battery_received_ns{0};
  std::unique_ptr<unitree::robot::g1::LocoClient> client;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>> bms;
  if (options.enabled) {
    unsetenv("CYCLONEDDS_URI");  // SDK uses its own DDS and the explicitly selected NIC.
    unitree::robot::ChannelFactory::Instance()->Init(0, options.network_interface);
    bms = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>>(
      "rt/lf/bmsstate");
    bms->InitChannel([&battery_soc, &battery_received_ns](const void *data) {
      const auto *state = static_cast<const unitree_hg::msg::dds_::BmsState_ *>(data);
      battery_soc.store(static_cast<int>(state->soc()), std::memory_order_relaxed);
      battery_received_ns.store(steady_ns(), std::memory_order_release);
    }, 1);
    client = std::make_unique<unitree::robot::g1::LocoClient>();
    client->Init();
    client->SetTimeout(1.0F);
  }

  std::cout << "Loco client listening on " << options.socket_path
            << "; actuation=" << (options.enabled ? "ENABLED" : "DISABLED") << '\n';
  g1_loco_cmdvel::VelocityPacket packet{};
  auto last_packet = std::chrono::steady_clock::now();
  bool active_velocity = false;
  bool stop_sent = false;
  auto battery_fresh = [&battery_soc, &battery_received_ns]() {
    const auto stamp = battery_received_ns.load(std::memory_order_acquire);
    const auto soc = battery_soc.load(std::memory_order_relaxed);
    return stamp > 0 && soc >= 20 && soc <= 100 &&
      (steady_ns() - stamp) <= 500'000'000;  // /lf/bmsstate is 20 Hz.
  };
  auto stop_motion = [&]() {
    if (options.enabled && !stop_sent) {
      const auto result = client->StopMove();
      if (result != 0) {
        std::cerr << "StopMove failed: " << result << '\n';
        stop_requested = 1;
      }
    }
    active_velocity = false;
    stop_sent = true;
  };
  while (!stop_requested) {
    const auto received = recv(socket_fd, &packet, sizeof(packet), MSG_DONTWAIT);
    if (received == static_cast<ssize_t>(sizeof(packet)) &&
        packet.magic == g1_loco_cmdvel::kVelocityPacketMagic) {
      const auto age_ns = steady_ns() - static_cast<std::int64_t>(packet.sent_steady_ns);
      if (!std::isfinite(packet.vx) || !std::isfinite(packet.vy) || !std::isfinite(packet.wz) ||
          std::abs(packet.vx) > 0.10F || std::abs(packet.vy) > 0.05F ||
          std::abs(packet.wz) > 0.20F || packet.sent_steady_ns == 0 ||
          age_ns < -100'000'000 || age_ns > 300'000'000) {
        std::cerr << "Rejected out-of-range or stale velocity packet\n";
        if (active_velocity) stop_motion();
        continue;
      }
      last_packet = std::chrono::steady_clock::now();
      const bool nonzero = packet.vx != 0.0F || packet.vy != 0.0F || packet.wz != 0.0F;
      std::cout << "cmd_vel vx=" << packet.vx << " vy=" << packet.vy
                << " wz=" << packet.wz << '\n';
      if (options.enabled) {
        if (!battery_fresh()) {
          std::cerr << "Command blocked: Unitree BMS missing, low, or stale\n";
          if (active_velocity) stop_motion();
          continue;
        }
        if (!nonzero) {
          stop_motion();
          continue;
        }
        const auto result = client->SetVelocity(packet.vx, packet.vy, packet.wz, 0.20F);
        if (result != 0) {
          std::cerr << "SetVelocity failed: " << result << '\n';
          stop_motion();
          stop_requested = 1;
        } else {
          active_velocity = true;
          stop_sent = false;
        }
      } else {
        active_velocity = nonzero;
      }
    }
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_packet).count();
    if (active_velocity && (age > options.timeout_sec ||
                            (options.enabled && !battery_fresh()))) {
      std::cout << "watchdog: stale velocity or battery; "
                << (options.enabled ? "StopMove" : "dry-run StopMove") << '\n';
      stop_motion();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (options.enabled) {
    stop_motion();  // SIGINT/SIGTERM also stop high-level walking.
    bms->CloseChannel();
  }
  close(socket_fd);
  unlink(address.sun_path);
  return 0;
}
