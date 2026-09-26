#include "g1_loco_cmdvel/velocity_packet.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

class CmdVelGateway final : public rclcpp::Node
{
public:
  CmdVelGateway()
  : Node("g1_loco_cmdvel_gateway")
  {
    socket_path_ = declare_parameter<std::string>("socket_path", "/tmp/g1_loco_cmdvel.sock");
    enabled_ = declare_parameter<bool>("enabled", false);
    max_vx_ = declare_parameter<double>("max_vx", 0.10);
    max_vy_ = declare_parameter<double>("max_vy", 0.05);
    max_wz_ = declare_parameter<double>("max_wz", 0.20);
    require_battery_ = declare_parameter<bool>("require_battery", true);
    battery_topic_ = declare_parameter<std::string>("battery_topic", "/battery_state");
    min_battery_percent_ = declare_parameter<double>("min_battery_percent", 0.20);
    battery_timeout_sec_ = declare_parameter<double>("battery_timeout_sec", 1.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.30);
    if (!std::isfinite(max_vx_) || max_vx_ <= 0 || !std::isfinite(max_vy_) || max_vy_ <= 0 ||
        !std::isfinite(max_wz_) || max_wz_ <= 0 ||
        !std::isfinite(min_battery_percent_) || min_battery_percent_ < 0 ||
        min_battery_percent_ > 1 ||
        !std::isfinite(battery_timeout_sec_) || battery_timeout_sec_ <= 0 ||
        !std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0) {
      throw std::invalid_argument("invalid velocity, battery, or timeout limits");
    }

    socket_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("cannot create Unix datagram socket");
    }
    std::memset(&destination_, 0, sizeof(destination_));
    destination_.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(destination_.sun_path)) {
      throw std::runtime_error("socket_path is too long");
    }
    std::strncpy(destination_.sun_path, socket_path_.c_str(), sizeof(destination_.sun_path) - 1);

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_command(*msg); });
    battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        battery_ok_ = std::isfinite(msg->percentage) &&
          msg->percentage >= static_cast<float>(min_battery_percent_) &&
          msg->percentage <= 1.0F;
        last_battery_receive_ = std::chrono::steady_clock::now();
        if (require_battery_ && !battery_ok_ && active_velocity_) {
          send(0.0F, 0.0F, 0.0F);
        }
      });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (active_velocity_ &&
          (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_)
            .count() > command_timeout_sec_ ||
           (require_battery_ && !battery_fresh()))) {
        RCLCPP_WARN(get_logger(), "stale command or battery; sending zero velocity");
        send(0.0F, 0.0F, 0.0F);
      }
    });

    RCLCPP_WARN(get_logger(), "High-level Loco gateway is %s", enabled_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(get_logger(), "limits: vx=%.3f vy=%.3f wz=%.3f, battery gate=%s",
      max_vx_, max_vy_, max_wz_, require_battery_ ? "required" : "disabled");
  }

  ~CmdVelGateway() override
  {
    send(0.0F, 0.0F, 0.0F);
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  static float clamp_finite(double value, double limit)
  {
    if (!std::isfinite(value)) {
      return 0.0F;
    }
    return static_cast<float>(std::clamp(value, -limit, limit));
  }

  bool battery_fresh() const
  {
    return battery_ok_ && last_battery_receive_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_battery_receive_).count() <= battery_timeout_sec_;
  }

  void on_command(const geometry_msgs::msg::Twist &msg)
  {
    const float vx = clamp_finite(msg.linear.x, max_vx_);
    const float vy = clamp_finite(msg.linear.y, max_vy_);
    const float wz = clamp_finite(msg.angular.z, max_wz_);

    last_command_ = std::chrono::steady_clock::now();
    if (!enabled_ || (require_battery_ && !battery_fresh())) {
      send(0.0F, 0.0F, 0.0F);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "command blocked: enabled=%s battery_fresh=%s",
        enabled_ ? "true" : "false", battery_fresh() ? "true" : "false");
      return;
    }
    send(vx, vy, wz);
  }

  void send(float vx, float vy, float wz)
  {
    g1_loco_cmdvel::VelocityPacket packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;
    packet.sequence = sequence_++;
    packet.sent_steady_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    active_velocity_ = vx != 0.0F || vy != 0.0F || wz != 0.0F;
    const auto sent = sendto(socket_fd_, &packet, sizeof(packet), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&destination_), sizeof(destination_));
    if (sent != static_cast<ssize_t>(sizeof(packet))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Loco client is not receiving packets at %s", socket_path_.c_str());
    }
  }

  int socket_fd_{-1};
  sockaddr_un destination_{};
  std::string socket_path_;
  bool enabled_{false};
  bool require_battery_{true};
  bool battery_ok_{false};
  bool active_velocity_{false};
  std::string battery_topic_;
  std::chrono::steady_clock::time_point last_battery_receive_{};
  std::chrono::steady_clock::time_point last_command_{};
  double max_vx_{0.10};
  double max_vy_{0.05};
  double max_wz_{0.20};
  double min_battery_percent_{0.20};
  double battery_timeout_sec_{1.0};
  double command_timeout_sec_{0.30};
  std::uint32_t sequence_{0};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelGateway>());
  rclcpp::shutdown();
  return 0;
}
