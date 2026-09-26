// Read-only SDK-side verification of the G1 battery channel. No LocoClient.
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/BmsState_.hpp>

int main(int argc, char **argv)
{
  if (argc != 2 || std::string(argv[1]) == "lo") {
    std::cerr << "Usage: g1_bms_probe <wired-robot-interface>\n";
    return 2;
  }
  std::atomic<int> soc{-1};
  unsetenv("CYCLONEDDS_URI");  // SDK process must not inherit ROS DDS settings.
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_> subscriber(
    "rt/lf/bmsstate");
  subscriber.InitChannel([&soc](const void *data) {
    const auto *msg = static_cast<const unitree_hg::msg::dds_::BmsState_ *>(data);
    soc.store(static_cast<int>(msg->soc()));
  }, 1);
  for (int tick = 0; tick < 50; ++tick) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const int latest = soc.load();
    if (latest >= 0 && latest <= 100) {
      std::cout << "SDK battery SOC: " << latest << "%\n";
      subscriber.CloseChannel();
      return 0;
    }
  }
  std::cerr << "No valid rt/lf/bmsstate within 5 seconds; actuation must stay disabled\n";
  subscriber.CloseChannel();
  return 1;
}
