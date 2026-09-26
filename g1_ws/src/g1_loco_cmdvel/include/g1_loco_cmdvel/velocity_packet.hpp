#pragma once

#include <cstdint>

namespace g1_loco_cmdvel
{

constexpr std::uint32_t kVelocityPacketMagic = 0x47315631;  // G1V1

struct VelocityPacket
{
  std::uint32_t magic{kVelocityPacketMagic};
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
  std::uint32_t sequence{0};
  std::uint32_t reserved{0};
  std::uint64_t sent_steady_ns{0};
};

static_assert(sizeof(VelocityPacket) == 32, "unexpected velocity packet layout");

}  // namespace g1_loco_cmdvel
