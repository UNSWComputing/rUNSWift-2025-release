#pragma once

#include <cstdint>
#include <runswift_interfaces/msg/comms_rcgcd.hpp>

using runswift_interfaces::msg::CommsRCGCD;

enum class GameState : uint8_t {
    Initial     = CommsRCGCD::STATE_INITIAL,
    Standby     = CommsRCGCD::STATE_STANDBY,
    Ready       = CommsRCGCD::STATE_READY,
    Set         = CommsRCGCD::STATE_SET,
    Playing     = CommsRCGCD::STATE_PLAYING,
    Finished    = CommsRCGCD::STATE_FINISHED 
};
