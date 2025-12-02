#ifndef ZEROERR_RMD__PROTOCOL__CANOPEN
#define ZEROERR_RMD__PROTOCOL__CANOPEN
#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "zeroerr_rmd/can/frame.hpp"

namespace zeroerr_rmd {

  // CANopen Command Specifiers (CS) for SDO
  enum class SdoCommand : std::uint8_t {
    DOWNLOAD_REQUEST = 0x20, // Write to node
    DOWNLOAD_RESPONSE = 0x60,
    UPLOAD_REQUEST = 0x40,   // Read from node
    UPLOAD_RESPONSE = 0x40,  // Server response with data (expedited)
    ABORT_TRANSFER = 0x80
  };

  // CANopen Object Dictionary Indices
  enum class ObjectIndex : std::uint16_t {
    CONTROLWORD = 0x6040,
    STATUSWORD = 0x6041,
    MODES_OF_OPERATION = 0x6060,
    MODES_OF_OPERATION_DISPLAY = 0x6061,
    POSITION_ACTUAL_VALUE = 0x6064,
    VELOCITY_ACTUAL_VALUE = 0x606C,
    TARGET_POSITION = 0x607A,
    TARGET_VELOCITY = 0x60FF,
    TARGET_TORQUE = 0x6071,
    MOTOR_RATED_TORQUE = 0x6076,
    PROFILE_ACCELERATION = 0x6083,
    PROFILE_DECELERATION = 0x6084,
    QUICK_STOP_DECELERATION = 0x6085,
    MOTION_PROFILE_TYPE = 0x6086,
    PROFILE_VELOCITY = 0x6081
  };

  class SdoRequest {
    public:
      static can::Frame read(std::uint16_t node_id, std::uint16_t index, std::uint8_t subindex) {
        std::array<std::uint8_t, 8> data;
        data[0] = static_cast<std::uint8_t>(SdoCommand::UPLOAD_REQUEST);
        data[1] = static_cast<std::uint8_t>(index & 0xFF);
        data[2] = static_cast<std::uint8_t>((index >> 8) & 0xFF);
        data[3] = subindex;
        std::memset(&data[4], 0, 4);
        return can::Frame(0x600 + node_id, data);
      }

      template<typename T>
      static can::Frame write(std::uint16_t node_id, std::uint16_t index, std::uint8_t subindex, T value) {
        std::array<std::uint8_t, 8> data;
        
        // Determine size and CS
        std::uint8_t size_indicator = 0;
        if constexpr (sizeof(T) == 1) size_indicator = 0x0F; // 4-1=3 bytes unused
        else if constexpr (sizeof(T) == 2) size_indicator = 0x0B; // 4-2=2 bytes unused
        else if constexpr (sizeof(T) == 4) size_indicator = 0x03; // 4-4=0 bytes unused
        
        data[0] = static_cast<std::uint8_t>(SdoCommand::DOWNLOAD_REQUEST) | size_indicator;
        data[1] = static_cast<std::uint8_t>(index & 0xFF);
        data[2] = static_cast<std::uint8_t>((index >> 8) & 0xFF);
        data[3] = subindex;
        
        std::memcpy(&data[4], &value, sizeof(T));
        return can::Frame(0x600 + node_id, data);
      }
  };

}

#endif // ZEROERR_RMD__PROTOCOL__CANOPEN
