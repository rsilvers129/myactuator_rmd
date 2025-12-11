/**
 * \file can_custom.hpp
 * \brief
 *    ZeroErr CAN-Custom Protocol Implementation
 *    Based on eRunner User Manual V1.7
 *    NOTE: Protocol uses BIG ENDIAN byte order for both Index and Data
 */

#ifndef ZEROERR_RMD__PROTOCOL__CAN_CUSTOM
#define ZEROERR_RMD__PROTOCOL__CAN_CUSTOM
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <algorithm> // for std::reverse if needed

#include "zeroerr_rmd/can/frame.hpp"

namespace zeroerr_rmd {

  /**
   * \brief CAN-Custom Protocol COB-ID definitions
   */
  namespace CobId {
    constexpr std::uint16_t TX_BASE = 0x640;  // Master -> Slave
    constexpr std::uint16_t RX_BASE = 0x5C0;  // Slave -> Master
  }

  /**
   * \brief CAN-Custom End Delimiters
   */
  namespace EndDelimiter {
    constexpr std::uint8_t SUCCESS = 0x3E;
    constexpr std::uint8_t ERROR = 0x80;
  }

  /**
   * \brief Parameter Indices (Read/Write)
   * Note: These are 16-bit values. Protocol expects Big Endian transmission.
   */
  namespace ParamIndex {
    // System Info
    constexpr std::uint16_t HARDWARE_VERSION = 0x0001;  // Sub-index 3
    constexpr std::uint16_t FIRMWARE_VERSION = 0x0001;  // Sub-index 4
    
    // Feedback (Read Only)
    constexpr std::uint16_t ACTUAL_POSITION = 0x0002;
    constexpr std::uint16_t ACTUAL_SPEED = 0x0005;     // Sub-index 1
    constexpr std::uint16_t ACTUAL_CURRENT = 0x0008;
    constexpr std::uint16_t U_PHASE_CURRENT = 0x0009;
    constexpr std::uint16_t V_PHASE_CURRENT = 0x000A;
    constexpr std::uint16_t ERROR_CODES = 0x001F;
    constexpr std::uint16_t OPERATION_STATE = 0x0020;
    constexpr std::uint16_t DC_BUS_VOLTAGE = 0x0024;
    constexpr std::uint16_t POWER_TEMPERATURE = 0x0026;
    constexpr std::uint16_t MOTOR_ANGLE = 0x009C;
    constexpr std::uint16_t LOAD_ANGLE = 0x009F;       // Single-turn 0-360°
    
    // Configuration (Read/Write)
    constexpr std::uint16_t CAN_ADDRESS = 0x0043;
    constexpr std::uint16_t OPERATION_MODE = 0x004E;   // 1=Torque, 2=Velocity, 3=Position
    constexpr std::uint16_t MAX_SPEED = 0x0050;
    constexpr std::uint16_t POSITION_LIMIT_MIN = 0x0052;
    constexpr std::uint16_t POSITION_LIMIT_MAX = 0x0053;
    constexpr std::uint16_t MAX_POSITION_ERROR = 0x0054;
    constexpr std::uint16_t MAX_SPEED_ERROR = 0x0055;
    constexpr std::uint16_t MAX_PHASE_CURRENT = 0x0062;
    constexpr std::uint16_t MAX_MOTOR_CURRENT = 0x0063;
    
    // Motion Control
    constexpr std::uint16_t TARGET_ABSOLUTE_POSITION = 0x0086;
    constexpr std::uint16_t RELATIVE_POSITION = 0x0087;
    constexpr std::uint16_t PROFILE_ACCELERATION = 0x0088;
    constexpr std::uint16_t PROFILE_DECELERATION = 0x0089;
    constexpr std::uint16_t PROFILE_SPEED = 0x008A;
    constexpr std::uint16_t JERK_SETTING = 0x008B;
    constexpr std::uint16_t EMERGENCY_STOP_DECEL = 0x008C;
    constexpr std::uint16_t MOTION_MODE = 0x008D;      // 0=Continuous, 1=Absolute, 2=Repetitive
    constexpr std::uint16_t STOP_TIME = 0x0093;
    constexpr std::uint16_t LIMIT_MOVE = 0x009E;       // 0=Speed only, 1=Accel/Decel limited
    
    // Velocity/Torque Control
    constexpr std::uint16_t CONTROL_SOURCE = 0x0112;   // 0=Not use, 1=EtherCAT
    constexpr std::uint16_t ANALOG_QUANTITY_SOURCE = 0x01FD;  // 0=Internal
    constexpr std::uint16_t ANALOG_QUANTITY = 0x01FE;  // Target speed (vel mode) or current (torque mode)
    constexpr std::uint16_t MAX_SPEED_LIMIT = 0x0204;  // Torque mode speed limit
    
    // PID Gains
    constexpr std::uint16_t POSITION_LOOP_GAIN = 0x0064;
    constexpr std::uint16_t SPEED_LOOP_GAIN = 0x0066;
    constexpr std::uint16_t SPEED_LOOP_INTEGRAL = 0x0067;
    
    // Current Limits
    constexpr std::uint16_t CONTINUOUS_CURRENT = 0x0033;
    constexpr std::uint16_t PEAK_CURRENT = 0x0034;
    constexpr std::uint16_t PEAK_CURRENT_DURATION = 0x0035;
    
    // Positioning Control
    constexpr std::uint16_t POSITIONING_THRESHOLD = 0x0109;
    constexpr std::uint16_t POSITIONING_TIME = 0x010A;
    constexpr std::uint16_t TARGET_STATE = 0x010C;     // 0=Off, 1=On, 2=Moving, 3=Wait, 4=Achieved
  }

  /**
   * \brief Command Indices (Write only, no value)
   */
  namespace CmdIndex {
    constexpr std::uint16_t START_MOTION = 0x0083;
    constexpr std::uint16_t STOP_MOTION = 0x0084;
    constexpr std::uint16_t SAVE_PARAMETERS = 0x00E8;
    constexpr std::uint16_t ENABLE_MOTOR = 0x0100;     // Value: 0=Disable, 1=Enable
    constexpr std::uint16_t DISENGAGE_BRAKE = 0x014F;
  }

  /**
   * \brief Operation Modes
   */
  enum class OperationMode : std::uint32_t {
    TORQUE = 1,
    VELOCITY = 2,
    POSITION = 3
  };

  /**
   * \brief Motion Modes (for position control)
   */
  enum class MotionMode : std::uint32_t {
    CONTINUOUS = 0,    // Like velocity mode, runs at set speed
    ABSOLUTE = 1,      // Move to absolute position
    REPETITIVE = 2     // Repeat motion
  };
  
  /**
   * \brief CAN-Custom Response structure
   */
  struct CanCustomResponse {
    bool success{false};
    std::array<std::uint8_t, 4> data{};
    std::uint8_t data_size{0};
    
    template<typename T>
    T getValue() const {
      // Not safe for Big Endian protocol -> Little Endian host
      // Use getInt32 or getUint32 which handle conversion
      return static_cast<T>(getUint32());
    }
    
    std::int32_t getInt32() const {
      return static_cast<std::int32_t>(getUint32());
    }
    
    std::uint32_t getUint32() const {
      // Reassemble Big Endian bytes to Little Endian host integer
      // data[0] is MSB, data[3] is LSB
      if (data_size < 4) return 0;
      
      return (static_cast<std::uint32_t>(data[0]) << 24) |
             (static_cast<std::uint32_t>(data[1]) << 16) |
             (static_cast<std::uint32_t>(data[2]) << 8) |
             static_cast<std::uint32_t>(data[3]);
    }
  };

  /**
   * \brief Parse a CAN-custom response frame
   */
  inline CanCustomResponse parseResponse(can::Frame const& frame) {
    CanCustomResponse response;
    auto const& data = frame.getData();
    
    // Check for error
    if (data[0] == EndDelimiter::ERROR) {
      response.success = false;
      return response;
    }

    // Check for single-byte success response (write acknowledgment)
    // Sometimes 3E is at byte 0
    if (data[0] == EndDelimiter::SUCCESS) {
      response.success = true;
      response.data_size = 0;
      return response;
    }
    
    // Find the end delimiter 0x3E
    // Read Data Response: Byte 0-3 Value, Byte 4 Delimiter
    // Or sometimes just Value if delimiter omitted (rare)
    for (int i = 0; i < 8; ++i) {
      if (data[i] == EndDelimiter::SUCCESS) {
        response.success = true;
        // Data is everything before delimiter
        // Usually 4 bytes at data[0]..data[3]
        if (i >= 4) {
          response.data_size = 4;
          response.data[0] = data[0];
          response.data[1] = data[1];
          response.data[2] = data[2];
          response.data[3] = data[3];
        } else {
            // Unexpected short response
            response.data_size = static_cast<std::uint8_t>(i);
            for(int k=0; k<i; ++k) response.data[k] = data[k];
        }
        return response;
      }
    }
    
    // No end delimiter found
    // If we have data but no delimiter, assume success but warn?
    // For now assuming success 
    response.success = true;
    response.data_size = 4;
    for(int k=0; k<4; ++k) response.data[k] = data[k];
    
    return response;
  }

  /**
   * \brief CAN-Custom Request builder class
   */
  class CanCustomRequest {
    public:
      /**
       * \brief Create a read request (without sub-index)
       */
      static can::Frame read(std::uint16_t node_id, std::uint16_t index) {
        std::array<std::uint8_t, 8> data{};
        data[0] = static_cast<std::uint8_t>((index >> 8) & 0xFF); // HI (Big Endian)
        data[1] = static_cast<std::uint8_t>(index & 0xFF);        // LO
        return can::Frame(CobId::TX_BASE + node_id, data, 2);  // DLC=2
      }
      
      /**
       * \brief Create a read request (with sub-index)
       */
      static can::Frame read(std::uint16_t node_id, std::uint16_t index, std::uint16_t subindex) {
        std::array<std::uint8_t, 8> data{};
        data[0] = static_cast<std::uint8_t>((index >> 8) & 0xFF); // Index HI
        data[1] = static_cast<std::uint8_t>(index & 0xFF);        // Index LO
        data[2] = static_cast<std::uint8_t>((subindex >> 8) & 0xFF); // Sub HI
        data[3] = static_cast<std::uint8_t>(subindex & 0xFF);        // Sub LO
        return can::Frame(CobId::TX_BASE + node_id, data, 4);  // DLC=4
      }
      
      /**
       * \brief Create a write request (without sub-index, 4-byte value)
       */
      static can::Frame write(std::uint16_t node_id, std::uint16_t index, std::uint32_t value) {
        std::array<std::uint8_t, 8> data{};
        data[0] = static_cast<std::uint8_t>((index >> 8) & 0xFF); // Index HI
        data[1] = static_cast<std::uint8_t>(index & 0xFF);        // Index LO
        // Value Big Endian, starting at Byte 2
        data[2] = static_cast<std::uint8_t>((value >> 24) & 0xFF); // MSB
        data[3] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
        data[4] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
        data[5] = static_cast<std::uint8_t>(value & 0xFF);         // LSB
        return can::Frame(CobId::TX_BASE + node_id, data, 6);  // DLC=6
      }
      
      /**
       * \brief Create a write request (with sub-index, 4-byte value)
       * NOTE: Most parameters don't use this. Check manual if SubIndex is required.
       */
      static can::Frame write(std::uint16_t node_id, std::uint16_t index, std::uint16_t subindex, std::uint32_t value) {
        std::array<std::uint8_t, 8> data{};
        data[0] = static_cast<std::uint8_t>((index >> 8) & 0xFF); // Index HI
        data[1] = static_cast<std::uint8_t>(index & 0xFF);        // Index LO
        data[2] = static_cast<std::uint8_t>((subindex >> 8) & 0xFF); // Sub HI
        data[3] = static_cast<std::uint8_t>(subindex & 0xFF);        // Sub LO
        // Value Big Endian at 4..7
        data[4] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
        data[5] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
        data[6] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
        data[7] = static_cast<std::uint8_t>(value & 0xFF);
        return can::Frame(CobId::TX_BASE + node_id, data);
      }
      
      /**
       * \brief Create a command request (no parameters)
       */
      static can::Frame command(std::uint16_t node_id, std::uint16_t cmd_index) {
        std::array<std::uint8_t, 8> data{};
        data[0] = static_cast<std::uint8_t>((cmd_index >> 8) & 0xFF); // HI
        data[1] = static_cast<std::uint8_t>(cmd_index & 0xFF);        // LO
        return can::Frame(CobId::TX_BASE + node_id, data, 2);  // DLC=2
      }
  };

  /**
   * \brief eRob specific constants
   */
  namespace ErobConstants {
    // Encoder resolution (per eRob manual)
    constexpr std::int32_t OUTPUT_ENCODER_RESOLUTION = 524288;  // pulses/revolution (load side)
    
    // Position scaling for eRob: position values are in encoder counts
    // 524288 counts = 360 degrees
    constexpr float COUNTS_PER_DEGREE = 524288.0f / 360.0f;     // ~1456.36 counts/degree
    constexpr float DEGREES_PER_COUNT = 360.0f / 524288.0f;
    constexpr float COUNTS_PER_RADIAN = 524288.0f / 6.28318530718f;  // ~83443.0 counts/rad
    constexpr float RADIANS_PER_COUNT = 6.28318530718f / 524288.0f;
  }

} // namespace zeroerr_rmd

#endif // ZEROERR_RMD__PROTOCOL__CAN_CUSTOM
