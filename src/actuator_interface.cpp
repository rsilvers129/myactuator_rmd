#include "zeroerr_rmd/actuator_interface.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "zeroerr_rmd/actuator_state/can_baud_rate.hpp"
#include "zeroerr_rmd/actuator_state/control_mode.hpp"
#include "zeroerr_rmd/actuator_state/feedback.hpp"
#include "zeroerr_rmd/actuator_state/gains.hpp"
#include "zeroerr_rmd/actuator_state/motor_status_1.hpp"
#include "zeroerr_rmd/actuator_state/motor_status_2.hpp"
#include "zeroerr_rmd/actuator_state/motor_status_3.hpp"
#include "zeroerr_rmd/driver/driver.hpp"
#include "zeroerr_rmd/protocol/can_custom.hpp"
#include "zeroerr_rmd/exceptions.hpp"


namespace zeroerr_rmd {

  ActuatorInterface::ActuatorInterface(Driver& driver, std::uint32_t const actuator_id)
  : driver_{driver}, actuator_id_{actuator_id} {
    driver.addId(actuator_id);
    // ZeroErr driver automatically adds offset 0x5C0 for RX filter.
    // driver.addId(actuator_id) adds filter for 0x5C0 + actuator_id.
    return;
  }

  std::int32_t ActuatorInterface::getAcceleration() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::PROFILE_ACCELERATION);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      throw Exception("Failed to read acceleration");
    }
    return response.getInt32();
  }

  std::uint16_t ActuatorInterface::getCanId() {
    return static_cast<std::uint16_t>(actuator_id_);
  }

  Gains ActuatorInterface::getControllerGains() {
    // ZeroErr exposes position/speed loop gains at different indices
    // For now returning default values - would need multiple reads
    return Gains{0, 0, 0, 0, 0, 0}; 
  }

  ControlMode ActuatorInterface::getControlMode() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::OPERATION_MODE);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      return ControlMode::NONE;
    }
    std::uint32_t mode = response.getUint32();
    switch (mode) {
      case 1: return ControlMode::CURRENT;   // Torque mode uses current control
      case 2: return ControlMode::VELOCITY;
      case 3: return ControlMode::POSITION;
      default: return ControlMode::NONE;
    }
  }

  std::string ActuatorInterface::getMotorModel() {
    return "ZeroErr eRob";
  }

  float ActuatorInterface::getMotorPower() {
    return 0.0f; // Not available via CAN-custom
  }

  MotorStatus1 ActuatorInterface::getMotorStatus1() {
    MotorStatus1 status{};
    
    // Read temperature
    auto temp_frame = CanCustomRequest::read(actuator_id_, ParamIndex::POWER_TEMPERATURE);
    auto temp_response = driver_.sendRecv(temp_frame);
    auto temp = parseResponse(temp_response);
    if (temp.success) {
      status.temperature = static_cast<std::int8_t>(temp.getInt32());
    }
    
    // Read voltage
    auto volt_frame = CanCustomRequest::read(actuator_id_, ParamIndex::DC_BUS_VOLTAGE);
    auto volt_response = driver_.sendRecv(volt_frame);
    auto volt = parseResponse(volt_response);
    if (volt.success) {
      // Voltage in mV, convert to 0.1V units used by status
      status.voltage = static_cast<std::uint16_t>(volt.getUint32() / 100);
    }
    
    // Read error codes
    auto err_frame = CanCustomRequest::read(actuator_id_, ParamIndex::ERROR_CODES);
    auto err_response = driver_.sendRecv(err_frame);
    auto err = parseResponse(err_response);
    if (err.success) {
      // Map error code - cast to ErrorCode directly (0 = NO_ERROR)
      status.error_code = static_cast<ErrorCode>(err.getUint32() & 0xFFFF);
    }
    
    return status;
  }

  MotorStatus2 ActuatorInterface::getMotorStatus2() {
    return getFeedback();
  }

  MotorStatus3 ActuatorInterface::getMotorStatus3() {
    MotorStatus3 status{};
    
    // Read temperature
    auto temp_frame = CanCustomRequest::read(actuator_id_, ParamIndex::POWER_TEMPERATURE);
    auto temp_response = driver_.sendRecv(temp_frame);
    auto temp = parseResponse(temp_response);
    if (temp.success) {
      status.temperature = static_cast<std::int8_t>(temp.getInt32());
    }
    
    return status;
  }

  float ActuatorInterface::getMultiTurnAngle() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::ACTUAL_POSITION);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      throw Exception("Failed to read position");
    }
    // Convert counts to degrees
    return static_cast<float>(response.getInt32()) * ErobConstants::DEGREES_PER_COUNT;
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderPosition() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::ACTUAL_POSITION);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      throw Exception("Failed to read encoder position");
    }
    return response.getInt32();
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderOriginalPosition() {
    return getMultiTurnEncoderPosition();
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderZeroOffset() {
    return 0; // Not directly available in CAN-custom
  }

  std::chrono::milliseconds ActuatorInterface::getRuntime() {
    return std::chrono::milliseconds(0); // Not available
  }

  float ActuatorInterface::getSingleTurnAngle() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::LOAD_ANGLE);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      return 0.0f;
    }
    // Load angle is 0-524287 for 0-360 degrees
    return static_cast<float>(response.getInt32()) * ErobConstants::DEGREES_PER_COUNT;
  }

  std::int16_t ActuatorInterface::getSingleTurnEncoderPosition() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::LOAD_ANGLE);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      return 0;
    }
    // This returns a 32-bit value, we truncate to 16-bit
    return static_cast<std::int16_t>(response.getInt32() & 0xFFFF);
  }

  std::uint32_t ActuatorInterface::getVersionDate() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::FIRMWARE_VERSION, 0x0004);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      return 0;
    }
    return response.getUint32();
  }

  void ActuatorInterface::lockBrake() {
    // Disable motor to engage brake
    auto frame = CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 0u);
    driver_.sendRecv(frame);
  }

  void ActuatorInterface::releaseBrake() {
    // Enable motor to disengage brake
    auto frame = CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u);
    driver_.sendRecv(frame);
  }

  void ActuatorInterface::reset() {
    // Stop motion and disable motor
    stopMotor();
    shutdownMotor();
  }

  Feedback ActuatorInterface::sendCurrentSetpoint(float const current) {
    // ==========================================
    // Torque Control Mode Sequence
    // ==========================================
    
    // 1. Set operation mode to Torque (1)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::OPERATION_MODE, 
                                         static_cast<std::uint32_t>(OperationMode::TORQUE)));
    
    // 2. Set control source to not use external
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::CONTROL_SOURCE, 0u));
    
    // 3. Set analog quantity to internal
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY_SOURCE, 0u));
    
    // 4. Set target current (analog quantity) in mA
    std::int32_t current_ma = static_cast<std::int32_t>(current * 1000.0f);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY, 
                                         static_cast<std::uint32_t>(current_ma)));
    
    // 5. Enable motor (if not already)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u));
    
    // Read and return feedback
    return getFeedback();
  }

  Feedback ActuatorInterface::sendPositionAbsoluteSetpoint(float const position, float const max_speed) {
    // ==========================================
    // Position Control Mode Sequence (Move Absolute)
    // ==========================================
    
    // 1. Set operation mode to Position (3)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::OPERATION_MODE, 
                                         static_cast<std::uint32_t>(OperationMode::POSITION)));
    
    // 2. Set motion mode to Absolute (1)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::MOTION_MODE, 
                                         static_cast<std::uint32_t>(MotionMode::ABSOLUTE)));
    
    // 3. Set profile speed (counts/s)
    std::uint32_t speed_counts = static_cast<std::uint32_t>(max_speed * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::PROFILE_SPEED, speed_counts));
    
    // 4. Set relative position to 0 (required before absolute move)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::RELATIVE_POSITION, 0u));
    
    // 5. Enable motor (if not already)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u));
    
    // 6. Set target absolute position (counts)
    std::int32_t pos_counts = static_cast<std::int32_t>(position * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::TARGET_ABSOLUTE_POSITION, 
                                         static_cast<std::uint32_t>(pos_counts)));
    
    // 7. Start motion
    driver_.sendRecv(CanCustomRequest::command(actuator_id_, CmdIndex::START_MOTION));
    
    // Read and return feedback
    return getFeedback();
  }

  Feedback ActuatorInterface::sendTorqueSetpoint(float const torque, float const torque_constant) {
    // Convert torque (Nm) to current (A) using torque constant
    float current = torque / torque_constant;
    return sendCurrentSetpoint(current);
  }

  Feedback ActuatorInterface::sendVelocitySetpoint(float const speed) {
    // ==========================================
    // Velocity Control Mode Sequence
    // ==========================================
    
    // 1. Set operation mode to Velocity (2)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::OPERATION_MODE, 
                                         static_cast<std::uint32_t>(OperationMode::VELOCITY)));
    
    // 2. Set control source to not use external
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::CONTROL_SOURCE, 0u));
    
    // 3. Set analog quantity to internal
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY_SOURCE, 0u));
    
    // 4. Set target speed (analog quantity) in counts/s
    std::int32_t speed_counts = static_cast<std::int32_t>(speed * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY, 
                                         static_cast<std::uint32_t>(speed_counts)));
    
    // 5. Enable motor (if not already)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u));
    
    // Read and return feedback
    return getFeedback();
  }

  void ActuatorInterface::setAcceleration(std::uint32_t const acceleration, AccelerationType const mode) {
    // Set both acceleration and deceleration
    // Convert from dps² to counts/s² (input is in degrees/s²)
    std::uint32_t accel_counts = static_cast<std::uint32_t>(acceleration * ErobConstants::COUNTS_PER_DEGREE);
    
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::PROFILE_ACCELERATION, accel_counts));
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::PROFILE_DECELERATION, accel_counts));
  }

  void ActuatorInterface::setCanId(std::uint16_t const can_id) {
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::CAN_ADDRESS, 
                                         static_cast<std::uint32_t>(can_id)));
    // Note: After changing CAN ID, need to save parameters and reboot
  }

  std::int32_t ActuatorInterface::setCurrentPositionAsEncoderZero() {
    // Not directly available in CAN-custom protocol
    return getMultiTurnEncoderPosition();
  }

  void ActuatorInterface::setEncoderZero(std::int32_t const encoder_offset) {
    // Not directly available in CAN-custom protocol
  }

  void ActuatorInterface::setCanBaudRate(CanBaudRate const baud_rate) {
    // CAN baud rate is fixed at 1 Mbps per eRunner manual
  }

  Gains ActuatorInterface::setControllerGains(Gains const& gains, bool const is_persistent) {
    // Set position loop gain
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::POSITION_LOOP_GAIN, 
                                         static_cast<std::uint32_t>(gains.position.kp)));
    
    // Set speed loop gain
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::SPEED_LOOP_GAIN, 
                                         static_cast<std::uint32_t>(gains.speed.kp)));
    
    // Set speed loop integral
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::SPEED_LOOP_INTEGRAL, 
                                         static_cast<std::uint32_t>(gains.speed.ki)));
    
    if (is_persistent) {
      // Save parameters to flash
      driver_.sendRecv(CanCustomRequest::command(actuator_id_, CmdIndex::SAVE_PARAMETERS));
      // Wait 3 seconds for save to complete (per eRunner manual)
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    
    return gains;
  }

  void ActuatorInterface::setTimeout(std::chrono::milliseconds const& timeout) {
    // Store timeout for use in waitForResponse
    // This is handled at the driver level
  }

  void ActuatorInterface::shutdownMotor() {
    // Disable motor (engages brake)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 0u));
  }

  void ActuatorInterface::stopMotor() {
    // Send stop motion command
    driver_.sendRecv(CanCustomRequest::command(actuator_id_, CmdIndex::STOP_MOTION));
  }
  
  void ActuatorInterface::waitForResponse() {
     // Removed as we use sendRecv
  }
  
  Feedback ActuatorInterface::getFeedback() {
    Feedback feedback{};

    // Read actual position
    try {
      auto pos_frame = CanCustomRequest::read(actuator_id_, ParamIndex::ACTUAL_POSITION);
      auto pos_response = driver_.sendRecv(pos_frame);
      auto pos = parseResponse(pos_response);
      if (pos.success) {
        feedback.shaft_angle = static_cast<float>(pos.getInt32()) * ErobConstants::DEGREES_PER_COUNT;
      }
    } catch (...) {}
    
    // Read actual speed
    try {
      auto speed_frame = CanCustomRequest::read(actuator_id_, ParamIndex::ACTUAL_SPEED, 0x0001);
      auto speed_response = driver_.sendRecv(speed_frame);
      auto speed = parseResponse(speed_response);
      if (speed.success) {
        feedback.shaft_speed = static_cast<float>(speed.getInt32()) * ErobConstants::DEGREES_PER_COUNT;
      }
    } catch (...) {}
    
    // Read actual current
    try {
      auto curr_frame = CanCustomRequest::read(actuator_id_, ParamIndex::ACTUAL_CURRENT);
      auto curr_response = driver_.sendRecv(curr_frame);
      auto curr = parseResponse(curr_response);
      if (curr.success) {
        feedback.current = static_cast<float>(curr.getInt32()) / 1000.0f;  // mA to A
      }
    } catch (...) {}
    
    return feedback;
  }


  void ActuatorInterface::initializePositionControl(float const profile_speed, float const profile_accel, float const profile_decel) {
    // 1. Set operation mode to Position (3)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::OPERATION_MODE, 
                                         static_cast<std::uint32_t>(OperationMode::POSITION)));
    
    // 2. Set motion mode to Absolute (1)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::MOTION_MODE, 
                                         static_cast<std::uint32_t>(MotionMode::ABSOLUTE)));
    
    // 3. Set profile speed (counts/s)
    std::uint32_t speed_counts = static_cast<std::uint32_t>(profile_speed * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::PROFILE_SPEED, speed_counts));

    // 4. Set acceleration/deceleration
    std::uint32_t accel_counts = static_cast<std::uint32_t>(profile_accel * ErobConstants::COUNTS_PER_DEGREE);
    std::uint32_t decel_counts = static_cast<std::uint32_t>(profile_decel * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::PROFILE_ACCELERATION, accel_counts));
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::PROFILE_DECELERATION, decel_counts));
    
    // 5. Set relative position to 0 (required before absolute move)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::RELATIVE_POSITION, 0u));
    
    // 6. Enable motor
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u));
    
    // 7. Start motion (Enable profile generator)
    driver_.sendRecv(CanCustomRequest::command(actuator_id_, CmdIndex::START_MOTION));
    
    // Check for errors
    auto status = getMotorStatus1();
    if (status.error_code != ErrorCode::NO_ERROR) {
      // We can't use RCLCPP here, but we can throw
      throw Exception("Motor error on init: " + std::to_string(static_cast<int>(status.error_code)));
    }
  }

  Feedback ActuatorInterface::updatePositionSetpoint(float const position) {
    // Set target absolute position (counts)
    std::int32_t pos_counts = static_cast<std::int32_t>(position * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::TARGET_ABSOLUTE_POSITION, 
                                         static_cast<std::uint32_t>(pos_counts)));
    
    // Trigger motion to new target
    driver_.sendRecv(CanCustomRequest::command(actuator_id_, CmdIndex::START_MOTION));
    
    return getFeedback();
  }
  
  std::uint32_t ActuatorInterface::getTargetState() {
    auto frame = CanCustomRequest::read(actuator_id_, ParamIndex::TARGET_STATE);
    auto response_frame = driver_.sendRecv(frame);
    auto response = parseResponse(response_frame);
    if (!response.success) {
      return 0; // Error or Off
    }
    return response.getUint32();
  }

  void ActuatorInterface::initializeVelocityControl() {
    // 1. Set operation mode to Velocity (2)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::OPERATION_MODE, 
                                         static_cast<std::uint32_t>(OperationMode::VELOCITY)));
    
    // 2. Set control source to not use external
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::CONTROL_SOURCE, 0u));
    
    // 3. Set analog quantity to internal
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY_SOURCE, 0u));
    
    // 4. Enable motor
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u));
  }

  Feedback ActuatorInterface::updateVelocitySetpoint(float const velocity) {
    // Set target speed (analog quantity) in counts/s
    std::int32_t speed_counts = static_cast<std::int32_t>(velocity * ErobConstants::COUNTS_PER_DEGREE);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY, 
                                         static_cast<std::uint32_t>(speed_counts)));
    return getFeedback();
  }

  void ActuatorInterface::initializeTorqueControl() {
    // 1. Set operation mode to Torque (1)
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::OPERATION_MODE, 
                                         static_cast<std::uint32_t>(OperationMode::TORQUE)));
    
    // 2. Set control source to not use external
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::CONTROL_SOURCE, 0u));
    
    // 3. Set analog quantity to internal
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY_SOURCE, 0u));
    
    // 4. Enable motor
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, CmdIndex::ENABLE_MOTOR, 1u));
  }

  Feedback ActuatorInterface::updateTorqueSetpoint(float const torque, float const torque_constant) {
    // Convert torque (Nm) to current (A)
    float current = torque / torque_constant;
    
    // Set target current (analog quantity) in mA
    std::int32_t current_ma = static_cast<std::int32_t>(current * 1000.0f);
    driver_.sendRecv(CanCustomRequest::write(actuator_id_, ParamIndex::ANALOG_QUANTITY, 
                                         static_cast<std::uint32_t>(current_ma)));
    return getFeedback();
  }

} // namespace zeroerr_rmd
