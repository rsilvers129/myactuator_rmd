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
#include "zeroerr_rmd/protocol/canopen.hpp"
#include "zeroerr_rmd/exceptions.hpp"


namespace zeroerr_rmd {

  ActuatorInterface::ActuatorInterface(Driver& driver, std::uint32_t const actuator_id)
  : driver_{driver}, actuator_id_{actuator_id} {
    driver.addId(actuator_id); 
    // Initialize motor: Set to Operational state?
    // NMT Start Node: 0x01 + NodeID
    // For now, we assume the user handles NMT or we do it lazily.
    return;
  }

  std::int32_t ActuatorInterface::getAcceleration() {
    auto frame = SdoRequest::read(actuator_id_, static_cast<uint16_t>(ObjectIndex::PROFILE_ACCELERATION), 0);
    auto response = driver_.sendRecv(frame);
    // Parse response (simplified, assuming SDO upload response)
    // Data starts at byte 4
    std::int32_t acc;
    std::memcpy(&acc, &response.getData()[4], sizeof(acc));
    return acc;
  }

  std::uint16_t ActuatorInterface::getCanId() {
    return static_cast<uint16_t>(actuator_id_);
  }

  Gains ActuatorInterface::getControllerGains() {
    // Zero-Err might not expose gains in the same structure. Returning default for now.
    return Gains{0,0,0,0,0,0}; 
  }

  ControlMode ActuatorInterface::getControlMode() {
    auto frame = SdoRequest::read(actuator_id_, static_cast<uint16_t>(ObjectIndex::MODES_OF_OPERATION_DISPLAY), 0);
    auto response = driver_.sendRecv(frame);
    std::int8_t mode;
    std::memcpy(&mode, &response.getData()[4], sizeof(mode));
    // Map DS402 modes to ControlMode enum if possible. 
    // For now returning UNKNOWN or casting if compatible.
    return ControlMode::NONE; 
  }

  std::string ActuatorInterface::getMotorModel() {
    return "ZeroErr eRob";
  }

  float ActuatorInterface::getMotorPower() {
    return 0.0f; // Not standard DS402
  }

  MotorStatus1 ActuatorInterface::getMotorStatus1() {
    return MotorStatus1{}; // Placeholder
  }

  MotorStatus2 ActuatorInterface::getMotorStatus2() {
    return MotorStatus2{}; // Placeholder
  }

  MotorStatus3 ActuatorInterface::getMotorStatus3() {
    return MotorStatus3{}; // Placeholder
  }

  float ActuatorInterface::getMultiTurnAngle() {
    auto frame = SdoRequest::read(actuator_id_, static_cast<uint16_t>(ObjectIndex::POSITION_ACTUAL_VALUE), 0);
    auto response = driver_.sendRecv(frame);
    std::int32_t pos;
    std::memcpy(&pos, &response.getData()[4], sizeof(pos));
    return static_cast<float>(pos) * 0.01f; // Scaling?
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderPosition() {
    auto frame = SdoRequest::read(actuator_id_, static_cast<uint16_t>(ObjectIndex::POSITION_ACTUAL_VALUE), 0);
    auto response = driver_.sendRecv(frame);
    std::int32_t pos;
    std::memcpy(&pos, &response.getData()[4], sizeof(pos));
    return pos;
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderOriginalPosition() {
    return getMultiTurnEncoderPosition();
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderZeroOffset() {
    return 0;
  }

  std::chrono::milliseconds ActuatorInterface::getRuntime() {
    return std::chrono::milliseconds(0);
  }

  float ActuatorInterface::getSingleTurnAngle() {
    return 0.0f;
  }

  std::int16_t ActuatorInterface::getSingleTurnEncoderPosition() {
    return 0;
  }

  std::uint32_t ActuatorInterface::getVersionDate() {
    return 0;
  }

  void ActuatorInterface::lockBrake() {
    // 0x6040 Controlword bit 0? Or specific object?
  }

  void ActuatorInterface::releaseBrake() {
  }

  void ActuatorInterface::reset() {
    // NMT Reset Node
    std::array<std::uint8_t, 8> data{};
    data[0] = 0x81; // Reset Node
    data[1] = static_cast<uint8_t>(actuator_id_);
    driver_.send(can::Frame(0x000, data));
  }

  Feedback ActuatorInterface::sendCurrentSetpoint(float const current) {
    // Map current to Target Torque (0x6071)
    // Assuming 1000 = 100% torque? Or raw units?
    // Zero-Err manual says 0x6071 is Target Torque in 0.1% of rated torque.
    // We need to know rated current/torque to convert.
    // For now, just sending raw value cast to int16.
    std::int16_t torque = static_cast<std::int16_t>(current * 100.0f); // Guess scaling
    
    // Also need to set Controlword to enable operation if not already?
    // And set Mode of Operation to Torque Profile (4)
    
    // Set Mode to Torque (4)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::MODES_OF_OPERATION), 0, static_cast<int8_t>(4)));
    
    // Enable Operation (Controlword 0x6040 = 0x0F)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::CONTROLWORD), 0, static_cast<uint16_t>(0x0F)));

    // Set Target Torque
    auto frame = SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::TARGET_TORQUE), 0, torque);
    auto response = driver_.sendRecv(frame);
    
    // Read Statusword and Position for Feedback
    // This is slow (multiple SDOs). Ideally use PDO.
    return Feedback{}; 
  }

  Feedback ActuatorInterface::sendPositionAbsoluteSetpoint(float const position, float const max_speed) {
    // Set Mode to Profile Position (1)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::MODES_OF_OPERATION), 0, static_cast<int8_t>(1)));
    
    // Set Target Position
    std::int32_t pos = static_cast<std::int32_t>(position * 100.0f); // Scaling?
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::TARGET_POSITION), 0, pos));
    
    // Set Profile Velocity
    std::uint32_t vel = static_cast<std::uint32_t>(max_speed * 100.0f);
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::PROFILE_VELOCITY), 0, vel)); // 0x6081

    // Controlword: New Setpoint (bit 4) + Enable (0x0F) -> 0x1F -> 0x0F (edge)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::CONTROLWORD), 0, static_cast<uint16_t>(0x0F)));
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::CONTROLWORD), 0, static_cast<uint16_t>(0x1F)));
    
    return Feedback{};
  }

  Feedback ActuatorInterface::sendTorqueSetpoint(float const torque, float const torque_constant) {
    return sendCurrentSetpoint(torque); // Simplified
  }

  Feedback ActuatorInterface::sendVelocitySetpoint(float const speed) {
    // Set Mode to Profile Velocity (3)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::MODES_OF_OPERATION), 0, static_cast<int8_t>(3)));
    
    // Enable
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::CONTROLWORD), 0, static_cast<uint16_t>(0x0F)));

    // Set Target Velocity
    std::int32_t vel = static_cast<std::int32_t>(speed * 100.0f);
    auto frame = SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::TARGET_VELOCITY), 0, vel);
    driver_.sendRecv(frame);
    
    return Feedback{};
  }

  void ActuatorInterface::setAcceleration(std::uint32_t const acceleration, AccelerationType const mode) {
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::PROFILE_ACCELERATION), 0, acceleration));
  }

  void ActuatorInterface::setCanId(std::uint16_t const can_id) {
    // Not standard CANopen to change ID via SDO usually, or vendor specific.
  }

  std::int32_t ActuatorInterface::setCurrentPositionAsEncoderZero() {
    return 0;
  }

  void ActuatorInterface::setEncoderZero(std::int32_t const encoder_offset) {
  }

  void ActuatorInterface::setCanBaudRate(CanBaudRate const baud_rate) {
  }

  Gains ActuatorInterface::setControllerGains(Gains const& gains, bool const is_persistent) {
    return gains;
  }

  void ActuatorInterface::setTimeout(std::chrono::milliseconds const& timeout) {
  }

  void ActuatorInterface::shutdownMotor() {
    // Controlword Shutdown (0x06)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::CONTROLWORD), 0, static_cast<uint16_t>(0x06)));
  }

  void ActuatorInterface::stopMotor() {
    // Quick Stop (0x02)
    driver_.send(SdoRequest::write(actuator_id_, static_cast<uint16_t>(ObjectIndex::CONTROLWORD), 0, static_cast<uint16_t>(0x02)));
  }

}
