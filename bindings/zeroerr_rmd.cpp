/**
 * \file zeroerr_rmd.cpp
 * \mainpage
 *    Python bindings for C++ library
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#include <cstdint>
#include <string>
#include <sstream>
#include <tuple>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "zeroerr_rmd/actuator_state/acceleration_type.hpp"
#include "zeroerr_rmd/actuator_state/can_baud_rate.hpp"
#include "zeroerr_rmd/actuator_state/control_mode.hpp"
#include "zeroerr_rmd/actuator_state/error_code.hpp"
#include "zeroerr_rmd/actuator_state/feedback.hpp"
#include "zeroerr_rmd/actuator_state/gains.hpp"
#include "zeroerr_rmd/actuator_state/motor_status_1.hpp"
#include "zeroerr_rmd/actuator_state/motor_status_2.hpp"
#include "zeroerr_rmd/actuator_state/motor_status_3.hpp"
#include "zeroerr_rmd/can/exceptions.hpp"
#include "zeroerr_rmd/can/frame.hpp"
#include "zeroerr_rmd/can/node.hpp"
#include "zeroerr_rmd/driver/can_driver.hpp"
#include "zeroerr_rmd/driver/driver.hpp"
#include "zeroerr_rmd/actuator_constants.hpp"
#include "zeroerr_rmd/actuator_interface.hpp"
#include "zeroerr_rmd/exceptions.hpp"
#include "zeroerr_rmd/io.hpp"


namespace zeroerr_rmd {
  namespace bindings {

    /**\fn declareActuator
     * \brief
     *    Helper function for declaring the actuator constants for a given actuator
     * 
     * \tparam T
     *    The class containing the actuator constants
     * \param[in] m
     *    Pybind11 module that the actuator constants should be declared into
     * \param[in] class_name
     *    Class name of the corresponding Python bindings
    */
    template<typename T>
    void declareActuator(pybind11::module& m, std::string const& class_name) {
      pybind11::class_<T>(m, class_name.c_str())
        .def_readonly_static("reducer_ratio", &T::reducer_ratio)
        .def_readonly_static("rated_speed", &T::rated_speed)
        .def_readonly_static("rated_current", &T::rated_current)
        .def_readonly_static("rated_power", &T::rated_power)
        .def_readonly_static("rated_torque", &T::rated_torque)
        .def_readonly_static("torque_constant", &T::torque_constant)
        .def_readonly_static("rotor_inertia", &T::rotor_inertia);
      return;
    }

  }
}

PYBIND11_MODULE(zeroerr_rmd_py, m) {

  m.doc() = "Python bindings for ZeroErr RMD-X actuator series";
  pybind11::class_<zeroerr_rmd::Driver>(m, "Driver");
  pybind11::class_<zeroerr_rmd::CanDriver, zeroerr_rmd::Driver>(m, "CanDriver")
    .def(pybind11::init<std::string const&>());
  pybind11::class_<zeroerr_rmd::ActuatorInterface>(m, "ActuatorInterface")
    .def(pybind11::init<zeroerr_rmd::Driver&, std::uint32_t>(), pybind11::keep_alive<1, 2>())
    .def("getAcceleration", &zeroerr_rmd::ActuatorInterface::getAcceleration)
    .def("getCanId", &zeroerr_rmd::ActuatorInterface::getCanId)
    .def("getControllerGains", &zeroerr_rmd::ActuatorInterface::getControllerGains)
    .def("getControlMode", &zeroerr_rmd::ActuatorInterface::getControlMode)
    .def("getMotorModel", &zeroerr_rmd::ActuatorInterface::getMotorModel)
    .def("getMotorPower", &zeroerr_rmd::ActuatorInterface::getMotorPower)
    .def("getMotorStatus1", &zeroerr_rmd::ActuatorInterface::getMotorStatus1)
    .def("getMotorStatus2", &zeroerr_rmd::ActuatorInterface::getMotorStatus2)
    .def("getMotorStatus3", &zeroerr_rmd::ActuatorInterface::getMotorStatus3)
    .def("getMultiTurnAngle", &zeroerr_rmd::ActuatorInterface::getMultiTurnAngle)
    .def("getMultiTurnEncoderPosition", &zeroerr_rmd::ActuatorInterface::getMultiTurnEncoderPosition)
    .def("getMultiTurnEncoderOriginalPosition", &zeroerr_rmd::ActuatorInterface::getMultiTurnEncoderOriginalPosition)
    .def("getMultiTurnEncoderZeroOffset", &zeroerr_rmd::ActuatorInterface::getMultiTurnEncoderZeroOffset)
    .def("getRuntime", &zeroerr_rmd::ActuatorInterface::getRuntime)
    .def("getSingleTurnAngle", &zeroerr_rmd::ActuatorInterface::getSingleTurnAngle)
    .def("getSingleTurnEncoderPosition", &zeroerr_rmd::ActuatorInterface::getSingleTurnEncoderPosition)
    .def("getVersionDate", &zeroerr_rmd::ActuatorInterface::getVersionDate)
    .def("lockBrake", &zeroerr_rmd::ActuatorInterface::lockBrake)
    .def("releaseBrake", &zeroerr_rmd::ActuatorInterface::releaseBrake)
    .def("reset", &zeroerr_rmd::ActuatorInterface::reset)
    .def("sendCurrentSetpoint", &zeroerr_rmd::ActuatorInterface::sendCurrentSetpoint)
    .def("sendPositionAbsoluteSetpoint", &zeroerr_rmd::ActuatorInterface::sendPositionAbsoluteSetpoint)
    .def("sendTorqueSetpoint", &zeroerr_rmd::ActuatorInterface::sendTorqueSetpoint)
    .def("sendVelocitySetpoint", &zeroerr_rmd::ActuatorInterface::sendVelocitySetpoint)
    .def("setAcceleration", &zeroerr_rmd::ActuatorInterface::setAcceleration)
    .def("setCanBaudRate", &zeroerr_rmd::ActuatorInterface::setCanBaudRate)
    .def("setCanId", &zeroerr_rmd::ActuatorInterface::setCanId)
    .def("setControllerGains", &zeroerr_rmd::ActuatorInterface::setControllerGains)
    .def("setCurrentPositionAsEncoderZero", &zeroerr_rmd::ActuatorInterface::setCurrentPositionAsEncoderZero)
    .def("setEncoderZero", &zeroerr_rmd::ActuatorInterface::setEncoderZero)
    .def("setTimeout", &zeroerr_rmd::ActuatorInterface::setTimeout)
    .def("shutdownMotor", &zeroerr_rmd::ActuatorInterface::shutdownMotor)
    .def("stopMotor", &zeroerr_rmd::ActuatorInterface::stopMotor);
  pybind11::register_exception<zeroerr_rmd::Exception>(m, "ActuatorException");
  pybind11::register_exception<zeroerr_rmd::ProtocolException>(m, "ProtocolException");
  pybind11::register_exception<zeroerr_rmd::ValueRangeException>(m, "ValueRangeException");

  auto m_actuator_state = m.def_submodule("actuator_state", "Submodule for actuator state structures");
  pybind11::enum_<zeroerr_rmd::AccelerationType>(m_actuator_state, "AccelerationType")
    .value("POSITION_PLANNING_ACCELERATION", zeroerr_rmd::AccelerationType::POSITION_PLANNING_ACCELERATION)
    .value("POSITION_PLANNING_DECELERATION", zeroerr_rmd::AccelerationType::POSITION_PLANNING_DECELERATION)
    .value("VELOCITY_PLANNING_ACCELERATION", zeroerr_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION)
    .value("VELOCITY_PLANNING_DECELERATION", zeroerr_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION);
  pybind11::enum_<zeroerr_rmd::CanBaudRate>(m_actuator_state, "CanBaudRate")
    .value("KBPS500", zeroerr_rmd::CanBaudRate::KBPS500)
    .value("MBPS1", zeroerr_rmd::CanBaudRate::MBPS1);
  pybind11::enum_<zeroerr_rmd::ControlMode>(m_actuator_state, "ControlMode")
    .value("NONE", zeroerr_rmd::ControlMode::NONE)
    .value("CURRENT", zeroerr_rmd::ControlMode::CURRENT)
    .value("VELOCITY", zeroerr_rmd::ControlMode::VELOCITY)
    .value("POSITION", zeroerr_rmd::ControlMode::POSITION);
  pybind11::enum_<zeroerr_rmd::ErrorCode>(m_actuator_state, "ErrorCode")
    .value("NO_ERROR", zeroerr_rmd::ErrorCode::NO_ERROR)
    .value("MOTOR_STALL", zeroerr_rmd::ErrorCode::MOTOR_STALL)
    .value("LOW_VOLTAGE", zeroerr_rmd::ErrorCode::LOW_VOLTAGE)
    .value("OVERVOLTAGE", zeroerr_rmd::ErrorCode::OVERVOLTAGE)
    .value("OVERCURRENT", zeroerr_rmd::ErrorCode::OVERCURRENT)
    .value("POWER_OVERRUN", zeroerr_rmd::ErrorCode::POWER_OVERRUN)
    .value("SPEEDING", zeroerr_rmd::ErrorCode::SPEEDING)
    .value("UNSPECIFIED_1", zeroerr_rmd::ErrorCode::UNSPECIFIED_1)
    .value("UNSPECIFIED_2", zeroerr_rmd::ErrorCode::UNSPECIFIED_2)
    .value("UNSPECIFIED_3", zeroerr_rmd::ErrorCode::UNSPECIFIED_3)
    .value("OVERTEMPERATURE", zeroerr_rmd::ErrorCode::OVERTEMPERATURE)
    .value("ENCODER_CALIBRATION_ERROR", zeroerr_rmd::ErrorCode::ENCODER_CALIBRATION_ERROR);
  pybind11::class_<zeroerr_rmd::Gains>(m_actuator_state, "Gains")
    .def(pybind11::init<zeroerr_rmd::PiGains const&, zeroerr_rmd::PiGains const&, zeroerr_rmd::PiGains const&>())
    .def(pybind11::init<std::uint8_t const, std::uint8_t const, std::uint8_t const, std::uint8_t const, std::uint8_t const, std::uint8_t const>())
    .def_readwrite("current", &zeroerr_rmd::Gains::current)
    .def_readwrite("speed", &zeroerr_rmd::Gains::speed)
    .def_readwrite("position", &zeroerr_rmd::Gains::position)
    .def("__repr__", [](zeroerr_rmd::Gains const& gains) -> std::string { 
      std::ostringstream ss {};
      ss << gains;
      return ss.str();
    });
  pybind11::class_<zeroerr_rmd::MotorStatus1>(m_actuator_state, "MotorStatus1")
    .def(pybind11::init<int const, bool const, float const, zeroerr_rmd::ErrorCode const>())
    .def_readonly("temperature", &zeroerr_rmd::MotorStatus1::temperature)
    .def_readonly("is_brake_released", &zeroerr_rmd::MotorStatus1::is_brake_released)
    .def_readonly("voltage", &zeroerr_rmd::MotorStatus1::voltage)
    .def_readonly("error_code", &zeroerr_rmd::MotorStatus1::error_code)
    .def("__repr__", [](zeroerr_rmd::MotorStatus1 const& motor_status) -> std::string { 
      std::ostringstream ss {};
      ss << motor_status;
      return ss.str();
    });
  pybind11::class_<zeroerr_rmd::MotorStatus2>(m_actuator_state, "MotorStatus2")
    .def(pybind11::init<int const, float const, float const, float const>())
    .def_readonly("temperature", &zeroerr_rmd::MotorStatus2::temperature)
    .def_readonly("current", &zeroerr_rmd::MotorStatus2::current)
    .def_readonly("shaft_speed", &zeroerr_rmd::MotorStatus2::shaft_speed)
    .def_readonly("shaft_angle", &zeroerr_rmd::MotorStatus2::shaft_angle)
    .def("__repr__", [](zeroerr_rmd::MotorStatus2 const& motor_status) -> std::string { 
      std::ostringstream ss {};
      ss << motor_status;
      return ss.str();
    });
  pybind11::class_<zeroerr_rmd::MotorStatus3>(m_actuator_state, "MotorStatus3")
    .def(pybind11::init<int const, float const, float const, float const>())
    .def_readonly("temperature", &zeroerr_rmd::MotorStatus3::temperature)
    .def_readonly("current_phase_a", &zeroerr_rmd::MotorStatus3::current_phase_a)
    .def_readonly("current_phase_b", &zeroerr_rmd::MotorStatus3::current_phase_b)
    .def_readonly("current_phase_c", &zeroerr_rmd::MotorStatus3::current_phase_c)
    .def("__repr__", [](zeroerr_rmd::MotorStatus3 const& motor_status) -> std::string { 
      std::ostringstream ss {};
      ss << motor_status;
      return ss.str();
    });
  pybind11::class_<zeroerr_rmd::PiGains>(m_actuator_state, "PiGains")
    .def(pybind11::init<std::uint8_t const, std::uint8_t const>())
    .def_readwrite("kp", &zeroerr_rmd::PiGains::kp)
    .def_readwrite("ki", &zeroerr_rmd::PiGains::ki)
    .def("__repr__", [](zeroerr_rmd::PiGains const& pi_gains) -> std::string { 
      std::ostringstream ss {};
      ss << pi_gains;
      return ss.str();
    });

  auto m_can = m.def_submodule("can", "Submodule for basic CAN communication");
  pybind11::class_<zeroerr_rmd::can::Frame>(m_can, "Frame")
    .def(pybind11::init<std::uint32_t const, std::array<std::uint8_t,8> const&>())
    .def("getId", &zeroerr_rmd::can::Frame::getId)
    .def("getData", &zeroerr_rmd::can::Frame::getData);
  pybind11::class_<zeroerr_rmd::can::Node>(m_can, "Node")
    .def(pybind11::init<std::string const&>())
    .def("setRecvFilter", &zeroerr_rmd::can::Node::setRecvFilter)
    .def("read", &zeroerr_rmd::can::Node::read)
    .def("write", pybind11::overload_cast<zeroerr_rmd::can::Frame const&>(&zeroerr_rmd::can::Node::write));
  pybind11::register_exception<zeroerr_rmd::can::SocketException>(m_can, "SocketException");
  pybind11::register_exception<zeroerr_rmd::can::Exception>(m_can, "CanException");
  pybind11::register_exception<zeroerr_rmd::can::TxTimeoutError>(m_can, "TxTimeoutError");
  pybind11::register_exception<zeroerr_rmd::can::LostArbitrationError>(m_can, "LostArbitrationError");
  pybind11::register_exception<zeroerr_rmd::can::ControllerProblemError>(m_can, "ControllerProblemError");
  pybind11::register_exception<zeroerr_rmd::can::ProtocolViolationError>(m_can, "ProtocolViolationError");
  pybind11::register_exception<zeroerr_rmd::can::TransceiverStatusError>(m_can, "TransceiverStatusError");
  pybind11::register_exception<zeroerr_rmd::can::NoAcknowledgeError>(m_can, "NoAcknowledgeError");
  pybind11::register_exception<zeroerr_rmd::can::BusOffError>(m_can, "BusOffError");
  pybind11::register_exception<zeroerr_rmd::can::BusError>(m_can, "BusError");
  pybind11::register_exception<zeroerr_rmd::can::ControllerRestartedError>(m_can, "ControllerRestartedError");

  auto m_actuator_constants = m.def_submodule("actuator_constants", "Submodule for actuator constants");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X4V2>(m_actuator_constants,     "X4V2");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X4V3>(m_actuator_constants,     "X4V3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X4_3>(m_actuator_constants,     "X4_3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X4_24>(m_actuator_constants,    "X4_24");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X6V2>(m_actuator_constants,     "X6V2");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X6S2V2>(m_actuator_constants,   "X6S2V2");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X6V3>(m_actuator_constants,     "X6V3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X6_7>(m_actuator_constants,     "X6_7");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X6_8>(m_actuator_constants,     "X6_8");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X6_40>(m_actuator_constants,    "X6_40");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8V2>(m_actuator_constants,     "X8V2");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8ProV2>(m_actuator_constants,  "X8ProV2");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8S2V3>(m_actuator_constants,   "X8S2V3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8HV3>(m_actuator_constants,    "X8HV3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8ProHV3>(m_actuator_constants, "X8ProHV3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8_20>(m_actuator_constants,    "X8_20");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8_25>(m_actuator_constants,    "X8_25");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8_60>(m_actuator_constants,    "X8_60");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X8_90>(m_actuator_constants,    "X8_90");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X10V3>(m_actuator_constants,    "X10V3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X10S2V3>(m_actuator_constants,  "X10S2V3");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X10_40>(m_actuator_constants,   "X10_40");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X10_100>(m_actuator_constants,  "X10_100");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X12_150>(m_actuator_constants,  "X12_150");
  zeroerr_rmd::bindings::declareActuator<zeroerr_rmd::X15_400>(m_actuator_constants,  "X15_400");

}
