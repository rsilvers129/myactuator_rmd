/**
 * \file feedback.hpp
 * \mainpage
 *    Contains the struct for closed-loop control feedback
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef ZEROERR_RMD__ACTUATOR_STATE__FEEDBACK
#define ZEROERR_RMD__ACTUATOR_STATE__FEEDBACK
#pragma once

#include "zeroerr_rmd/actuator_state/motor_status_2.hpp"


namespace zeroerr_rmd {

  // The feedback struct for any closed-loop control corresponds to motor status 2
  using Feedback = MotorStatus2;

}

#endif // ZEROERR_RMD__ACTUATOR_STATE__FEEDBACK
