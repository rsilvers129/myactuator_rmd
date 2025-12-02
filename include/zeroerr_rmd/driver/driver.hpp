/**
 * \file driver.hpp
 * \mainpage
 *    Contains the driver base class defining the interface
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef ZEROERR_RMD__DRIVER__DRIVER
#define ZEROERR_RMD__DRIVER__DRIVER
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "zeroerr_rmd/can/frame.hpp"


namespace zeroerr_rmd {

  class ActuatorInterface;

  /**\class Driver
   * \brief
   *    Pure abstract base class for drivers
  */
  class Driver {
    public:
      /**\fn addId
       * \brief
       *    Updates the id as well as the send and receive ids in a consistent manner
       * 
       * \param[in] actuator_id
       *    The id of the actuator
      */
      virtual void addId(std::uint32_t const actuator_id) = 0;

      /**\fn send
       * \brief
       *    Writes the given frame to the CAN bus
       * 
       * \param[in] frame
       *    The frame that should be sent
       */
      virtual void send(can::Frame const& frame) = 0;

      /**\fn sendRecv
       * \brief
       *    Writes the given frame and waits for a response
       * 
       * \param[in] frame
       *    The frame that should be sent
       * \return
       *    The response frame
       */
      [[nodiscard]]
      virtual can::Frame sendRecv(can::Frame const& frame) = 0;

    protected:
      Driver() = default;
      Driver(Driver const&) = default;
      Driver& operator = (Driver const&) = default;
      Driver(Driver&&) = default;
      Driver& operator = (Driver&&) = default;

      friend ActuatorInterface;
  };

}

#endif // ZEROERR_RMD__DRIVER__DRIVER
