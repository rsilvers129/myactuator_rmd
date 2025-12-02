/**
 * \file actuator_mock.hpp
 * \mainpage
 *    Contains a mock for the actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef ZEROERR_RMD__TEST__MOCK__ACTUATOR_MOCK
#define ZEROERR_RMD__TEST__MOCK__ACTUATOR_MOCK
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <gmock/gmock.h>

#include "zeroerr_rmd/protocol/responses.hpp"
#include "actuator_adaptor.hpp"


namespace zeroerr_rmd {
  namespace test {

    /**\class ActuatorMock
     * \brief
     *    Mock of the real actuator that can be used for testing the driver over a (virtual) CAN network interface
    */
    class ActuatorMock: public ActuatorAdaptor {
      public:
        /**\fn ActuatorMock
         * \brief
         *    Class constructor
         * 
         * \param[in] ifname
         *    The name of the (virtual) CAN network interface that should be used as a loopback device
         * \param[in] actuator_id
         *    The actuator id for the actuator and the actuator mock
        */
        ActuatorMock(std::string const& ifname, std::uint32_t const actuator_id);
        ActuatorMock() = delete;
        ActuatorMock(ActuatorMock const&) = delete;
        ActuatorMock& operator = (ActuatorMock const&) = default;
        ActuatorMock(ActuatorMock&&) = default;
        ActuatorMock& operator = (ActuatorMock&&) = default;

        // Register all the virtual methods of the adaptor as mock methods so we can control their behavior
        MOCK_METHOD(zeroerr_rmd::GetVersionDateResponse, getVersionDate, (), (const, override));
    };

  }
}

#endif // ZEROERR_RMD__TEST__MOCK__ACTUATOR_MOCK
