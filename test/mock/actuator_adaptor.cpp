#include "actuator_adaptor.hpp"

#include <array>
#include <cstdint>
#include <string>

#include "zeroerr_rmd/can/frame.hpp"
#include "zeroerr_rmd/driver/can_driver.hpp"
#include "zeroerr_rmd/protocol/command_type.hpp"
#include "zeroerr_rmd/protocol/responses.hpp"
#include "zeroerr_rmd/exceptions.hpp"


namespace zeroerr_rmd {
  namespace test {

    void ActuatorAdaptor::handleRequest() {
      can::Frame const frame {read()};
      std::array<std::uint8_t,8> const data {frame.getData()};
      
      if (data[0] == zeroerr_rmd::CommandType::READ_SYSTEM_SOFTWARE_VERSION_DATE) {
        zeroerr_rmd::GetVersionDateResponse const response {getVersionDate()};
        send(response, actuator_id_);
      } else {
        throw zeroerr_rmd::Exception("Unrecognized request");
      }
      return;
    }

    ActuatorAdaptor::ActuatorAdaptor(std::string const& ifname, std::uint32_t const actuator_id)
    : CanNode{ifname}, actuator_id_{actuator_id} {
      this->addId(actuator_id_);
      return;
    }

  }
}
