/**
 * \file can_node.hpp
 * \mainpage
 *    Contains the base class for the CAN driver as well as the actuator mock
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef ZEROERR_RMD__DRIVER__CAN_NODE
#define ZEROERR_RMD__DRIVER__CAN_NODE
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "zeroerr_rmd/can/frame.hpp"
#include "zeroerr_rmd/can/node.hpp"
#include "zeroerr_rmd/driver/driver.hpp"
#include "zeroerr_rmd/protocol/message.hpp"
#include "zeroerr_rmd/exceptions.hpp"


namespace zeroerr_rmd {

  /**\class CanNode
   * \brief
   *    Base class for the CAN driver as well as the actuator mock
  */
  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  class CanNode: public Driver, protected can::Node {
    protected:
      /**\fn CanNode
       * \brief
       *    Class constructor
       * 
       * \param[in] ifname
       *    The name of the network interface that should communicated over
      */
      CanNode(std::string const& ifname);
      CanNode() = delete;
      CanNode(CanNode const&) = delete;
      CanNode& operator = (CanNode const&) = default;
      CanNode(CanNode&&) = default;
      CanNode& operator = (CanNode&&) = default;

      /**\fn addId
       * \brief
       *    Updates the id as well as the send and receive ids in a consistent manner
       * 
       * \param[in] actuator_id
       *    The id of the actuator [0, 32]
      */
      void addId(std::uint32_t const actuator_id) override;

      /**\fn send
       * \brief
       *    Writes a given CAN frame to the bus
       * 
       * \param[in] frame
       *    The frame that should be sent
       */
      inline void send(can::Frame const& frame) override;

      /**\fn sendRecv
       * \brief
       *    Writes a given CAN frame and waits for a reply
       * 
       * \param[in] frame
       *    The frame that should be sent
       * \return
       *    The response frame
       */
      [[nodiscard]]
      inline can::Frame sendRecv(can::Frame const& frame) override;

    protected:
      /**\fn getCanSendId
       * \brief
       *    Get the CAN id that a message should be sent to
       * 
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be sent to
       * \return
       *    The CAN id that the message should be sent to
      */
      [[nodiscard]]
      constexpr std::uint32_t getCanSendId(std::uint32_t const actuator_id) noexcept;

      /**\fn getCanReceiveId
       * \brief
       *    Get the CAN id that a message will be received from
       * 
       * \param[in] actuator_id
       *    The ID of the actuator that the message should be received from
       * \return
       *    The CAN id that the message should be received from
      */
      [[nodiscard]]
      constexpr std::uint32_t getCanReceiveId(std::uint32_t const actuator_id) noexcept;

      std::vector<std::uint32_t> actuator_ids_;
  };

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::CanNode(std::string const& ifname)
  : can::Node{ifname}, Driver{} {
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::addId(std::uint32_t const actuator_id) {
    if ((actuator_id < 1) || (actuator_id > 32)) {
      throw Exception("Given actuator id '" + std::to_string(actuator_id) + "' out of admittable range [1, 32]!");
    }
    actuator_ids_.push_back(actuator_id);
    std::vector<std::uint32_t> can_receive_ids {};
    for (auto const& id: actuator_ids_){
      can_receive_ids.emplace_back(getCanReceiveId(id));
    }
    setRecvFilter(can_receive_ids);
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  void CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::send(can::Frame const& frame) {
    write(frame);
    return;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  can::Frame CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::sendRecv(can::Frame const& frame) {
    write(frame);
    return can::Node::read();
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  constexpr std::uint32_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCanSendId(std::uint32_t const actuator_id) noexcept {
    return SEND_ID_OFFSET + actuator_id;
  }

  template <std::uint32_t SEND_ID_OFFSET, std::uint32_t RECEIVE_ID_OFFSET>
  constexpr std::uint32_t CanNode<SEND_ID_OFFSET,RECEIVE_ID_OFFSET>::getCanReceiveId(std::uint32_t const actuator_id) noexcept {
    return RECEIVE_ID_OFFSET + actuator_id;
  }

}

#endif // ZEROERR_RMD__DRIVER__CAN_NODE
