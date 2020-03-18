/********************************************************************************
 *  File Name:
 *    rf24_network_ping.hpp
 *
 *  Description:
 *    Defines processes for Ping operations
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_NETWORK_PING_HPP
#define RF24_NODE_NETWORK_PING_HPP

/* RF24 Includes */
#include <RF24Node/src/interfaces/network_intf.hpp>
#include <RF24Node/src/network/frame/frame.hpp>

namespace RF24::Network::Internal::Processes
{
  /**
   *	Handles a ping request from another node
   *
   *	@param[in]	obj     The network driver doing the handling
   *  @param[in]  frame   The ping frame that was received
   *	@return bool
   */
  bool handlePingRequest( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame );
}    // namespace RF24::Network::Internal::Processes

#endif /* !RF24_NODE_NETWORK_PING_HPP */