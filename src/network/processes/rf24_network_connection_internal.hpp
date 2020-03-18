/********************************************************************************
 *  File Name:
 *    rf24_network_connection_internal.hpp
 *
 *  Description:
 *    Defines the internal interface for network level connection processing. Not
 *    to be included at the project level.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_NETWORK_CONNECTION_PROCESSING_INTERNAL_HPP
#define RF24_NODE_NETWORK_CONNECTION_PROCESSING_INTERNAL_HPP

/* RF24 Includes */
#include <RF24Node/src/interfaces/network_intf.hpp>
#include <RF24Node/src/network/frame/frame.hpp>

namespace RF24::Network::Internal::Processes::Connection
{
  /**
   *  Initializes the connection sub-system	
   *	
   *	@return void
   */
  void init();

  /**
   *	Handles processing a bind request from another node on the network
   *
   *	@param[in]	obj           The endpoint object that is processing the request
   *	@param[in]	frame         The connection frame that was received
   *	@return void
   */
  void requestHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame );

  /**
   *	Handles processing a bind ACK from another node on the network
   *
   *	@param[in]	obj           The endpoint object that is processing the request
   *	@param[in]	frame         The connection frame that was received
   *	@return void
   */
  void ackHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame );

  /**
   *	Handles processing a bind NACK from another node on the network
   *
   *	@param[in]	obj           The endpoint object that is processing the request
   *	@param[in]	frame         The connection frame that was received
   *	@return void
   */
  void nackHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame );
}    // namespace RF24::Network::Internal::Processes

#endif /* !RF24_NODE_NETWORK_CONNECTION_PROCESSING_INTERNAL_HPP */
