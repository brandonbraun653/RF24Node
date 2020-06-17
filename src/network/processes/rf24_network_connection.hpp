/********************************************************************************
 *  File Name:
 *    rf24_network_connection.hpp
 *
 *  Description:
 *    Defines the interface for network level connection processing
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_NETWORK_CONNECTION_PROCESSING_HPP
#define RF24_NODE_NETWORK_CONNECTION_PROCESSING_HPP


/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/interfaces/network_intf.hpp>
#include <RF24Node/src/network/frame/frame.hpp>

namespace RF24::Network::Internal::Processes::Connection
{
  /**
   *	Starts the process of creating a new connection to a node
   *
   *	@param[in]  obj         The network driver instance to utilize
   *	@param[in]	node        The node to try and connect to
   *	@param[in]	callback    Callback to execute upon success/fail/timeout
   *	@param[in]	timeout     How long to wait for the connection to succeed
   *	@return bool
   */
  bool startConnect( RF24::Network::Interface &obj, const RF24::LogicalAddress node,
                     RF24::Connection::OnCompleteCallback callback, const size_t timeout );

  /**
   *	Disconnects from a previously connected bind site (parent or child)
   *
   *	@param[in]  obj         The network driver instance to utilize
   *	@param[in]	id          The bindsite that is being disconnected from
   *	@param[in]	callback    Callback to execute upon success/fail/timeout
   *	@param[in]	timeout     How long to wait for the disconnect to succeed
   *	@return bool
   */
  bool startDisconnect( RF24::Network::Interface &obj, RF24::Connection::BindSite id,
                        RF24::Connection::OnCompleteCallback callback, const size_t timeout );

  /**
   *  Runs the connection handling processing logic to allow connections to form
   *  between this node and parents/children.
   *
   *	@param[in]	obj         The network driver instance to utilize
   *	@param[in]	frame       The frame that was received (optional)
   *	@return void
   */
  void runConnectProcess( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType *frame );

}    // namespace RF24::Network::Internal::Processes

#endif /* !RF24_NODE_NETWORK_CONNECTION_PROCESSING_HPP */
