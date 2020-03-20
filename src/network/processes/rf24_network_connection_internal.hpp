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
   *	Handles processing a new bind request from another node on the network
   *
   *	@param[in]	obj           The endpoint object that is processing the request
   *	@param[in]	frame         The connection frame that was received
   *  @param[in]  connection    The connection control block to use
   *	@return void
   */
  void requestHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection );

  /**
   *	Handles processing a bind ACK from another node on the network
   *
   *	@param[in]	obj           The endpoint object that is processing the request
   *	@param[in]	frame         The connection frame that was received
   *  @param[in]  connection    The connection control block to use
   *	@return void
   */
  void ackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection );

  /**
   *	Handles processing a bind NACK from another node on the network
   *
   *	@param[in]	obj           The endpoint object that is processing the request
   *	@param[in]	frame         The connection frame that was received
   *  @param[in]  connection    The connection control block to use
   *	@return void
   */
  void nackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection );

  /**
   *  Gets the connection identifier for the given node address. 
   *
   *  @note Assumes that the address has already been validated to have a direct parent or 
   *        child relationship with the node that is executing this check.
   *	
   *	@param[in]	address   The address to get the connection id for
   *	@return RF24::Network::ConnectionId
   */
  ConnectionId getDirectConnectionID( RF24::LogicalAddress address );

  /**
   *	Modifies a received packet from some node and converts it into a connection
   *  NACK packet that can be sent back to that node.
   *	
   *	@param[in]	obj       The network driver that received the packet
   *	@param[in]	frame     The frame to be modified
   *	@return void
   */
  void buildNackPacket( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame );

  /**
   *	Modifies a received packed from some node and converts it into a connection
   *  ACK packet that can be sent back to that node.
   *	
   *	@param[in]	obj       The network driver that received the packet
   *	@param[in]	frame     The frame to be modified
   *	@return void
   */
  void buildAckPacket( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame );

}    // namespace RF24::Network::Internal::Processes

#endif /* !RF24_NODE_NETWORK_CONNECTION_PROCESSING_INTERNAL_HPP */
