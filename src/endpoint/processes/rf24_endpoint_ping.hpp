/********************************************************************************
 *  File Name:
 *    rf24_endpoint_ping.hpp
 *
 *  Description:
 *    Defines the ping algorithm interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_PING_HPP
#define RF24_NODE_PING_HPP

/* RF24 Includes */
#include <RF24Node/src/interfaces/endpoint_intf.hpp>

namespace RF24::Endpoint::Internal::Processes
{
  /**
   *	Kicks off a state machine that handles pinging any node on the network
   *
   *	@param[in]	obj       The endpoint object to use for the pinging process
   *	@param[in]	node      Which node to try and ping
   *	@param[in]	timeout   How long the processor can take to succeed at a ping (mS)
   *	@return bool
   */
  bool dispatchPing( ::RF24::Endpoint::Interface &obj, const ::RF24::LogicalAddress node, const size_t timeout );

}    // namespace RF24::Endpoint::Internal::Processor

#endif /* !RF24_NODE_PING_HPP */
