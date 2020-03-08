/********************************************************************************
 *  File Name:
 *    rf24_net_msg.hpp
 *
 *  Description:
 *    Defines helper structures for interacting with message payloads
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_NETWORKING_MESSAGES_HPP
#define RF24_NODE_NETWORKING_MESSAGES_HPP

/* RF24 Includes */
#include <RF24Node/src/network/frame/frame.hpp>

namespace RF24::Network::Messages
{
  bool serialize( ::RF24::Network::Frame::FrameType &frame, const void *const data, const size_t length );
  
}

#endif  /* !RF24_NODE_NETWORKING_MESSAGES_HPP */
