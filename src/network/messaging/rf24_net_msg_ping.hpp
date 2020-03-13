/********************************************************************************
 *  File Name:
 *    rf24_net_msg_ping.hpp
 *
 *  Description:
 *    Defines methods for interacting with Ping payloads
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_MESSAGING_PING_HPP
#define RF24_NODE_MESSAGING_PING_HPP

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/network/frame/types.hpp>

namespace RF24::Network::Messages::Ping
{
  /* Sub ids for message group type MSG_NETWORK_PING */
  enum class SubId : uint8_t
  {
    SUB_ID_PING_REQUEST  = 0x3A,
    SUB_ID_PING_RESPONSE = 0xA3
  };

  struct RawResponse
  {
    uint8_t sub_id;           /**< Identifier for this ping message */
    uint8_t ack;              /**< Did the node decide to ACK or NACK? (boolean)*/
    LogicalAddress responder; /**< The node that is responding to the ack */

    RawResponse() : sub_id( 0 ), ack( 0 ), responder( ::RF24::Network::RSVD_ADDR_INVALID )
    {
    }

    void clear()
    {
      sub_id    = 0;
      ack       = 0;
      responder = ::RF24::Network::RSVD_ADDR_INVALID;
    }
  };

  struct RawRequest
  {
    uint8_t sub_id;            /**< Identifier for this ping message */
    uint8_t pad;               /**< Pad for byte alignment */
    LogicalAddress dispatcher; /**< Node that is sending the ping request */

    RawRequest() : sub_id( 0 ), pad( 0 ), dispatcher( ::RF24::Network::RSVD_ADDR_INVALID )
    {
    }

    void clear()
    {
      sub_id     = 0;
      pad        = 0;
      dispatcher = ::RF24::Network::RSVD_ADDR_INVALID;
    }
  };

  bool isPingRequest( ::RF24::Network::Frame::FrameType &frame );

  bool isPingResponse( ::RF24::Network::Frame::FrameType &frame );

  /**
   *	Builds a Ping Request frame for the user
   *
   *	@param[in]	frame   The frame to be modified
   *	@param[in]	dst     The destination node to try and ping
   *	@param[in]	src     The node that is sending the ping
   *	@return void
   */
  void requestFactory( ::RF24::Network::Frame::FrameType &frame, const LogicalAddress dst, const LogicalAddress src );

  /**
   *  Builds a Ping Response frame for the user
   *
   *	@param[in]	frame   The frame to be modified
   *	@param[in]	dst     The destination node to send the response back to
   *	@param[in]	src     The source node that is generating the ping response
   *	@param[in]	ack     Whether or not the ACK is valid
   *	@return void
   */
  void responseFactory( ::RF24::Network::Frame::FrameType &frame, const LogicalAddress dst, const LogicalAddress src, const bool ack );

  /**
   *	Validates if the response contains a valid ack from the expected node
   *
   *	@param[in]	response    The raw payload buffer from the response frame
   *	@param[in]	expected    The expected node to have sent the ack
   *	@return bool
   */
  bool responseValidator( const void *const response, const LogicalAddress expected );

}    // namespace RF24::Network::Messages::Ping

#endif /* !RF24_NODE_MESSAGING_PING_HPP */
