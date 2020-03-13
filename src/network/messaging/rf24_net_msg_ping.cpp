/********************************************************************************
 *  File Name:
 *    rf24_net_msg_ping.cpp
 *
 *  Description:
 *    Declares the Ping message module
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* RF24 Includes */
#include <RF24Node/src/network/message>

namespace RF24::Network::Messages::Ping
{
  bool isPingRequest( ::RF24::Network::Frame::FrameType &frame )
  {
    uint8_t tmp = 0;
    memcpy( &tmp, frame.peekPayload() + offsetof( RawRequest, sub_id ), sizeof( RawRequest::sub_id ) );
    return tmp == static_cast<uint8_t>( SubId::SUB_ID_PING_REQUEST );
  }

  bool isPingResponse(::RF24::Network::Frame::FrameType &frame )
  {
    uint8_t tmp = 0;
    memcpy( &tmp, frame.peekPayload() + offsetof( RawRequest, sub_id ), sizeof( RawRequest::sub_id ) );
    return tmp == static_cast<uint8_t>( SubId::SUB_ID_PING_RESPONSE );
  }

  void requestFactory( ::RF24::Network::Frame::FrameType &frame, const LogicalAddress dst, const LogicalAddress src )
  {
    RawRequest tempPayload;
    tempPayload.clear();
    tempPayload.dispatcher = src;
    tempPayload.sub_id     = static_cast<uint8_t>( SubId::SUB_ID_PING_REQUEST );

    frame.setDst( dst );
    frame.setSrc( src );
    frame.setType( Network::MSG_NETWORK_PING );
    frame.setPayload( &tempPayload, sizeof( RawRequest ) );
    frame.updateCRC();
  }

  void responseFactory( ::RF24::Network::Frame::FrameType &frame, const LogicalAddress dst, const LogicalAddress src, const bool ack )
  {
    RawResponse tempResponse;
    tempResponse.clear();

    tempResponse.ack       = ack;
    tempResponse.responder = src;
    tempResponse.sub_id    = static_cast<uint8_t>( SubId::SUB_ID_PING_RESPONSE );

    frame.setDst( dst );
    frame.setSrc( src );
    frame.setType( Network::MSG_NETWORK_PING );
    frame.setPayload( &tempResponse, sizeof( RawResponse ) );
    frame.updateCRC();
  }

  bool responseValidator( const void *const response, const LogicalAddress expected )
  {
    RawResponse tempResponse;
    tempResponse.clear();

    memcpy( &tempResponse, response, sizeof( RawResponse ) );

    /* clang-format off */
    if ( ( tempResponse.sub_id == static_cast<uint8_t>( SubId::SUB_ID_PING_RESPONSE ) ) && 
         ( tempResponse.responder == expected ) &&
         ( tempResponse.ack )
       ) /* clang-format on */
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}    // namespace RF24::Network::Messages::Ping