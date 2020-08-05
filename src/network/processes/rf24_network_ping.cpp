/********************************************************************************
 *  File Name:
 *    rf24_network_ping.cpp
 *
 *  Description:
 *    Declares Ping process implementations
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* RF24 Includes */
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/network/message>
#include <RF24Node/src/network/processes/rf24_network_ping.hpp>


namespace RF24::Network::Internal::Processes
{
  bool handlePingRequest( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame )
  {
    using namespace ::RF24::Network::Messages::Ping;

    /*------------------------------------------------
    Pull out the request information from the frame payload
    ------------------------------------------------*/
    RawRequest tmp;
    tmp.clear();
    memcpy( &tmp, frame.peekPayload(), sizeof( RawRequest ) );

    /*------------------------------------------------
    Build up a response and send it back through the network
    ------------------------------------------------*/
    ::RF24::Network::Messages::Ping::responseFactory( frame, tmp.dispatcher, obj.thisNode(), true );
    return obj.write( frame, ROUTE_NORMALLY );
  }


  void handlePingResponse( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame )
  {
  }
}    // namespace RF24::Network::Internal::Processes
