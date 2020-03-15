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

  void bindRequestHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame )
  {
    /*------------------------------------------------
    Repurpose the existing frame for the response
    ------------------------------------------------*/
    auto cachedSrc = frame.getSrc();
    frame.setSrc( obj.thisNode() );
    frame.setDst( cachedSrc );
    frame.setType( Network::MSG_NET_REQUEST_BIND_FULL );
    frame.setPayload( nullptr, 0 );

    /*------------------------------------------------
    This will only update if the source address is a direct descendant
    ------------------------------------------------*/
    if ( obj.updateRouteTable( cachedSrc ) )
    {
      frame.setType( Network::MSG_NET_REQUEST_BIND_ACK );
    }

    /*------------------------------------------------
    Send the response back using direct routing because only bind requests
    from immediate children are allowed. Perhaps in the future virtual
    paths will be allowed, but that's currently not the case.
    ------------------------------------------------*/
    obj.write( frame, RF24::Network::RoutingStyle::ROUTE_DIRECT );
  }

}    // namespace RF24::Network::Internal::Processes
