/********************************************************************************
 *  File Name:
 *    rf24_network_connection.cpp
 *
 *  Description:
 *    Declares implementation of the RF24 network connection interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/interfaces/endpoint_intf.hpp>
#include <RF24Node/src/network/definitions.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection_internal.hpp>

namespace RF24::Network::Internal::Processes::Connection
{
  enum class State
  {
    CONNECT_IDLE,
    CONNECT_REQUEST,
    CONNECT_WAIT_FOR_RESPONSE,
    CONNECT_RESPONSE,
    CONNECT_SUCCESS,
    CONNECT_TERMINATE,
    CONNECT_EXIT_LOOP
  };

  struct ControlBlock
  {
    State currentState;
    RF24::Endpoint::Connection::Callback onEventCallback;
    size_t timeout;

    ControlBlock()
    {
      currentState = State::CONNECT_IDLE;
      onEventCallback = nullptr;
      timeout = 0;
    }
  };

  static bool s_process_initialized;
  static ControlBlock s_system_connections[ RF24::Network::MAX_CONNECTIONS ];

  bool begin( RF24::Network::Interface &obj, const RF24::LogicalAddress node, RF24::Endpoint::Connection::Callback callback, const size_t timeout )
  {
    /*------------------------------------------------
    Startup conditions
    ------------------------------------------------*/
    if ( !s_process_initialized )
    {
      init();
    }
    else if ( !isAddressValid( node ) || !timeout || !callback )
    {
      return false;
    }
    else if ( obj.isConnectedTo( node ) )
    {
      // Already registered, awesome!
      callback( RF24::Endpoint::Connection::Result::CONNECTION_SUCCESS );
      return true;
    }

    /*------------------------------------------------
    Process the connection request to see if we can fit it in
    ------------------------------------------------*/
    auto thisNode = obj.thisNode();

    if ( isDirectDescendent( node, thisNode ) )
    {
      /*------------------------------------------------
      Register the connection request as thisNode->parent
      ------------------------------------------------*/
      s_system_connections[ CONNECTION_PARENT ].currentState = State::CONNECT_REQUEST;
      s_system_connections[ CONNECTION_PARENT ].onEventCallback = callback;
      s_system_connections[ CONNECTION_PARENT ].timeout = timeout;
    }
    else if ( isDirectDescendent( thisNode, node ) )
    {
      /*------------------------------------------------
      Great! We know it's a child...which one? Thankfully the last number
      in the node's address is equal to the child number of it's parent.
      ------------------------------------------------*/
      auto level = getIdAtLevel( node, getLevel( node ) );
      auto childId = ConnectionId::CONNECTION_PARENT;

      if ( ConnectionId::CONNECTION_CHILD_1 <= level <= ConnectionId::CONNECTION_CHILD_5 )
      {
        childId = static_cast<ConnectionId>( level );
      }
      else
      {
        return false;
      }

      /*------------------------------------------------
      Register the connection request as child->thisNode
      ------------------------------------------------*/
      s_system_connections[ childId ].currentState = State::CONNECT_REQUEST;
      s_system_connections[ childId ].onEventCallback = callback;
      s_system_connections[ childId ].timeout = timeout;
    }
    
    /*------------------------------------------------
    Fall-through condition. If we get here, the desired node
    to connect with didn't have a direct relationship in the tree.
    ------------------------------------------------*/
    return false;
  }

  void run( ::RF24::Network::Interface& obj, ::RF24::Network::Frame::FrameType& frame, const HeaderMessage type )
  {
    /*------------------------------------------------
    Ensure the process has been initialized properly
    ------------------------------------------------*/
    if ( !s_process_initialized )
    {
      init();
    }

    /*------------------------------------------------
    Handle the message appropriately
    ------------------------------------------------*/
    switch ( type )
    {
      case MSG_NET_REQUEST_BIND:
        requestHandler( obj, frame );
        break;

      case MSG_NET_REQUEST_BIND_ACK:
        ackHandler( obj, frame );
        break;

      case MSG_NET_REQUEST_BIND_NACK:
        nackHandler( obj, frame );
        break;

      default:
        break;
    }
  }

  void init()
  {
    /*------------------------------------------------
    Initialize the process memory to defaults
    ------------------------------------------------*/
    for ( auto& item : s_system_connections )
    {
      item = {};
    }

    s_process_initialized = true;
  }

  void requestHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame )
  {
    /*------------------------------------------------
    Repurpose the existing frame for the response
    ------------------------------------------------*/
    auto cachedSrc = frame.getSrc();
    frame.setSrc( obj.thisNode() );
    frame.setDst( cachedSrc );
    frame.setType( Network::MSG_NET_REQUEST_BIND_NACK );
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
  
  void ackHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame )
  {
  
  }

  void nackHandler( ::RF24::Network::Interface &obj, ::RF24::Network::Frame::FrameType &frame )
  {
  
  }
}
