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
#include <RF24Node/src/network/definitions.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection_internal.hpp>

namespace RF24::Network::Internal::Processes::Connection
{
  bool begin( RF24::Network::Interface &obj, const RF24::LogicalAddress node, RF24::Connection::Callback callback,
              const size_t timeout )
  {
    /*------------------------------------------------
    Startup conditions
    ------------------------------------------------*/
    if ( !isAddressValid( node ) || !timeout || !callback )
    {
      return false;
    }
    else if ( obj.isConnectedTo( node ) )
    {
      // Already registered, awesome!
      callback( RF24::Connection::Result::CONNECTION_SUCCESS );
      return true;
    }

    /*------------------------------------------------
    Process the connection request only if the node is
    a direct parent || a direct child.
    ------------------------------------------------*/
    auto thisNode                    = obj.thisNode();
    auto connectToParent             = isDirectDescendent( node, thisNode );
    auto connectToChild              = isDirectDescendent( thisNode, node );
    ControlBlockList &connectionList = obj.getConnectionList();

    if ( connectToParent || connectToChild )
    {
      auto connectID = getDirectConnectionID( node );

      connectionList[ connectID ].connectId       = connectID;
      connectionList[ connectID ].connectTo       = ( connectToParent ? node : thisNode );
      connectionList[ connectID ].connectFrom     = ( connectToParent ? thisNode : node );
      connectionList[ connectID ].currentState    = State::CONNECT_REQUEST;
      connectionList[ connectID ].onEventCallback = callback;
      connectionList[ connectID ].startTime       = Chimera::millis();
      connectionList[ connectID ].timeout         = timeout;

      obj.setConnectionInProgress( connectID, true );

      return true;
    }

    /*------------------------------------------------
    Fall-through condition. If we get here, the desired node
    to connect with didn't have a direct relationship in the tree.
    ------------------------------------------------*/
    return false;
  }

  void run( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType *frame )
  {
    /*------------------------------------------------
    Initialize any variables needed for the algorithm
    ------------------------------------------------*/
    RF24::Network::Frame::FrameType tmpFrame;

    /*------------------------------------------------
    Handle the message appropriately
    ------------------------------------------------*/
    ControlBlockList& connectionList = obj.getConnectionList();
    for ( ControlBlock &connection : connectionList )
    {
      /*------------------------------------------------
      Handle processing states/events where this node is receiving data
      ------------------------------------------------*/
      if ( frame && ( frame->getDst() == obj.thisNode() ) )
      {
        switch ( frame->getType() )
        {
          case MSG_NET_REQUEST_BIND:
            requestHandler( obj, *frame, connection );
            break;

          case MSG_NET_REQUEST_BIND_ACK:
            ackHandler( obj, *frame, connection );
            break;

          case MSG_NET_REQUEST_BIND_NACK:
            nackHandler( obj, *frame, connection );
            break;

          default:
            break;
        }
      }

      /*------------------------------------------------
      Handle processing states where this node takes action of some kind
      ------------------------------------------------*/
      switch ( connection.currentState )
      {
        case State::CONNECT_IDLE:
          continue;
          break;

        /*------------------------------------------------
        Handles making a connection request from this node
        to another node on the network
        ------------------------------------------------*/
        case State::CONNECT_REQUEST:
        case State::CONNECT_REQUEST_RETRY:
          tmpFrame.setDst( connection.connectTo );
          tmpFrame.setSrc( connection.connectFrom );
          tmpFrame.setType( Network::MSG_NET_REQUEST_BIND );
          tmpFrame.setPayload( nullptr, 0 );

          obj.write( tmpFrame, Network::RoutingStyle::ROUTE_DIRECT );
          connection.previousState = connection.currentState;
          connection.currentState = State::CONNECT_WAIT_FOR_BIND_PARENT_RESPONSE;
          break;

        case State::CONNECT_SUCCESS:
          obj.setConnectionInProgress( connection.connectId, false );

          if ( connection.onEventCallback )
          {
            connection.onEventCallback( RF24::Connection::Result::CONNECTION_SUCCESS );
          }

          //TODO: Need to reset the channel with the right connection ID
          connection = {};
          break;

        case State::CONNECT_TIMEOUT:
          
          break;

        case State::CONNECT_TERMINATE:

          break;

        default:
          break;
      }
    }
  }

  void requestHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
    /*------------------------------------------------
    Input & state protection
    ------------------------------------------------*/
    // You were about to try and properly handle a request to bind that came out of the blue.
    // The whole connnectTo thing doesn't work cause THIS PROCESS HASN'T STARTED YET.

    auto connectTo = getDirectConnectionID( frame.getSrc() );
    if ( connectTo != connection.connectId )
    {
      /*------------------------------------------------
      The node that's trying to connect doesn't match this
      particular control block, so don't continue.
      ------------------------------------------------*/
      return;
    }
    else if ( connection.currentState != State::CONNECT_IDLE )
    {
      /*------------------------------------------------
      The connection is in the process of doing something else, so
      we can't allow the requesting node to start a connection.
      ------------------------------------------------*/
      buildNackPacket( obj, frame );
      obj.write( frame, RF24::Network::RoutingStyle::ROUTE_DIRECT );
      return;
    }

    /*------------------------------------------------
    Try and bind the requesting node into the network
    ------------------------------------------------*/
    auto childToBind = frame.getSrc();
    if ( obj.updateRouteTable( childToBind ) )
    {
      buildAckPacket( obj, frame );
      connection.connectTo = childToBind;
      connection.connectFrom = obj.thisNode();
    }
    else
    {
      buildNackPacket( obj, frame );
    }

    /*------------------------------------------------
    Send the response back using direct routing because only bind requests
    from immediate children are allowed. Perhaps in the future virtual
    paths will be allowed, but that's currently not the case.
    ------------------------------------------------*/
    obj.write( frame, RF24::Network::RoutingStyle::ROUTE_DIRECT );
    connection.currentState = State::CONNECT_WAIT_FOR_CHILD_NODE_ACK;
  }

  void ackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
    /*------------------------------------------------
    Input & state protection
    ------------------------------------------------*/
    if ( frame.getSrc() != connection.connectTo )
    {
      return;
    }

    /*------------------------------------------------
    An ACK in a connection process can happen for many reasons
    ------------------------------------------------*/
    switch ( connection.currentState )
    {
      /*------------------------------------------------
      Child Node Perspective: The parent sent some kind
      of response to us in regards to the bind request.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_BIND_PARENT_RESPONSE:

        /*------------------------------------------------
        Pull out the connection process result
        ------------------------------------------------*/
        if ( frame.getType() == Network::HeaderMessage::MSG_NET_REQUEST_BIND_ACK )
        {
          connection.currentState = State::CONNECT_SUCCESS;
        }
        else
        {
          connection.currentState = State::CONNECT_TERMINATE;
        }

        /*------------------------------------------------
        Let the parent know we received the message
        ------------------------------------------------*/
        buildAckPacket( obj, frame );
        obj.write( frame, Network::RoutingStyle::ROUTE_DIRECT );
        break;

      /*------------------------------------------------
      Parent Node Perspective: The child binding to us has 
      just acknowledged the bind status we sent them.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_CHILD_NODE_ACK:

        // TODO: Need to properly reset with the connection ID
        connection = {};
        break;

      default:
        return;
        break;
    }
  }

  void nackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
  }

  ConnectionId getDirectConnectionID( RF24::LogicalAddress address )
  {
    auto level = getIdAtLevel( address, getLevel( address ) );
    auto id    = ConnectionId::CONNECTION_PARENT;

    if ( ConnectionId::CONNECTION_CHILD_1 <= level <= ConnectionId::CONNECTION_CHILD_5 )
    {
      id = static_cast<ConnectionId>( level );
    }

    return id;
  }

  void buildNackPacket( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame )
  {
    auto cachedSrc = frame.getSrc();
    frame.setSrc( obj.thisNode() );
    frame.setDst( cachedSrc );
    frame.setType( Network::MSG_NET_REQUEST_BIND_NACK );
    frame.setPayload( nullptr, 0 );
  }

  void buildAckPacket( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame )
  {
    auto cachedSrc = frame.getSrc();
    frame.setSrc( obj.thisNode() );
    frame.setDst( cachedSrc );
    frame.setType( Network::MSG_NET_REQUEST_BIND_ACK );
    frame.setPayload( nullptr, 0 );
  }
}    // namespace RF24::Network::Internal::Processes::Connection
