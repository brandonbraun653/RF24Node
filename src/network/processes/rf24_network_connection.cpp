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
  bool begin( RF24::Network::Interface &obj, const RF24::LogicalAddress node, RF24::Connection::OnCompleteCallback callback,
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
      callback( RF24::Connection::Result::CONNECTION_SUCCESS, RF24::Connection::BindSite::INVALID );
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
      auto index     = static_cast<size_t>( connectID );

      connectionList[ index ].bindId             = connectID;
      connectionList[ index ].connectToAddress   = ( connectToParent ? node : thisNode );
      connectionList[ index ].connectFromAddress = ( connectToParent ? thisNode : node );
      connectionList[ index ].currentState       = State::CONNECT_REQUEST;
      connectionList[ index ].onConnectComplete  = callback;
      connectionList[ index ].startTime          = Chimera::millis();
      connectionList[ index ].timeout            = timeout;

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
    ControlBlockList &connectionList = obj.getConnectionList();
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
          tmpFrame.setDst( connection.connectToAddress );
          tmpFrame.setSrc( connection.connectFromAddress );
          tmpFrame.setType( Network::MSG_NET_REQUEST_BIND );
          tmpFrame.setPayload( nullptr, 0 );

          obj.write( tmpFrame, Network::RoutingStyle::ROUTE_DIRECT );
          connection.currentState = State::CONNECT_WAIT_FOR_PARENT_RESPONSE;
          break;

        /*------------------------------------------------
        Either the child's connection attempt to the parent was
        successful or the parent asynchronously bound a child.
        ------------------------------------------------*/
        case State::CONNECT_SUCCESS_DIRECT:     // Child's perspective
        case State::CONNECT_SUCCESS_ASYNC: {    // Parent's perspective
          /*------------------------------------------------
          Inform the user that the connection completed
          ------------------------------------------------*/
          if ( connection.onConnectComplete )
          {
            if ( connection.currentState == State::CONNECT_SUCCESS_DIRECT )
            {
              // Child perspective: The connection to the parent succeeded
              connection.onConnectComplete( RF24::Connection::Result::CONNECTION_SUCCESS, connection.bindId );
            }
            else if ( connection.currentState == State::CONNECT_SUCCESS_ASYNC )
            {
              // Parent perspective: A child node just bound to me
              connection.onConnectComplete( RF24::Connection::Result::CONNECTION_BOUND, connection.bindId );
            }
          }
          // else no callback was registered

          /*------------------------------------------------
          Reset the tracking block information to defaults
          ------------------------------------------------*/
          auto cachedId = connection.bindId;

          obj.setConnectionInProgress( cachedId, false );
          connection        = {};
          connection.bindId = cachedId;
          break;
        }

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
    auto connectTo = getDirectConnectionID( frame.getSrc() );
    if ( connectTo != connection.bindId )
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
      connection.connectToAddress   = childToBind;
      connection.connectFromAddress = obj.thisNode();

      obj.setConnectionInProgress( connection.bindId, true );
    }
    else
    {
      buildNackPacket( obj, frame );
    }

    /*------------------------------------------------
    Inform the binding node of the result and transition
    to waiting for their acknowledgment
    ------------------------------------------------*/
    obj.write( frame, RF24::Network::RoutingStyle::ROUTE_DIRECT );
    connection.currentState = State::CONNECT_WAIT_FOR_CHILD_ACK;
  }

  void ackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
    /*------------------------------------------------
    If this particular connection control block isn't
    the intended handler of the frame, exit early.
    ------------------------------------------------*/
    if ( frame.getSrc() != connection.connectToAddress )
    {
      return;
    }

    /*------------------------------------------------
    Otherwise process the ACK according to the node type
    ------------------------------------------------*/
    switch ( connection.currentState )
    {
      /*------------------------------------------------
      Child Node Perspective: The parent sent some kind
      of response to us in regards to the bind request.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_PARENT_RESPONSE:
        if ( frame.getType() == Network::HeaderMessage::MSG_NET_REQUEST_BIND_ACK )
        {
          connection.currentState = State::CONNECT_SUCCESS_DIRECT;
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
      just acknowledged the bind status we sent them. The
      connection process is complete, so reset the handler.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_CHILD_ACK: {
        auto cachedId     = connection.bindId;
        connection        = {};
        connection.bindId = cachedId;
        break;
      }

      default:
        return;
        break;
    }
  }

  void nackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
  }

  RF24::Connection::BindSite getDirectConnectionID( RF24::LogicalAddress address )
  {
    using namespace RF24::Connection;

    // Can explicitly convert the pipe identifier (id at level) to a BindSite
    auto level = static_cast<BindSite>( getIdAtLevel( address, getLevel( address ) ) );
    auto id    = BindSite::PARENT;

    if ( ( BindSite::CHILD_1 <= level ) && ( level <= BindSite::CHILD_5 ) )
    {
      id =  level;
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
