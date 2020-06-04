/********************************************************************************
 *  File Name:
 *    rf24_network_connection.cpp
 *
 *  Description:
 *    Declares implementation of the RF24 network connection interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

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
      connectionList[ index ].lastEventTime      = Chimera::millis();
      connectionList[ index ].processTimeout     = timeout;
      connectionList[ index ].startTime          = Chimera::millis();

      obj.setConnectionInProgress( connectID, true );

      return true;
    }

    /*------------------------------------------------
    Fall-through condition. If we get here, the desired node
    to connect with didn't have a direct relationship in the tree.
    ------------------------------------------------*/
    uLog::getRootSink()->flog( uLog::Level::LVL_DEBUG, "Invalid configuration. Can't start connection\n" );
    return false;
  }

  void run( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType *frame )
  {
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
      Handle high level process timeout behavior
      ------------------------------------------------*/
      auto deltaT = Chimera::millis() - connection.lastEventTime;
      if ( ( deltaT > connection.processTimeout ) && ( connection.currentState != State::CONNECT_IDLE ) )
      {
        connection.currentState = State::CONNECT_TIMEOUT;
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
          connection.frameCache.setDst( connection.connectToAddress );
          connection.frameCache.setSrc( connection.connectFromAddress );
          connection.frameCache.setType( Network::MSG_NET_REQUEST_BIND );
          connection.frameCache.setPayload( nullptr, 0 );

          obj.write( connection.frameCache, Network::RoutingStyle::ROUTE_DIRECT );

          connection.lastEventTime = Chimera::millis();
          connection.connectAttempts += 1;
          connection.currentState = State::CONNECT_WAIT_FOR_PARENT_RESPONSE;
          break;

        /*------------------------------------------------
        Handles timeout and retransmit operations when the sending 
        node is waiting on the receiver to send something back
        ------------------------------------------------*/
        case State::CONNECT_WAIT_FOR_CHILD_ACK:
        case State::CONNECT_WAIT_FOR_PARENT_RESPONSE:
          /*------------------------------------------------
          We've spent too long waiting for the other node to respond
          ------------------------------------------------*/
          if ( deltaT > connection.netTimeout )
          {
            /*------------------------------------------------
            Try and send the last packet again if we can, otherwise just fail out
            ------------------------------------------------*/
            if( connection.connectAttempts < connection.maxAttempts )
            {
              obj.write( connection.frameCache, Network::RoutingStyle::ROUTE_DIRECT );

              connection.lastEventTime = Chimera::millis();
              connection.connectAttempts += 1;
            }
            else
            {
              connection.currentState = State::CONNECT_TERMINATE;
              connection.result       = RF24::Connection::Result::CONNECTION_NO_RESPONSE;
            }
          }
          break;

        /*------------------------------------------------
        The process timed out
        ------------------------------------------------*/
        case State::CONNECT_TIMEOUT:
          connection.result       = RF24::Connection::Result::CONNECTION_TIMEOUT;
          connection.currentState = State::CONNECT_TERMINATE;
          break;

        /*------------------------------------------------
        Handle the various process exit conditions
        ------------------------------------------------*/
        case State::CONNECT_SUCCESS_DIRECT:    // Child's perspective
        case State::CONNECT_SUCCESS_ASYNC:     // Parent's perspective
        case State::CONNECT_TERMINATE: {       // Parent or Child perspective    
          /*------------------------------------------------
          Inform the user that the connection completed
          ------------------------------------------------*/
          if ( connection.onConnectComplete )
          {
            connection.onConnectComplete( connection.result, connection.bindId );
          }

          /*------------------------------------------------
          Reset the tracking block information to defaults
          ------------------------------------------------*/
          auto cachedId = connection.bindId;

          obj.setConnectionInProgress( cachedId, false );
          connection        = {};
          connection.bindId = cachedId;
          break;
        }

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
    connection.frameCache = frame;
    obj.write( connection.frameCache, RF24::Network::RoutingStyle::ROUTE_DIRECT );
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
          connection.result       = RF24::Connection::Result::CONNECTION_SUCCESS;
        }
        else
        {
          connection.currentState = State::CONNECT_TERMINATE;
          connection.result       = RF24::Connection::Result::CONNECTION_FAILED;
        }

        /*------------------------------------------------
        Let the parent know we received the message
        ------------------------------------------------*/
        buildAckPacket( obj, frame );

        connection.frameCache = frame;
        obj.write( connection.frameCache, Network::RoutingStyle::ROUTE_DIRECT );
        break;

      /*------------------------------------------------
      Parent Node Perspective: The child binding to us has
      just acknowledged the bind status we sent them.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_CHILD_ACK:
        connection.currentState = State::CONNECT_SUCCESS_ASYNC;
        connection.result       = RF24::Connection::Result::CONNECTION_BOUND;
        break;

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
