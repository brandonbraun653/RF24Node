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
#include <RF24Node/src/network/types.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection_internal.hpp>


namespace RF24::Network::Internal::Processes::Connection
{
  /*-------------------------------------------------------------------------------
  Make A Connection
  -------------------------------------------------------------------------------*/
  bool startConnect( RF24::Network::Interface &obj, const RF24::LogicalAddress node,
                     RF24::Connection::OnCompleteCallback callback, const size_t timeout )
  {
    /*------------------------------------------------
    Startup conditions
    ------------------------------------------------*/
    if ( !isAddressValid( node ) || !timeout )
    {
      return false;
    }
    else if ( obj.isConnectedTo( node ) )
    {
      // Already registered, awesome!
      callback( RF24::Connection::Result::CONNECT_PROC_SUCCESS, RF24::Connection::BindSite::INVALID );
      return true;
    }

    /*------------------------------------------------
    Ensure we have exclusive access to the network driver
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::LockGuard( obj );

    /*------------------------------------------------
    Process the connection request only if the node is
    a direct parent || a direct child.
    ------------------------------------------------*/
    auto thisNode        = obj.thisNode();
    auto connectToParent = isDirectDescendent( node, thisNode );
    auto connectToChild  = isDirectDescendent( thisNode, node );

    if ( connectToParent || connectToChild )
    {
      if constexpr ( DBG_LOG_NET_PROC_CONNECT )
      {
        obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Start connect\n", Chimera::millis() );
      }

      auto connectID = getDirectConnectionID( obj, node );
      auto index     = static_cast<size_t>( connectID );

      obj.unsafe_ConnectionList[ index ].bindId            = connectID;
      obj.unsafe_ConnectionList[ index ].toAddress         = ( connectToParent ? node : thisNode );
      obj.unsafe_ConnectionList[ index ].fromAddress       = ( connectToParent ? thisNode : node );
      obj.unsafe_ConnectionList[ index ].currentState      = State::CONNECT_REQUEST;
      obj.unsafe_ConnectionList[ index ].onConnectComplete = callback;
      obj.unsafe_ConnectionList[ index ].lastEventTime     = Chimera::millis();
      obj.unsafe_ConnectionList[ index ].processTimeout    = timeout;
      obj.unsafe_ConnectionList[ index ].startTime         = Chimera::millis();
      obj.unsafe_ConnectionList[ index ].direction         = RF24::Connection::Direction::CONNECT;

      obj.setConnectionInProgress( connectID, RF24::Connection::Direction::CONNECT, true );

      return true;
    }

    /*------------------------------------------------
    Fall-through condition. If we get here, the desired node
    to connect with didn't have a direct relationship in the tree.
    ------------------------------------------------*/
    return false;
  }


  bool startDisconnect( RF24::Network::Interface &obj, RF24::Connection::BindSite id,
                        RF24::Connection::OnCompleteCallback callback, const size_t timeout )
  {
    auto lockGuard = Chimera::Threading::LockGuard( obj );

    /*------------------------------------------------
    Startup conditions
    ------------------------------------------------*/
    auto bindSite = obj.getBindSiteCBSafe( id );
    if ( !bindSite.connected )
    {
      return true;
    }

    if constexpr ( DBG_LOG_NET_PROC_CONNECT )
    {
      obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Start disconnect\n", Chimera::millis() );
    }

    /*------------------------------------------------
    Process the connection request only
    ------------------------------------------------*/
    auto index = static_cast<size_t>( id );

    obj.unsafe_ConnectionList[ index ].bindId            = id;
    obj.unsafe_ConnectionList[ index ].toAddress         = bindSite.address;
    obj.unsafe_ConnectionList[ index ].fromAddress       = obj.thisNode();
    obj.unsafe_ConnectionList[ index ].currentState      = State::CONNECT_REQUEST;
    obj.unsafe_ConnectionList[ index ].onConnectComplete = callback;
    obj.unsafe_ConnectionList[ index ].lastEventTime     = Chimera::millis();
    obj.unsafe_ConnectionList[ index ].processTimeout    = timeout;
    obj.unsafe_ConnectionList[ index ].startTime         = Chimera::millis();
    obj.unsafe_ConnectionList[ index ].direction         = RF24::Connection::Direction::DISCONNECT;

    obj.setConnectionInProgress( id, RF24::Connection::Direction::DISCONNECT, true );

    return true;
  }


  void runConnectProcess( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType *frame )
  {
    /*------------------------------------------------
    Ensure we have exclusive access to the network driver
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::LockGuard( obj );

    /*------------------------------------------------
    Handle the message appropriately
    ------------------------------------------------*/
    for ( ControlBlock &connection : obj.unsafe_ConnectionList )
    {
      const bool isConnect = ( connection.direction == RF24::Connection::Direction::CONNECT );

      /*------------------------------------------------
      Handle processing states/events where this node is receiving data
      ------------------------------------------------*/
      if ( frame && ( frame->getDst() == obj.thisNode() ) )
      {
        switch ( frame->getType() )
        {
          case MSG_NET_REQUEST_CONNECT:
          case MSG_NET_REQUEST_DISCONNECT:
            requestHandler( obj, *frame, connection );
            break;

          case MSG_NET_REQUEST_CONNECT_ACK:
          case MSG_NET_REQUEST_DISCONNECT_ACK:
            ackHandler( obj, *frame, connection );
            break;

          case MSG_NET_REQUEST_CONNECT_NACK:
          case MSG_NET_REQUEST_DISCONNECT_NACK:
            nackHandler( obj, *frame, connection );
            break;

          default:
            break;
        }
      }

      /*------------------------------------------------
      Handle high level process timeout behavior that can
      override any pre-existing state.
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
          /*------------------------------------------------
          Decide what kind of request to generate
          ------------------------------------------------*/
          if ( isConnect )
          {
            connection.frameCache.setType( MSG_NET_REQUEST_CONNECT );

            if constexpr ( DBG_LOG_NET_PROC_CONNECT )
            {
              obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: TX connect request \n", Chimera::millis() );
            }
          }
          else
          {
            connection.frameCache.setType( MSG_NET_REQUEST_DISCONNECT );

            if constexpr ( DBG_LOG_NET_PROC_CONNECT )
            {
              obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: TX disconnect request\n", Chimera::millis() );
            }
          }

          /*------------------------------------------------
          Fill out the network parameters and send it off
          ------------------------------------------------*/
          connection.frameCache.setDst( connection.toAddress );
          connection.frameCache.setSrc( connection.fromAddress );
          connection.frameCache.setPayload( nullptr, 0 );

          obj.write( connection.frameCache, Network::RoutingStyle::ROUTE_DIRECT );

          /*------------------------------------------------
          Move the connection state forward
          ------------------------------------------------*/
          connection.lastEventTime = Chimera::millis();
          connection.attempts += 1;
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
            if ( connection.attempts < connection.maxAttempts )
            {
              obj.write( connection.frameCache, Network::RoutingStyle::ROUTE_DIRECT );

              connection.lastEventTime = Chimera::millis();
              connection.attempts += 1;
            }
            else
            {
              connection.currentState = State::CONNECT_TERMINATE;
              connection.result       = RF24::Connection::Result::CONNECT_PROC_NO_RESPONSE;
            }
          }
          break;

        /*------------------------------------------------
        The process timed out
        ------------------------------------------------*/
        case State::CONNECT_TIMEOUT:
          connection.result       = RF24::Connection::Result::CONNECT_PROC_TIMEOUT;
          connection.currentState = State::CONNECT_TERMINATE;

          if constexpr ( DBG_LOG_NET_PROC_CONNECT )
          {
            obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Connection proc timeout\n", Chimera::millis() );
          }
          break;

        /*------------------------------------------------
        Handle the various process exit conditions
        ------------------------------------------------*/
        case State::CONNECT_SUCCESS_DIRECT:    // Child's perspective
        case State::CONNECT_SUCCESS_ASYNC:     // Parent's perspective
        case State::CONNECT_TERMINATE:         // Parent or Child perspective
        {
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

          obj.setConnectionInProgress( cachedId, connection.direction, false );
          connection              = {};
          connection.bindId       = cachedId;
          connection.currentState = State::CONNECT_IDLE;

          if constexpr ( DBG_LOG_NET_PROC_CONNECT )
          {
            obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Proc exit\n", Chimera::millis() );
          }
          break;
        }

        default:
          break;
      }
    }
  }


  /*-------------------------------------------------------------------------------
  Internal Helper Methods
  -------------------------------------------------------------------------------*/
  void requestHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
    /*------------------------------------------------
    Input & state protection
    ------------------------------------------------*/
    auto connectTo = getDirectConnectionID( obj, frame.getSrc() );
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
      buildNackPacket( obj, frame, connection.direction, connection.currentState );
      obj.write( frame, RF24::Network::RoutingStyle::ROUTE_DIRECT );
      return;
    }

    /*------------------------------------------------
    Try and bind/remove the requesting node on the network
    ------------------------------------------------*/
    auto child            = frame.getSrc();
    bool connectedAlready = obj.isConnectedTo( child );

    switch ( frame.getType() )
    {
      case MSG_NET_REQUEST_CONNECT:
        /*-------------------------------------------------
        Optional logging
        -------------------------------------------------*/
        if constexpr ( DBG_LOG_NET_PROC_CONNECT )
        {
          obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: RX connect request from [%04o]\n", Chimera::millis(), frame.getSrc() );
        }

        /*-------------------------------------------------
        Check on the current node connection status. Try and
        attach if possible, otherwise NACK the process.
        -------------------------------------------------*/
        if ( connectedAlready )
        {
          connection.currentState = State::CONNECT_SUCCESS_DIRECT;
          connection.lastEventTime = Chimera::millis();
          connection.startTime     = Chimera::millis();

          buildAckPacket( obj, frame, connection.direction, connection.currentState );

          if constexpr ( DBG_LOG_NET_PROC_CONNECT )
          {
            obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Node [%04o] already connected\n", Chimera::millis(), frame.getDst() );
          }
        }
        else if ( obj.updateRouteTable( child, true ) )
        {
          connection.toAddress    = child;
          connection.fromAddress  = obj.thisNode();
          connection.direction    = RF24::Connection::Direction::CONNECT;
          connection.currentState = State::CONNECT_WAIT_FOR_CHILD_ACK;

          buildAckPacket( obj, frame, connection.direction, connection.currentState );
          obj.setConnectionInProgress( connection.bindId, connection.direction, true );
        }
        else
        {
          connection.currentState = State::CONNECT_TERMINATE;
          buildNackPacket( obj, frame, connection.direction, connection.currentState );
        }
        break;

      case MSG_NET_REQUEST_DISCONNECT:
        /*-------------------------------------------------
        Optional logging
        -------------------------------------------------*/
        if constexpr ( DBG_LOG_NET_PROC_CONNECT )
        {
          obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: RX disconnect request from [%04o]\n", Chimera::millis(), frame.getSrc() );
        }

        /*-------------------------------------------------
        Check on the current node connection status. Try and
        dettach if possible, otherwise NACK the process.
        -------------------------------------------------*/
        if ( !connectedAlready )
        {
          connection.currentState = State::CONNECT_SUCCESS_DIRECT;
          buildAckPacket( obj, frame, connection.direction, connection.currentState );
        }
        else if ( obj.updateRouteTable( child, false ) )
        {
          connection.toAddress    = child;
          connection.fromAddress  = obj.thisNode();
          connection.direction    = RF24::Connection::Direction::DISCONNECT;
          connection.currentState = State::CONNECT_WAIT_FOR_CHILD_ACK;

          buildAckPacket( obj, frame, connection.direction, connection.currentState );
          obj.setConnectionInProgress( connection.bindId, connection.direction, true );
        }
        else
        {
          connection.currentState = State::CONNECT_TERMINATE;
          buildNackPacket( obj, frame, connection.direction, connection.currentState );
        }
        break;

      default:
        return;
        break;
    };

    connection.frameCache = frame;
    obj.write( connection.frameCache, RF24::Network::RoutingStyle::ROUTE_DIRECT );
  }


  void ackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
    /*------------------------------------------------
    If this particular connection control block isn't
    the intended handler of the frame, exit early.
    ------------------------------------------------*/
    if ( frame.getSrc() != connection.toAddress )
    {
      return;
    }

    /*------------------------------------------------
    Select the proper expected frame type based on the
    kind of connection that is in progress.
    ------------------------------------------------*/
    auto frameTypeACK = Network::HeaderMessage::MSG_NET_REQUEST_CONNECT_ACK;
    if ( connection.direction == RF24::Connection::Direction::DISCONNECT )
    {
      frameTypeACK = Network::HeaderMessage::MSG_NET_REQUEST_DISCONNECT_ACK;
    }

    /*------------------------------------------------
    Process the ACK message based on the current state
    ------------------------------------------------*/
    auto bindIdx = static_cast<size_t>( connection.bindId );

    switch ( connection.currentState )
    {
      /*------------------------------------------------
      Child Node Perspective: The parent sent some kind
      of response to us in regards to the bind request.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_PARENT_RESPONSE:
        if ( frame.getType() == frameTypeACK )
        {
          connection.currentState = State::CONNECT_SUCCESS_DIRECT;
          connection.result       = RF24::Connection::Result::CONNECT_PROC_SUCCESS;

          if constexpr ( DBG_LOG_NET_PROC_CONNECT )
          {
            obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: ACK\n", Chimera::millis() );
          }
        }
        else
        {
          // Unexpected packet. Shouldn't be handled here at all.
          if constexpr ( DBG_LOG_NET_PROC_CONNECT )
          {
            obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Unexpected Packet-%d\n", Chimera::millis(),
                                   frame.getType() );
          }
          break;
        }

        /*------------------------------------------------
        Let the parent know we received the message
        ------------------------------------------------*/
        buildAckPacket( obj, frame, connection.direction, connection.currentState );

        connection.frameCache = frame;
        obj.write( connection.frameCache, Network::RoutingStyle::ROUTE_DIRECT );
        break;

      /*------------------------------------------------
      Parent Node Perspective: The child binding to us has
      just acknowledged the bind status we sent them.
      ------------------------------------------------*/
      case State::CONNECT_WAIT_FOR_CHILD_ACK:
        connection.currentState = State::CONNECT_SUCCESS_ASYNC;
        connection.result       = RF24::Connection::Result::CONNECT_PROC_SUCCESS;
        break;

      default:
        return;
        break;
    }

    /*------------------------------------------------
    Update the bind site's notion of connection status
    ------------------------------------------------*/
    if ( connection.result == RF24::Connection::Result::CONNECT_PROC_SUCCESS )
    {
      if ( connection.direction == RF24::Connection::Direction::DISCONNECT )
      {
        obj.unsafe_BindSiteList[ bindIdx ].clear();

        if constexpr ( DBG_LOG_NET_PROC_CONNECT )
        {
          obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Site %d disconnected\n", Chimera::millis(), bindIdx );
        }
      }
      else
      {
        obj.unsafe_BindSiteList[ bindIdx ].bindId     = connection.bindId;
        obj.unsafe_BindSiteList[ bindIdx ].address    = connection.toAddress;
        obj.unsafe_BindSiteList[ bindIdx ].connected  = true;
        obj.unsafe_BindSiteList[ bindIdx ].lastActive = Chimera::millis();
        obj.unsafe_BindSiteList[ bindIdx ].valid      = true;

        if constexpr ( DBG_LOG_NET_PROC_CONNECT )
        {
          obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: Site %d connected\n", Chimera::millis(), bindIdx );
        }
      }
    }
  }


  void nackHandler( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame, ControlBlock &connection )
  {
    /*------------------------------------------------
    If this particular connection control block isn't
    the intended handler of the frame, exit early.
    ------------------------------------------------*/
    if ( frame.getSrc() != connection.toAddress )
    {
      return;
    }

    /*------------------------------------------------
    Select the proper expected frame type based on the
    kind of connection that is in progress.
    ------------------------------------------------*/
    auto frameTypeNACK = Network::HeaderMessage::MSG_NET_REQUEST_CONNECT_NACK;
    if ( connection.direction == RF24::Connection::Direction::DISCONNECT )
    {
      frameTypeNACK = Network::HeaderMessage::MSG_NET_REQUEST_DISCONNECT_NACK;
    }

    /*------------------------------------------------
    Process the NACK message
    ------------------------------------------------*/
    if ( frame.getType() == frameTypeNACK )
    {
      /*-------------------------------------------------
      Move the connection process to the termination state
      -------------------------------------------------*/
      connection.currentState = State::CONNECT_TERMINATE;
      connection.result       = RF24::Connection::Result::CONNECT_PROC_FAIL;

      /*-------------------------------------------------
      Ensure the bindsite won't show it's connected to anything
      -------------------------------------------------*/
      auto bindIdx = static_cast<size_t>( connection.bindId );
      obj.unsafe_BindSiteList[ bindIdx ].clear();

      /*-------------------------------------------------
      Optionally log the event
      -------------------------------------------------*/
      if constexpr ( DBG_LOG_NET_PROC_CONNECT )
      {
        obj.getLogger()->flog( uLog::Level::LVL_DEBUG, "%d-NET-Connect: NACK\n", Chimera::millis() );
      }
    }
  }


  RF24::Connection::BindSite getDirectConnectionID( RF24::Network::Interface &obj, RF24::LogicalAddress address )
  {
    using namespace RF24::Connection;

    BindSite connectSite = BindSite::INVALID;

    /*------------------------------------------------
    Figure out which direction this connection is being made
    ------------------------------------------------*/
    if ( address == getParent( obj.thisNode() ) )
    {
      connectSite = BindSite::PARENT;
    }
    else if ( isDirectDescendent( obj.thisNode(), address ) )
    {
      /*------------------------------------------------
      Otherwise, it's a possible child node. Convert the ID
      at the connecting node's level directly into a bind site.
      They are really the same thing anyways.
      ------------------------------------------------*/
      auto level = static_cast<BindSite>( getIdAtLevel( address, getLevel( address ) ) );

      if ( ( BindSite::CHILD_1 <= level ) && ( level <= BindSite::CHILD_5 ) )
      {
        connectSite = level;
      }
    }
    // Else invalid

    return connectSite;
  }


  void buildNackPacket( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame,
                        const RF24::Connection::Direction dir,
                        const RF24::Network::Internal::Processes::Connection::State state )
  {
    auto cachedSrc = frame.getSrc();
    frame.setSrc( obj.thisNode() );
    frame.setDst( cachedSrc );
    frame.setPayload( &state, sizeof( state ) );

    if ( dir == RF24::Connection::Direction::CONNECT )
    {
      frame.setType( Network::MSG_NET_REQUEST_CONNECT_NACK );
    }
    else
    {
      frame.setType( Network::MSG_NET_REQUEST_DISCONNECT_NACK );
    }
  }


  void buildAckPacket( RF24::Network::Interface &obj, RF24::Network::Frame::FrameType &frame,
                       const RF24::Connection::Direction dir,
                       const RF24::Network::Internal::Processes::Connection::State state )
  {
    auto cachedSrc = frame.getSrc();
    frame.setSrc( obj.thisNode() );
    frame.setDst( cachedSrc );
    frame.setPayload( &state, sizeof( state ) );

    if ( dir == RF24::Connection::Direction::CONNECT )
    {
      frame.setType( Network::MSG_NET_REQUEST_CONNECT_ACK );
    }
    else
    {
      frame.setType( Network::MSG_NET_REQUEST_DISCONNECT_ACK );
    }
  }

}    // namespace RF24::Network::Internal::Processes::Connection
