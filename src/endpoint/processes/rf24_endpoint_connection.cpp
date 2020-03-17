/********************************************************************************
 *  File Name:
 *    endpoint_connection.cpp
 *
 *  Description:
 *    Implements the connection management subsystem for an endpoint
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* RF24 Includes */
#include <RF24Node/src/common/utility.hpp>
#include <RF24Node/src/endpoint/endpoint.hpp>
#include <RF24Node/src/endpoint/processes/rf24_endpoint_connection.hpp>

namespace RF24::Endpoint::Internal::Processor
{
  /*------------------------------------------------
  Static connection state machine possible actions
  ------------------------------------------------*/
  enum class Static
  {
    CONNECT_BEGIN,
    CONNECT_REQUEST,
    CONNECT_WAIT_FOR_RESPONSE,
    CONNECT_RESPONSE,
    CONNECT_SUCCESS,
    CONNECT_TERMINATE,
    CONNECT_EXIT_LOOP
  };

  /*------------------------------------------------
  Mesh connection state machine possible actions
  ------------------------------------------------*/
  enum class Mesh
  {
    CONNECT_BEGIN,
    CONNECT_TERMINATE,
    CONNECT_EXIT_LOOP
  };

  Chimera::Status_t makeStaticConnection( ::RF24::Endpoint::Interface &obj, const ::RF24::LogicalAddress node, const size_t timeout )
  {
    /*------------------------------------------------
    Protect against multi-threaded access
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::TimedLockGuard( obj );
    if ( !lockGuard.try_lock_for( 100 ) )
    {
      return ::Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Algorithm vars
    ------------------------------------------------*/
    static constexpr size_t requestTimeout = 100;

    size_t startTime      = Chimera::millis();
    size_t requestTime    = Chimera::millis();
    auto connectionResult = Chimera::CommonStatusCodes::OK;
    Static currentState   = Static::CONNECT_BEGIN;
    bool packetValidity   = false;
    Network::Frame::FrameType frame;

    auto netCfg = obj.getCurrentState();
    auto network = obj.getNetworkingDriver();

    while ( currentState != Static::CONNECT_EXIT_LOOP )
    {
      /*------------------------------------------------
      Run the network layer so we don't stall communication
      ------------------------------------------------*/
      obj.processNetworking();

      /*------------------------------------------------
      Handle the current state
      ------------------------------------------------*/
      switch ( currentState )
      {
        /*------------------------------------------------
        Do any preparation work to get the system ready for connection
        ------------------------------------------------*/
        case Static::CONNECT_BEGIN:

          /*------------------------------------------------
          No point in continuing if parent/child addresses
          are invalid. Parent is allowed to be any valid address.
          ------------------------------------------------*/
          if ( !( isAddressValid( node ) &&
                  isAddressChild( netCfg.endpointAddress ) ) )
          {
            currentState = Static::CONNECT_TERMINATE;
          }

          currentState = Static::CONNECT_REQUEST;
          break;

        /*------------------------------------------------
        Make the request to the configured nodes 
        ------------------------------------------------*/
        case Static::CONNECT_REQUEST:
          requestTime = Chimera::millis();

          frame.setDst( netCfg.parentAddress );
          frame.setSrc( netCfg.endpointAddress );
          frame.setType( Network::MSG_NET_REQUEST_BIND );
          frame.setPayload( nullptr, 0 );

          network->write( frame, Network::RoutingStyle::ROUTE_DIRECT );
          currentState = Static::CONNECT_WAIT_FOR_RESPONSE;
          break;

        /*------------------------------------------------
        Wait for the potential parent node to respond
        ------------------------------------------------*/
        case Static::CONNECT_WAIT_FOR_RESPONSE:
          if ( network->available() )
          {
            network->peek( frame );

            if ( frame.getType() == Network::MSG_NET_REQUEST_BIND_ACK )
            {
              if ( frame.getSrc() == netCfg.parentAddress )
              { 
                /*------------------------------------------------
                The expected ACK packet came from the node this state machine 
                is trying to connect to. Move the state forward.
                ------------------------------------------------*/
                currentState = Static::CONNECT_RESPONSE;
              }
              else
              {
                /*------------------------------------------------
                Another node tried (and succeeded) connecting to us while
                this state machine was in progress. Acknowledge this by 
                removing the ACK frame from the queue.
                ------------------------------------------------*/
                network->removeRXFrame();

                //TODO: Actually I think this might be a bug. Setting a break point here reveals that
                //      the source address is the same as the network->thisNode() address and the
                //      destination is for a child address. This means we are somehow receiving our
                //      own packet in a loopback mode??? 
                //      
                //      Investigate, why this packet ends up in the RX queue...maybe add more tracing...
              }
            }
          }
          else if ( ( Chimera::millis() - requestTime ) > requestTimeout )
          {
            requestTime = 0;
            currentState = Static::CONNECT_REQUEST;
          }
          break;

        /*------------------------------------------------
        Check the response from the parent node to see if we connected
        ------------------------------------------------*/
        case Static::CONNECT_RESPONSE:
          packetValidity = network->read( frame );

          if ( packetValidity && ( frame.getType() == Network::MSG_NET_REQUEST_BIND_ACK ) )
          {
            frame.setDst( netCfg.parentAddress );
            frame.setSrc( netCfg.endpointAddress );
            frame.setType( Network::MSG_NETWORK_ACK );
            frame.setPayload( nullptr, 0 );

            network->write( frame, Network::RoutingStyle::ROUTE_DIRECT );

            currentState = Static::CONNECT_SUCCESS;
          }
          else
          {
            currentState = Static::CONNECT_TERMINATE;
          }
          break;

        /*------------------------------------------------
        Connected and registered with the parent device
        ------------------------------------------------*/
        case Static::CONNECT_SUCCESS:
          network->updateRouteTable( node );

          connectionResult = true;
          currentState     = Static::CONNECT_EXIT_LOOP;
          startTime        = Chimera::millis();    // Prevent the timeout check from firing
          break;

        /*------------------------------------------------
        Last stop before this loop exits
        ------------------------------------------------*/
        case Static::CONNECT_TERMINATE:
          connectionResult = false;
          currentState     = Static::CONNECT_EXIT_LOOP;
          startTime        = Chimera::millis();    // Prevent the timeout check from firing
          break;

        /*------------------------------------------------
        Immediately signal the loop to go through exit process
        ------------------------------------------------*/
        default:
          currentState = Static::CONNECT_TERMINATE;
          break;
      }

      /*------------------------------------------------
      Make sure we haven't exceeded the user connect timeout
      ------------------------------------------------*/
      if ( ( Chimera::millis() - startTime ) > timeout )
      {
        currentState = Static::CONNECT_TERMINATE;
      }

      /* Yield cause we want to run as fast as possible without blocking other threads */
      Chimera::Threading::this_thread::yield();
    }

    return connectionResult;
  }

  Chimera::Status_t makeMeshConnection( const size_t timeout )
  {
    return Chimera::CommonStatusCodes::TIMEOUT;
  }


}    // namespace RF24::Endpoint