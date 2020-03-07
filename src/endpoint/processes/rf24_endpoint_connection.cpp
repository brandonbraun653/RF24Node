/********************************************************************************
 *  File Name:
 *    endpoint_connection.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* RF24 Includes */
#include <RF24Node/src/common/utility.hpp>
#include <RF24Node/src/endpoint/processes/rf24_endpoint_connection.hpp>
#include <RF24Node/src/endpoint/endpoint.hpp>

namespace RF24::Endpoint::Internal
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

  
  ConnectionManager::ConnectionManager( ::RF24::Endpoint::Device &endpoint ) : endpoint( endpoint )
  {
    
  }

  ConnectionManager::~ConnectionManager()
  {
  }

  Chimera::Status_t ConnectionManager::makeStaticConnection( const size_t timeout )
  {
    /*------------------------------------------------
    Protect against multi-threaded access
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::TimedLockGuard( endpoint );
    if ( !lockGuard.try_lock_for( 100 ) )
    {
      return ::Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Algorithm vars
    ------------------------------------------------*/
    size_t startTime      = Chimera::millis();
    auto connectionResult = Chimera::CommonStatusCodes::OK;
    Static currentState   = Static::CONNECT_BEGIN;
    Static lastState      = currentState;
    bool packetValidity   = false;
    Network::Frame::FrameType frame;

    while ( currentState != Static::CONNECT_EXIT_LOOP )
    {
      /*------------------------------------------------
      Run the network layer so we don't stall communication
      ------------------------------------------------*/
      endpoint.network->updateRX();

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
          if ( !( isAddressValid( endpoint.mConfig.network.parentStaticAddress ) &&
                  isAddressChild( endpoint.mConfig.network.nodeStaticAddress ) ) )
          {
            currentState = Static::CONNECT_TERMINATE;
          }

          currentState = Static::CONNECT_REQUEST;
          break;

        /*------------------------------------------------
        Make the request to the configured nodes 
        ------------------------------------------------*/
        case Static::CONNECT_REQUEST:
          frame.setDst( endpoint.mConfig.network.parentStaticAddress );
          frame.setSrc( endpoint.mConfig.network.nodeStaticAddress );
          frame.setType( Network::MSG_NET_REQUEST_BIND );
          frame.setLength( RF24::Network::Frame::EMPTY_PAYLOAD_SIZE );

          endpoint.network->write( frame, Network::RoutingStyle::ROUTE_DIRECT );
          currentState = Static::CONNECT_WAIT_FOR_RESPONSE;
          break;

        /*------------------------------------------------
        Wait for the potential parent node to respond
        ------------------------------------------------*/
        case Static::CONNECT_WAIT_FOR_RESPONSE:
          if ( endpoint.network->available() )
          {
            endpoint.network->peek( frame );

            if ( ( frame.getType() == Network::MSG_NET_REQUEST_BIND_ACK ) && ( frame.getSrc() == endpoint.mParentAddress ) )
            {
              currentState = Static::CONNECT_RESPONSE;
            }
          }
          break;

        /*------------------------------------------------
        Check the response from the parent node to see if we connected
        ------------------------------------------------*/
        case Static::CONNECT_RESPONSE:
          packetValidity = endpoint.network->read( frame );

          if ( packetValidity && ( frame.getType() == Network::MSG_NET_REQUEST_BIND_ACK ) )
          {
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

  Chimera::Status_t ConnectionManager::makeMeshConnection( const size_t timeout )
  {
    return Chimera::CommonStatusCodes::TIMEOUT;
  }
}    // namespace RF24::Endpoint