/********************************************************************************
*  File Name:
*    endpoint_connection.cpp
*
*  Description:
*    
*
*  2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* C++ Includes */

/* Chimera Includes */
#include <Chimera/chimera.hpp>

/* RF24 Includes */
#include <RF24Node/common/utility.hpp>
#include <RF24Node/endpoint/endpoint.hpp>

namespace RF24::Endpoint
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

  /**
   *  State machine for making a static (direct) connection to a node
   *  
   */
  Chimera::Status_t Device::makeStaticConnection( const size_t timeout )
  {
    /*------------------------------------------------
    Protect against multi-threaded access
    ------------------------------------------------*/
    auto lockGuard        = Chimera::Threading::LockGuard( *this );
    if ( !lockGuard.lock() )
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
      network->updateRX();
      // The network layer might need to disable certain kinds of communication until it knows we are connected...
      // perhaps use "services" to describe this? Bit field....forward to parent, consume, DHCP, etc.

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
          if ( !( isAddressValid( mConfig.network.parentStaticAddress ) &&
                  isAddressChild( mConfig.network.nodeStaticAddress ) ) )
          {
            currentState = Static::CONNECT_TERMINATE;
          }

          currentState = Static::CONNECT_REQUEST;
          break;

        case Static::CONNECT_REQUEST:
          frame.setDst( mConfig.network.parentStaticAddress );
          frame.setSrc( mConfig.network.nodeStaticAddress );
          frame.setType( Network::MSG_NET_REQUEST_BIND );
          
          // TODO: Update this function to be intelligent......
          frame.setLength( sizeof( RF24::Network::Frame::PackedData ) - RF24::Network::Frame::PAYLOAD_SIZE );

          network->write( frame, Network::RoutingStyle::ROUTE_DIRECT );
          currentState = Static::CONNECT_WAIT_FOR_RESPONSE;
          break;

        case Static::CONNECT_WAIT_FOR_RESPONSE:
          
          if ( network->available() )
          {
            network->peek( frame );

            if ( ( frame.getType() == Network::MSG_NET_REQUEST_BIND_ACK ) &&
                 ( frame.getSrc() == mParentAddress ) )
            {
              currentState = Static::CONNECT_RESPONSE;
            }
          }

          break;

        /*------------------------------------------------
        Check the response from the parent node to see if we connected
        ------------------------------------------------*/
        case Static::CONNECT_RESPONSE:
          packetValidity = network->read( frame );

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
          // Send ACK back.

          connectionResult = true;
          currentState = Static::CONNECT_EXIT_LOOP;
          startTime = Chimera::millis(); // Prevent the timeout check from firing
          break;

        /*------------------------------------------------
        Last stop before this loop exits
        ------------------------------------------------*/
        case Static::CONNECT_TERMINATE:
          connectionResult = false;
          currentState = Static::CONNECT_EXIT_LOOP;
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
      
      Chimera::Threading::yield();
    }

    return connectionResult;
  }

  Chimera::Status_t Device::makeMeshConnection( const size_t timeout )
  {
    return Chimera::CommonStatusCodes::TIMEOUT;
  }

  
  bool Device::bindChildNode( const ::RF24::LogicalAddress address )
  {
    return true;
  }
}