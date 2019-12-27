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
#include <RF24Node/network/header/header.hpp>

namespace RF24
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

  static Static selectNextStateFromAck( const Static current, const Static last, bool ack);

  /**
   *  State machine for making a static (direct) connection to a node
   *  
   */
  Chimera::Status_t Endpoint::makeStaticConnection( const size_t timeout )
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
    Network::HeaderHelper header;

    while ( currentState != Static::CONNECT_EXIT_LOOP )
    {
      /*------------------------------------------------
      Run the network layer so we don't stall communication
      ------------------------------------------------*/
      network->update();
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

          header.setDestinationNode( mConfig.network.parentStaticAddress );
          header.setSourceNode( mConfig.network.nodeStaticAddress );
          header.setType( Network::MSG_NET_REQUEST_BIND );

          network->write( header, nullptr, 0 );

          break;

        case Static::CONNECT_WAIT_FOR_RESPONSE:
          
          if ( network->available() )
          {
            network->peek( header, nullptr, 0 );
            if ( ( header.getType() == Network::MSG_NET_REQUEST_BIND_ACK ) &&
                 ( header.getSourceNode() == mConfig.network.parentStaticAddress ) )
            {
              currentState = Static::CONNECT_RESPONSE;
            }
          }

          break;

        /*------------------------------------------------
        Check the response from the parent node to see if we connected
        ------------------------------------------------*/
        case Static::CONNECT_RESPONSE:
          packetValidity = network->read( header, nullptr, 0 );

          if ( packetValidity && ( header.getType() == Network::MSG_NET_REQUEST_BIND_ACK ) )
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

  Chimera::Status_t Endpoint::makeMeshConnection( const size_t timeout )
  {
    return Chimera::CommonStatusCodes::TIMEOUT;
  }


}