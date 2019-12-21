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
#include <RF24Node/endpoint/endpoint.hpp>

namespace RF24
{
  enum class ConnectionUpdate
  {
    CONNECT_BEGIN,
    CONNECT_TERMINATE,
    CONNECT_EXIT_LOOP
  };

  /**
   *  State machine for making a static (direct) connection to a node
   *  
   */
  ::Chimera::Status_t Endpoint::makeStaticConnection( const size_t timeout )
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
    const size_t startTime        = Chimera::millis();
    auto connectionResult         = Chimera::CommonStatusCodes::OK;
    ConnectionUpdate connectState = ConnectionUpdate::CONNECT_BEGIN;

    while ( connectState != ConnectionUpdate::CONNECT_EXIT_LOOP )
    {
      /*------------------------------------------------
      Run the network layer so we don't stall communication
      ------------------------------------------------*/
      network->update();
      // The network layer might need to disable certain kinds of communication until it knows we are connected...
      // perhaps use "services" to describe this? Bit field....forward to parent, consume, dhcp, etc.

      /*------------------------------------------------
      Handle the current state
      ------------------------------------------------*/
      switch ( connectState )
      {
        /*------------------------------------------------
        Do any preparation work to get the system ready for connection
        ------------------------------------------------*/
        case ConnectionUpdate::CONNECT_BEGIN:

          break;


        /*------------------------------------------------
        Last stop before this loop exits
        ------------------------------------------------*/
        case ConnectionUpdate::CONNECT_TERMINATE:
          // Do special things to terminate?
          // Check termination flags?
          // Set connect result??
          connectState = ConnectionUpdate::CONNECT_EXIT_LOOP;
          break;
      
        default:
          connectState = ConnectionUpdate::CONNECT_TERMINATE;
          break;
      }

      if ( ( Chimera::millis() - startTime ) > timeout )
      {
        connectState = ConnectionUpdate::CONNECT_TERMINATE;
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