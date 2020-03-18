/********************************************************************************
 *  File Name:
 *    rf24_endpoint_ping.cpp
 *
 *  Description:
 *    Declares the ping algorithm functionality
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/endpoint/processes/rf24_endpoint_ping.hpp>
#include <RF24Node/src/network/message>

namespace RF24::Endpoint::Internal::Processes
{
  enum class StateMachine
  {
    BEGIN,
    SEND_REQUEST,
    WAIT_FOR_ACK,
    TERMINATE,
    EXIT_LOOP
  };


  bool dispatchPing( ::RF24::Endpoint::Interface& obj, const ::RF24::LogicalAddress node, const size_t timeout )
  {
    size_t startTime = Chimera::millis();
    bool pingResult = false;

    /*------------------------------------------------
    Protect against multi-threaded access and early timeout
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::TimedLockGuard( obj );
    if ( !lockGuard.try_lock_for( timeout ) )
    {
      return pingResult;
    }
    else if ( ( Chimera::millis() - startTime ) >= timeout )
    {
      return pingResult;
    }

    /*------------------------------------------------
    State Machine Variables 
    ------------------------------------------------*/
    auto currentState = StateMachine::BEGIN;
    auto thisNode     = obj.getCurrentState();
    auto netDriver    = obj.getNetworkingDriver();

    
    Network::Frame::FrameType frame;

    /*------------------------------------------------
    Run the state machine
    ------------------------------------------------*/
    while ( currentState != StateMachine::EXIT_LOOP )
    {
      /*------------------------------------------------
      Run the network layer so we don't stall communication
      ------------------------------------------------*/
      obj.doAsyncProcessing();

      /*------------------------------------------------
      Handle the current state
      ------------------------------------------------*/
      switch ( currentState )
      {
        /*------------------------------------------------
        Initialize the data for the transfer
        ------------------------------------------------*/
        case StateMachine::BEGIN:
          ::RF24::Network::Messages::Ping::requestFactory( frame, node, thisNode.endpointAddress );
          currentState = StateMachine::SEND_REQUEST;
          break;

        /*------------------------------------------------
        Send the ping request through the network
        ------------------------------------------------*/
        case StateMachine::SEND_REQUEST:
          netDriver->write( frame, Network::RoutingStyle::ROUTE_NORMALLY );
          currentState = StateMachine::WAIT_FOR_ACK;
          break;

        /*------------------------------------------------
        Wait for the end node to respond
        ------------------------------------------------*/
        case StateMachine::WAIT_FOR_ACK:
          if ( netDriver->available() )
          {
            netDriver->peek( frame );

            if ( ( frame.getType() == Network::MSG_NETWORK_PING ) &&
                 ::RF24::Network::Messages::Ping::responseValidator( frame.peekPayload(), node ) )
            {
              pingResult = true;
              netDriver->removeRXFrame();
              currentState = StateMachine::EXIT_LOOP;
            }
          }
          break;

        /*------------------------------------------------
        Prematurely terminate the event handler loop (Fail case)
        ------------------------------------------------*/
        case StateMachine::TERMINATE:
          pingResult = false;
          currentState = StateMachine::EXIT_LOOP;
          break;
      
        default:
          currentState = StateMachine::TERMINATE;
          break;
      }

      /*------------------------------------------------
      Make sure we haven't exceeded the user connect timeout
      ------------------------------------------------*/
      if ( ( ( Chimera::millis() - startTime ) > timeout ) && ( currentState != StateMachine::EXIT_LOOP ) )
      {
        pingResult = false;
        currentState = StateMachine::TERMINATE;
      }

      /* Yield cause we want to run as fast as possible without blocking other threads */
      Chimera::Threading::this_thread::yield();
    }
    
    return pingResult;
  }
}
