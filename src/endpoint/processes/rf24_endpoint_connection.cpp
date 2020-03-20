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
#include <RF24Node/src/network/processes/rf24_network_connection.hpp>

namespace RF24::Endpoint::Internal::Processes::Connection
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

  Chimera::Status_t makeStaticConnection( RF24::Endpoint::Interface &obj, const RF24::LogicalAddress node,
                                          RF24::Connection::OnCompleteCallback callback,
                                          const size_t timeout )
  {
    auto network = obj.getNetworkingDriver();
    RF24::Network::Internal::Processes::Connection::begin( *network, node, callback, timeout );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t makeMeshConnection( const size_t timeout )
  {
    return Chimera::CommonStatusCodes::TIMEOUT;
  }


}    // namespace RF24::Endpoint