/********************************************************************************
 *  File Name:
 *    connection.hpp
 *
 *  Description:
 *    Interface for making connections to an existing RF24 network
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef RF24_NODE_ENDPOINT_CONNECTION_HPP
#define RF24_NODE_ENDPOINT_CONNECTION_HPP

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/endpoint/fwd.hpp>
#include <RF24Node/src/interfaces/endpoint_intf.hpp>

namespace RF24::Endpoint::Internal::Processes::Connection
{
  /**
   *	Makes a connection to the network using previously configured addressing
   *  information.
   *
   *  @note   setEndpointStaticAddress() and setParentStaticAddress() need to be
   *          called before this function will succeed.
   *  
   *  @param[in]  obj           The endpoint object to connect with
   *  @param[in]  node          The node to make the connection to
   *  @param[in]  callback      The callback to be invoked upon success/fail/timeout
   *	@param[in]	timeout       Timeout in milliseconds for connection to succeed
   *	@return Chimera::Status_t
   */
  Chimera::Status_t makeStaticConnection( RF24::Endpoint::Interface &obj, const RF24::LogicalAddress node,
                                          RF24::Connection::Callback callback,
                                          const size_t timeout );

  /**
   *	Makes a connection to the network and requests an address from the DHCP provider.
   *  This is done dynamically, so no previously configured addressing information is needed.
   *
   *	@param[in]	timeout       Timeout in milliseconds for connection to succeed
   *	@return Chimera::Status_t
   */
  Chimera::Status_t makeMeshConnection( const size_t timeout );
  
}    // namespace RF24::Endpoint::Internal::Processor

#endif /* !RF24_NODE_ENDPOINT_CONNECTION_HPP */
