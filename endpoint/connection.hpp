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
#include <RF24Node/endpoint/fwd.hpp>

namespace RF24::Endpoint::Internal
{
  class ConnectionManager
  {
  public:
    ConnectionManager( Device &endpoint );
    ~ConnectionManager();

    /**
     *	Makes a connection to the network using previously configured addressing
     *  information.
     *
     *  @note   setEndpointStaticAddress() and setParentStaticAddress() need to be
     *          called before this function will succeed.
     *
     *	@param[in]	timeout
     *	@return Chimera::Status_t
     */
    Chimera::Status_t makeStaticConnection( const size_t timeout );

    /**
     *	Makes a connection to the network and requests an address from the DHCP provider.
     *  This is done dynamically, so no previously configured addressing information is needed.
     *
     *	@param[in]	timeout       Timeout in milliseconds for connection to succeed
     *	@return Chimera::Status_t
     */
    Chimera::Status_t makeMeshConnection( const size_t timeout );

  private:
    Device &endpoint;
  };
}    // namespace RF24::Endpoint::Internal

#endif /* !RF24_NODE_ENDPOINT_CONNECTION_HPP */
