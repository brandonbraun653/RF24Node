/********************************************************************************
 *  File Name:
 *    endpoint.hpp
 *
 *  Description:
 *    Describes a node endpoint implementation. The goal is for the user instantiate
 *    only this class and be able to create/connect to an NRF24L01 network without
 *    too much hassle and start sending data around.
 *
 *  Notes:
 *    Is thread-safe for all function calls.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_HPP
#define RF24_NODE_ENDPOINT_HPP

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/interfaces/endpoint_intf.hpp>
#include <RF24Node/interfaces/network_intf.hpp>
#include <RF24Node/interfaces/physical_intf.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24
{
  class Endpoint : public EndpointInterface, public Chimera::Threading::Lockable
  {
  public:
    Endpoint();
    ~Endpoint();

    ::Chimera::Status_t attachLogger( uLog::SinkHandle sink );

    ::Chimera::Status_t configure( const EndpointConfig &cfg ) override;
    ::Chimera::Status_t setNetworkingMode( const Network::Mode mode ) final override;
    ::Chimera::Status_t setEnpointStaticAddress( const LogicalAddress address ) final override;
    ::Chimera::Status_t setParentStaticAddress( const LogicalAddress address ) final override;
    ::Chimera::Status_t requestAddress() final override;
    ::Chimera::Status_t renewAddressReservation() final override;
    ::Chimera::Status_t releaseAddress() final override;
    ::Chimera::Status_t connect( const size_t timeout ) final override;
    ::Chimera::Status_t disconnect() final override;
    ::Chimera::Status_t reconnect() final override;
    ::Chimera::Status_t onEvent( const Event event, const EventFuncPtr_t function ) final override;
    ::Chimera::Status_t processMessageBuffers() final override;
    ::Chimera::Status_t processDHCPServer() final override;
    ::Chimera::Status_t processMessageRequests() final override;
    ::Chimera::Status_t processEventHandlers() final override;
    ::Chimera::Status_t write( const LogicalAddress dst, const void *const data, const size_t length ) final override;
    ::Chimera::Status_t read( void *const data, const size_t length ) final override;
    bool packetAvailable() final override;
    size_t nextPacketLength() final override;
    EndpointStatus getEndpointStatus() final override;
    Chimera::Status_t isConnected() final override;

  protected:
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
    EndpointConfig mConfig; /**< User endpoint configuration for this node */
    Network::Mode mNetMode; /**< User configured networking mode */

    Physical::Interface_sPtr physical; /**< Physical layer object */
    Network::Interface_uPtr network;   /**< Network layer object */
    uLog::SinkHandle logger;           /**< Logger object */
  };
}    // namespace RF24

/*------------------------------------------------
Exported functions for construction/destruction of Endpoints
------------------------------------------------*/
extern "C" RF24API RF24::Endpoint *new__Endpoint();
extern "C" RF24API void delete__Endpoint( RF24::Endpoint *obj );

#endif /* !RF24_NODE_ENDPOINT_HPP*/