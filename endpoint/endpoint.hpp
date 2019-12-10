/********************************************************************************
 *  File Name:
 *    endpoint.hpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_HPP
#define RF24_NODE_ENDPOINT_HPP

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* RF24 Includes */
#include <RF24Node/interfaces/endpoint_intf.hpp>
#include <RF24Node/interfaces/network_intf.hpp>
#include <RF24Node/interfaces/physical_intf.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24
{
  
  class Endpoint : EndpointInterface
  {
  public:
    Endpoint();
    ~Endpoint();

    Chimera::Status_t configure( const EndpointConfig &cfg ) override;
    Chimera::Status_t setNetworkingMode( const Network::Mode mode ) final override;
    Chimera::Status_t setEnpointStaticAddress( const LogicalAddress address ) final override;
    Chimera::Status_t setParentStaticAddress( const LogicalAddress address ) final override;
    Chimera::Status_t requestAddress() final override;
    Chimera::Status_t renewAddressReservation() final override;
    Chimera::Status_t connect() final override;
    Chimera::Status_t disconnect() final override;
    Chimera::Status_t reconnect() final override;
    Chimera::Status_t onEvent( const Event event, const EventFuncPtr_t function ) final override;
    Chimera::Status_t processMessageBuffers() final override;
    Chimera::Status_t processDHCPServer() final override;
    Chimera::Status_t processMessageRequests() final override;
    Chimera::Status_t processEventHandlers() final override;
    Chimera::Status_t write( const LogicalAddress dst, const void *const data, const size_t length ) final override;
    Chimera::Status_t read( void *const data, const size_t length ) final override;
    bool packetAvailable() final override;
    size_t nextPacketLength() final override;
    EndpointStatus getEndpointStatus() final override;
    Chimera::Status_t isConnected() final override;

  private:
    EndpointConfig mConfig;

    Physical::Interface_sPtr physical;
    Network::Interface_uPtr network;
  };
}

/*------------------------------------------------
Exported functions for construction/destruction of Endpoints
------------------------------------------------*/
extern "C" RF24API RF24::Endpoint *new__Endpoint();
extern "C" RF24API void delete__Endpoint( RF24::Endpoint *obj );

#endif /* !RF24_NODE_ENDPOINT_HPP*/