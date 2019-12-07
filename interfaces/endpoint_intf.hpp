/********************************************************************************
 *  File Name:
 *    endpoint.hpp
 *
 *  Description:
 *    Describes a high level interface to configuring and using the NRF24L01
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_INTERFACE_HPP
#define RF24_NODE_ENDPOINT_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* RF24 Includes */
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24
{
  
  class RF24API EndpointInterface
  {
  public:
    virtual ~EndpointInterface() = default;

    virtual Chimera::Status_t assignNetworkingMode() = 0;

    virtual Chimera::Status_t setLogicalAddress() = 0;


    virtual Chimera::Status_t write( const uint16_t endpointAddress, const void *const data, const size_t length );

    virtual Chimera::Status_t read( void *const data );

    virtual Chimera::Status_t periodicProcessing();

    virtual Chimera::Status_t requestAddress();

    virtual Chimera::Status_t isConnected();

    virtual Chimera::Status_t reconnect();

    virtual Chimera::Status_t onEvent(/* Event list + functionpointer*/);
  };
}

#endif /* !RF24_NODE_ENDPOINT_INTERFACE_HPP*/