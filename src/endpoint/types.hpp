/********************************************************************************
 *  File Name:
 *    types.hpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_TYPES_HPP
#define RF24_NODE_ENDPOINT_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <string>

/* RF24 Includes */
#include <RF24Node/src/physical/types.hpp>
#include <RF24Node/src/network/types.hpp>
#include <RF24Node/src/network/definitions.hpp>

namespace RF24::Endpoint
{

  struct SystemInit
  {
    uint8_t version;           /**< Version ID */
    uint32_t linkTimeout;      /**< How long to wait for any network connections to timeout */
    Network::Config network;   /**< Network layer configuration */
    Physical::Config physical; /**< Physical layer configuration */
  };

  struct ConnectionStatus
  {
    bool connected;     /**< The endpoint is connected to the network */
    size_t expiredTick; /**< Tick when the connection state is due to expire */
  };

  struct SystemState
  {
    LogicalAddress parentAddress;
    LogicalAddress endpointAddress;
    ConnectionStatus linkStatus;

    std::string name;
  };

  struct Status
  {
    bool connected;
  };

  
}

#endif /* !RF24_NODE_ENDPOINT_TYPES_HPP */