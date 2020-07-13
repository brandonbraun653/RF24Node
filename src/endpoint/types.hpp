/********************************************************************************
 *  File Name:
 *    types.hpp
 *
 *  Description:
 *    Endpoint types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_TYPES_HPP
#define RF24_NODE_ENDPOINT_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

/* RF24 Includes */
#include <RF24Node/src/physical/types.hpp>
#include <RF24Node/src/network/types.hpp>
#include <RF24Node/src/network/definitions.hpp>

namespace RF24::Endpoint
{
  /**
   *  System initialization data for configuring an Endpoint node
   */
  struct SystemInit
  {
    uint8_t version;           /**< Version ID */
    uint32_t linkTimeout;      /**< How long to wait for any network connections to timeout */
    Network::Config network;   /**< Network layer configuration */
    Physical::Config physical; /**< Physical layer configuration */

    void clear()
    {
      version     = 0;
      linkTimeout = std::numeric_limits<uint32_t>::max();
      physical.clear();
      memset( &network, 0, sizeof( Network::Config ) );
    }
  };


  /**
   *  Details information about the current system state, from a high level
   */
  struct SystemState
  {
    LogicalAddress endpointAddress;          /**< The address of this node */
    Connection::BindSiteList connectedNodes; /**< The addresses of which this node is connected to */

    std::string name;
  };
}    // namespace RF24::Endpoint

#endif /* !RF24_NODE_ENDPOINT_TYPES_HPP */