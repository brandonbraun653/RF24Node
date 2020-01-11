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

/* RF24 Includes */
#include <RF24Node/physical/types.hpp>
#include <RF24Node/network/types.hpp>

namespace RF24::Endpoint
{
  struct Config
  {
    uint32_t version; /**< Version ID */

    Network::Config network;   /**< Network layer configuration */
    Physical::Config physical; /**< Physical layer configuration */
  };

  struct Status
  {
    bool connected;
  };

  struct Node
  {
    LogicalAddress logicalAddress;
    PhysicalAddress physicalAddress;
  };
}

#endif /* !RF24_NODE_ENDPOINT_TYPES_HPP */