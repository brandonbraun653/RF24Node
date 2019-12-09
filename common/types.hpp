/********************************************************************************
 *   File Name:
 *     types.hpp
 *
 *   Description:
 *     Common types used across the RF24Node project
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_COMMON_TYPES_HPP
#define RF24_NODE_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/types/spi_types.hpp>

/* RF24 Includes */
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24
{
  /**
   *  Represents a node's address in the network in octal format
   */
  using LogicalAddress = uint16_t;


  enum class NetworkMode : size_t
  {
    NET_MODE_STATIC,
    NET_MODE_MESH
  };

  enum NetworkServices : uint8_t
  {
    NET_SERVICE_DHCP,
    NET_SERVICE_MSG_FORWARDING
  };


  /**
   *  Function pointer callback to be invoked on an event occurrence
   */
  using EventFuncPtr_t = void ( * )( void );

  enum class Event
  {
    CONNECT,
    DISCONECT,
    MSG_TX,
    MSG_RX
  };


  struct EndpointConfig
  {
    uint32_t version; /**< Version ID */

    void *rxQueueBuffer;  /**< (Optional) External buffer for the network rx queue */
    uint32_t rxQueueSize; /**< (Required) Size of the network rx queue in bytes */

    void *txQueueBuffer;  /**< (Optional) External buffer for the network tx queue */
    uint32_t txQueueSize; /**< (Required) Size of the network tx queue in bytes */

#if defined( RF24API )

#else
    Chimera::SPI::DriverConfig spiConfig;
#endif /* RF24API */
  };

  struct EndpointStatus
  {
    bool connected;
  };

}    // namespace RF24

#endif /* !RF24_NODE_COMMON_TYPES_HPP */