/********************************************************************************
 *   File Name:
 *    types.hpp
 *
 *   Description:
 *    Types used in implementing the network driver for the NRF24 radios.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_TYPES_HPP
#define NRF24L01_NETWORK_LAYER_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* RF24 Includes */
#include <RF24Node/common/types.hpp>

namespace RF24::Network
{
  enum class Mode : size_t
  {
    NET_MODE_INVALID,
    NET_MODE_STATIC,
    NET_MODE_MESH
  };

  enum Services : uint8_t
  {
    NET_SERVICE_DHCP,
    NET_SERVICE_MSG_FORWARDING
  };

  struct TransferControlBlock
  {
    // Frame? Pointer to frame buffer?
    // Mode?
  };

  struct Config
  {
    void *rxQueueBuffer;                /**< (Optional) External buffer for the network rx queue */
    size_t rxQueueSize;                 /**< (Required) Size of the network rx queue in bytes */
    void *txQueueBuffer;                /**< (Optional) External buffer for the network tx queue */
    size_t txQueueSize;                 /**< (Required) Size of the network tx queue in bytes */
    Mode mode;                   /**< Networking mode to use */
    LogicalAddress nodeStaticAddress;   /**< Static address for the node (Only available in NET_MODE_STATIC) */
    LogicalAddress parentStaticAddress; /**< Static address of the parent (Only available in NET_MODE_STATIC) */
  };
  static_assert( sizeof( Config ) % sizeof( size_t ) == 0, "Struct not aligned properly" );

}    // namespace RF24::Network


#endif /* !NRF24L01_NETWORK_LAYER_TYPES_HPP */