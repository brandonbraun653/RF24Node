/********************************************************************************
 *   File Name:
 *
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_HEADER_TYPES_HPP
#define NRF24L01_NETWORK_LAYER_HEADER_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <RF24Node/network/header/header_definitions.hpp>

namespace RF24::Network
{   

#pragma pack( push )
#pragma pack( 1 )
  /**
   *   The header payload, forcefully packed and aligned to a 32bit width so we can have
   *   consistent data representation across multiple systems. This structure is the bread
   *   and butter of the class.
   */
  struct Header_t
  {
    uint16_t id;      /**< Sequential message ID, incremented every time a new frame is constructed. */
    uint16_t dstNode; /**< Logical address (OCTAL) describing where the message is going */
    uint16_t srcNode; /**< Logical address (OCTAL) describing where the message was generated */
    uint8_t msgType;  /**< Message type for the header */
    uint8_t reserved; /**< Reserved for system use: Can carry either the fragmentID or headerType */
  };
#pragma pack( pop )

  static_assert( sizeof( Header_t ) % 32 != 0, "Payload_t structure not aligned to 32bit width" );
  static_assert( sizeof( Header_t ) <= HEADER_SIZE, "Payload_t structure is too large!" );
}

#endif  /* NRF24L01_NETWORK_LAYER_HEADER_TYPES_HPP */