/********************************************************************************
 *   File Name:
 *    types.hpp
 *
 *   Description:
 *    Types for interacting with frames.
 *
 *   Note:
 *    There are two variants of the structures
 *    used to define headers and the full frame data. One variant is a system
 *    level header that allows for memory access in whatever alignment the compiler
 *    deems necessary. The other is a tightly packed (unaligned members) structure
 *    that really should have its access guarded to prevent runtime faults.
 *
 *    On some CPUs the unaligned memory isn't going to be a problem due to special
 *    assembly instructions, but others will fault. Due to this code executing on
 *    a bunch of different CPUs I thought it best to protect the user against this
 *    and allow them to natively interact with properly aligned data structures.
 *
 *    The downside is that this introduces dual maintenance and a slight memory
 *    overhead, but these structures won't be changing much once they are set.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_FRAME_TYPES_HPP
#define NRF24L01_FRAME_TYPES_HPP

/* C++ Includes */
#include <array>
#include <cstddef>
#include <cstdint>

/* Boost Includes */
#include <boost/circular_buffer_fwd.hpp>
#include <boost/circular_buffer.hpp>

/* Driver Includes */
#include <RF24Node/src/common/definitions.hpp>
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/hardware/definitions.hpp>
#include <RF24Node/src/network/definitions.hpp>
#include <RF24Node/src/network/frame/definitions.hpp>

namespace RF24::Network::Frame
{
  using Buffer  = std::array<uint8_t, Frame::size()>;
  using Cache   = boost::circular_buffer<Buffer>;
  using CRC16_t = uint16_t;
  using Length  = uint8_t;
  using Payload = std::array<uint8_t, PAYLOAD_SIZE>;

  /*------------------------------------------------
  Header network/system structure (see documentation above)
  ------------------------------------------------*/
  struct Header
  {
    LogicalAddress dstNode; /**< Logical address (OCTAL) describing where the message is going */
    LogicalAddress srcNode; /**< Logical address (OCTAL) describing where the message was generated */
    HeaderMessage type;     /**< Indicates the purpose of this particular packet */
  };

#pragma pack( push )
#pragma pack( 1 )
  struct PackedHeader
  {
    LogicalAddress dstNode;
    LogicalAddress srcNode;
    HeaderMessage type;
  };
#pragma pack( pop )

  static_assert( sizeof( PackedHeader ) == HEADER_SIZE, "Header structure is too large!" );
  static_assert( sizeof( PackedHeader::dstNode ) == HEADER_DST_SIZE, "Destination node wrong size" );
  static_assert( sizeof( PackedHeader::srcNode ) == HEADER_SRC_SIZE, "Source node wrong size" );
  static_assert( sizeof( PackedHeader::type ) == HEADER_MSG_SIZE, "Type wrong size" );
  
  static_assert( offsetof( PackedHeader, dstNode ) == HEADER_DST_OFFSET, "Destination node offset incorrect" );
  static_assert( offsetof( PackedHeader, srcNode ) == HEADER_SRC_OFFSET, "Source node offset incorrect" );
  static_assert( offsetof( PackedHeader, type ) == HEADER_MSG_OFFSET, "Type offset is incorrect" );

  /*------------------------------------------------
  Frame network/system structures (see documentation above)
  ------------------------------------------------*/
  struct Data
  {
    CRC16_t crc;                     /**< CRC of the entire frame, minus the crc field */
    PackedHeader header;             /**< Header describing the frame */
    Length payloadLength;            /**< Length of the payload in bytes */
    uint8_t payload[ PAYLOAD_SIZE ]; /**< User defined payload message */
  };

#pragma pack( push )
#pragma pack( 1 )
  struct PackedData
  {
    CRC16_t crc;
    PackedHeader header;
    Length payloadLength;
    uint8_t payload[ PAYLOAD_SIZE ];
  };
#pragma pack( pop )

  static_assert( sizeof( PackedData ) == ::RF24::Network::Frame::size(), "Frame data structure is the wrong size" );
  static_assert( offsetof( PackedData, crc ) == CRC_OFFSET, "CRC offset incorrect" );
  static_assert( offsetof( PackedData, header ) == HEADER_OFFSET, "Header offset incorrect" );
  static_assert( offsetof( PackedData, payloadLength ) == MSG_LEN_OFFSET, "Length offset incorrect" );
  static_assert( offsetof( PackedData, payload ) == PAYLOAD_OFFSET, "Payload offset incorrect" );

}    // namespace RF24::Network::Frame

#endif /* NRF24L01_NETWORK_LAYER_TYPES_HPP */