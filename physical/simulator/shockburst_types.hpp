/********************************************************************************
 *  File Name:
 *    shockburst_types.hpp
 *
 *  Description:
 *    Types used in modeling ShockBurst
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_TYPES_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_TYPES_HPP

/* C++ Includes */
#include <array>
#include <string_view>

/* RF24 Includes */
#include <RF24Node/hardware/definitions.hpp>

namespace RF24::Physical::Shockburst
{
  /*------------------------------------------------
  Indicates that the end of a packet has been reached.
  Value randomly generated from atmospheric noise. (random.org)
  ------------------------------------------------*/
  static constexpr std::string_view POSTAMBLE_STR( "WczQ", 4 );
  static constexpr uint32_t POSTAMBLE_T32 = 0x57637a51;

  static constexpr size_t FIFO_QUEUE_MAX_SIZE = 3;
  static constexpr char INVALID_MEMORY        = 0xFF;

  /*------------------------------------------------
  Raw shockburst packet transmitted via sockets
  ------------------------------------------------*/
  struct _Data
  {
    uint16_t control;
    uint64_t address;
    uint8_t payload[ RF24::Hardware::MAX_PAYLOAD_WIDTH ];
    const uint32_t postamble = POSTAMBLE_T32;
  };

  struct Packet
  {
    uint32_t crc;
    _Data data;
  };

  static_assert( sizeof( Packet ) % sizeof( size_t ) == 0, "Struct has invalid alignment for current architecture" );

  /*------------------------------------------------
  Buffer used for transferring packets
  ------------------------------------------------*/
  using PacketBuffer = std::array<char, sizeof( Packet )>;
}    // namespace RF24::Physical::Shockburst

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_TYPES_HPP */
