/********************************************************************************
 *  File Name:
 *    shockburst.hpp
 *
 *  Description:
 *    
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP

/* C++ Includes */
#include <array>
#include <charconv>
#include <string_view>

/* RF24 Includes */
#include <RF24Node/hardware/definitions.hpp>


namespace RF24::Physical::Sim
{
  /*------------------------------------------------
  Indicates that the end of a packet has been reached.
  Value randomly generated from atmospheric noise. (random.org)
  ------------------------------------------------*/
  static constexpr std::string_view SBEndSequence("WczQ", 4);
  static constexpr uint32_t SBEndSequence_t32 = 0x57637a51;

  static constexpr size_t SB_FIFO_QUEUE_MAX_SIZE = 3;


//#pragma pack(push, 1)
  struct SBPacket
  {
    uint64_t address;
    uint16_t control;
    uint8_t payload[ RF24::Hardware::MAX_PAYLOAD_WIDTH ];
    uint16_t crc;
    const uint32_t postamble = SBEndSequence_t32;
  };
//#pragma pop
  static_assert( sizeof( SBPacket ) % sizeof( size_t ) == 0, "Struct has invalid alignment" );

  using SBArray = std::array<char, sizeof( SBPacket )>;

  class ShockBurstPacket
  {
  public:
    ShockBurstPacket();
    ~ShockBurstPacket();


    SBArray disassemble();

    SBPacket assemble( const SBArray &packet );

    bool isValid( const SBPacket &packet );

    const uint8_t *payload();

    size_t payloadSize();

  private:
    SBPacket pkt;

  };
}

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP */
