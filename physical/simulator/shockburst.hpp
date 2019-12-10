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

/* RF24 Includes */
#include <RF24Node/hardware/definitions.hpp>


namespace RF24::Physical::Sim
{
  /*------------------------------------------------
  Indicates that the end of a packet has been reached.
  Value randomly generated from atmospheric noise. (random.org)
  ------------------------------------------------*/
  static constexpr uint32_t SBEndSequence = 0x41a9efcc;


#pragma pack(push, 1)
  struct SBPacket
  {
    uint64_t address;
    uint16_t control;
    uint8_t payload[ RF24::Hardware::MAX_PAYLOAD_WIDTH ];
    uint16_t crc;
    const uint32_t postamble = SBEndSequence;
  };
#pragma pop
  static_assert( sizeof( SBPacket ) % sizeof( size_t ) == 0, "Struct has invalid alignment" );

  using SBArray = std::array<uint8_t, sizeof( SBPacket )>;

  class ShockBurst
  {
  public:
    ShockBurst();
    ~ShockBurst();


    SBPacket disassemble();

    SBPacket assemble( const SBArray &packet );

    void isValid( const SBPacket &packet );

  };
}

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP */
