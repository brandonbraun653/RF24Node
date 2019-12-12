/********************************************************************************
 *  File Name:
 *    shockburst.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* RF24 Includes */
#include <RF24Node/physical/simulator/shockburst.hpp>


namespace RF24::Physical::Sim
{
  ShockBurstPacket::ShockBurstPacket()
  {
  }

  ShockBurstPacket::~ShockBurstPacket()
  {
  }

  SBArray ShockBurstPacket::disassemble()
  {
    return SBArray{};
  }

  SBPacket ShockBurstPacket::assemble( const SBArray &packet )
  {
    return SBPacket{};
  }

  bool ShockBurstPacket::isValid( const SBPacket &packet )
  {
    return false;
  }

  const uint8_t *ShockBurstPacket::payload()
  {
    return pkt.payload;
  }

  size_t ShockBurstPacket::payloadSize()
  {
    return pkt.control & 0;
  }

}    // namespace RF24::Physical::Sim