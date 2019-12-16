/********************************************************************************
 *  File Name:
 *    conversion.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */

/* Boost Includes */
#include <boost/asio.hpp>

/* RF24 Includes */
#include <RF24Node/physical/simulator/conversion.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Conversion
{
  static constexpr uint64_t IP_MASK         = 0xFFFFFFFF;
  static constexpr uint64_t IP_POS          = 0;
  static constexpr uint64_t ENCODED_IP_MASK = IP_MASK << IP_POS;

  static constexpr uint64_t PORT_MASK         = 0xFFFF;
  static constexpr uint64_t PORT_POS          = 32;
  static constexpr uint64_t ENCODED_PORT_MASK = PORT_MASK << PORT_POS;

  uint64_t encodeAddress( const std::string &ip, const uint16_t port )
  {
    uint32_t ip_as_uint = boost::asio::ip::address_v4::from_string( ip ).to_uint();

    uint64_t maskedIP = ( static_cast<uint64_t>( ip_as_uint ) << IP_POS ) & ENCODED_IP_MASK;
    uint64_t maskedPort = ( static_cast<uint64_t>( port ) << PORT_POS ) & ENCODED_PORT_MASK;

    return ( maskedIP | maskedPort );
  }

  std::string decodeIP( const uint64_t &address )
  {
    auto ip = boost::asio::ip::address_v4( static_cast<uint32_t>( ( address >> IP_POS ) & IP_MASK ) );
    return ip.to_string();
  }

  std::string decodeIP( const RF24::Physical::Shockburst::PacketBuffer &pkt )
  {
    /*------------------------------------------------
    Convert the raw packet buffer into the struct form
    ------------------------------------------------*/
    RF24::Physical::Shockburst::Packet packet;
    memcpy( &packet, pkt.data(), sizeof( RF24::Physical::Shockburst::Packet ) );

    return decodeIP( packet.address );
  }

  uint16_t decodePort( const uint64_t &address )
  {
    auto port = static_cast<uint16_t>( ( address >> PORT_POS ) & PORT_MASK );
    return port;
  }

  uint16_t decodePort( const RF24::Physical::Shockburst::PacketBuffer &pkt )
  {
    /*------------------------------------------------
    Convert the raw packet buffer into the struct form
    ------------------------------------------------*/
    RF24::Physical::Shockburst::Packet packet;
    memcpy( &packet, pkt.data(), sizeof( RF24::Physical::Shockburst::Packet ) );

    return decodePort( packet.address );
  }

}    // namespace RF24::Physical::Conversion
