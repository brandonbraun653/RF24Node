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
#if defined( RF24_SIMULATOR )
#include <boost/asio.hpp>
#endif

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

/* Chimera Includes */
#include <Chimera/common>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/common/utility.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>
#include <RF24Node/network/definitions.hpp>

namespace RF24::Physical::Conversion
{
  RF24::Hardware::PipeNumber getPipeOnParent( const ::RF24::LogicalAddress address )
  {
    auto level = getLevel( address );

    if ( level == RF24::NODE_LEVEL_1 )
    {
      /*------------------------------------------------
      This node is directly connected to a root. The ID returned
      here is exactly equal to root node RX pipe number.
      ------------------------------------------------*/
      auto id = getIdAtLevel( address, level );
      return static_cast<RF24::Hardware::PipeNumber>( level );
    }
    else
    {
      /*------------------------------------------------
      By using the level just one above this node, we can get the
      id of the pipe that this node is connected to on the parent.
      ------------------------------------------------*/
      auto id = getIdAtLevel( address, level - 1 );
      return static_cast<RF24::Hardware::PipeNumber>( id );
    }
  }


#if defined( RF24_SIMULATOR )

  static constexpr uint64_t IP_MASK         = 0xFFFFFFFF;
  static constexpr uint64_t IP_POS          = 0;
  static constexpr uint64_t ENCODED_IP_MASK = IP_MASK << IP_POS;

  static constexpr uint64_t PORT_MASK         = 0xFFFF;
  static constexpr uint64_t PORT_POS          = 32;
  static constexpr uint64_t ENCODED_PORT_MASK = PORT_MASK << PORT_POS;

  static constexpr uint16_t BASE_PORT = 13000;


  uint64_t encodeAddress( const std::string &ip, const uint16_t port )
  {
    uint32_t ip_as_uint = boost::asio::ip::address_v4::from_string( ip ).to_uint();

    uint64_t maskedIP   = ( static_cast<uint64_t>( ip_as_uint ) << IP_POS ) & ENCODED_IP_MASK;
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

    return decodeIP( packet.data.address );
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

    return decodePort( packet.data.address );
  }

  PhysicalAddress getPhysicalAddress( const LogicalAddress nodeID, const Hardware::PipeNumber pipeNum )
  {
    /*------------------------------------------------
    The ports have to be unique across all nodeIDs, so
    map each nodeID into a range of values that are mutually
    exclusive from one another.
    ------------------------------------------------*/
    uint16_t actualPort  = BASE_PORT + ( nodeID * ( RF24::Hardware::MAX_NUM_PIPES + 1 ) ) + static_cast<uint16_t>( pipeNum );
    std::string actualIP = "127.0.0.1";

    return encodeAddress( actualIP, actualPort );
  }

#else /* !RF24_SIMULATOR */

  RF24::PhysicalAddress getPhysicalAddress( const RF24::LogicalAddress address, const RF24::Hardware::PipeNumber pipeNum )
  {
    uint64_t physicalAddress = 0u;

    /*------------------------------------------------
    Bytes to map a node id at a given level into a physical address
    ------------------------------------------------*/
    /* clang-format off */
    static std::array<uint8_t, RF24::Network::MAX_CHILDREN + 1> addressBytePool = {
      0x5c,   /**< Byte for a root node address */
      0xec,   /**< Byte for child 1 of a node */
      0xc9,   /**< Byte for child 2 of a node */
      0x3c,   /**< Etc.... */
      0x33, 
      0xce
    };

    static std::array<uint8_t, RF24::Hardware::MAX_NUM_PIPES> addressPipePool = {
      0xC0,
      0xC1,
      0xC2,
      0xC3,
      0xC4,
      0xC5
    };

    static constexpr uint8_t empty_address_byte = 0xe7;
    /* clang-format on */

    /*------------------------------------------------
    Replace the octal node address number with the appropriate address byte
    ------------------------------------------------*/
    const auto nodeLevel    = getLevel( address );
    LogicalID nodeIdAtLevel = 0;
    std::array<uint8_t, sizeof( uint64_t )> addressBuffer;

    /* Preemptively default unused bytes to the datasheet's example address */
    addressBuffer.fill( empty_address_byte );

    /* The device can only use up to 5 bytes, so clear the remaining bytes to
    avoid confusion during debugging. */
    addressBuffer[ 5 ] = 0;
    addressBuffer[ 6 ] = 0;
    addressBuffer[ 7 ] = 0;

    /* The first byte indicates which pipe to direct the data into */
    addressBuffer[ 0 ] = addressPipePool[ static_cast<size_t>( pipeNum ) ];

    /* The remaining bytes up to the node level are then filled in from the pool.
    Essentially we are just using the nodeID as a lookup index. */
    for ( LogicalLevel x = 1; x <= nodeLevel; x++ )
    {
      nodeIdAtLevel = getIdAtLevel( address, x );

      if ( nodeIdAtLevel < addressBytePool.size() )
      {
        /* Single byte offset due to pipe being the first byte */
        addressBuffer[ x ] = addressBytePool[ nodeIdAtLevel ];
      }
    }

    /*------------------------------------------------
    Copy over the newly formed memory and return it
    ------------------------------------------------*/
    memcpy( &physicalAddress, addressBuffer.data(), addressBuffer.size() );
    return physicalAddress;
  }

#endif /* RF24_SIMULATOR */

}    // namespace RF24::Physical::Conversion
