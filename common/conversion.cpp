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

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

/* Chimera Includes */
#include <Chimera/chimera.hpp>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>
#include <RF24Node/network/definitions.hpp>
#include <RF24Node/network/node/node.hpp>

namespace RF24::Physical::Conversion
{
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

  uint64_t getPipeAddress( const RF24::LogicalAddress nodeID, const RF24::Hardware::PipeNumber_t pipeNum )
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


  #else  /* !RF24_SIMULATOR */

  uint64_t getPipeAddress( const RF24::LogicalAddress nodeID, const RF24::Hardware::PipeNumber_t pipeNum )
  {
  /*------------------------------------------------
    These bytes take on many uses. They can represent pipe numbers, device numbers,
    parent/child numbers, etc. They are really just a way to identify where a device
    is inside the network. As needed, a byte can be used to compose an address.
    ------------------------------------------------*/
    static uint8_t addressBytePool[] = { 0xec, 0xc3, 0x3c, 0x33, 0xce, 0x3e };

    /*------------------------------------------------
    The base, 5 byte address we start off with. 0xCC is reserved to indicate
    the master node.
    ------------------------------------------------*/
    uint64_t result  = 0xCCCCCCCCCC;
    uint8_t *address = reinterpret_cast<uint8_t *>( &result );

    /*------------------------------------------------
    If we want the master node (00), calculating the pipe address is easy. Simply
    replace the first bytes of the base address with the appropriate addressByte[].
    ------------------------------------------------*/
    if ( !nodeID )
    {
      address[ 0 ] = addressBytePool[ pipeNum ];
    }
    else
    {
      RF24::Network::Node node;
      node.fromOctal( nodeID );

      /*------------------------------------------------
      Calculate what byte goes at each level of the address scheme. The first byte
      will always be associated with the pipe number. The remaining bytes are determined
      like so (MSB -> LSB):

      Level 0: (Direct child of master)
          [ 0xCC, 0xCC, 0xCC, childID, pipeNum ]

      Level 1: (Has a parent, ie 011: 1st child of the 1st level 0 node)
          [ 0xCC, 0xCC, childID, parentID, pipeNum ]

      Level 2: (Has a parent and grandparent)
          [ 0xCC, childID, parentID, grandParentID, pipeNum ]

      Level 3: (You get the idea)
          [ childID, parentID, grandParentID, greatGrandParentID, pipeNum ]

      Each level is limited to the range indicated by MIN_NODE_ID and MAX_NODE_ID.
      ------------------------------------------------*/
      address[ 0 ] = addressBytePool[ pipeNum ];

      for ( uint8_t i = 1; i < RF24::Hardware::MAX_ADDRESS_WIDTH; i++ )
      {
        if ( node.greatGrandParentID_isValid() )
        {
          address[ i ]            = addressBytePool[ node.greatGrandParentID ];
          node.greatGrandParentID = RF24::Network::INVALID_NODE_ID;
          continue;
        }
        else if ( node.grandParentID_isValid() )
        {
          address[ i ]       = addressBytePool[ node.grandParentID ];
          node.grandParentID = RF24::Network::INVALID_NODE_ID;
          continue;
        }
        else if ( node.parentID_isValid() )
        {
          address[ i ]  = addressBytePool[ node.parentID ];
          node.parentID = RF24::Network::INVALID_NODE_ID;
          continue;
        }
        else if ( node.childID_isValid() )
        {
          address[ i ] = addressBytePool[ node.childID ];
          node.childID = RF24::Network::INVALID_NODE_ID;
          continue;
        }
        else
        {
          break;
        }
      }
    }

    IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has address 0x%05x\n\r", Chimera::millis(),
                                 pipeNum, nodeID, result ); );

    return result;
  }

  #endif /* RF24_SIMULATOR */

}    // namespace RF24::Physical::Conversion
