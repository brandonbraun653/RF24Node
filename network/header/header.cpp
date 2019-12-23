/********************************************************************************
 *   File Name:
 *    header.cpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>


/* Driver Includes */
#include <RF24Node/network/header/header.hpp>
#include <RF24Node/network/definitions.hpp>

namespace RF24::Network
{
  /**
   *   Reference variable for the Header class constructor. This lets a new Header object
   *   know which sequential ID number it should have. This is shared across all header instances.
   */
  static uint16_t universalHeaderID = 0u;
  static bool initialized = false;

  void HeaderHelper::initialize()
  {
    if ( !initialized )
    {
      universalHeaderID = 0u;
      initialized       = true;
    }
  }

  HeaderHelper::HeaderHelper( const FrameBuffer &buffer )
  {
    memcpy( &data, buffer.data(), sizeof( FrameHeaderField ) );
  }

  HeaderHelper::HeaderHelper( const RF24::LogicalAddress dstNode, const HeaderMessage msgType )
  {
    /*------------------------------------------------
    Initialize the payload structure fully
    ------------------------------------------------*/
    data.reserved = 0u;
    data.srcNode  = EMPTY_LOGICAL_ADDRESS;
    data.dstNode  = dstNode;
    data.type  = msgType;

    /*------------------------------------------------
    Grab our ID number and then update the global reference
    ------------------------------------------------*/
    data.number = universalHeaderID++;
  }

  HeaderHelper::HeaderHelper() : HeaderHelper( EMPTY_LOGICAL_ADDRESS, 0 )
  {
  }

  HeaderHelper::~HeaderHelper()
  {
  }

  void HeaderHelper::operator()( const FrameBuffer &buffer )
  {
    memcpy( &data, buffer.data(), sizeof( FrameHeaderField ) );
  }

  void HeaderHelper::operator=( const HeaderHelper &headerClass )
  {
    memcpy( &data, &headerClass.data, sizeof( FrameHeaderField ) );
  }

  void HeaderHelper::operator=( const FrameHeaderField &headerData )
  {
    memcpy( &data, &headerData, sizeof( FrameHeaderField ) );
  }

  const char *HeaderHelper::toString() const
  {
    constexpr int bufferLen = 45;
    static char buffer[ bufferLen ];
    snprintf( buffer, bufferLen, "Id: %u, Src: 0%o, Dst: 0%o, Type: %d, Reserved: %d", data.number, data.srcNode, data.dstNode,
             ( int )data.type, ( int )data.reserved );
    return buffer;
  }

  void HeaderHelper::setDestinationNode( const RF24::LogicalAddress destination )
  {
    data.dstNode = destination;
  }

  void HeaderHelper::setSourceNode( const RF24::LogicalAddress source )
  {
    data.srcNode = source;
  }

  void HeaderHelper::setType( const HeaderMessage type )
  {
    data.type = type;
  }

  RF24::LogicalAddress HeaderHelper::getDestinationNode()
  {
    return data.dstNode;
  }

  RF24::LogicalAddress HeaderHelper::getSourceNode()
  {
    return data.srcNode;
  }

  RF24::Network::HeaderMessage HeaderHelper::getType()
  {
    return data.type;
  }

}    // namespace RF24::Network
