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

namespace RF24::Network
{
  /**
   *   Reference variable for the Header class constructor. This lets a new Header object
   *   know which sequential ID number it should have. This is shared across all header instances.
   */
  static uint16_t universalHeaderID = 0u;
  static bool initialized = false;

  void Header::initialize()
  {
    if ( !initialized )
    {
      universalHeaderID = 0u;
      initialized       = true;
    }
  }

  Header::Header( const FrameBuffer_t &buffer )
  {
    memcpy( &data, buffer.cbegin(), sizeof( Header_t ) );
  }

  Header::Header( const uint16_t dstNode, const uint8_t msgType )
  {
    /*------------------------------------------------
    Initialize the payload structure fully
    ------------------------------------------------*/
    data.reserved = 0u;
    data.srcNode  = EMPTY_LOGICAL_ADDRESS;
    data.dstNode  = dstNode;
    data.msgType  = msgType;

    /*------------------------------------------------
    Grab our ID number and then update the global reference
    ------------------------------------------------*/
    data.id = universalHeaderID++;
  }

  Header::Header( const uint16_t dstNode, const MessageType type ) : Header( dstNode, static_cast<uint8_t>( type ) )
  {
  }

  Header::Header() : Header( EMPTY_LOGICAL_ADDRESS, 0 )
  {
  }

  Header::~Header()
  {
  }

  void Header::operator()( const FrameBuffer_t &buffer )
  {
    memcpy( &data, buffer.cbegin(), sizeof( Header_t ) );
  }

  void Header::operator=( const Header &headerClass )
  {
    memcpy( &data, &headerClass.data, sizeof( Header_t ) );
  }

  void Header::operator=( const Header_t &headerData )
  {
    memcpy( &data, &headerData, sizeof( Header_t ) );
  }

  const char *Header::toString() const
  {
    static char buffer[ 45 ];
    sprintf( buffer, "Id: %u, Src: 0%o, Dst: 0%o, Type: %d, Reserved: %d", data.id, data.srcNode, data.dstNode,
             ( int )data.msgType, ( int )data.reserved );
    return buffer;
  }
}    // namespace RF24::Network
