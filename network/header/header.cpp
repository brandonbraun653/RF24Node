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

  HeaderHelper::HeaderHelper( const uint16_t dstNode, const NetHdrMsgType msgType )
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
             ( int )data.msgType, ( int )data.reserved );
    return buffer;
  }
}    // namespace RF24::Network
