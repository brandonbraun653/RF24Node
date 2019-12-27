/********************************************************************************
 *   File Name:
 *    frame.cpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* CRC Includes */
#include "CRC.h"

/* Driver Includes */
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/frame/frame_definitions.hpp>
#include <RF24Node/network/frame/frame_types.hpp>

namespace RF24::Network
{
  FrameHelper::FrameHelper( const FrameHeaderField &header, const FrameLengthField &msgLen, const void *const message ) : staleData( true )
  {
    memcpy( &data.header, &header, sizeof( FrameHeaderField ) );
    memcpy( &data.messageLength, &msgLen, sizeof( FrameLengthField ) );

    memset( data.message.data(), 0, data.message.size() );
    memcpy( data.message.data(), message, msgLen );
  }

  FrameHelper::FrameHelper( const FrameBuffer &buffer ) : staleData( true )
  {
    memcpy( &data.header, buffer.data() + FRAME_HEADER_OFFSET, sizeof( FrameHeaderField ) );
    memcpy( &data.messageLength, buffer.data() + FRAME_MSG_LEN_OFFSET, sizeof( data.messageLength ) );
    memcpy( data.message.data(), buffer.data() + FRAME_MESSAGE_OFFSET, sizeof( FramePayloadField ) );
  }

  FrameHelper::FrameHelper() : staleData( true )
  {

    data.messageLength = 0u;
    data.message.fill( 0u );
  }

  FrameHelper::~FrameHelper(){};

  void FrameHelper::operator()( const FrameBuffer &buffer )
  {
    staleData = true;

    memcpy( &data.header, buffer.data() + FRAME_HEADER_OFFSET, sizeof( FrameHeaderField ) );
    memcpy( &data.messageLength, buffer.data() + FRAME_MSG_LEN_OFFSET, sizeof( data.messageLength ) );
    memcpy( data.message.data(), buffer.data() + FRAME_MESSAGE_OFFSET, sizeof( FramePayloadField ) );
  }

  void FrameHelper::clear()
  {
    staleData = true;

    memset( &data, 0, sizeof( data ) );
  }

  void FrameHelper::build( const FrameHeaderField &header, const FrameLengthField &msgLen, const void *const message )
  {
    staleData = true;

    /* Header Field */
    memcpy( &data.header, &header, sizeof( FrameHeaderField ) );

    /* Length Field */
    memcpy( &data.messageLength, &msgLen, sizeof( FrameLengthField ) );

    /* Message Field */
    memset( data.message.data(), 0, data.message.size() );
    memcpy( data.message.data(), message, msgLen );
  }

  void FrameHelper::updateCRC()
  {
    if ( staleData )
    {
      data.crc = calculateCRC();
      staleData = false;
    }
  }

  uint8_t *FrameHelper::getBuffer()
  {
    return mBuffer.data();
  }

  bool FrameHelper::validateCRC()
  {
    return ( ( data.crc - calculateCRC() ) == 0 );
  }

  uint16_t FrameHelper::calculateCRC()
  {
    return CRC::Calculate( &data.header, ( sizeof( FrameData ) - sizeof( FrameData::crc ) ), CRC::CRC_16_ARC() );
  }

  void FrameHelper::commitBuffer()
  {
    staleData = true;
    memcpy( &data, mBuffer.data(), mBuffer.size() );
  }

  HeaderHelper FrameHelper::getHeader()
  {
    return HeaderHelper( data.header );
  }

  FrameLengthField FrameHelper::getPayloadLength()
  {
    return data.messageLength;
  }

  FramePayloadField FrameHelper::getPayload()
  {
    /* Yes, the copy is intentional */
    return data.message;
  }

}    // namespace RF24::Network
