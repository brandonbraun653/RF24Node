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

/* Driver Includes */
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/frame/frame_definitions.hpp>
#include <RF24Node/network/frame/frame_types.hpp>

namespace RF24::Network
{
  FrameHelper::FrameHelper( const FrameHeaderField &header, const FrameLengthField &msgLen, const void *const message )
  {
    memcpy( &data.header, &header, sizeof( FrameHeaderField ) );
    memcpy( &data.messageLength, &msgLen, sizeof( FrameLengthField ) );

    memset( data.message.data(), 0, data.message.size() );
    memcpy( data.message.data(), message, msgLen );
  }

  FrameHelper::FrameHelper( const FrameBuffer &buffer )
  {
    memcpy( &data.header, buffer.data() + FRAME_HEADER_OFFSET, sizeof( FrameHeaderField ) );
    memcpy( &data.messageLength, buffer.data() + FRAME_MSG_LEN_OFFSET, sizeof( data.messageLength ) );
    memcpy( data.message.data(), buffer.data() + FRAME_MESSAGE_OFFSET, sizeof( FramePayloadField ) );
  }

  FrameHelper::FrameHelper()
  {
    data.messageLength = 0u;
    data.message.fill( 0u );
  }

  FrameHelper::~FrameHelper(){};

  void FrameHelper::operator()( const FrameBuffer &buffer )
  {
    memcpy( &data.header, buffer.data() + FRAME_HEADER_OFFSET, sizeof( FrameHeaderField ) );
    memcpy( &data.messageLength, buffer.data() + FRAME_MSG_LEN_OFFSET, sizeof( data.messageLength ) );
    memcpy( data.message.data(), buffer.data() + FRAME_MESSAGE_OFFSET, sizeof( FramePayloadField ) );
  }

  void FrameHelper::clear()
  {
    memset( &data, 0, sizeof( data ) );
  }

  void FrameHelper::build( const FrameHeaderField &header, const FrameLengthField &msgLen, const void *const message )
  {
    /* Header Field */
    memcpy( &data.header, &header, sizeof( FrameHeaderField ) );

    /* Length Field */
    memcpy( &data.messageLength, &msgLen, sizeof( FrameLengthField ) );

    /* Message Field */
    memset( data.message.data(), 0, data.message.size() );
    memcpy( data.message.data(), message, msgLen );
  }

}    // namespace RF24::Network
