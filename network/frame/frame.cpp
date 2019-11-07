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
  Frame::Frame( const Header_t &header, const FrameLength_t &msgLen, const void *const message )
  {
    memcpy( &data.header, &header, sizeof( Header_t ) );
    memcpy( &data.messageLength, &msgLen, sizeof( FrameLength_t ) );
    memcpy( data.message.begin(), message, msgLen );
  }

  Frame::Frame( const FrameBuffer_t &buffer )
  {
    memcpy( &data.header, buffer.cbegin() + FRAME_HEADER_OFFSET, sizeof( Header_t ) );
    memcpy( &data.messageLength, buffer.cbegin() + FRAME_MSG_LEN_OFFSET, sizeof( data.messageLength ) );
    memcpy( data.message.begin(), buffer.cbegin() + FRAME_MESSAGE_OFFSET, sizeof( FramePayload_t ) );
  }

  Frame::Frame()
  {
    data.messageLength = 0u;
    data.message.fill( 0u );
  }

  Frame::~Frame(){};

  void Frame::operator()( const FrameBuffer_t &buffer )
  {
    memcpy( &data.header, buffer.cbegin() + FRAME_HEADER_OFFSET, sizeof( Header_t ) );
    memcpy( &data.messageLength, buffer.cbegin() + FRAME_MSG_LEN_OFFSET, sizeof( data.messageLength ) );
    memcpy( data.message.begin(), buffer.cbegin() + FRAME_MESSAGE_OFFSET, sizeof( FramePayload_t ) );
  }
}    // namespace RF24::Network
