/********************************************************************************
*  File Name:
*    sim_header.cpp
*
*  Description:
*    
*
*  2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/


#include <RF24Node/network/header/sim_header.hpp>

using namespace RF24::Network;

RF24::Network::HeaderHelper *new__HeaderHelper()
{
  return new RF24::Network::HeaderHelper();
}

RF24::Network::HeaderHelper *new__HeaderHelperFromFrameBuffer( const uint8_t *frameBuffer )
{
  FrameBuffer tempBuffer;
  tempBuffer.fill( 0 );

  memcpy( tempBuffer.data(), frameBuffer, tempBuffer.size() );

  return new RF24::Network::HeaderHelper( tempBuffer );
}

RF24::Network::HeaderHelper *new__HeaderHelperFromSpec( const uint16_t dstNode, const uint8_t type )
{
  // TODO: This isn't gonna work right....
  return new RF24::Network::HeaderHelper();
}


void delete__HeaderHelper( RF24::Network::HeaderHelper *obj )
{
  delete obj;
}

void copyFromFrameBuffer( RF24::Network::HeaderHelper *const obj, const uint8_t *frameBuffer )
{
  FrameBuffer tempBuffer;
  tempBuffer.fill( 0 );

  memcpy( tempBuffer.data(), frameBuffer, tempBuffer.size() );

  ( *obj )( tempBuffer );
}

void copyInstance( RF24::Network::HeaderHelper *const obj, const RF24::Network::HeaderHelper *const classInstance )
{
  ( *obj ) = ( *classInstance );
}

void copyFrameHeader( RF24::Network::HeaderHelper *const obj, const RF24::Network::FrameHeaderField *const frameHeader )
{
  ( *obj ) = ( *frameHeader );
}