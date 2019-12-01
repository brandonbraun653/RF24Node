/********************************************************************************
 *   File Name:
 *    path.cpp
 *
 *   Description:
 *    
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Driver Includes */
#include <RF24Node/network/path/path.hpp>
#include <RF24Node/network/frame/frame_definitions.hpp>

namespace RF24::Network
{
  Path::Path( std::array<uint8_t, MAX_FRAME_PAYLOAD_SIZE> messagePath )
  {
    memcpy( &msgPath, messagePath.data(), sizeof( MessagePath ) );
  }

  void Path::process()
  {
    uint8_t currentLevel = Level::LEVEL0;
    uint8_t *iter = reinterpret_cast<uint8_t *>( &msgPath );

    for ( uint8_t i = 0; i < sizeof( MessagePath ); i += sizeof( MessagePath::hop0 ) )
    {
      if ( iter[ i ] != INVALID_NODE_ID )
      {
        currentLevel++;
      }
    }

    networkLevel = static_cast<Level>( currentLevel );
  }
}
