/********************************************************************************
 *  File Name:
 *    rf24_endpoint_write.cpp
 *
 *  Description:
 *    Implements the algorithm needed to write data to another node
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* RF24 Includes */
#include <RF24Node/endpoint>

namespace RF24::Endpoint
{


  Chimera::Status_t Device::write( const ::RF24::LogicalAddress dst, const void *const data, const size_t length )
  {

    //need to implement the is connected function as well as connect timeout, reconnect, refresh connect.

    return Chimera::Status_t();
  }

  
}