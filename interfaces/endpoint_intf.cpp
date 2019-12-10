/********************************************************************************
 *  File Name:
 *    endpoint_intf.cpp
 *
 *  Description:
 *    Implements the exported dll wrappers for the endpoint interface.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* RF24 Includes */
#include <RF24Node/interfaces/endpoint_intf.hpp>

#if defined( RF24DLL )

Chimera::Status_t EP_configure( RF24::EndpointInterface *obj, const RF24::EndpointConfig *const cfg )
{
  RF24::EndpointConfig localCfg;

  memset( &localCfg, 0, sizeof( RF24::EndpointConfig ) );
  memcpy( &localCfg, cfg, sizeof( RF24::EndpointConfig ) );

  return obj->configure( localCfg );
}

Chimera::Status_t EP_setNetworkingMode( RF24::EndpointInterface *obj, const RF24::Network::Mode mode )
{
  return obj->setNetworkingMode( mode );
}

Chimera::Status_t EP_setEnpointStaticAddress( RF24::EndpointInterface *obj, const RF24::LogicalAddress address )
{
  return obj->setEnpointStaticAddress( address );
}

Chimera::Status_t EP_setParentStaticAddress( RF24::EndpointInterface *obj, const RF24::LogicalAddress address )
{
  return obj->setParentStaticAddress( address );
}

Chimera::Status_t EP_requestAddress( RF24::EndpointInterface *obj )
{
  return obj->requestAddress();
}

Chimera::Status_t EP_renewAddressReservation( RF24::EndpointInterface *obj )
{
  return obj->renewAddressReservation();
}

Chimera::Status_t EP_connect( RF24::EndpointInterface *obj )
{
  return obj->connect();
}

Chimera::Status_t EP_disconnect( RF24::EndpointInterface *obj )
{
  return obj->disconnect();
}

Chimera::Status_t EP_reconnect( RF24::EndpointInterface *obj )
{
  return obj->reconnect();
}

Chimera::Status_t EP_onEvent( RF24::EndpointInterface *obj, const RF24::Event event, const RF24::EventFuncPtr_t function )
{
  return obj->onEvent( event, function );
}

Chimera::Status_t EP_processMessageBuffers( RF24::EndpointInterface *obj )
{
  return obj->processMessageBuffers();
}

Chimera::Status_t EP_processDHCPServer( RF24::EndpointInterface *obj )
{
  return obj->processDHCPServer();
}

Chimera::Status_t EP_processMessageRequests( RF24::EndpointInterface *obj )
{
  return obj->processMessageRequests();
}

Chimera::Status_t EP_processEventHandlers( RF24::EndpointInterface *obj )
{
  return obj->processEventHandlers();
}

Chimera::Status_t EP_write( RF24::EndpointInterface *obj, const RF24::LogicalAddress dst, const void *const data,
                            const size_t length )
{
  return obj->write( dst, data, length );
}

Chimera::Status_t EP_read( RF24::EndpointInterface *obj, void *const data, const size_t length )
{
  return obj->read( data, length );
}

bool EP_packetAvailable( RF24::EndpointInterface *obj )
{
  return obj->packetAvailable();
}

size_t EP_nextPacketLength( RF24::EndpointInterface *obj )
{
  return obj->nextPacketLength();
}

RF24::EndpointStatus EP_getEndpointStatus( RF24::EndpointInterface *obj )
{
  return obj->getEndpointStatus();
}

Chimera::Status_t EP_isConnected( RF24::EndpointInterface *obj )
{
  return obj->isConnected();
}

#endif /* RF24DLL */
