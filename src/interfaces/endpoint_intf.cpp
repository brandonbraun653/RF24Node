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
#include <RF24Node/src/interfaces/endpoint_intf.hpp>

#if defined( RF24DLL )

Chimera::Status_t EP_configure( ::RF24::Endpoint::Interface *obj, const ::RF24::Endpoint::SystemInit *const cfg )
{
  ::RF24::Endpoint::SystemInit localCfg;

  memset( &localCfg, 0, sizeof( ::RF24::Endpoint::SystemInit ) );
  memcpy( &localCfg, cfg, sizeof( ::RF24::Endpoint::SystemInit ) );

  return obj->configure( localCfg );
}

Chimera::Status_t EP_setNetworkingMode( ::RF24::Endpoint::Interface *obj, const RF24::Network::Mode mode )
{
  return obj->setNetworkingMode( mode );
}

Chimera::Status_t EP_setEnpointStaticAddress( ::RF24::Endpoint::Interface *obj, const RF24::LogicalAddress address )
{
  return obj->setEnpointStaticAddress( address );
}

Chimera::Status_t EP_setParentStaticAddress( ::RF24::Endpoint::Interface *obj, const RF24::LogicalAddress address )
{
  return obj->setParentStaticAddress( address );
}

Chimera::Status_t EP_requestAddress( ::RF24::Endpoint::Interface *obj )
{
  return obj->requestAddress();
}

Chimera::Status_t EP_renewAddressReservation( ::RF24::Endpoint::Interface *obj )
{
  return obj->renewAddressReservation();
}

Chimera::Status_t EP_connect( ::RF24::Endpoint::Interface *obj, const size_t timeout )
{
  return obj->connect( timeout );
}

Chimera::Status_t EP_disconnect( ::RF24::Endpoint::Interface *obj )
{
  return obj->disconnect();
}

Chimera::Status_t EP_reconnect( ::RF24::Endpoint::Interface *obj )
{
  return obj->reconnect();
}

Chimera::Status_t EP_onEvent( ::RF24::Endpoint::Interface *obj, const RF24::Event event, const RF24::EventFuncPtr_t function )
{
  return obj->onEvent( event, function );
}

Chimera::Status_t EP_write( ::RF24::Endpoint::Interface *obj, const RF24::LogicalAddress dst, const void *const data,
                            const size_t length )
{
  return obj->write( dst, data, length );
}

Chimera::Status_t EP_read( ::RF24::Endpoint::Interface *obj, void *const data, const size_t length )
{
  return obj->read( data, length );
}

bool EP_packetAvailable( ::RF24::Endpoint::Interface *obj )
{
  return obj->packetAvailable();
}

size_t EP_nextPacketLength( ::RF24::Endpoint::Interface *obj )
{
  return obj->nextPacketLength();
}

RF24::Endpoint::Status EP_getEndpointStatus( ::RF24::Endpoint::Interface *obj )
{
  return obj->getStatus();
}

Chimera::Status_t EP_isConnected( ::RF24::Endpoint::Interface *obj )
{
  return obj->isConnected();
}

#endif /* RF24DLL */
