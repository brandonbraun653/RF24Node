/********************************************************************************
 *  File Name:
 *    endpoint.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/


/* RF24 Includes */
#include <RF24Node/endpoint/endpoint.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/network/queue/queue.hpp>

#include <RF24Node/physical/physical.hpp>
#include <RF24Node/physical/simulator/sim_physical.hpp>

RF24::Endpoint *new__Endpoint()
{
  return new RF24::Endpoint();
}

void delete__Endpoint( RF24::Endpoint *obj )
{
  delete obj;
}

namespace RF24
{
  Endpoint::Endpoint()
  {
    memset( &mConfig, 0, sizeof( EndpointConfig ) );


    network = std::make_unique<Network::Network>();
    

#if defined( RF24_SIMULATOR )
    physical = std::make_shared<Physical::SimulatorDriver>();
#else
    #error Need to initialize the hardware based physical driver
#endif 
  }

  Endpoint::~Endpoint()
  {
  }

  Chimera::Status_t Endpoint::configure( const EndpointConfig &cfg )
  {
    auto configResult = Chimera::CommonStatusCodes::OK;

    mConfig = cfg;

    /*------------------------------------------------
    Initialize the hardware layer
    ------------------------------------------------*/
#if !defined( RF24_SIMULATOR )
#error Need initialization of the hardware layer
#endif 

    /*------------------------------------------------
    Initialize the physical layer
    ------------------------------------------------*/
    configResult |= physical->initialize( cfg.physical );

    /*------------------------------------------------
    Initialize the network layer
    ------------------------------------------------*/
    configResult |= network->attachPhysicalDriver(physical);

    /* Queues must be attached before startup */
    configResult |= network->initRXQueue( cfg.network.rxQueueBuffer, cfg.network.rxQueueSize );
    configResult |= network->initTXQueue( cfg.network.txQueueBuffer, cfg.network.txQueueSize );

    /* Start up with the given settings and start listening */
    configResult |= network->begin( cfg.physical.rfChannel, cfg.network.nodeStaticAddress, cfg.physical.dataRate,
                                    cfg.physical.powerAmplitude );

    /*------------------------------------------------
    Initialize the mesh layer
    ------------------------------------------------*/
    if ( cfg.network.mode == Network::Mode::NET_MODE_MESH )
    {
      // TODO: Once mesh is actually working
    }

    return configResult;
  }

  Chimera::Status_t Endpoint::setNetworkingMode( const Network::Mode mode )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::setEnpointStaticAddress( const LogicalAddress address )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::setParentStaticAddress( const LogicalAddress address )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::requestAddress()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::renewAddressReservation()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::connect()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::disconnect()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::reconnect()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::onEvent( const Event event, const EventFuncPtr_t function )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::processMessageBuffers()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::processDHCPServer()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::processMessageRequests()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::processEventHandlers()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::write( const LogicalAddress dst, const void *const data, const size_t length )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::read( void *const data, const size_t length )
  {
    return Chimera::Status_t();
  }

  bool Endpoint::packetAvailable()
  {
    return false;
  }

  size_t Endpoint::nextPacketLength()
  {
    return size_t();
  }

  EndpointStatus Endpoint::getEndpointStatus()
  {
    return EndpointStatus();
  }

  Chimera::Status_t Endpoint::isConnected()
  {
    return Chimera::Status_t();
  }

}    // namespace RF24
