/********************************************************************************
 *  File Name:
 *    endpoint.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/threading.hpp>

/* uLog Includes */
#include <uLog/types.hpp>
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_cout.hpp>

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
    /*------------------------------------------------
    Initialize class vars
    ------------------------------------------------*/
    memset( &mConfig, 0, sizeof( EndpointConfig ) );
    mNetMode = Network::Mode::NET_MODE_INVALID;

    /*------------------------------------------------
    Initialize class composite objects
    ------------------------------------------------*/
    logger = nullptr;
    network = std::make_unique<Network::Driver>();
    
#if defined( RF24_SIMULATOR )
    physical = std::make_shared<Physical::SimulatorDriver>();
#else
    #error Need to initialize the hardware based physical driver
#endif 
  }

  Endpoint::~Endpoint()
  {
  }

  Chimera::Status_t Endpoint::attachLogger( uLog::SinkHandle sink )
  {
    if ( Chimera::Threading::LockGuard( *this ).lock( 100 ) )
    {
      logger = sink;
      network->attachLogger( logger );
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Endpoint::configure( const EndpointConfig &cfg )
  {
    auto configResult = Chimera::CommonStatusCodes::OK;
    auto lockGuard    = Chimera::Threading::LockGuard( *this );

    /*------------------------------------------------
    Make sure we can actually move forward with configuration
    ------------------------------------------------*/
    if ( !lockGuard.lock() )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }
    else if ( logger == nullptr )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Initialize the hardware layer
    ------------------------------------------------*/
    mConfig = cfg;

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
    mNetMode = mConfig.network.mode;

    network->setNetworkingMode( mNetMode );

    configResult |= setNetworkingMode( mNetMode );
    configResult |= network->attachPhysicalDriver( physical );

    /* Queues must be attached before startup */
    configResult |= network->initRXQueue( cfg.network.rxQueueBuffer, cfg.network.rxQueueSize );
    configResult |= network->initTXQueue( cfg.network.txQueueBuffer, cfg.network.txQueueSize );

    /* Start up with the given settings and start listening */
    configResult |= network->begin( cfg.physical.rfChannel, cfg.network.nodeStaticAddress, cfg.physical.dataRate,
                                    cfg.physical.powerAmplitude );

    /*------------------------------------------------
    Initialize the mesh network layer
    ------------------------------------------------*/
    if ( cfg.network.mode == Network::Mode::NET_MODE_MESH )
    {
      logger->flog( uLog::Level::LVL_ERROR, "ERR: Configured as mesh, but currently not supported\n" );
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

  Chimera::Status_t Endpoint::releaseAddress()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::connect( const size_t timeout )
  {
    /*------------------------------------------------
    Make sure someone can't interrupt us
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::LockGuard( *this );
    if ( !lockGuard.lock() )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    The connection algorithm greatly depends upon what kind of network we are.
    Direct connections are pretty easy, but mesh ones might take a hot second 
    if other nodes have to pass messages around before arriving at the DCHP
    server, allocate a new network address, and then send a response back.
    ------------------------------------------------*/
    switch ( mNetMode )
    {
      case Network::Mode::NET_MODE_STATIC:
        return makeStaticConnection( timeout );
        break;

      case Network::Mode::NET_MODE_MESH:
        return makeMeshConnection( timeout );
        break;

      case Network::Mode::NET_MODE_INVALID:
      default:
        return Chimera::CommonStatusCodes::FAIL;
        break;
    }
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
