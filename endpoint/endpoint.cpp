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
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/common/utility.hpp>
#include <RF24Node/endpoint/endpoint.hpp>
#include <RF24Node/hardware/types.hpp>
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

    /*------------------------------------------------
    Initialize class composite objects
    ------------------------------------------------*/
    logger = nullptr;
    network = std::make_unique<Network::Driver>( this );
    
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
    else
    {
      mConfig = cfg;
    }

    /*------------------------------------------------
    Initialize the various layers
    ------------------------------------------------*/
    configResult |= initHardwareLayer();
    configResult |= initPhysicalLayer();
    configResult |= initNetworkLayer();
    configResult |= initMeshLayer();

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
    switch ( mConfig.network.mode )
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

  EndpointStatus Endpoint::getStatus()
  {
    return EndpointStatus();
  }

  EndpointConfig &Endpoint::getConfig()
  {
    return mConfig;
  }

  LogicalAddress Endpoint::getLogicalAddress()
  {
    return mCurrentAddress;
  }

  Chimera::Status_t Endpoint::isConnected()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Endpoint::initHardwareLayer()
  {
#if !defined( RF24_SIMULATOR )
#error Need initialization of the hardware layer
#endif
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Endpoint::initPhysicalLayer()
  {
    using namespace Physical::Conversion;

    Chimera::Status_t configResult = Chimera::CommonStatusCodes::OK;
    auto nodeAddress               = mConfig.network.nodeStaticAddress;

    /*------------------------------------------------
    Turn on the radio. By default, this wipes all pre-existing settings.
    ------------------------------------------------*/
    if ( physical->initialize( mConfig.physical ) != Chimera::CommonStatusCodes::OK )
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_ERROR, "ERR: Phy failed init\n" ); );
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Reconfigure the radio's base settings
    ------------------------------------------------*/
    configResult |= physical->setChannel( mConfig.physical.rfChannel );
    configResult |= physical->setPALevel( mConfig.physical.powerAmplitude );
    configResult |= physical->setDataRate( mConfig.physical.dataRate );
    configResult |= physical->toggleAutoAck( false, RF24::Hardware::PIPE_NUM_0 );
    configResult |= physical->toggleDynamicPayloads( true );
    configResult |= physical->setStaticPayloadSize( RF24::Hardware::MAX_PAYLOAD_WIDTH );

    /*------------------------------------------------
    Open all pipes in listening mode
    ------------------------------------------------*/
    IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_DEBUG, "%d: NET: Opening all pipes for listening\n", Chimera::millis() ); );

    for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
    {
      auto pipe = static_cast<Hardware::PipeNumber_t>( i );
      auto addr = getPhysicalAddress( nodeAddress, pipe );

      configResult |= physical->openReadPipe( pipe, addr, true );

      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has IP[%s] and Port[%d]\n",
                                     Chimera::millis(), pipe, nodeAddress, decodeIP( addr ).c_str(), decodePort( addr ) ); );
    }

    /*------------------------------------------------
    Instruct the radio to begin listening now that pipes are configured
    ------------------------------------------------*/
    configResult |= physical->startListening();
    return configResult;
  }

  Chimera::Status_t Endpoint::initNetworkLayer()
  {
    Chimera::Status_t configResult = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Queues must be attached before startup
    ------------------------------------------------*/
    configResult |= network->initRXQueue( mConfig.network.rxQueueBuffer, mConfig.network.rxQueueSize );
    configResult |= network->initTXQueue( mConfig.network.txQueueBuffer, mConfig.network.txQueueSize );

    /*------------------------------------------------
    Bring up the network layer
    ------------------------------------------------*/
    configResult |= setNetworkingMode( mConfig.network.mode );
    configResult |= network->attachPhysicalDriver( physical );
    configResult |= network->initialize();

    return configResult;
  }

  Chimera::Status_t Endpoint::initMeshLayer()
  {
    if ( mConfig.network.mode == Network::Mode::NET_MODE_MESH )
    {
      logger->flog( uLog::Level::LVL_ERROR, "ERR: Configured as mesh, but currently not supported\n" );
    }

    return Chimera::CommonStatusCodes::OK;
  }

}    // namespace RF24
