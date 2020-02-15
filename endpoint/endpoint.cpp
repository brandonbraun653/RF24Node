/********************************************************************************
 *  File Name:
 *    endpoint.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <string_view>

/* Chimera Includes */
#include <Chimera/thread>

/* uLog Includes */
#include <uLog/types.hpp>
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_cout.hpp>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/common/utility.hpp>
#include <RF24Node/endpoint/connection.hpp>
#include <RF24Node/endpoint/endpoint.hpp>
#include <RF24Node/endpoint/registration.hpp>
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/network/queue/queue.hpp>
#include <RF24Node/physical/physical.hpp>
#include <RF24Node/physical/simulator/sim_physical.hpp>


#if defined( RF24_SIMULATOR )
RF24::Endpoint::Device *new__Endpoint()
{
  return new RF24::Endpoint::Device();
}

void delete__Endpoint( RF24::Endpoint::Device *obj )
{
  delete obj;
}
#endif 

namespace RF24::Endpoint
{
  Device::Device() : mConectionManager( *this )
  {
    /*------------------------------------------------
    Initialize class vars
    ------------------------------------------------*/
    memset( &mConfig, 0, sizeof( Config ) );
    memset( mDeviceName, 0, sizeof( mDeviceName ) );
    mDeviceAddress = 0;
    mParentAddress = 0;

    /*------------------------------------------------
    Initialize class composite objects
    ------------------------------------------------*/
    logger = nullptr;
    network = std::make_unique<Network::Driver>( this );
    
#if defined( RF24_SIMULATOR )
    physical = std::make_shared<Physical::SimulatorDriver>();
#else
    physical = std::make_shared<Physical::HardwareDriver>();
#endif 
  }

  Device::~Device()
  {
  }

  Chimera::Status_t Device::attachLogger( uLog::SinkHandle sink )
  {
    if ( Chimera::Threading::TimedLockGuard( *this ).try_lock_for( 100 ) )
    {
      logger = sink;
      network->attachLogger( logger );
      physical->attachLogger( logger );
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Device::configure( const Config &cfg )
  {
    auto configResult = Chimera::CommonStatusCodes::OK;
    auto lockGuard    = Chimera::Threading::TimedLockGuard( *this );

    /*------------------------------------------------
    Make sure we can actually move forward with configuration
    ------------------------------------------------*/
    if ( !lockGuard.try_lock_for( 100 ) )
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
#if defined( RF24_SIMULATOR )
      mConfig.physical.deviceName = cfg.physical.deviceName;
#endif 
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

  void Device::setName( const std::string_view &name )
  {
    /*------------------------------------------------
    Make sure we leave a character for null termination
    ------------------------------------------------*/
    size_t bytesToCopy = name.size();
    if ( bytesToCopy > MAX_CHARS_IN_DEVICE_NAME )
    {
      static_assert( sizeof( mDeviceName ) == MAX_CHARS_IN_DEVICE_NAME + 1u, "Invalid device name array length" );
      bytesToCopy = MAX_CHARS_IN_DEVICE_NAME;
    }

    /*------------------------------------------------
    Force null terminate regardless of previous data
    ------------------------------------------------*/
    memset( mDeviceName, 0, sizeof( mDeviceName ) );
    memcpy( mDeviceName, name.data(), bytesToCopy );
  }

  Chimera::Status_t Device::setNetworkingMode( const ::RF24::Network::Mode mode )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::setEnpointStaticAddress( const ::RF24::LogicalAddress address )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::setParentStaticAddress( const ::RF24::LogicalAddress address )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::requestAddress()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::renewAddressReservation()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::releaseAddress()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::connect( const size_t timeout )
  {
    /*------------------------------------------------
    Make sure someone can't interrupt us
    ------------------------------------------------*/
    auto lockGuard = Chimera::Threading::TimedLockGuard( *this );
    if ( !lockGuard.try_lock_for( 100 ) )
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
        return mConectionManager.makeStaticConnection( timeout );
        break;

      case Network::Mode::NET_MODE_MESH:
        return mConectionManager.makeMeshConnection( timeout );
        break;

      case Network::Mode::NET_MODE_INVALID:
      default:
        return Chimera::CommonStatusCodes::FAIL;
        break;
    }
  }

  Chimera::Status_t Device::disconnect()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::reconnect()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::onEvent( const ::RF24::Event event, const ::RF24::EventFuncPtr_t function )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::doAsyncProcessing()
  {
    auto result = Chimera::CommonStatusCodes::OK;
    RF24::Network::Frame::FrameType frame;

    result |= processNetworking();
   
    if ( network->read( frame ) )
    {
      result |= processMessageRequests( frame );
      result |= processDHCPServer( frame );
      result |= processEventHandlers( frame );
    }

    return result;
  }

  Chimera::Status_t Device::processDHCPServer( RF24::Network::Frame::FrameType &frame )
  {
    auto result = Chimera::CommonStatusCodes::OK;

    return result;
  }

  Chimera::Status_t Device::processMessageRequests( RF24::Network::Frame::FrameType &frame )
  {
    auto result = Chimera::CommonStatusCodes::OK;
    RF24::Network::Frame::FrameType response;

    switch ( frame.getType() )
    {
      case Network::MSG_NET_REQUEST_BIND:
        response.setSrc( mDeviceAddress );
        response.setDst( frame.getSrc() );
        response.setType( Network::MSG_NET_REQUEST_BIND_FULL );
        response.setLength( RF24::Network::Frame::EMPTY_PAYLOAD_SIZE );

        // Will likely need to get original sender for multi hop??? Maybe add it in the payload.
        if ( mRegistrationManager.bind( frame.getSrc() ) )
        {
          response.setType( Network::MSG_NET_REQUEST_BIND_ACK );
        }

        network->write( response, RF24::Network::RoutingStyle::ROUTE_DIRECT );
        break;

      default:
        break;
    }

    return result;
  }

  Chimera::Status_t Device::processEventHandlers( RF24::Network::Frame::FrameType &frame )
  {
    auto result = Chimera::CommonStatusCodes::OK;

    return result;
  }

  Chimera::Status_t Device::processNetworking()
  {
    network->updateRX();
    network->updateTX();
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Device::write( const ::RF24::LogicalAddress dst, const void *const data, const size_t length )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::read( void *const data, const size_t length )
  {
    return Chimera::Status_t();
  }

  bool Device::packetAvailable()
  {
    return false;
  }

  size_t Device::nextPacketLength()
  {
    return size_t();
  }

  Status Device::getStatus()
  {
    return Status();
  }

  Config &Device::getConfig()
  {
    return mConfig;
  }

  ::RF24::LogicalAddress Device::getLogicalAddress()
  {
    return mDeviceAddress;
  }

  Chimera::Status_t Device::isConnected()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t Device::initHardwareLayer()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Device::initPhysicalLayer()
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
    configResult |= physical->toggleDynamicPayloads( false );
    configResult |= physical->setStaticPayloadSize( RF24::Hardware::MAX_PAYLOAD_WIDTH );

    /*------------------------------------------------
    Open all pipes in listening mode
    ------------------------------------------------*/
    IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_DEBUG, "%d: NET: Opening all pipes for listening\n", Chimera::millis() ); );

    for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
    {
      auto pipe = static_cast<Hardware::PipeNumber>( i );
      auto addr = getPhysicalAddress( nodeAddress, pipe );

      configResult |= physical->openReadPipe( pipe, addr, true );

      #if defined( RF24_SIMULATOR )
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has IP[%s] and Port[%d]\n",
                                     Chimera::millis(), pipe, nodeAddress, decodeIP( addr ).c_str(), decodePort( addr ) ); );
      #else
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has address [0x%.8X]\n",
                                     Chimera::millis(), pipe, nodeAddress, addr ); );
      #endif
    }

    /*------------------------------------------------
    Instruct the radio to begin listening now that pipes are configured
    ------------------------------------------------*/
    configResult |= physical->startListening();
    return configResult;
  }

  Chimera::Status_t Device::initNetworkLayer()
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

    /*------------------------------------------------
    Track our current network address
    ------------------------------------------------*/
    if ( configResult == Chimera::CommonStatusCodes::OK )
    {
      mDeviceAddress = mConfig.network.nodeStaticAddress;
      mParentAddress = mConfig.network.parentStaticAddress;
    }

    return configResult;
  }

  Chimera::Status_t Device::initMeshLayer()
  {
    if ( mConfig.network.mode == Network::Mode::NET_MODE_MESH )
    {
      logger->flog( uLog::Level::LVL_ERROR, "ERR: Configured as mesh, but currently not supported\n" );
    }

    return Chimera::CommonStatusCodes::OK;
  }




}    // namespace RF24
