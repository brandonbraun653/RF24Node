/********************************************************************************
 *  File Name:
 *    endpoint.cpp
 *
 *  Description:
 *    Implements the endpoint driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
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
#include <RF24Node/src/common/conversion.hpp>
#include <RF24Node/src/common/utility.hpp>
#include <RF24Node/src/endpoint/endpoint.hpp>
#include <RF24Node/src/endpoint/processes/rf24_endpoint_connection.hpp>
#include <RF24Node/src/endpoint/processes/rf24_endpoint_ping.hpp>
#include <RF24Node/src/hardware/types.hpp>
#include <RF24Node/src/network/network.hpp>
#include <RF24Node/src/network/queue/queue.hpp>
#include <RF24Node/src/physical/physical.hpp>

namespace RF24::Endpoint
{
  Device::Device() : mNetworkDriver( nullptr ), mPhysicalDriver( nullptr ), mLogger( nullptr )
  {
    /*------------------------------------------------
    Initialize class vars
    ------------------------------------------------*/
    mEndpointInit = {};
    mState = {};
  }

  Device::~Device()
  {
  }

  void Device::initNetworkingStack( ::RF24::Network::Interface_sPtr net, ::RF24::Physical::Interface_sPtr phy )
  {
    mNetworkDriver = net;
    mPhysicalDriver = phy;

    net->attachPhysicalDriver( phy );
  }


  Chimera::Status_t Device::attachLogger( uLog::SinkHandle sink )
  {
    if ( Chimera::Threading::TimedLockGuard( *this ).try_lock_for( 100 ) )
    {
      mLogger = sink;
      mNetworkDriver->attachLogger( mLogger );
      mPhysicalDriver->attachLogger( mLogger );
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Device::configure( const SystemInit &cfg )
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
    else if ( mLogger == nullptr )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      mEndpointInit                     = cfg;
      mEndpointInit.physical.deviceName = cfg.physical.deviceName;
    }

    /*------------------------------------------------
    Constrain the input parameters
    ------------------------------------------------*/
    bool inputParametersChanged = false;

    if ( mEndpointInit.linkTimeout < Minimum_LinkTimeout )
    {
      inputParametersChanged = true;
      mEndpointInit.linkTimeout = Default_LinkTimeout;
    }

    if ( inputParametersChanged )
    {
      mLogger->flog( uLog::Level::LVL_DEBUG, "Endpoint cfg params out of bounds\n" );
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

  void Device::setName( const std::string &name )
  {
    mState.name = name;
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
    auto connectResult = Chimera::CommonStatusCodes::FAIL;
    switch ( mEndpointInit.network.mode )
    {
      case Network::Mode::NET_MODE_STATIC:
        connectResult = Internal::Processor::makeStaticConnection( *this, mEndpointInit.network.parentStaticAddress, timeout );
        break;

      case Network::Mode::NET_MODE_MESH:
        connectResult = Internal::Processor::makeMeshConnection( timeout );
        break;

      case Network::Mode::NET_MODE_INVALID:
      default:
        connectResult = Chimera::CommonStatusCodes::FAIL;
        break;
    }

    /*------------------------------------------------
    Now that the connection has been made, update anyone who needs to know
    ------------------------------------------------*/
    if ( connectResult == Chimera::CommonStatusCodes::OK )
    {
      //mNetworkDriver->updateCache( *this );
    }

    return connectResult;
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

    if ( mNetworkDriver->read( frame ) )
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
    mNetworkDriver->updateRX(0);
    mNetworkDriver->updateTX(0);
    return Chimera::CommonStatusCodes::OK;
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

  SystemInit &Device::getConfig()
  {
    return mEndpointInit;
  }

  ::RF24::LogicalAddress Device::getLogicalAddress()
  {
    return mState.endpointAddress;
  }

  bool Device::isConnected()
  {
    const bool expired = ( Chimera::millis() < mState.linkStatus.expiredTick );
    const bool cachedConnection = mState.linkStatus.connected;

    if ( cachedConnection && !expired )
    {
      // Most likely scenario: Connection is good.
      return true;
    }
    else if ( cachedConnection && expired && ping( mState.parentAddress, 150 ) )
    {
      // We were previously connected but have expired. Try to refresh.
      mState.linkStatus.expiredTick = Chimera::millis() + mEndpointInit.linkTimeout;
      return true;
    }
    else
    {
      // No matter what, all other states are invalid:
      //  1. No cached connection
      //  2. The ping failed to reach the parent
      //  3. Edge case where cachedConnection == false and expired == true
      return false;
    }
  }

  Chimera::Status_t Device::initHardwareLayer()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Device::initPhysicalLayer()
  {
    using namespace Physical::Conversion;

    Chimera::Status_t configResult = Chimera::CommonStatusCodes::OK;
    auto nodeAddress               = mEndpointInit.network.nodeStaticAddress;

    /*------------------------------------------------
    Reconfigure the radio's base settings
    ------------------------------------------------*/
    configResult |= mPhysicalDriver->setChannel( mEndpointInit.physical.rfChannel );
    configResult |= mPhysicalDriver->setPALevel( mEndpointInit.physical.powerAmplitude );
    configResult |= mPhysicalDriver->setDataRate( mEndpointInit.physical.dataRate );
    configResult |= mPhysicalDriver->toggleAutoAck( false, RF24::Hardware::PIPE_NUM_0 );
    configResult |= mPhysicalDriver->toggleDynamicPayloads( false );
    configResult |= mPhysicalDriver->setStaticPayloadSize( RF24::Hardware::MAX_PAYLOAD_WIDTH );

    /*------------------------------------------------
    Open all pipes in listening mode
    ------------------------------------------------*/
    if constexpr ( DBG_LOG_APP )
    {
        mLogger->flog( uLog::Level::LVL_DEBUG, "%d: NET: Opening all pipes for listening\n", Chimera::millis() );
    }

    for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
    {
      auto pipe = static_cast<Hardware::PipeNumber>( i );
      auto addr = getPhysicalAddress( nodeAddress, pipe );

      configResult |= mPhysicalDriver->openReadPipe( pipe, addr, true );

#if defined( RF24_SIMULATOR )
      if constexpr ( DBG_LOG_APP )
      {
        mLogger->flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has IP[%s] and Port[%d]\n", Chimera::millis(), pipe,
                       nodeAddress, decodeIP( addr ).c_str(), decodePort( addr ) );
      }
#else
      if constexpr ( DBG_LOG_APP )
      {
        mLogger->flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has address [0x%.8X]\n",
                                      Chimera::millis(), pipe, nodeAddress, addr );
      }
#endif
    }

    /*------------------------------------------------
    Instruct the radio to begin listening now that pipes are configured
    ------------------------------------------------*/
    configResult |= mPhysicalDriver->startListening();
    return configResult;
  }

  Chimera::Status_t Device::initNetworkLayer()
  {
    Chimera::Status_t configResult = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Track our current network address
    ------------------------------------------------*/
    if ( configResult == Chimera::CommonStatusCodes::OK )
    {
      mState.endpointAddress = mEndpointInit.network.nodeStaticAddress;
      mState.parentAddress   = mEndpointInit.network.parentStaticAddress;
    
      mNetworkDriver->setNodeAddress( mState.endpointAddress );
    }

    return configResult;
  }

  Chimera::Status_t Device::initMeshLayer()
  {
    if ( mEndpointInit.network.mode == Network::Mode::NET_MODE_MESH )
    {
      mLogger->flog( uLog::Level::LVL_ERROR, "ERR: Configured as mesh, but currently not supported\n" );
    }

    return Chimera::CommonStatusCodes::OK;
  }

  RF24::Network::Interface_sPtr Device::getNetworkingDriver()
{
    return mNetworkDriver;
  }

  RF24::Endpoint::SystemState Device::getCurrentState()
  {
    // TODO: Eventually transition this to a safe copy or something if it gets too big
    return mState;
  }

  bool Device::ping( const ::RF24::LogicalAddress node, const size_t timeout )
  {
    return Internal::Processor::dispatchPing( *this, node, timeout );
  }

}    // namespace RF24::Endpoint
