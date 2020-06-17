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
  Device::Device() : mPhysicalDriver( nullptr ), mNetworkDriver( nullptr ),  mLogger( nullptr ), mEndpointInit( {} ), mState( {} )
  {
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

  Chimera::Status_t Device::connectAsync( RF24::Connection::OnCompleteCallback callback, const size_t timeout )
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
    switch ( mEndpointInit.network.mode )
    {
      case Network::Mode::NET_MODE_STATIC:
        mState.linkStatus.expiresAt = Chimera::millis() + mEndpointInit.linkTimeout;
        return Internal::Processes::Connection::makeStaticConnection( *this, mEndpointInit.network.parentStaticAddress, callback, timeout );
        break;

      case Network::Mode::NET_MODE_MESH:
        return Chimera::CommonStatusCodes::FAIL;
        break;

      case Network::Mode::NET_MODE_INVALID:
      default:
        return Chimera::CommonStatusCodes::FAIL;
        break;
    }
  }

  Chimera::Status_t Device::connectBlocking( const size_t timeout )
  {
    const size_t startTime = Chimera::millis();
    bool connected = false;

    /*-------------------------------------------------
    Kick off the connection process
    -------------------------------------------------*/
    connectAsync( nullptr, timeout );

    /*-------------------------------------------------
    Wait for the connection to complete
    -------------------------------------------------*/
    while ( !connected )
    {
      /*------------------------------------------------
      Handle connections taking too long
      ------------------------------------------------*/
      if ( ( Chimera::millis() - startTime ) > timeout )
      {
        break;
      }

      /*------------------------------------------------
      Poll the network layer to see if we've connected
      to the parent node
      ------------------------------------------------*/
      connected = isConnected( RF24::Connection::BindSite::PARENT );

      /*------------------------------------------------
      Keep the network alive
      ------------------------------------------------*/
      processNetworking();
      Chimera::delayMilliseconds( 10 );
    }

    /*-------------------------------------------------
    Parse that result
    -------------------------------------------------*/
    if( connected )
    {
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
  }

  Chimera::Status_t Device::disconnect()
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
    Select the disconnection method based on the network 
    operational mode
    ------------------------------------------------*/
    switch ( mEndpointInit.network.mode )
    {
      case Network::Mode::NET_MODE_STATIC:
        return Internal::Processes::Connection::disconnect( *this, nullptr, Chimera::Threading::TIMEOUT_BLOCK );
        break;

      case Network::Mode::NET_MODE_MESH:
        return Chimera::CommonStatusCodes::FAIL;
        break;

      case Network::Mode::NET_MODE_INVALID:
      default:
        return Chimera::CommonStatusCodes::FAIL;
        break;
    }
  }

  Chimera::Status_t Device::reconnect( RF24::Connection::OnCompleteCallback callback, const size_t timeout )
  {
    /*-------------------------------------------------
    Threading protection
    -------------------------------------------------*/
    auto lockGuard = Chimera::Threading::TimedLockGuard( *this );
    if ( !lockGuard.try_lock_for( 100 ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Reset any needed connection parameters to the "unconnected" state
    ------------------------------------------------*/
    switch ( mEndpointInit.network.mode )
    {
      case Network::Mode::NET_MODE_STATIC:
        mState.linkStatus.connected = false;
        mNetworkDriver->resetConnection( RF24::Connection::BindSite::PARENT );
        break;

      case Network::Mode::NET_MODE_MESH:
        return Chimera::CommonStatusCodes::FAIL;
        break;

      case Network::Mode::NET_MODE_INVALID:
      default:
        return Chimera::CommonStatusCodes::FAIL;
        break;
    }

    if constexpr( DBG_LOG_NET_TRACE )
    {
      mLogger->flog( uLog::Level::LVL_INFO, "%d-NET: Begin reconnect\n", Chimera::millis() );
    }

    return connectAsync( callback, timeout );
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
    mNetworkDriver->pollNetStack();
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

  bool Device::isConnected( const RF24::Connection::BindSite site )
  {
    /*------------------------------------------------
    Device is connected if it hasn't expired and the 
    control block says it's connected
    ------------------------------------------------*/
    auto tmp = mNetworkDriver->getBindSiteCBSafe( site );
    const bool expired = ( Chimera::millis() > ( tmp.lastActive + tmp.expirationDelta ) );
    const bool connected = tmp.connected && !expired;

    return connected;
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
        // This section tends to be pretty fast, so give time for the log sink to write
        Chimera::delayMilliseconds( 100 );
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
    return Internal::Processes::dispatchPing( *this, node, timeout );
  }

  void Device::refreshConnection( const RF24::Connection::BindSite site )
  {
    /*------------------------------------------------
    Grab the site control block
    ------------------------------------------------*/
    auto tmp = mNetworkDriver->getBindSiteCBSafe( site );

    /*------------------------------------------------
    Ping the configured node to see if it says hello
    ------------------------------------------------*/
    if ( tmp.valid && ping( tmp.address, Chimera::Threading::TIMEOUT_500MS ) )
    {
      auto lockGuard = Chimera::Threading::LockGuard( *mNetworkDriver );
      auto idx = static_cast<size_t>( site );

      mNetworkDriver->unsafe_BindSiteList[ idx ].lastActive = Chimera::millis();
    }
  }

}    // namespace RF24::Endpoint
