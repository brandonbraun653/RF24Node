/********************************************************************************
 *  File Name:
 *    network.cpp
 *
 *  Description:
 *    Implements the NRF24L01 network layer driver.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cmath>
#include <memory>

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Aurora Includes */
#include <Aurora/logging/serial_sink.hpp>

/* Chimera Includes */
#include <Chimera/common>

/* RF24 Includes */
#include <RF24Node/src/common/conversion.hpp>
#include <RF24Node/src/common/logging.hpp>
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/common/utility.hpp>
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/network/messaging/rf24_net_msg_ping.hpp>
#include <RF24Node/src/network/network.hpp>
#include <RF24Node/src/network/processes/rf24_network_connection.hpp>
#include <RF24Node/src/network/processes/rf24_network_ping.hpp>
#include <RF24Node/src/network/types.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>
#include <uLog/sinks/sink_cout.hpp>

namespace RF24::Network
{
  Interface_sPtr createShared( const RF24::Network::Config &cfg )
  {
    auto temp = std::make_shared<Driver>();
    temp->initialize( cfg );
    return temp;
  }


  Interface_uPtr createUnique( const RF24::Network::Config &cfg )
  {
    Driver_uPtr temp = std::make_unique<Driver>();
    temp->initialize( cfg );

    return std::move( temp );
  }


  Driver::Driver() : mPhysicalDriver( nullptr ), mLogger( nullptr )
  {
    mInitialized           = false;
    mReturnSystemMessages  = false;
    mMulticastRelay        = false;
    mLastTxTime            = 0;
    mConnectionsInProgress = 0;
    mDisconnectsInProgress = 0;

    unsafe_DriverCB.clear();

    /*-------------------------------------------------
    Initialize the transfer control blocks
    -------------------------------------------------*/
    mNetTXTCB.flag      = StatusFlag::SF_IDLE;
    mNetTXTCB.lastTime  = 0;
    mNetTXTCB.startTime = 0;

    /*------------------------------------------------
    Initialize the connection tracker with the proper data
    ------------------------------------------------*/
    for ( size_t x = 0; x < unsafe_ConnectionList.size(); x++ )
    {
      unsafe_ConnectionList[ x ]        = {};
      unsafe_ConnectionList[ x ].bindId = static_cast<RF24::Connection::BindSite>( x );
    }

    /*------------------------------------------------
    Initialize the bind site control blocks
    ------------------------------------------------*/
    for ( size_t x = 0; x < unsafe_BindSiteList.size(); x++ )
    {
      unsafe_BindSiteList[ x ].reset();
    }
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attachLogger( uLog::SinkHandle sink )
  {
    mLogger = sink;
    return Chimera::CommonStatusCodes::OK;
  }


  Chimera::Status_t Driver::attachPhysicalDriver( RF24::Physical::Interface_sPtr physicalLayer )
  {
    if ( !physicalLayer )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    mPhysicalDriver = physicalLayer;
    return Chimera::CommonStatusCodes::OK;
  }


  Chimera::Status_t Driver::initAppRXQueue( void *buffer, const size_t size )
  {
    return mAppRXQueue.attachHeap( buffer, size );
  }


  Chimera::Status_t Driver::initNetTXQueue( void *buffer, const size_t size )
  {
    return mNetTXQueue.attachHeap( buffer, size );
  }


  Chimera::Status_t Driver::initialize( const RF24::Network::Config &cfg )
  {
    Chimera::Status_t configResult = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Queues must be attached before startup
    ------------------------------------------------*/
    configResult |= initAppRXQueue( cfg.rxQueueBuffer, cfg.rxQueueSize );
    configResult |= initNetTXQueue( cfg.txQueueBuffer, cfg.txQueueSize );


    if ( configResult == Chimera::CommonStatusCodes::OK )
    {
      mInitialized = true;
    }

    return configResult;
  }


  RF24::Network::HeaderMessage Driver::updateRX()
  {
    /*-------------------------------------------------
    Initialize the RX proc
    -------------------------------------------------*/
    size_t payloadSize              = 0u;
    HeaderMessage sysMsg            = MSG_TX_NORMAL;
    RF24::Hardware::PipeNumber pipe = mPhysicalDriver->payloadAvailable();

    /*------------------------------------------------
    Process the incoming data
    ------------------------------------------------*/
    while ( pipe < RF24::Hardware::PIPE_NUM_MAX )
    {
      /*------------------------------------------------
      Get the raw data and validate its CRC
      ------------------------------------------------*/
      Frame::Buffer buffer;
      buffer.fill( 0 );

      payloadSize = mPhysicalDriver->getPayloadSize( pipe );
      mPhysicalDriver->readPayload( buffer, payloadSize );

      auto reportedCRC   = Frame::getCRCFromBuffer( buffer );
      auto calculatedCRC = Frame::calculateCRCFromBuffer( buffer );
      if ( reportedCRC != calculatedCRC )
      {
        DBG_MSG_NET( mLogger, "%d-NET: Pkt dropped due to CRC failure\n", Chimera::millis() );
        continue;
      }

      /*------------------------------------------------
      Handle the message or pass it on to the next node
      ------------------------------------------------*/
      auto frame = Frame::FrameType( buffer );
      DBG_MSG_NET( mLogger, "%d-NET: Process RX pkt of type [%d] from pipe %d\n", Chimera::millis(), frame.getType(), pipe )

      if ( frame.getDst() == thisNode() )
      {
        sysMsg = handleDestination( frame );
      }
      else
      {
        sysMsg = handlePassthrough( frame );
      }

      /*------------------------------------------------
      Check to see if a new message is available
      ------------------------------------------------*/
      pipe = mPhysicalDriver->payloadAvailable();
    }

    return sysMsg;
  }


  void Driver::updateTX()
  {
    /*-------------------------------------------------
    Don't process anything if not necessary
    -------------------------------------------------*/
    if ( mNetTXQueue.empty() )
    {
      return;
    }

    /*------------------------------------------------
    Peek the next element and verify it contains data
    ------------------------------------------------*/
    Frame::FrameType frame;
    Frame::Buffer tempBuffer;
    Queue::Element element = mNetTXQueue.peek();

    if ( !element.payload || !element.size || ( element.size > tempBuffer.size() ) )
    {
      mLogger->flog( uLog::Level::LVL_ERROR, "%d-NET: TX queue contains garbage\n", Chimera::millis() );
      return;
    }

    /*-------------------------------------------------
    Ensure we can read the status data. This should
    never go invalid, but if it does, be noisy.
    -------------------------------------------------*/
    auto sts = mPhysicalDriver->getStatus();
    if ( !sts.validity )
    {
      mLogger->flog( uLog::Level::LVL_ERROR, "%d-NET: Failed to read radio status\n", Chimera::millis() );
    }

    /*------------------------------------------------
    Build a new frame to transmit with
    ------------------------------------------------*/
    tempBuffer.fill( 0 );
    memcpy( tempBuffer.data(), element.payload, element.size );
    frame = tempBuffer;

    /*------------------------------------------------
    Determine next hop and kick off the transfer
    ------------------------------------------------*/
    auto jumpMeta = nextHop( frame.getDst() );
    if ( !writeDirect( frame, jumpMeta.hopAddress, jumpMeta.routing ) )
    {
      mLogger->flog( uLog::Level::LVL_ERROR, "%d-NET: Frame TX failed\n", Chimera::millis() );
    }

    mNetTXQueue.pop( element.payload, element.size );
  }


  void Driver::pollNetStack()
  {
    using namespace Internal::Processes;

    /*------------------------------------------------
    Receive/Transmit any queued data first
    ------------------------------------------------*/
    updateRX();
    updateTX();

    /*------------------------------------------------
    Handle any ongoing connection state machines
    ------------------------------------------------*/
    if ( connectionsInProgress() )
    {
      Connection::runConnectProcess( *this, nullptr );
    }

    /*-------------------------------------------------
    Handle network connection timeouts
    -------------------------------------------------*/
    processStaleConnections();
    processRefreshConnection();
  }


  bool Driver::available()
  {
    return !mAppRXQueue.empty();
  }


  bool Driver::peek( Frame::FrameType &frame )
  {
    return readWithPop( frame, false );
  }


  bool Driver::read( Frame::FrameType &frame )
  {
    return readWithPop( frame, true );
  }


  bool Driver::write( Frame::FrameType &frame, const RoutingStyle route )
  {
    enqueueTXPacket( frame );
    return true;
  }


  void Driver::removeRXFrame()
  {
    mAppRXQueue.removeFront();
  }


  JumpType Driver::nextHop( const LogicalAddress dst )
  {
    JumpType meta;

    /*------------------------------------------------
    Destination must be valid!
    ------------------------------------------------*/
    if ( !isAddressValid( dst ) )
    {
      meta.hopAddress = RSVD_ADDR_INVALID;
      meta.routing    = ROUTE_INVALID;
      return meta;
    }

    /*------------------------------------------------
    Next hop is the actual destination node
    ------------------------------------------------*/
    auto thisAddress    = thisNode();
    auto isDirectChild  = isDirectDescendent( thisAddress, dst );
    auto isDirectParent = isDirectDescendent( dst, thisAddress );

    if ( isDirectChild || isDirectParent )
    {
      meta.hopAddress = dst;
      meta.routing    = ROUTE_DIRECT;
    }
    else
    {
      /*------------------------------------------------
      Simple forwarding rule:
        1) If destination node is a descendant of the
           current node, send to child for forwarding.

        2) Else send directly to the parent. The data
           must go up the tree. Eventually a node that
           has the destination as a descendant will be hit.
      ------------------------------------------------*/
      if ( isDescendent( thisAddress, dst ) )
      {
        // Determine which child of this node to send the message to
        meta.hopAddress = getAddressAtLevel( dst, getLevel( thisAddress ) + 1 );
        meta.routing    = ROUTE_NORMALLY;
      }
      else
      {
        meta.hopAddress = getParent( thisAddress );
        meta.routing    = ROUTE_NORMALLY;
      }
    }

    return meta;
  }


  bool Driver::updateRouteTable( const LogicalAddress address, const bool attach )
  {
    if ( attach )
    {
      return mRouteTable.attach( address );
    }
    else
    {
      mRouteTable.detach( address );
      return true;
    }
  }


  void Driver::setNodeAddress( const LogicalAddress address )
  {
    mRouteTable.updateCentralNode( address );
  }


  LogicalAddress Driver::thisNode()
  {
    return mRouteTable.getCentralNode().getLogicalAddress();
  }


  bool Driver::isConnectedTo( const LogicalAddress toCheck )
  {
    /*------------------------------------------------
    Check the simplest option first: Is the parent?
    ------------------------------------------------*/
    if ( toCheck == mRouteTable.getParentNode().getLogicalAddress() )
    {
      return true;
    }

    /*------------------------------------------------
    Iterate over the registration list to see if maybe it's there
    ------------------------------------------------*/
    for ( auto const &child : mRouteTable.getRegistrationList() )
    {
      if ( child.getLogicalAddress() == toCheck )
      {
        return true;
      }
    }

    return false;
  }


  void Driver::resetConnection( const RF24::Connection::BindSite id )
  {
    this->lock();
    if ( id < Connection::BindSite::MAX )
    {
      size_t idx = static_cast<size_t>( id );

      /*-------------------------------------------------
      Erase all notion that this bind site had a connection
      -------------------------------------------------*/
      mRouteTable.detach( unsafe_BindSiteList[ idx ].address );
      unsafe_BindSiteList[ idx ].reset();
    }
    this->unlock();
  }


  /*------------------------------------------------
  Data Getters
  ------------------------------------------------*/
  bool Driver::connectionsInProgress()
  {
    return static_cast<bool>( mConnectionsInProgress ) || static_cast<bool>( mDisconnectsInProgress );
  }


  bool Driver::bindSiteConnected( const RF24::Connection::BindSite site )
  {
    /*------------------------------------------------
    Device is connected if:
    1. The site received a ping from the connected node in an appropriate amount of time
    2. The site has previously connected to the network
    ------------------------------------------------*/
    auto tmp             = getBindSiteCBSafe( site );
    const size_t  time   = Chimera::millis();
    const bool expired   = ( time > ( tmp.lastRXActive + tmp.staleExpirationDelta ) );
    const bool connected = tmp.connected && !expired;

    return connected;
  }


  RF24::Network::BindSiteCB Driver::getBindSiteCBSafe( const RF24::Connection::BindSite site )
  {
    /*------------------------------------------------
    Make sure an invalid bind site isn't accessed
    ------------------------------------------------*/
    if ( site >= Connection::BindSite::MAX )
    {
      return {};
    }

    /*-------------------------------------------------
    Grab the latest network status
    -------------------------------------------------*/
    RF24::Network::BindSiteCB tmp;
    tmp.reset();

    lock();
    tmp = unsafe_BindSiteList[ static_cast<size_t>( site ) ];
    unlock();

    return tmp;
  }


  RF24::Connection::BindSite Driver::getBindSite( const RF24::LogicalAddress address )
  {
    lock();
    RF24::Connection::BindSite siteCopy = RF24::Connection::BindSite::INVALID;
    for ( size_t x = 0; x < unsafe_BindSiteList.size(); x++ )
    {
      if ( unsafe_BindSiteList[ x ].address != address )
      {
        continue;
      }
      else
      {
        siteCopy = unsafe_BindSiteList[ x ].bindId;
        break;
      }
    }
    unlock();

    return siteCopy;
  }

  void Driver::getSCBUnsafe( SystemCB &scb )
  {
    scb = unsafe_DriverCB;
  }


  SystemCB Driver::getSCBSafe()
  {
    SystemCB scbCopy;
    scbCopy.clear();

    this->lock();
    getSCBUnsafe( scbCopy );
    this->unlock();

    return scbCopy;
  }



  uLog::SinkHandle Driver::getLogger()
  {
    return mLogger;
  }

  /*------------------------------------------------
  Data Setters
  ------------------------------------------------*/
  void Driver::setConnectionInProgress( const RF24::Connection::BindSite id, const RF24::Connection::Direction dir,
                                        const bool enabled )
  {
    switch ( dir )
    {
      case RF24::Connection::Direction::CONNECT:
        if ( enabled )
        {
          mConnectionsInProgress |= ( static_cast<size_t>( 1u ) << static_cast<size_t>( id ) );
        }
        else
        {
          mConnectionsInProgress &= ~( static_cast<size_t>( 1u ) << static_cast<size_t>( id ) );
        }
        break;

      case RF24::Connection::Direction::DISCONNECT:
        if ( enabled )
        {
          mDisconnectsInProgress |= ( static_cast<size_t>( 1u ) << static_cast<size_t>( id ) );
        }
        else
        {
          mDisconnectsInProgress &= ~( static_cast<size_t>( 1u ) << static_cast<size_t>( id ) );
        }
        break;

      default:
        break;
    }
  }


  void Driver::setSCBUnsafe( const SystemCB &scb )
  {
    unsafe_DriverCB = scb;
  }


  /*------------------------------------------------
  Callbacks
  ------------------------------------------------*/
  void Driver::onNodeHasBound( const RF24::Connection::BindSite id, RF24::Connection::OnCompleteCallback listener )
  {
    if ( ( RF24::Connection::BindSite::CHILD_1 <= id ) && ( id <= RF24::Connection::BindSite::CHILD_5 ) )
    {
      unsafe_ConnectionList[ static_cast<size_t>( id ) ].onConnectComplete = listener;
    }
  }


  bool Driver::isDescendantOfRegisteredChild( const LogicalAddress toCheck, LogicalAddress &which )
  {
    for ( auto const &child : mRouteTable.getRegistrationList() )
    {
      if ( RF24::isDescendent( child.getLogicalAddress(), toCheck ) )
      {
        which = child.getLogicalAddress();
        return true;
      }
    }

    return false;
  }


  // TODO: Move this function to utility.hpp
  Hardware::PipeNumber Driver::getDestinationRXPipe( const LogicalAddress destination, const LogicalAddress source )
  {
    /*------------------------------------------------
    Figure out what coarse Level these nodes are at in the network
    ------------------------------------------------*/
    const auto dstNetLevel = getLevel( destination );
    const auto srcNetLevel = getLevel( source );

    /*------------------------------------------------
    A lower level means higher up in the tree
    ------------------------------------------------*/
    if ( dstNetLevel <= srcNetLevel )
    {
      /*------------------------------------------------
      When the destination is higher in the tree (parent node) or another node on the same
      level, the data has to at a minimum pass through the parent first.
      The pipe number on the parent node is exactly equal to the source node's
      registration ID at it's network level.
      ------------------------------------------------*/
      return static_cast<Hardware::PipeNumber>( getIdAtLevel( source, srcNetLevel ) );
    }
    else
    {
      /*------------------------------------------------
      When the destination is lower in the network tree (child node), the
      appropriate pipe number is 0 because pipes 1-5 are reserved for children.
      ------------------------------------------------*/
      return Hardware::PipeNumber::PIPE_NUM_0;
    }
  }


  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  void Driver::enqueueRXPacket( Frame::FrameType &frame )
  {
    if ( !mAppRXQueue.full() )
    {
      auto buffer = frame.toBuffer();
      mAppRXQueue.push( buffer.data(), buffer.size() );
    }
    else
    {
      DBG_MSG_NET( mLogger, "%d-NET: **Drop RX Payload** Buffer Full\n", Chimera::millis() );
    }
  }


  void Driver::enqueueTXPacket( Frame::FrameType &frame )
  {
    if ( !mNetTXQueue.full() )
    {
      auto buffer = frame.toBuffer();
      mNetTXQueue.push( buffer.data(), buffer.size() );
    }
    else
    {
      DBG_MSG_NET( mLogger, "%d-NET: **Drop TX Payload** Buffer Full\n", Chimera::millis() );
    }
  }


  void Driver::toggleSystemMessageReturn( const bool state )
  {
    mReturnSystemMessages = state;
  }


  void Driver::toggleMulticastRelay( const bool state )
  {
    mMulticastRelay = state;
  }


  HeaderMessage Driver::handleDestination( Frame::FrameType &frame )
  {
    bool handledInternally = true;
    HeaderMessage message  = frame.getType();
    LogicalAddress source  = frame.getSrc();

    /*-------------------------------------------------
    Check if we can receive the packet
    -------------------------------------------------*/
    const bool connectedToSource = isConnectedTo( source );

    if( !connectedToSource && ( message != MSG_NET_REQUEST_CONNECT ) && ( message == MSG_NET_REQUEST_DISCONNECT ) )
    {
      DBG_MSG_NET( mLogger, "%d-NET: Rejected packet at destination. Not connected to sender.\n", Chimera::millis() );
      return MSG_INVALID;
    }

    /*-------------------------------------------------
    Update activity stats assuming connection is ok
    -------------------------------------------------*/
    if ( auto site = getBindSite( source ); site != RF24::Connection::BindSite::INVALID )
    {
      unsafe_BindSiteList[ static_cast<size_t>( site ) ].lastRXActive = Chimera::millis();
    }

    /*------------------------------------------------
    Handle whatever messages can/should be handled immediately at the network layer.
    Everything else will be pushed in the network RX queue.
    ------------------------------------------------*/
    switch ( message )
    {
      /*------------------------------------------------
      Simple ping packet
      ------------------------------------------------*/
      case MSG_NETWORK_PING:
        if ( Messages::Ping::isPingRequest( frame ) )
        {
          Internal::Processes::handlePingRequest( *this, frame );
        }
        else if( Messages::Ping::isPingResponse( frame ) )
        {
          Internal::Processes::handlePingResponse( *this, frame );
        }
        // Else unhandled
        break;

      /*-------------------------------------------------
      On-going connection process
      -------------------------------------------------*/
      case MSG_NET_REQUEST_CONNECT:
      case MSG_NET_REQUEST_CONNECT_ACK:
      case MSG_NET_REQUEST_CONNECT_NACK:
      case MSG_NET_REQUEST_DISCONNECT:
      case MSG_NET_REQUEST_DISCONNECT_ACK:
      case MSG_NET_REQUEST_DISCONNECT_NACK:
        Internal::Processes::Connection::runConnectProcess( *this, &frame );
        break;

      default:
        handledInternally = false;
        break;
    }

    /*------------------------------------------------
    Enqueue any unhandled packets
    ------------------------------------------------*/
    if ( !handledInternally )
    {
      enqueueRXPacket( frame );
    }

    return message;
  }


  HeaderMessage Driver::handlePassthrough( Frame::FrameType &frame )
  {
    DBG_MSG_NET( mLogger, "%d-NET: Fwd pkt type [%d] dst for [%04o]\n", Chimera::millis(), frame.getType(), frame.getDst() );

    auto jumpMeta = nextHop( frame.getDst() );
    writeDirect( frame, jumpMeta.hopAddress, jumpMeta.routing );

    return frame.getType();
  }


  bool Driver::writeDirect( Frame::FrameType &frame, const LogicalAddress hopAddress, const RoutingStyle routeType )
  {
    using namespace ::RF24::Physical::Conversion;

    // Getting the wrong pipe here because I'm not differentiating on the routing style

    auto txFromAddress = RSVD_ADDR_INVALID;
    auto originator    = frame.getSrc();
    auto sender        = thisNode();

    switch ( routeType )
    {
      case ROUTE_DIRECT:
        /*------------------------------------------------
        The current node is the last stop for this message
        before it reaches its target. There are two possible
        nodes that could be sending this kind of message:

        1. The originator of the message also happens to be
        directly connected to the receiver.

        2. The originator is several nodes away, meaning the
        message is hopping through this node to arrive at
        the receiver.
        ------------------------------------------------*/
        if ( originator != sender )
        {
          txFromAddress = sender;
        }
        else
        {
          txFromAddress = originator;
        }
        break;

      case ROUTE_NORMALLY:
        /*------------------------------------------------
        The current node is just one hop along the route
        this message will take to arrive at its destination.
        There are two nodes types that could transmit this way:

        1. The originator IS the sender, meaning this is the
        first time the message is being transmitted.

        2. The current node is just passing the message along.

        In either case, the transmitter is the same.
        ------------------------------------------------*/
        txFromAddress = sender;
        break;

      default:
        /*------------------------------------------------
        Unhandled condition. This shouldn't be happening
        ------------------------------------------------*/
        Chimera::insert_debug_breakpoint();
        return false;
        break;
    }

    auto pipeOnParent    = getDestinationRXPipe( hopAddress, txFromAddress );
    auto physicalAddress = getPhysicalAddress( hopAddress, static_cast<Hardware::PipeNumber>( pipeOnParent ) );

    frame.updateCRC();

    /*-------------------------------------------------
    Spit out some debug info to help development
    -------------------------------------------------*/
    if constexpr ( DBG_LOG_NET_TRACE )
    {
      auto type = frame.getType();
      auto dst  = frame.getDst();
      auto tick = Chimera::millis();

      switch ( routeType )
      {
        case ROUTE_DIRECT:
          mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: TX direct packet of type [%d] from [%04o] to [%04o]\n", tick, type,
                         txFromAddress, dst );
          break;

        case ROUTE_NORMALLY:
          mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: TX routed packet of type [%d] from [%04o] to [%04o] through [%04o]\n",
                         tick, type, txFromAddress, dst, hopAddress );
          break;

        default:
          break;
      }
    }

    /*------------------------------------------------
    For the moment, the Chinese clones of the RF24 chips won't work with
    dynamic payload lengths, so default to the full payload size.
    ------------------------------------------------*/
    auto buffer = frame.toBuffer();
    return transferToPipe( physicalAddress, buffer, buffer.size(), true );
  }


  bool Driver::transferToPipe( const ::RF24::PhysicalAddress address, const Frame::Buffer &buffer, const size_t length,
                               const bool autoAck )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    If we are multi casting, turn off auto acknowledgment.
    We don't care how the message gets out as long as it does.
    ------------------------------------------------*/
    result |= mPhysicalDriver->stopListening();
    result |= mPhysicalDriver->toggleAutoAck( autoAck, RF24::Hardware::PIPE_NUM_0 );
    result |= mPhysicalDriver->openWritePipe( address );
    result |= mPhysicalDriver->immediateWrite( buffer, length );
    result |= mPhysicalDriver->txStandBy( Hardware::MAX_DELAY_AUTO_RETRY + 6, true );
    result |= mPhysicalDriver->startListening();

    return ( result == Chimera::CommonStatusCodes::OK );
  }

  bool Driver::readWithPop( Frame::FrameType &frame, const bool pop )
  {
    if ( !mAppRXQueue.empty() )
    {
      /*------------------------------------------------
      Peek the next element and verify it contains data
      ------------------------------------------------*/
      Frame::Buffer tempBuffer;
      auto element  = mAppRXQueue.peek();
      bool validity = false;

      if ( !element.payload || !element.size || ( element.size > tempBuffer.size() ) )
      {
        return false;
      }

      /*------------------------------------------------
      Fill the buffer with zeros since it's likely not a full packet
      ------------------------------------------------*/
      tempBuffer.fill( 0 );
      memcpy( tempBuffer.data(), element.payload, element.size );

      /*------------------------------------------------
      Refresh the user's frame with the buffer data and check the CRC
      ------------------------------------------------*/
      frame    = tempBuffer;
      validity = frame.valid();

      /*------------------------------------------------
      Optionally pop the data off the queue
      ------------------------------------------------*/
      if ( pop )
      {
        mAppRXQueue.pop( element.payload, element.size );
      }

      return validity;
    }

    return false;
  }


  void Driver::processStaleConnections()
  {
//    RF24::Connection::BindSite bs = RF24::Connection::BindSite::INVALID;
    RF24::Connection::BindSite bs = RF24::Connection::BindSite::PARENT;

    /*-------------------------------------------------
    Check if each connected node has been pinged recently
    -------------------------------------------------*/
    //    for ( size_t x = 0; x < unsafe_BindSiteList.size(); x++ )
    //    {
    //    bs = static_cast<RF24::Connection::BindSite>( x );
    if ( bindSiteConnected( bs ) )
    {
      //continue;
      return;
    }

    /*-------------------------------------------------
    Reset the connection block, but only if previously
    connected. This node went AWOL.
    -------------------------------------------------*/
    this->lock();
    if ( unsafe_BindSiteList[ static_cast<size_t>( bs ) ].connected )
    {
      resetConnection( bs );
      DBG_MSG_NET( mLogger, "%d-NET: Lost connection on site %d due to timeout\n", Chimera::millis(), bs );
    }
    this->unlock();
    //    }
  }


  void Driver::processRefreshConnection()
  {
    static size_t lastRefresh = Chimera::millis();

    /*-------------------------------------------------
    Nothing to refresh if a root node
    -------------------------------------------------*/
    if ( isAddressRoot( thisNode() ) )
    {
      return;
    }

    /*-------------------------------------------------
    Ping the parent node so it keeps our connection
    -------------------------------------------------*/
    auto parentNode = getParent( thisNode() );

    if ( isConnectedTo( parentNode ) )
    {
      if ( ( Chimera::millis() - lastRefresh ) > DFLT_CONNECTION_REFRESH_PERIOD )
      {
        lastRefresh = Chimera::millis();
        DBG_MSG_NET( mLogger, "%d-NET: Refreshing network connection\n", Chimera::millis() );

        Network::Frame::FrameType frame;
        ::RF24::Network::Messages::Ping::requestFactory( frame, parentNode, thisNode() );
        write( frame, Network::RoutingStyle::ROUTE_DIRECT );
      }
      else
      {
        return;
      }
    }

  }

}    // namespace RF24::Network
