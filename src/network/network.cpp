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
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/common/utility.hpp>
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/network/messaging/rf24_net_msg_ping.hpp>
#include <RF24Node/src/network/network.hpp>
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
    mInitialized          = false;
    mReturnSystemMessages = false;
    mMulticastRelay       = false;
    mLastTxTime           = 0;
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

  Chimera::Status_t Driver::initRXQueue( void *buffer, const size_t size )
  {
    return mRXQueue.attachHeap( buffer, size );
  }

  Chimera::Status_t Driver::initTXQueue( void *buffer, const size_t size )
  {
    return mTXQueue.attachHeap( buffer, size );
  }

  Chimera::Status_t Driver::initialize( const RF24::Network::Config &cfg )
  {
    Chimera::Status_t configResult = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Queues must be attached before startup
    ------------------------------------------------*/
    configResult |= initRXQueue( cfg.rxQueueBuffer, cfg.rxQueueSize );
    configResult |= initTXQueue( cfg.txQueueBuffer, cfg.txQueueSize );


    if ( configResult == Chimera::CommonStatusCodes::OK )
    {
      mInitialized = true;
    }

    return configResult;
  }

  HeaderMessage Driver::updateRX()
  {
    if ( !mInitialized )
    {
      if constexpr ( DBG_LOG_NET )
      {
        mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: Not initialized\n", Chimera::millis() );
      }
      return MSG_NETWORK_ERR;
    }

    uint8_t pipeNum                 = 0u;
    size_t payloadSize              = 0u;
    HeaderMessage sysMsg            = MSG_TX_NORMAL;
    RF24::Hardware::PipeNumber pipe = mPhysicalDriver->payloadAvailable();

    /*------------------------------------------------
    Process the incoming data
    ------------------------------------------------*/
    while ( pipe < RF24::Hardware::PIPE_NUM_MAX )
    {
      /*------------------------------------------------
      Get the raw data and validate that CRC
      ------------------------------------------------*/
      Frame::Buffer buffer;
      buffer.fill( 0 );

      payloadSize = mPhysicalDriver->getPayloadSize( pipe );
      mPhysicalDriver->readPayload( buffer, payloadSize );

      auto reportedCRC   = Frame::getCRCFromBuffer( buffer );
      auto calculatedCRC = Frame::calculateCRCFromBuffer( buffer );
      if ( reportedCRC != calculatedCRC )
      {
        if constexpr ( DBG_LOG_NET )
        {
          mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: Pkt dropped due to CRC failure\n", Chimera::millis() );
        }
        continue;
      }

      /*------------------------------------------------
      Assuming the frame is valid, handle the message appropriately
      ------------------------------------------------*/
      auto frame = Frame::FrameType( buffer );
      if ( frame.getDst() == mRouteTable.getCentralNode().getLogicalAddress() )
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
  
  }

  bool Driver::available()
  {
    return !mRXQueue.empty();
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
    /*------------------------------------------------
    Allows time for requests (RF24Mesh) to get through between failed writes on busy nodes
    ------------------------------------------------*/
    //while ( Chimera::millis() - mLastTxTime < 25 )
    //{
    //  if ( updateRX() > MSG_MAX_USER_DEFINED_HEADER_TYPE )
    //  {
    //    break;
    //  }
    //}

    /*------------------------------------------------
    Invoke the appropriate transfer method
    ------------------------------------------------*/
    bool writeSuccess = false;

    // TODO: I need to push these through the TX queuing system

    switch ( route )
    {
      case ROUTE_DIRECT:
        writeSuccess = writeDirect( frame );
        break;

      case ROUTE_NORMALLY:
        writeSuccess = writeRouted( frame );
        break;

      case ROUTE_MULTICAST:
        writeSuccess = writeMulticast( frame );
        break;
    }

    return writeSuccess;
  }

  void Driver::removeRXFrame()
  {
    mRXQueue.removeFront();
  }

  LogicalAddress Driver::nextHop( const LogicalAddress dst )
  {
    LogicalAddress ancestor;

    if ( isRegisteredDirectly( dst ) )
    {
      // Next hop IS the destination node
      return dst;
    }
    else if ( getLevel( dst ) >= mRouteTable.getCentralNode().getLevel() )
    {
      // The data must be routed through the parent node to get data to a node 
      // at the same level or higher.
      return mRouteTable.getParentNode().getLogicalAddress();
    }
    else if ( isDescendantOfRegisteredChild( dst, ancestor ) )
    {
      // This address is an ancestor of the destination node
      return ancestor;
    }
    else
    {
      return Network::RSVD_ADDR_INVALID;
    }
  }

  bool Driver::updateRouteTable( const LogicalAddress address )
  {
    return mRouteTable.attach( address );
  }

  void Driver::setNodeAddress(  const LogicalAddress address )
  {
    mRouteTable.updateCentralNode( address );
  }

  LogicalAddress Driver::thisNode()
  {
    return mRouteTable.getCentralNode().getLogicalAddress();
  }

  bool Driver::isRegisteredDirectly( const LogicalAddress toCheck )
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

  bool Driver::transferToPipe( const ::RF24::PhysicalAddress address, const Frame::Buffer &buffer, const size_t length, const bool autoAck )
  {
    Chimera::Status_t result  = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    If we are multi casting, turn off auto acknowledgment. 
    We don't care how the message gets out as long as it does.
    ------------------------------------------------*/
    result |= mPhysicalDriver->stopListening();
    result |= mPhysicalDriver->toggleAutoAck( autoAck, RF24::Hardware::PIPE_NUM_0 );
    result |= mPhysicalDriver->openWritePipe( address );
    result |= mPhysicalDriver->immediateWrite( buffer, length );
    result |= mPhysicalDriver->txStandBy( 10, true );
    result |= mPhysicalDriver->startListening();

    return ( result == Chimera::CommonStatusCodes::OK );
  }

  void Driver::enqueueRXPacket( Frame::FrameType &frame )
  {
    if ( !mRXQueue.full())
    {
      auto buffer = frame.toBuffer();
      mRXQueue.push( buffer.data(), buffer.size() );
    }
    else if constexpr ( DBG_LOG_NET )
    {
      mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: **Drop Payload** Buffer Full\n", Chimera::millis() );
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
    HeaderMessage message = frame.getType();

    /*------------------------------------------------
    Handle whatever messages can/should be handled at the network layer.
    Everything else will be propagated up to the application layer.
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
          return message;
        }
        break;
      
      default:
        break;
    }

    /*------------------------------------------------
    Getting here means the frame is destined for the application layer
    ------------------------------------------------*/
    enqueueRXPacket( frame );

    return message;
  }

  HeaderMessage Driver::handlePassthrough( Frame::FrameType &frame )
  {
    return MSG_NETWORK_ERR;
  }

  bool Driver::writeDirect( Frame::FrameType &frame )
  {
    using namespace ::RF24::Physical::Conversion;

    auto pipeOnParent    = getDestinationRXPipe( frame.getDst(), frame.getSrc() );
    auto physicalAddress = getPhysicalAddress( frame.getDst(), static_cast<Hardware::PipeNumber>( pipeOnParent ) );

    frame.updateCRC();

    if constexpr ( DBG_LOG_NET )
    {
      mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: TX direct packet of type [%d] from [%04o] to [%04o]\n", Chimera::millis(),
                    frame.getType(), frame.getSrc(), frame.getDst() );
    }

    return transferToPipe( physicalAddress, frame.toBuffer(), frame.getPayloadLength(), false );
  }

  bool Driver::writeRouted( Frame::FrameType &frame )
  {
    using namespace ::RF24::Physical::Conversion;

    /*------------------------------------------------
    Figure out where to send the packet along the tree
    ------------------------------------------------*/
    auto hopAddress = nextHop( frame.getDst() );
    if ( hopAddress != RSVD_ADDR_INVALID )
    {
      /*------------------------------------------------
      Grab the physical address of the pipe on the destination node 
      assuming we are sending from this (central) node.
      ------------------------------------------------*/
      auto destinationPipe = getDestinationRXPipe( hopAddress, mRouteTable.getCentralNode().getLogicalAddress() );
      auto physicalAddress = getPhysicalAddress( hopAddress, destinationPipe );

      frame.updateCRC();

      if constexpr ( DBG_LOG_NET )
      {
        mLogger->flog( uLog::Level::LVL_DEBUG, "%d-NET: TX routed packet of type [%d] from [%04o] to [%04o] through [%04o]\n",
                      Chimera::millis(), frame.getType(), frame.getSrc(), frame.getDst(), hopAddress );
      }
      
      return transferToPipe( hopAddress, frame.toBuffer(), frame.getPayloadLength(), false );
    }
    else
    {
      if constexpr ( DBG_LOG_NET )
      {
        mLogger->flog( uLog::Level::LVL_DEBUG, "Drop routed packet. Unable to deduce next destination.\n" );
      }
      return false;
    }
  }

  bool Driver::writeMulticast( Frame::FrameType &frame )
  {
    mPhysicalDriver->startListening();
    return false;
  }

  bool Driver::readWithPop( Frame::FrameType &frame, const bool pop )
  {
    if ( !mRXQueue.empty() )
    {
      /*------------------------------------------------
      Peek the next element and verify it contains data
      ------------------------------------------------*/
      Frame::Buffer tempBuffer;
      auto element = mRXQueue.peek();
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
      frame = tempBuffer;
      validity = frame.valid();

      /*------------------------------------------------
      Optionally pop the data off the queue
      ------------------------------------------------*/
      if ( pop )
      {
        mRXQueue.pop( element.payload, element.size );
      }

      return validity;
    }

    return false;
  }



}    // namespace RF24::Network
