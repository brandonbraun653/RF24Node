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
#include <RF24Node/src/network/network.hpp>
#include <RF24Node/src/network/types.hpp>
#include <RF24Node/src/network/frame/frame.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>
#include <uLog/sinks/sink_cout.hpp>

namespace RF24::Network
{

  Driver::Driver( ::RF24::Endpoint::Interface *const node ) : mNode( node )
  {
    mInitialized          = false;
    mReturnSystemMessages = false;
    mMulticastRelay       = false;

    mLastTxTime = 0;

    logger = nullptr;
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attachLogger( uLog::SinkHandle sink )
  {
    logger = sink;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::attachPhysicalDriver( RF24::Physical::Interface_sPtr &physicalLayer )
  {
    if ( !physicalLayer )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    radio = physicalLayer;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::initRXQueue( void *buffer, const size_t size )
  {
    return rxQueue.attachHeap( buffer, size );
  }

  Chimera::Status_t Driver::initTXQueue( void *buffer, const size_t size )
  {
    return txQueue.attachHeap( buffer, size );
  }

  Chimera::Status_t Driver::initialize()
  {
    if ( radio ) // TODO: Add queue checks 
    {
      mInitialized = true;
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      mInitialized = false;
      return Chimera::CommonStatusCodes::FAIL;
    }
  }

  HeaderMessage Driver::updateRX()
  {
    if ( !mInitialized )
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_DEBUG, "%d: NET Not initialized\n", Chimera::millis() ); );
      return MSG_NETWORK_ERR;
    }

    uint8_t pipeNum                 = 0u;
    size_t payloadSize              = 0u;
    HeaderMessage sysMsg            = MSG_TX_NORMAL;
    RF24::Hardware::PipeNumber pipe = radio->payloadAvailable();

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

      payloadSize = radio->getPayloadSize( pipe );
      radio->readPayload( buffer, payloadSize );

      auto reportedCRC   = Frame::getCRCFromBuffer( buffer );
      auto calculatedCRC = Frame::calculateCRCFromBuffer( buffer );
      if ( reportedCRC != calculatedCRC )
      {
        IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d-NET: Pkt dropped due to CRC failure\n", Chimera::millis() ); );
        continue;
      }

      /*------------------------------------------------
      Assuming the frame is valid, handle the message appropriately
      ------------------------------------------------*/
      if ( Frame::getDestinationFromBuffer( buffer ) == mNode->getLogicalAddress() )
      {
        sysMsg = handleDestination( buffer );
      }
      else
      {
        sysMsg = handlePassthrough( buffer );
      }

      pipe = radio->payloadAvailable();
    }

    return sysMsg;
  }

  void Driver::updateTX()
  {
  
  }

  bool Driver::available()
  {
    return !rxQueue.empty();
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
    rxQueue.removeFront();
  }

  LogicalAddress Driver::nextHop( const LogicalAddress dst )
  {
    /* Network Forwarding Rules:

    Ok so when forwarding a packet through the network, I will have two options: Forward to a parent or forward to a Child

    What determines when I should pass to a parent vs child?
      1. Check first if the destination node is any of the registered nodes
      2. If it's a descendant, will pass to one of the children, otherwise it goes to the parent
          a. Question for later: Is a descendant guaranteed to exist? Do we need to load the network with data only for the message to hit a dead end?
          b. Thought: I guess that would mean each node needs to have the whole tree structure in a table so they know what does and does not exist...yikes.
              - I think that might be an "advanced" feature for mesh.
              - Automatic networking structure update packets sound cool

    When you pass to the parent:
      1. Not much to do here. There is only one parent so pass it and move on.
          a. Do I need to wait for an ACK?

    When you pass to a child:
      1. Need to figure out which of the registered children act as a parent (either directly or farther up the tree) to the destination node.
      2. I suppose that can be answered by the "is_descendent" functions


    Pseudo Code:

    LogicalAddress returnAddress = invalid;

    if(destination is registered with node) // Create a function here with signature bool isRegisteredDirectly(const LogicalAddress a, LogicalAddress *const b );
    {
      return output from function;
    }
    else if( destination has an equal or  higher level than current level )
    {
      return parentAddress;
    }
    else if( destination is descendant of node) // Also function: bool isRegisteredDescendant( const LogicalAddress a, LogicalAddress *const b );
    {
      return output from function;
    }
    else  // Destination is somewhere in the descendant tree
    {
      return invalid;
    } 
    
    return returnAddress;
    */

    LogicalAddress ancestor;

    if ( isRegisteredDirectly( dst ) )
    {
      // Next hop IS the destination node
      return dst;
    }
    else if ( getLevel( dst ) >= routeTable.getCentralNode().getLevel() )
    {
      // The data must be routed through the parent node to get data to a node 
      // at the same level or higher.
      return routeTable.getParentNode().getLogicalAddress();
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
    return routeTable.attach( address );
  }

  void Driver::setNodeAddress(  const LogicalAddress address )
  {
    routeTable.updateCentralNode( address );
  }

  bool Driver::isRegisteredDirectly( const LogicalAddress toCheck )
  {
    /*------------------------------------------------
    Check the simplest option first: Is the parent?
    ------------------------------------------------*/
    if ( toCheck == routeTable.getParentNode().getLogicalAddress() )
    {
      return true;
    }

    /*------------------------------------------------
    Iterate over the registration list to see if maybe it's there
    ------------------------------------------------*/
    for ( auto const &child : routeTable.getRegistrationList() )
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
    for ( auto const &child : routeTable.getRegistrationList() )
    {
      if ( RF24::isDescendent( child.getLogicalAddress(), toCheck ) )
      {
        which = child.getLogicalAddress();
        return true;
      }
    }

    return false;
  }

  bool Driver::transferToPipe( const ::RF24::PhysicalAddress address, const Frame::Buffer &buffer, const size_t length, const bool autoAck )
  {
    Chimera::Status_t result  = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    If we are multi casting, turn off auto acknowledgment. 
    We don't care how the message gets out as long as it does.
    ------------------------------------------------*/
    result |= radio->stopListening();
    result |= radio->toggleAutoAck( autoAck, RF24::Hardware::PIPE_NUM_0 );
    result |= radio->openWritePipe( address );
    result |= radio->immediateWrite( buffer, length );
    result |= radio->txStandBy( 10, true );
    result |= radio->startListening();

    return ( result == Chimera::CommonStatusCodes::OK );
  }

  void Driver::enqueueRXPacket( Frame::Buffer &buffer )
  {
    if ( !rxQueue.full())
    {
      auto size = Frame::getFrameLengthFromBuffer( buffer );
      rxQueue.push( buffer.data(), size );
    }
    else
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d-NET: **Drop Payload** Buffer Full\n", Chimera::millis() ); );
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

  HeaderMessage Driver::handleDestination( Frame::Buffer &buffer )
  {
    HeaderMessage message = Frame::getHeaderTypeFromBuffer( buffer );

    switch ( message )
    {
      /*------------------------------------------------
      Simple ping. ACK handled automatically in hardware.
      ------------------------------------------------*/
      case MSG_NETWORK_PING:
        break;
      
      default:
        break;
    }

    /*------------------------------------------------
    Getting here means the frame is destined for the user
    ------------------------------------------------*/
    enqueueRXPacket( buffer );

    return message;
  }

  HeaderMessage Driver::handlePassthrough( Frame::Buffer &buffer )
  {
    return MSG_NETWORK_ERR;
  }

  bool Driver::writeDirect( Frame::FrameType &frame )
  {
    using namespace ::RF24::Physical::Conversion;

    auto pipeOnParent  = getIdAtLevel( frame.getSrc(), getLevel( frame.getSrc() ) );
    auto physicalAddress = getPhysicalAddress( frame.getDst(), static_cast<Hardware::PipeNumber>( pipeOnParent ) );

    frame.updateCRC();

    IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d-NET: TX direct packet of type [%d] from [%04o] to [%04o]\n",
                                   Chimera::millis(), frame.getType(), frame.getSrc(), frame.getDst() ); );

    return transferToPipe( physicalAddress, frame.toBuffer(), frame.getPayloadLength(), false );
  }

  bool Driver::writeRouted( Frame::FrameType &frame )
  {
    using namespace ::RF24::Physical::Conversion;

    auto hopAddress = nextHop( frame.getDst() );
    if ( hopAddress != RSVD_ADDR_INVALID )
    {
      // The level ID is directly convertible to the pipe number
      auto pipeOnHop = getIdAtLevel( hopAddress, getLevel( hopAddress ) );
      auto physicalAddress = getPhysicalAddress( hopAddress, static_cast<Hardware::PipeNumber>( pipeOnHop ) );

      frame.updateCRC();

      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d-NET: TX routed packet of type [%d] from [%04o] to [%04o] through [%04o]\n",
                                     Chimera::millis(), frame.getType(), frame.getSrc(), frame.getDst(), hopAddress ); );
      
      return transferToPipe( hopAddress, frame.toBuffer(), frame.getPayloadLength(), false );
    }
    else
    {
      logger->flog( uLog::Level::LVL_INFO, "Drop routed packet. Unable to deduce next destination.\n" );
      return false;
    }
  }

  bool Driver::writeMulticast( Frame::FrameType &frame )
  {
    radio->startListening();
    return false;
  }

  bool Driver::readWithPop( Frame::FrameType &frame, const bool pop )
  {
    if ( !rxQueue.empty() )
    {
      /*------------------------------------------------
      Peek the next element and verify it contains data
      ------------------------------------------------*/
      Frame::Buffer tempBuffer;
      auto element = rxQueue.peek();
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
        rxQueue.pop( element.payload, element.size );
      }

      return validity;
    }

    return false;
  }



}    // namespace RF24::Network
