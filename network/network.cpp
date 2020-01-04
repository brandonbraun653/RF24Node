/********************************************************************************
 *   File Name:
 *    network.cpp
 *
 *   Description:
 *    Implements the NRF24L01 network layer driver.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cmath>

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/modules/ulog/serial_sink.hpp>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/common/types.hpp>
#include <RF24Node/common/utility.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/network/types.hpp>
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/node/node.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>
#include <uLog/sinks/sink_cout.hpp>

namespace RF24::Network
{

  Driver::Driver( ::RF24::EndpointInterface *const node ) : mNode( node ), frameQueue( Frame::Cache( 3 ) )
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

    uint8_t pipeNum         = 0u;
    HeaderMessage returnVal = MSG_TX_NORMAL;

    /*------------------------------------------------
    Process the incoming data
    ------------------------------------------------*/
    while ( radio->payloadAvailable() < RF24::Hardware::PIPE_NUM_MAX )
    {
      /*------------------------------------------------
      Get the raw data and parse it into a frame
      ------------------------------------------------*/
      Frame::FrameType frame;
      Frame::Buffer buffer;
      radio->readPayload( buffer.data(), buffer.size(), radio->getDynamicPayloadSize() );
      
      frame = buffer;

      /*------------------------------------------------
      Assuming the frame is valid, handle the message appropriately
      ------------------------------------------------*/
      if ( !frame.valid() )
      {
        IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d NET: Pkt dropped due to CRC failure\n", Chimera::millis() ); );
        continue;
      }
      else if ( frame.getDst() == mNode->getLogicalAddress() )
      {
        returnVal = handleDestination( frame );
      }
      else
      {
        returnVal = handlePassthrough( frame );
      }

      /*------------------------------------------------
      Does the result of handling the frame require more 
      interaction from the caller?
      ------------------------------------------------*/
      if ( mReturnSystemMessages )
      {
        return returnVal;
      }
    }

    return returnVal;
  }

  void Driver::updateTX()
  {
  
  }

  bool Driver::available() const
  {
    return !frameQueue.empty();
  }

  bool Driver::peek( Frame::FrameType &frame )
  {
    if ( available() )
    {
      frame = frameQueue.front();
      return frame.valid();
    }

    return false;
  }

  bool Driver::read( Frame::FrameType &frame )
  {
    /*------------------------------------------------
    Pop off the data and perform a CRC check
    ------------------------------------------------*/
    if ( available() )
    {
      frame = frameQueue.front();
      frameQueue.pop_front();
      return frame.valid();
    }

    return false;
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

  bool Driver::transferToPipe( const ::RF24::PhysicalAddress address, const Frame::Buffer &buffer, const bool autoAck )
  {
    Chimera::Status_t result  = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    If we are multi casting, turn off auto acknowledgment. 
    We don't care how the message gets out as long as it does.
    ------------------------------------------------*/
    result |= radio->stopListening();
    result |= radio->toggleAutoAck( autoAck, RF24::Hardware::PIPE_NUM_0 );
    result |= radio->openWritePipe( address );

    // TODO: Need to only transfer the exact amount
    result |= radio->immediateWrite( buffer.data(), buffer.size(), false );

    // TODO: Turn this into a literal constant
    result |= radio->txStandBy( 10 );

    return ( result == Chimera::CommonStatusCodes::OK );
  }

  void Driver::enqueue( Frame::FrameType &frame )
  {
    if ( !frameQueue.full() )
    {
      frameQueue.push_back( frame.toBuffer() );
    }
    else
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: NET **Drop Payload** Buffer Full\n", Chimera::millis() ); );
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

    switch ( message )
    {
      /*------------------------------------------------
      Simple ping. ACK handled automatically in hardware.
      ------------------------------------------------*/
      case MSG_NETWORK_PING:
        break;
            
      /*------------------------------------------------
      A node is trying to directly connect with this node
      ------------------------------------------------*/
      case MSG_NET_REQUEST_BIND:
        // Pass this up to the next layer.
        break;
    }

    /*------------------------------------------------
    Getting here means the frame is destined for the user
    ------------------------------------------------*/
    enqueue( frame );

    return MSG_NETWORK_ERR;
  }

  HeaderMessage Driver::handlePassthrough( Frame::FrameType &frame )
  {
    return MSG_NETWORK_ERR;
  }

  bool Driver::writeDirect( Frame::FrameType &frame )
  {
    using namespace ::RF24::Physical::Conversion;

    auto pipeOnParent  = getIdAtLevel( frame.getSrc(), getLevel( frame.getSrc() ) );
    auto directAddress = getPhysicalAddress( frame.getDst(), static_cast<Hardware::PipeNumber_t>( pipeOnParent ) );

    frame.updateCRC();

    transferToPipe( directAddress, frame.toBuffer(), false );
    radio->startListening();
    return false;
  }

  bool Driver::writeRouted( Frame::FrameType &frame )
  {
    radio->startListening();
    return false;
  }

  bool Driver::writeMulticast( Frame::FrameType &frame )
  {
    radio->startListening();
    return false;
  }

}    // namespace RF24::Network
