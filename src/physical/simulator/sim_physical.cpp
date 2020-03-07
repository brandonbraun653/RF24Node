/********************************************************************************
 *  File Name:
 *    sim_physical.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#if defined( RF24_SIMULATOR )

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Logger Includes */
#include <uLog/types.hpp>
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

/* RF24 Includes */
#include <RF24Node/src/physical/simulator/sim_physical.hpp>
#include <RF24Node/src/hardware/register.hpp>
#include <RF24Node/src/physical/simulator/shockburst.hpp>
#include <RF24Node/src/network/frame/frame.hpp>

namespace RF24::Physical
{
  SimulatorDriver::SimulatorDriver()
  {
    flagInitialized    = false;
    killFlag           = false;
    IPAddress          = "";
    mChannel           = 0;
    mStaticPayloadSize = 0;

    mDataRate = Hardware::DR_1MBPS;
    mPALevel  = Hardware::PA_LOW;
  }

  SimulatorDriver::~SimulatorDriver()
  {
  }

  Chimera::Status_t SimulatorDriver::initialize( const RF24::Physical::Config &cfg )
  {
    /*------------------------------------------------
    Create the low level IO transceivers
    ------------------------------------------------*/
    ioService.reset();
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ] =
          std::make_unique<Shockburst::Socket>( static_cast<RF24::Hardware::PipeNumber>( x ), ioService, cfg.deviceName );

      if ( logger )
      {
        mDataPipes[ x ]->attachLogger( logger );
      }
    }

    /*------------------------------------------------
    Start the thread that mimics hardware processing of
    the pipe TX/RX FIFO queues.
    ------------------------------------------------*/
    FIFOProcessingThread = std::thread( &SimulatorDriver::QueueHandler, this );

    flagInitialized = true;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::isInitialized()
  {
    return ( flagInitialized ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::NOT_INITIALIZED );
  }

  Chimera::Status_t SimulatorDriver::isConnected()
  {
    /*------------------------------------------------
    We are always connected in the simulator
    ------------------------------------------------*/
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count,
                                                 const bool validate )
  {
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->setRetryDelay( delay );
      mDataPipes[ x ]->setRetryLimit( count );
    }
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::setChannel( const size_t channel, const bool validate )
  {
    /*------------------------------------------------
    For our purposes, the channel is the IP address of the
    pipes. Given that this is fixed to local-host, there is nothing
    to do here. Left as a variable so minimal code changes
    will be needed later should I switch this up.
    ------------------------------------------------*/
    IPAddress = "127.0.0.1";
    mChannel  = channel;
    return Chimera::CommonStatusCodes::OK;
  }

  size_t SimulatorDriver::getChannel()
  {
    return mChannel;
  }

  Chimera::Status_t SimulatorDriver::setStaticPayloadSize( const size_t size )
  {
    mStaticPayloadSize = size;
    return Chimera::CommonStatusCodes::OK;
  }

  size_t SimulatorDriver::getStaticPayloadSize()
  {
    return mStaticPayloadSize;
  }

  size_t SimulatorDriver::getDynamicPayloadSize()
  {
    return RF24::Network::Frame::size();
  }

  Chimera::Status_t SimulatorDriver::startListening()
  {
    /*------------------------------------------------
    In hardware, this is done by toggling a physical pin
    and setting a few register bits.
    ------------------------------------------------*/
    logger->flog( uLog::Level::LVL_INFO, "Starting all pipes for listening\n" );
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->startListening();
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::stopListening()
  {
    /*------------------------------------------------
    In hardware, this is done by toggling a physical pin that
    places the chip in a standby mode. We don't have that in
    software, so simply tell the pipes to plug their ears.
    ------------------------------------------------*/
    logger->flog( uLog::Level::LVL_INFO, "Stopping all pipes from listening\n" );
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->stopListening();
    }

    Chimera::delayMilliseconds( 10 );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::pauseListening()
  {
    return stopListening();
  }

  Chimera::Status_t SimulatorDriver::resumeListening()
  {
    return startListening();
  }

  Chimera::Status_t SimulatorDriver::openWritePipe( const RF24::PhysicalAddress address )
  {
    return ( mDataPipes[ 0 ]->openWritePipe( address ) ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  Chimera::Status_t SimulatorDriver::closeWritePipe()
  {
    return ( mDataPipes[ 0 ]->closeWritePipe() ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  Chimera::Status_t SimulatorDriver::openReadPipe( const RF24::Hardware::PipeNumber pipe, const RF24::PhysicalAddress address,
                                                   const bool validate )
  {
    return ( mDataPipes[ pipe ]->openReadPipe( address ) ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  Chimera::Status_t SimulatorDriver::closeReadPipe( const RF24::Hardware::PipeNumber pipe )
  {
    return ( mDataPipes[ pipe ]->closeReadPipe() ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  RF24::Hardware::PipeNumber SimulatorDriver::payloadAvailable()
  {
    std::lock_guard<std::recursive_mutex> guard( FIFOLock );
    if ( !RxFIFO.empty() )
    {
      auto elem = RxFIFO.front();
      return elem.pipe;
    }
    else
    {
      return RF24::Hardware::PIPE_INVALID;
    }
  }

  size_t SimulatorDriver::getPayloadSize( const RF24::Hardware::PipeNumber pipe )
  {
    std::lock_guard<std::recursive_mutex> guard( FIFOLock );
    if ( !RxFIFO.empty() )
    {
      auto elem = RxFIFO.front();
      return RF24::Network::Frame::getFrameLengthFromBuffer( elem.payload );
    }
    else
    {
      return 0;
    }
  }

  Chimera::Status_t SimulatorDriver::readPayload( RF24::Network::Frame::Buffer &buffer, const size_t length )
  {
    std::lock_guard<std::recursive_mutex> guard( FIFOLock );
    if ( !RxFIFO.empty() )
    {
      logger->flog( uLog::Level::LVL_INFO, "length of %d\n", length );
      auto elem = RxFIFO.front();
      memcpy( buffer.data(), elem.payload.data(), length );
      RxFIFO.pop();
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      return Chimera::CommonStatusCodes::EMPTY;
    }
  }

  Chimera::Status_t SimulatorDriver::immediateWrite( const RF24::Network::Frame::Buffer &buffer, const size_t length )
  {
    std::lock_guard<std::recursive_mutex> guard( FIFOLock );
    if ( TxFIFO.size() < RF24::Physical::Shockburst::FIFO_QUEUE_MAX_SIZE )
    {
      FIFOElement elem;
      elem.pipe    = RF24::Hardware::PIPE_NUM_0;
      elem.payload = buffer;
      elem.size    = length;

      TxFIFO.push( elem );
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      logger->flog( uLog::Level::LVL_INFO, "%d PHY: Failed writing payload due to FIFO queue full\n", Chimera::millis() );
      return Chimera::CommonStatusCodes::FULL;
    }
  }

  Chimera::Status_t SimulatorDriver::txStandBy( const size_t timeout, const bool startTx )
  {
    Chimera::delayMilliseconds( timeout );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::stageAckPayload( const RF24::Hardware::PipeNumber pipe,
                                                      const RF24::Network::Frame::Buffer &buffer, size_t length )
  {
    // Will need to support this directly in the pipes
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Reg8_t SimulatorDriver::flushTX()
  {
    /*------------------------------------------------
    Normally this is done with a single command in hardware
    ------------------------------------------------*/
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->flushTX();
    }

    /* I'm not calculating register values, so simply return the default reset */
    return RF24::Hardware::STATUS_Reset;
  }

  Reg8_t SimulatorDriver::flushRX()
  {
    /*------------------------------------------------
    Normally this is done with a single command in hardware
    ------------------------------------------------*/
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->flushRX();
    }

    /* I'm not calculating register values, so simply return the default reset */
    return RF24::Hardware::STATUS_Reset;
  }

  Chimera::Status_t SimulatorDriver::toggleDynamicPayloads( const bool state )
  {
    /*------------------------------------------------
    Normally this is done with a single command in hardware
    ------------------------------------------------*/
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->toggleDynamicPayloads( state );
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SimulatorDriver::setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate )
  {
    /*------------------------------------------------
    HW only function. Means nothing for SW.
    ------------------------------------------------*/
    mPALevel = level;
    return Chimera::CommonStatusCodes::OK;
  }

  RF24::Hardware::PowerAmplitude SimulatorDriver::getPALevel()
  {
    return mPALevel;
  }

  Chimera::Status_t SimulatorDriver::setDataRate( const RF24::Hardware::DataRate speed )
  {
    /*------------------------------------------------
    HW only function. Means nothing for SW.
    ------------------------------------------------*/
    mDataRate = speed;
    return Chimera::CommonStatusCodes::OK;
  }

  RF24::Hardware::DataRate SimulatorDriver::getDataRate()
  {
    return mDataRate;
  }

  Chimera::Status_t SimulatorDriver::toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber pipe )
  {
    /*------------------------------------------------
    Normally this is done with a single command in hardware
    ------------------------------------------------*/
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->toggleAutoAck( state );
    }

    return Chimera::CommonStatusCodes::OK;
  }

  void SimulatorDriver::QueueHandler()
  {
    try
    {
      while ( !killFlag )
      {
        /*------------------------------------------------
        Handle RX Payloads
        ------------------------------------------------*/
        for ( size_t x = 0; x < mDataPipes.size(); x++ )
        {
          if ( mDataPipes[ x ]->available() )
          {
            FIFOElement elem;
            elem.pipe = static_cast<RF24::Hardware::PipeNumber>( x );
            mDataPipes[ x ]->read( elem.payload );

            if ( RxFIFO.size() < RF24::Physical::Shockburst::FIFO_QUEUE_MAX_SIZE )
            {
              std::lock_guard<std::recursive_mutex> guard( FIFOLock );
              logger->flog( uLog::Level::LVL_INFO, "%d PHY: Enqueue RX payload on pipe %d\n", Chimera::millis(), elem.pipe );
              RxFIFO.push( elem );
            }
            else
            {
              logger->flog( uLog::Level::LVL_INFO, "%d PHY: Lost packet from pipe %d due to FIFO queue full\n",
                            Chimera::millis(), x );
            }
          }
        }

        /*------------------------------------------------
        Handle TX Payloads
        ------------------------------------------------*/
        while ( !TxFIFO.empty() )
        {
          /*------------------------------------------------
          We can't transmit if the user enabled listening mode
          ------------------------------------------------*/
          if ( mDataPipes[ 0 ]->isListening() )
          {
            logger->flog(uLog::Level::LVL_ERROR, "%d-PHY: Cannot TX while radio is listening\n", Chimera::millis() );
            break;
          }

          auto elem = TxFIFO.front();
          mDataPipes[ 0 ]->write( elem.payload );
          TxFIFO.pop();
        }

        /*------------------------------------------------
        Make sure other threads can do work
        ------------------------------------------------*/
        Chimera::Threading::this_thread::yield();
        Chimera::delayMilliseconds( 5 );
      }
    }
    catch ( const std::exception & )
    {
      // Exiting
    }
  }

  Chimera::Status_t SimulatorDriver::attachLogger( uLog::SinkHandle sink )
  {
    logger = sink;
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      if ( mDataPipes[ x ] )
      {
        mDataPipes[ x ]->attachLogger( logger );
      }
    }
    return Chimera::CommonStatusCodes::OK;
  }

}    // namespace RF24::Physical

#endif /* RF24_SIMULATOR */
