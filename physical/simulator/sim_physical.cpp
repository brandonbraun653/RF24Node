/********************************************************************************
 *  File Name:
 *    sim_physical.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* RF24 Includes */
#include <RF24Node/physical/simulator/sim_physical.hpp>
#include <RF24Node/hardware/register.hpp>

namespace RF24::Physical
{
  SimulatorDriver::SimulatorDriver()
  {
    flagInitialized    = false;
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
    for ( size_t x = 0; x < mDataPipes.size(); x++ )
    {
      mDataPipes[ x ] = std::make_unique<Shockburst::DataPipe>( static_cast<RF24::Hardware::PipeNumber_t>( x ), ioService );
    }

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
    for( size_t x=0; x<mDataPipes.size(); x++ )
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
    mChannel = channel;
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
    //Will likely have to peek the RX queue
    return 0;
  }

  Chimera::Status_t SimulatorDriver::startListening()
  {
    /*------------------------------------------------
    In hardware, this is done by toggling a physical pin
    and setting a few register bits.
    ------------------------------------------------*/
    for( size_t x=0; x<mDataPipes.size(); x++ )
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
    for( size_t x=0; x<mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->stopListening();
    }

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

  Chimera::Status_t SimulatorDriver::openWritePipe( const uint64_t address )
  {
    return ( mDataPipes[ 0 ]->openWritePipe( address ) ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  Chimera::Status_t SimulatorDriver::closeWritePipe()
  {
    return ( mDataPipes[ 0 ]->closeWritePipe() ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  Chimera::Status_t SimulatorDriver::openReadPipe( const RF24::Hardware::PipeNumber_t pipe, const uint64_t address,
                                                   const bool validate )
  {
    return ( mDataPipes[ pipe ]->openReadPipe( address ) ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  Chimera::Status_t SimulatorDriver::closeReadPipe( const RF24::Hardware::PipeNumber_t pipe )
  {
    return ( mDataPipes[ pipe ]->closeReadPipe() ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL );
  }

  RF24::Hardware::PipeNumber_t SimulatorDriver::payloadAvailable()
  {
    if ( mPendingPipeData.empty() )
    {
      return Hardware::PIPE_INVALID;
    }
    else
    {
      return mPendingPipeData.front();
    }
  }

  size_t SimulatorDriver::getPayloadSize( const RF24::Hardware::PipeNumber_t pipe )
  {
    //Will likely have to peek the RX queue
    return 0;
  }

  Chimera::Status_t SimulatorDriver::readPayload( void *const buffer, const size_t bufferLength, const size_t payloadLength )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::immediateWrite( const void *const buffer, const size_t len, const bool multicast )
  {

    Oy vey....need to actually implement this function!!!!!!
    Also a write function for the Shockburst thing.

    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::txStandBy( const size_t timeout, const bool startTx )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::stageAckPayload( const uint8_t pipe, const uint8_t *const buffer, size_t len )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Reg8_t SimulatorDriver::flushTX()
  {
    /*------------------------------------------------
    Normally this is done with a single command in hardware
    ------------------------------------------------*/
    for( size_t x=0; x<mDataPipes.size(); x++ )
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
    for( size_t x=0; x<mDataPipes.size(); x++ )
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
    for( size_t x=0; x<mDataPipes.size(); x++ )
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
    return RF24::Hardware::DataRate();
  }

  Chimera::Status_t SimulatorDriver::toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber_t pipe )
  {
    /*------------------------------------------------
    Normally this is done with a single command in hardware
    ------------------------------------------------*/
    for( size_t x=0; x<mDataPipes.size(); x++ )
    {
      mDataPipes[ x ]->toggleAutoAck( state );
    }

    return Chimera::CommonStatusCodes::OK;
  }
}    // namespace RF24::Physical