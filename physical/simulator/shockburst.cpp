/********************************************************************************
 *  File Name:
 *    shockburst.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <mutex>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/physical/simulator/shockburst.hpp>


namespace RF24::Physical::Shockburst
{
  DataPipe::DataPipe( const RF24::Hardware::PipeNumber_t pipe, boost::asio::io_service &io_service ) :
      rxPipe( io_service ), txPipe( io_service ), pipeID( pipe )
  {
    mHasUserTxPipe = false;
    mUseDynamicPayloads = false;
    mUseAutoAcknowledge = false;
    mRetryLimit    = 0;
    mRetryDelay    = 0;
    cachedTXAddress = 0;

    /*------------------------------------------------
    On the NRF24L01, the only pipe that lets the user
    transmit data is the first pipe.
    ------------------------------------------------*/
    if ( pipeID == Hardware::PIPE_NUM_0 )
    {
      mHasUserTxPipe = true;
    }
  }

  DataPipe::~DataPipe()
  {
  }

  void DataPipe::setRetryLimit( const size_t limit )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    mRetryLimit = limit;
  }

  void DataPipe::setRetryDelay( const size_t delay )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    mRetryDelay = delay;
  }

  void DataPipe::startListening()
  {
    rxPipe.flush();
    rxPipe.startListening();
  }

  void DataPipe::stopListening()
  {
    rxPipe.stopListening();
  }

  bool DataPipe::openWritePipe( const uint64_t address )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    if (!address || !mHasUserTxPipe)
    {
      return false;
    }

    txPipe.openPipe();
    cachedTXAddress = address;
    return true;
  }

  bool DataPipe::closeWritePipe()
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    if (!mHasUserTxPipe)
    {
      return false;
    }

    txPipe.closePipe();
    cachedTXAddress = 0;
    return true;
  }

  bool DataPipe::openReadPipe( const uint64_t address )
  {
    closeWritePipe();
    rxPipe.openPipe( Conversion::decodeIP( address ), Conversion::decodePort( address ) );
    return true;
  }

  bool DataPipe::closeReadPipe()
  {
    rxPipe.closePipe();
    return true;
  }

  void DataPipe::flushTX()
  {
    txPipe.flush();
  }

  void DataPipe::flushRX()
  {
    rxPipe.flush();
  }

  void DataPipe::toggleDynamicPayloads( const bool state )
  {
    mUseDynamicPayloads = state;
  }

  void DataPipe::toggleAutoAck( const bool state )
  {
    mUseAutoAcknowledge = state;
  }

}    // namespace RF24::Physical::Shockburst