/********************************************************************************
 *  File Name:
 *    pipe.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Boost Includes */
#include <boost/chrono.hpp>
#include <boost/thread.hpp>

/* Logger Includes */
#include <uLog/types.hpp>
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/physical/simulator/pipe.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>


using namespace boost::asio::ip;
using namespace uLog;

namespace RF24::Physical::Pipe
{
  /*------------------------------------------------
  TX Pipe Implementation
  ------------------------------------------------*/
  TX::TX( boost::asio::io_service &io_service ) : mTXSocket( io_service ), mIOService( io_service )
  {
    mTXThread     = {};
    mUserCallback = nullptr;
    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );

    mLogger = uLog::getRootSink();
  }

  TX::~TX()
  {
    closePipe();
  }

  void TX::openPipe()
  {
    /*------------------------------------------------
    Handle an already open socket
    ------------------------------------------------*/
    if ( mTXSocket.is_open() )
    {
      mTXSocket.close();
    }

    /*------------------------------------------------
    Open the tx pipe and start the async worker
    ------------------------------------------------*/
    mTXSocket.open( udp::v4() );
    mTXThread = boost::thread( boost::bind( &TX::updateThread, this ) );
  }

  void TX::closePipe()
  {
    std::lock_guard<std::mutex> guard( mBufferLock );

    /*------------------------------------------------
    Destroy the update thread
    ------------------------------------------------*/
    mTXThread.interrupt();
    mTXThread.join();

    /*------------------------------------------------
    Kill the socket resources
    ------------------------------------------------*/
    mTXSocket.close();

    /*------------------------------------------------
    Clear our local memory buffers
    ------------------------------------------------*/
    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
  }

  void TX::write( const RF24::Physical::Shockburst::PacketBuffer &data )
  {
    /*------------------------------------------------
    Add work to process the TX request
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( mBufferLock );
    auto ip   = address_v4::from_string( Conversion::decodeIP( data ) );
    auto port = Conversion::decodePort( data );

    mLogger->flog( Level::LVL_DEBUG, "DBG: TX to IP[%s] on Port[%d]\n", ip.to_string().c_str(), port );

    memcpy( mBuffer.data(), data.data(), mBuffer.size() );
    mTXSocket.async_send_to( boost::asio::buffer( mBuffer ), udp::endpoint( ip, port ),
                             boost::bind( &TX::onAsyncTransmit, this, boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred ) );
  }

  void TX::onTransmit( TXPipeCallback &callback )
  {
    /*------------------------------------------------
    Make sure we aren't doing some kind of queue processing
    before assigning the callback.
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( mBufferLock );
    mUserCallback = callback;
  }

  void TX::flush()
  {
    std::lock_guard<std::mutex> guard( mBufferLock );
    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
  }

  void TX::updateThread()
  {
    try
    {
      mIOService.reset();

      /*------------------------------------------------
      Perform work while someone doesn't interrupt us
      ------------------------------------------------*/
      while ( true )
      {
        mIOService.run();
        boost::this_thread::sleep_for( boost::chrono::milliseconds( 10 ) );
      }
    }
    catch ( boost::thread_interrupted & )
    {
      mLogger->flog( Level::LVL_DEBUG, "DBG: TX thread interrupted...Exiting\n" );
    }
  }

  void TX::onAsyncTransmit( const boost::system::error_code &error, size_t bytes_transferred )
  {
    if ( error )
    {
      mLogger->flog( Level::LVL_ERROR, "ERR: TX failed on pipe\n" );
      mLogger->flog( Level::LVL_ERROR, "ERR:\t%s\n", error.message() );
      return;
    }

    /*------------------------------------------------
    Handle the user's callback
    ------------------------------------------------*/
    if ( mUserCallback )
    {
      mUserCallback( this );
    }
  }

  /*------------------------------------------------
  RX Pipe Implementation
  ------------------------------------------------*/
  RX::RX( boost::asio::io_service &io_service ) : mIOService( io_service ), mRXSocket( io_service )
  {
    mAllowListening   = false;
    mRXEventProcessed = false;

    mRXThread     = {};
    mUserCallback = nullptr;

    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );

    mLogger = uLog::getRootSink();
  }

  RX::~RX()
  {
    stopListening();
    closePipe();
  }

  void RX::openPipe( const std::string &ip, const uint16_t port )
  {
    /*------------------------------------------------
    Handle an already open socket
    ------------------------------------------------*/
    if ( mRXSocket.is_open() )
    {
      closePipe();
    }

    /*------------------------------------------------
    Open the socket and start listening
    ------------------------------------------------*/
    mRXSocket.open( udp::v4() );
    mRXSocket.bind( udp::endpoint( address::from_string( ip ), port ) );

    mRXThread = boost::thread( boost::bind( &RX::updateThread, this ) );
  }

  void RX::closePipe()
  {
    /*------------------------------------------------
    Destroy the update thread
    ------------------------------------------------*/
    mRXThread.interrupt();
    mRXThread.join();

    /*------------------------------------------------
    Kill the socket resources
    ------------------------------------------------*/
    mRXSocket.close();

    /*------------------------------------------------
    Clear our local memory buffers
    ------------------------------------------------*/
    mBuffer.fill( 0 );
  }

  void RX::startListening()
  {
    mAllowListening = true;
  }

  void RX::stopListening()
  {
    mAllowListening = false;
  }

  bool RX::isListening()
  {
    return mAllowListening;
  }

  bool RX::available()
  {
    /*------------------------------------------------
    Whenever we flush the buffer, it will always be filled with this pattern
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( mBufferLock );
    return !( ( mBuffer[ 0 ] == mBuffer[ mBuffer.size() - 1 ] ) && ( mBuffer[ 0 ] == RF24::Physical::Shockburst::INVALID_MEMORY ) );
  }

  RF24::Physical::Shockburst::PacketBuffer RX::read()
  {
    /*------------------------------------------------
    Obtain access to the data and copy it out
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( mBufferLock );
    RF24::Physical::Shockburst::PacketBuffer temp;

    if ( available() )
    {
      memcpy( temp.data(), mBuffer.data(), temp.size() );
      mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
    }

    return temp;
  }

  void RX::flush()
  {
    std::lock_guard<std::mutex> guard( mBufferLock );
    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
  }

  void RX::onReceive( RXPipeCallback &callback )
  {
    /*------------------------------------------------
    Make sure we aren't doing some kind of queue processing
    before assigning the callback.
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( mBufferLock );
    mUserCallback = callback;
  }

  void RX::updateThread()
  {
    /*------------------------------------------------
    Pre-allocate the callback/buffers for async RX handler
    ------------------------------------------------*/
    auto rxCallback = boost::bind( &RX::onAsyncReceive, this, boost::asio::placeholders::error,
                                   boost::asio::placeholders::bytes_transferred );

    auto rxBuffer = boost::asio::buffer( mBuffer );

    try
    {
      /*------------------------------------------------
      Queue up some work for the ioService
      ------------------------------------------------*/
      mRXSocket.async_receive( boost::asio::buffer( mBuffer ), rxCallback );
      mIOService.reset();

      /*------------------------------------------------
      Perform work while someone doesn't interrupt us
      ------------------------------------------------*/
      while ( true )
      {
        if ( mAllowListening )
        {
          /*------------------------------------------------
          Process async work if their events occurred
          ------------------------------------------------*/
          mIOService.run();

          /*------------------------------------------------
          Only add new work to the queue if we processed the last one
          ------------------------------------------------*/
          if ( mRXEventProcessed )
          {
            mRXSocket.async_receive( rxBuffer, rxCallback );
            mRXEventProcessed = false;
          }
        }

        boost::this_thread::sleep_for( boost::chrono::milliseconds( 10 ) );
      }
    }
    catch ( boost::thread_interrupted & )
    {
      mLogger->flog( Level::LVL_DEBUG, "DBG: RX thread interrupted...Exiting\n" );
    }
  }

  void RX::onAsyncReceive( const boost::system::error_code &error, size_t bytes_transferred )
  {
    if ( error )
    {
      mLogger->flog( Level::LVL_ERROR, "ERR: RX failed on pipe\n" );
      mLogger->flog( Level::LVL_ERROR, "ERR:\t%s\n", error.message() );
      return;
    }

    /*------------------------------------------------
    Protect the queue from other threads
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( mBufferLock );
    mLogger->flog( Level::LVL_DEBUG, "DBG: RX packet of size %lu\n", bytes_transferred );

    /*------------------------------------------------
    Handle the user's callback
    ------------------------------------------------*/
    if ( mUserCallback )
    {
      mUserCallback( this, mBuffer );
    }

    mRXEventProcessed = true;
  }

}    // namespace RF24::Physical::Pipe