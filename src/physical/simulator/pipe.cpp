/********************************************************************************
 *  File Name:
 *    pipe.cpp
 *
 *  Description:
 *
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#if defined( RF24_SIMULATOR )

/* C++ Includes */
#include <type_traits>

/* Boost Includes */
#include <boost/chrono.hpp>
#include <boost/thread.hpp>

/* Chimera Includes */
#include <Chimera/common>

/* Logger Includes */
#include <uLog/types.hpp>
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/physical/simulator/pipe.hpp>
#include <RF24Node/src/physical/simulator/shockburst_types.hpp>


using namespace boost::asio::ip;
using namespace uLog;

namespace RF24::Physical::Pipe
{
  /*------------------------------------------------
  TX Pipe Implementation
  ------------------------------------------------*/
  TX::TX( boost::asio::io_service &io_service, const std::string name, const size_t pipe ) :
      mTXSocket( io_service ), mIOService( io_service ), mName( name ), mPipeNumber( pipe )
  {
    mTXThread     = {};
    mTXCompleteCallback = nullptr;
    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );

    mLogger = nullptr;
  }

  TX::~TX()
  {
    closePipe();
  }

  Chimera::Status_t TX::attachLogger( uLog::SinkHandle sink )
  {
    mLogger = sink;
    return Chimera::CommonStatusCodes::OK;
  }

  void TX::openPipe()
  {
    /*------------------------------------------------
    Handle an already open socket
    ------------------------------------------------*/
    if ( mTXSocket.is_open() )
    {
      return;
    }

    /*------------------------------------------------
    Open the tx pipe and start the async worker
    ------------------------------------------------*/
    mTXSocket.open( udp::v4() );
    mTXThread = boost::thread( boost::bind( &TX::updateThread, this ) );
  }

  void TX::closePipe()
  {
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );

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

  bool TX::isOpen()
  {
    return mTXSocket.is_open();
  }

  void TX::write( const RF24::Physical::Shockburst::PacketBuffer &data )
  {
    /*------------------------------------------------
    Add work to process the TX request
    ------------------------------------------------*/
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
    auto ip   = address_v4::from_string( Conversion::decodeIP( data ) );
    auto port = Conversion::decodePort( data );

    memcpy( mBuffer.data(), data.data(), mBuffer.size() );
    mTXSocket.async_send_to( boost::asio::buffer( mBuffer ), udp::endpoint( ip, port ),
                             boost::bind( &TX::onAsyncTransmit, this, boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred ) );
  }

  void TX::onTransmitComplete( std::function<void( void )> callback )
  {
    /*------------------------------------------------
    Make sure we aren't doing some kind of queue processing
    before assigning the callback.
    ------------------------------------------------*/
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
    mTXCompleteCallback = callback;
  }

  void TX::flush()
  {
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
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
        mIOService.poll_one();
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
      mLogger->flog( Level::LVL_ERROR, "%d-PHY: ERROR TX failed on pipe %d\n", Chimera::millis(), mPipeNumber );
      mLogger->flog( Level::LVL_ERROR, "%d-PHY:\t%s\n", error.message() );
      return;
    }

    /*------------------------------------------------
    Handle the user's callback
    ------------------------------------------------*/
    if ( mTXCompleteCallback )
    {
      mTXCompleteCallback();
    }
  }


  /*------------------------------------------------
  RX Pipe Implementation
  ------------------------------------------------*/
  RX::RX( boost::asio::io_service &io_service, const std::string name, const size_t pipe  ) : 
    mIOService( io_service ), mRXSocket( io_service ), mName( name ), mPipeNumber( pipe )
  {
    mAllowListening   = false;
    mRXEventProcessed = false;

    mRXThread     = {};
    mRXCompleteCallback = nullptr;

    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );

    mLogger = uLog::getRootSink();
  }

  RX::~RX()
  {
    stopListening();
    closePipe();
  }

  Chimera::Status_t RX::attachLogger( uLog::SinkHandle sink )
  {
    mLogger = sink;
    return Chimera::CommonStatusCodes::OK;
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
    static_assert( sizeof( std::decay_t<decltype( mBuffer[ 0 ] )> ) ==
                       sizeof( std::decay_t<decltype( RF24::Physical::Shockburst::INVALID_MEMORY )> ),
                   "Invalid memory comparison types" );

    /*------------------------------------------------
    Whenever we flush the buffer, it will always be filled with this pattern.
    ------------------------------------------------*/
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
    return !( mBuffer[ 0 ] == RF24::Physical::Shockburst::INVALID_MEMORY );
  }

  RF24::Physical::Shockburst::PacketBuffer RX::read()
  {
    /*------------------------------------------------
    Obtain access to the data and copy it out
    ------------------------------------------------*/
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
    RF24::Physical::Shockburst::PacketBuffer temp;
    temp.fill( 0 );

    if ( available() )
    {
      memcpy( temp.data(), mBuffer.data(), temp.size() );
      mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
    }

    return temp;
  }

  void RX::flush()
  {
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
    mBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
  }

  void RX::onReceiveComplete( Chimera::Callback::DefaultFunction callback )
  {
    /*------------------------------------------------
    Make sure we aren't doing some kind of queue processing
    before assigning the callback.
    ------------------------------------------------*/
    std::lock_guard<std::recursive_mutex> guard( mBufferLock );
    mRXCompleteCallback = callback;
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
      mRXSocket.async_receive( rxBuffer, rxCallback );
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
          mIOService.poll_one();

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

    mLogger->flog( Level::LVL_INFO, "Pipe exiting RX thread handler\n" );
  }

  void RX::onAsyncReceive( const boost::system::error_code &error, size_t bytes_transferred )
  {
    if ( error )
    {
      mLogger->flog( Level::LVL_ERROR, "%d-PHY: ERROR RX failed on pipe %d\n", Chimera::millis(), mPipeNumber );
      mLogger->flog( Level::LVL_ERROR, "%d-PHY:\t%s\n", error.message() );
      return;
    }

    /*------------------------------------------------
    Handle the user's callback
    ------------------------------------------------*/
    if ( mRXCompleteCallback )
    {
      mRXCompleteCallback();
    }

    mRXEventProcessed = true;
  }

}    // namespace RF24::Physical::Pipe

#endif /* _WIN32 || _WIN64 */
