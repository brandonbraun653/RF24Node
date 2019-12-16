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
#include <uLog/ulog.hpp>

/* RF24 Includes */
#include <RF24Node/physical/simulator/pipe.hpp>
#include <RF24Node/physical/simulator/conversion.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>


using namespace boost::asio::ip;
using namespace uLog;

namespace RF24::Physical::Pipe
{
  /*------------------------------------------------
  TX Pipe Implementation
  ------------------------------------------------*/
  TX::TX( boost::asio::io_service &io_service ) : txSocket( io_service ), ioService( io_service )
  {
    txThread = {};
    userCallback = nullptr;
    networkBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
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
    if (txSocket.is_open())
    {
      txSocket.close();
    }

    /*------------------------------------------------
    Open the tx pipe and start the async worker
    ------------------------------------------------*/
    txSocket.open( udp::v4() );
    txThread = boost::thread( boost::bind( &TX::updateThread, this ) );
  }

  void TX::closePipe()
  {
    /*------------------------------------------------
    Destroy the update thread
    ------------------------------------------------*/
    txThread.interrupt();
    txThread.join();

    /*------------------------------------------------
    Kill the socket resources
    ------------------------------------------------*/
    txSocket.close();

    /*------------------------------------------------
    Clear our local memory buffers
    ------------------------------------------------*/
    networkBuffer.fill( 0 );
    txFIFO = {};
  }

  void TX::write( const RF24::Physical::Shockburst::PacketBuffer &data )
  {
    std::lock_guard<std::mutex> guard( FIFOLock );

    /*------------------------------------------------
    If the FIFO isn't empty, the ioService hasn't had the chance
    to process all the requested work yet. Queue up the data.
    ------------------------------------------------*/
    if ( !txFIFO.empty() )
    {
      /*------------------------------------------------
      Enqueue the data if we have room left. This is a 
      hardware constraint on the real NRF24 chip.
      ------------------------------------------------*/
      if ( txFIFO.size() < RF24::Physical::Shockburst::FIFO_QUEUE_MAX_SIZE )
      {
        txFIFO.push( data );
      }
      else
      {
        flog( Level::LVL_ERROR, "ERR: TX FIFO queue full, packet lost\n" );
      }
    }
    else
    {
      /*------------------------------------------------
      We are free, so add work to process the TX request
      ------------------------------------------------*/
      auto ip = address_v4::from_string( Conversion::decodeIP( data ) );
      auto port = Conversion::decodePort( data );

      flog( Level::LVL_DEBUG, "DBG: TX to IP[%s] on Port[%d]\n", ip.to_string().c_str(), port );

      memcpy( networkBuffer.data(), data.data(), networkBuffer.size() );
      txSocket.async_send_to( boost::asio::buffer( networkBuffer ), udp::endpoint( ip, port ),
                              boost::bind( &TX::onAsyncTransmit, this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred ) );
    }
  }

  void TX::onTransmit( TXPipeCallback &callback )
  {
    /*------------------------------------------------
    Make sure we aren't doing some kind of queue processing 
    before assigning the callback.
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( FIFOLock );
    userCallback = callback;
  }

  void TX::flush()
  {
    std::lock_guard<std::mutex> guard( FIFOLock );
    txFIFO = {};
  }

  void TX::updateThread()
  {
    try
    {
      ioService.reset();

      /*------------------------------------------------
      Perform work while someone doesn't interrupt us
      ------------------------------------------------*/
      while ( true )
      {
        ioService.run();
        boost::this_thread::sleep_for( boost::chrono::milliseconds( 25 ) );
      }
    }
    catch ( boost::thread_interrupted & )
    {
      flog( Level::LVL_DEBUG, "DBG: TX thread interrupted...Exiting\n" );
    }
  }

  void TX::onAsyncTransmit( const boost::system::error_code &error, size_t bytes_transferred )
  {
    if ( error )
    {
      flog( Level::LVL_ERROR, "ERR: TX failed on pipe\n" );
      flog( Level::LVL_ERROR, "ERR:\t%s\n", error.message() );
      return;
    }

    /*------------------------------------------------
    Protect the queue from other threads
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( FIFOLock );
    
    /*------------------------------------------------
    Handle the user's callback
    ------------------------------------------------*/
    if ( userCallback )
    {
      userCallback( this );
    }

    /*------------------------------------------------
    If we have packets in the queue, add more work
    ------------------------------------------------*/
    if ( !txFIFO.empty() )
    {
      auto nextPacket = txFIFO.front();
      auto ip = address::from_string( Conversion::decodeIP( nextPacket ) );
      auto port = Conversion::decodePort( nextPacket );

      flog( Level::LVL_DEBUG, "DBG: TX to IP[%s] on Port[%d]", ip, port );

      memcpy( networkBuffer.data(), nextPacket.data(), networkBuffer.size() );
      txSocket.async_send_to( boost::asio::buffer( networkBuffer ), udp::endpoint( ip, port ),
                              boost::bind( &TX::onAsyncTransmit, this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred ) );

      txFIFO.pop();
    }
  }

  /*------------------------------------------------
  RX Pipe Implementation 
  ------------------------------------------------*/
  RX::RX( boost::asio::io_service &io_service ) : ioService( io_service ), rxSocket( io_service )
  {
    allowListening = false;
    rxEventProcessed = false;

    rxThread = {};
    userCallback = nullptr;

    networkBuffer.fill( 0 );
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
    if ( rxSocket.is_open() )
    {
      closePipe();
    }

    /*------------------------------------------------
    Open the socket and start listening
    ------------------------------------------------*/
    rxSocket.open( udp::v4() );
    rxSocket.bind( udp::endpoint( address::from_string( ip ), port ) );

    rxThread = boost::thread( boost::bind( &RX::updateThread, this ) );
  }

  void RX::closePipe()
  {
    /*------------------------------------------------
    Destroy the update thread
    ------------------------------------------------*/
    rxThread.interrupt();
    rxThread.join();

    /*------------------------------------------------
    Kill the socket resources
    ------------------------------------------------*/
    rxSocket.close();

    /*------------------------------------------------
    Clear our local memory buffers
    ------------------------------------------------*/
    networkBuffer.fill( 0 );
    rxFIFO = {};
  }

  void RX::startListening()
  {
    allowListening = true;
  }

  void RX::stopListening()
  {
    allowListening = false;
  }

  bool RX::available()
  {
    return !rxFIFO.empty();
  }

  RF24::Physical::Shockburst::PacketBuffer RX::read()
  {
    /*------------------------------------------------
    Ensure we return invalid memory if we can't get anything
    ------------------------------------------------*/
    RF24::Physical::Shockburst::PacketBuffer packet;
    packet.fill( RF24::Physical::Shockburst::INVALID_MEMORY );

    /*------------------------------------------------
    Obtain access to the data and copy it out
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( FIFOLock );
    if ( available() )
    {
      packet = rxFIFO.front();
      rxFIFO.pop();
    }

    return packet;
  }

  void RX::flush()
  {
    std::lock_guard<std::mutex> guard( FIFOLock );
    rxFIFO = {};
  }

  void RX::onReceive( RXPipeCallback &callback )
  {
    /*------------------------------------------------
    Make sure we aren't doing some kind of queue processing 
    before assigning the callback.
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( FIFOLock );
    userCallback = callback;
  }

  void RX::updateThread()
  {
    /*------------------------------------------------
    Pre-allocate the callback/buffers for async RX handler
    ------------------------------------------------*/
    auto rxCallback = boost::bind( &RX::onAsyncReceive, this, boost::asio::placeholders::error,
                                   boost::asio::placeholders::bytes_transferred );

    auto rxBuffer = boost::asio::buffer( networkBuffer );

    try
    {
      /*------------------------------------------------
      Queue up some work for the ioService
      ------------------------------------------------*/
      rxSocket.async_receive( boost::asio::buffer( networkBuffer ), rxCallback );
      ioService.reset();

      /*------------------------------------------------
      Perform work while someone doesn't interrupt us
      ------------------------------------------------*/
      while ( true )
      {
        if ( allowListening )
        {
          /*------------------------------------------------
          Process async work if their events occurred
          ------------------------------------------------*/
          ioService.run();

          /*------------------------------------------------
          Only add new work to the queue if we processed the last one
          ------------------------------------------------*/
          if ( rxEventProcessed )
          {
            rxSocket.async_receive( rxBuffer, rxCallback );
            rxEventProcessed = false;
          }
        }

        boost::this_thread::sleep_for( boost::chrono::milliseconds( 25 ) );
      }
    }
    catch ( boost::thread_interrupted & )
    {
      flog( Level::LVL_DEBUG, "DBG: RX thread interrupted...Exiting\n" );
    }
  }

  void RX::onAsyncReceive( const boost::system::error_code &error, size_t bytes_transferred )
  {
    if ( error )
    {
      flog( Level::LVL_ERROR, "ERR: RX failed on pipe\n" );
      flog( Level::LVL_ERROR, "ERR:\t%s\n", error.message() );
      return;
    }

    /*------------------------------------------------
    Protect the queue from other threads
    ------------------------------------------------*/
    std::lock_guard<std::mutex> guard( FIFOLock );
    flog( Level::LVL_DEBUG, "DBG: RX packet of size %lu\n", bytes_transferred );

    /*------------------------------------------------
    Assuming we have space left, enqueue the data. This is
    mimicking the real hardware which has a fixed size RX FIFO.
    ------------------------------------------------*/
    if ( rxFIFO.size() < RF24::Physical::Shockburst::FIFO_QUEUE_MAX_SIZE )
    {
      rxFIFO.push( networkBuffer );
      networkBuffer.fill( RF24::Physical::Shockburst::INVALID_MEMORY );
    }
    else
    {
      flog( Level::LVL_INFO, "INF: Lost packet due to FIFO queue full\n" );
    }

    /*------------------------------------------------
    Handle the user's callback
    ------------------------------------------------*/
    if ( userCallback )
    {
      userCallback( this, networkBuffer );
    }

    rxEventProcessed = true;
  }

}    // namespace RF24::Physical::Pipe