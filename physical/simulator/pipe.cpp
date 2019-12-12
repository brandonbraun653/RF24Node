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


using namespace boost::asio::ip;

namespace RF24::Physical::Sim
{
  TXPipe::TXPipe( boost::asio::io_service &io_service ) : txSocket( io_service )
  {
  
  }

  TXPipe::~TXPipe()
  {
  
  }





  RXPipe::RXPipe( boost::asio::io_service &io_service ) : ioService( io_service ), rxSocket( io_service )
  {
    allowListening = false;
    killFlag = false;
    threadExecuting = false;

    packetBufferLock.unlock();
    networkBuffer.fill( 0 );
  }

  RXPipe::~RXPipe()
  {
    stopListening();
    kill();
  }

  void RXPipe::configure( const std::string &ip, const size_t port )
  {
    boost::asio::ip::address x;

    x.from_string( ip );


    acceptor = std::make_unique<tcp::acceptor>( ioService, tcp::endpoint( x, port ) );
    
    acceptor->async_accept( rxSocket, boost::bind( &RXPipe::__PrvOnAccept, this, boost::asio::placeholders::error ) );
    
  }

  

  void RXPipe::startListening()
  {
    allowListening = true;
  }

  void RXPipe::stopListening()
  {
    allowListening = false;
  }

  void RXPipe::__PrvUpdateThread()
  {
    using namespace RF24::Physical::Sim;

    try
    {
      threadExecuting = true;

      while ( !killFlag )
      {
        if ( allowListening ) 
        {
          /*------------------------------------------------
          Block until we read our packet delimiter
          ------------------------------------------------*/
          boost::asio::streambuf streamBuffer;
          boost::asio::read_until( socket, streamBuffer, RF24::Physical::Sim::SBEndSequence );

          /*------------------------------------------------
          Pull out the raw data and construct the packet 
          ------------------------------------------------*/
          SBArray temp;
          std::istream stream(&streamBuffer);
          stream.read(temp.data(), streamBuffer.size());
          streamBuffer.consume( streamBuffer.size() );

          ShockBurstPacket packet;
          packet.assemble( temp );

          /*------------------------------------------------
          Assuming we have space left, enqueue the data. This is 
          mimicking the real hardware which has a fixed size RX FIFO.
          If the FIFO is full on hardware, data is simply lost.
          ------------------------------------------------*/
          if ( rxFIFO.size() < RF24::Physical::Sim::SB_FIFO_QUEUE_MAX_SIZE )
          {
            rxFIFO.push(packet);
          }
          else
          {
            uLog::flog( uLog::Level::LVL_INFO, "Lost packet due to FIFO queue full\r\n" );
          }
        }

        boost::this_thread::sleep_for( boost::chrono::milliseconds( 25 ) );
      }
    }
    catch ( boost::thread_interrupted & )
    {
    }

    threadExecuting = false;
  }

  void RXPipe::__PrvOnAccept( const boost::system::error_code &error )
  {
    
    // If already listening, do nothing

    /*------------------------------------------------
    If the thread doesn't exist, start it up
    ------------------------------------------------*/
    if ( !threadExecuting )
    {
      killFlag = false;
      allowListening = false;

      rxThread = boost::thread( boost::bind( &RXPipe::__PrvUpdateThread, this ) );
    }
  }

  void RXPipe::kill()
  {
    killFlag = false;
    rxThread.join();
  }

  bool RXPipe::available()
  {
    return !rxFIFO.empty();
  }

  size_t RXPipe::read( void *const data, const size_t length )
  {
    ShockBurstPacket packet = rxFIFO.front();

    if ( available() || data || ( length > packet.payloadSize() ) )
    {
      memcpy( data, packet.payload(), packet.payloadSize() );
      rxFIFO.pop();
      return packet.payloadSize();
    }

    return 0;
  }

}