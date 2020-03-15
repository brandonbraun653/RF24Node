/********************************************************************************
 *  File Name:
 *    shockburst.cpp
 *
 *  Description:
 *    Implements a software version of the RF transmissions in the NRF24. This is 
 *    directly modeling the hardware transceiver and its associated packet logic.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#if defined( RF24_SIMULATOR )

/* C++ Includes */
#include <mutex>

/* CRC Includes */
#include "CRC.h"

/* RF24 Includes */
#include <RF24Node/src/common/conversion.hpp>
#include <RF24Node/src/physical/simulator/shockburst.hpp>


namespace RF24::Physical::Shockburst
{
  Socket::Socket( const RF24::Hardware::PipeNumber pipe, boost::asio::io_service &io_service, const std::string name ) :
      rxPipe( io_service, name, static_cast<size_t>( pipe ) ), txPipe( io_service, name, static_cast<size_t>( pipe ) ),
      pipeID( pipe )
  {
    mHasUserTxPipe      = false;
    mUseDynamicPayloads = false;
    mUseAutoAcknowledge = false;
    mRetryLimit         = 0;
    mRetryDelay         = 0;
    cachedTXAddress     = 0;

    /*------------------------------------------------
    On the NRF24L01, the only pipe that lets the user
    transmit data is the first pipe.
    ------------------------------------------------*/
    if ( pipeID == Hardware::PIPE_NUM_0 )
    {
      mHasUserTxPipe = true;
    }
  }

  Socket::~Socket()
  {
  }

  Chimera::Status_t Socket::attachLogger( uLog::SinkHandle sink )
  {
    logger = sink;
    txPipe.attachLogger( logger );
    rxPipe.attachLogger( logger );
    return Chimera::CommonStatusCodes::OK;
  }

  void Socket::setRetryLimit( const size_t limit )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    mRetryLimit = limit;
  }

  void Socket::setRetryDelay( const size_t delay )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    mRetryDelay = delay;
  }

  void Socket::startListening()
  {
    rxPipe.flush();
    rxPipe.startListening();
  }

  void Socket::stopListening()
  {
    rxPipe.stopListening();
  }
  
  bool Socket::isListening()
  {
    return rxPipe.isListening();
  }

  bool Socket::openWritePipe( const RF24::PhysicalAddress address )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    if ( !address || !mHasUserTxPipe )
    {
      return false;
    }
    else if ( txPipe.isOpen() )
    {
      /* Update the address cache so future calls to write() are correct */
      cachedTXAddress = address;
      return true;
    }
    else
    {
      txPipe.openPipe();

      /* Update the address cache so future calls to write() are correct */
      cachedTXAddress = address;
      return true;
    }
  }

  bool Socket::closeWritePipe()
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    if ( !mHasUserTxPipe )
    {
      return false;
    }

    txPipe.closePipe();
    cachedTXAddress = 0;
    return true;
  }

  bool Socket::openReadPipe( const RF24::PhysicalAddress address )
  {
    closeWritePipe();
    rxPipe.openPipe( Conversion::decodeIP( address ), Conversion::decodePort( address ) );
    return true;
  }

  bool Socket::closeReadPipe()
  {
    rxPipe.closePipe();
    return true;
  }

  void Socket::flushTX()
  {
    txPipe.flush();
  }

  void Socket::flushRX()
  {
    rxPipe.flush();
  }

  void Socket::toggleDynamicPayloads( const bool state )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    mUseDynamicPayloads = state;
    // Maybe set bit fields in a control variable instead
  }

  void Socket::toggleAutoAck( const bool state )
  {
    std::lock_guard<std::recursive_mutex> guard( mSettingsLock );
    mUseAutoAcknowledge = state;
    // Maybe set bit fields in a control variable instead
  }

  void Socket::write( const RF24::Network::Frame::Buffer &buffer )
  {
    /*------------------------------------------------
    Fill out the packet fields
    ------------------------------------------------*/
    Packet pkt;
    pkt.data.address = cachedTXAddress;
    pkt.data.control = 0;
    memcpy( &pkt.data.payload, buffer.data(), buffer.size() );

    pkt.crc = CRC::Calculate( &pkt.data, sizeof( _Data ), CRC::CRC_32() );

    /*------------------------------------------------
    Copy the packet data into a raw buffer and transmit it
    ------------------------------------------------*/
    PacketBuffer tmp;
    tmp.fill(0);
    memcpy( tmp.data(), &pkt, tmp.size() );
    txPipe.write( tmp );
  }

  bool Socket::read( RF24::Network::Frame::Buffer &buffer )
  {
    if ( !rxPipe.available() )
    {
      return false;
    }

    /*------------------------------------------------
    Pull out the packet from the buffer
    ------------------------------------------------*/
    Packet pkt;
    PacketBuffer temp = rxPipe.read();
    memcpy( &pkt, temp.data(), temp.size() );

    /*------------------------------------------------
    Assuming the CRC is good, copy the full packet to the user
    ------------------------------------------------*/
    auto pktCRC = CRC::Calculate( &pkt.data, sizeof( _Data ), CRC::CRC_32() );
    if ( pktCRC == pkt.crc )
    {
      memcpy( buffer.data(), pkt.data.payload, buffer.size() );
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Socket::available()
  {
    return rxPipe.available();
  }

}    // namespace RF24::Physical::Shockburst

#endif /* _WIN32 || _WIN64 */
