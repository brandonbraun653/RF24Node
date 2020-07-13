/********************************************************************************
 *  File Name:
 *    rf24_endpoint_write.cpp
 *
 *  Description:
 *    Implements the algorithms needed to read/write data to other nodes via the
 *    radio link.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/endpoint>
#include <RF24Node/network>

namespace RF24::Endpoint
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t Device::write( const ::RF24::LogicalAddress dst, const void *const data, const size_t length )
  {
    /*------------------------------------------------
    Entrance conditions
    ------------------------------------------------*/
    if ( !data || !length || !isAddressValid( dst ) )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else if ( !isConnected( Connection::BindSite::PARENT ) && !isAddressRoot( mNetworkDriver->thisNode() ) )
    {
      return Chimera::CommonStatusCodes::NOT_FOUND;
    }

    /*------------------------------------------------
    Enforce packet limit. Currently fragmented packets
    aren't supported.
    ------------------------------------------------*/
    if ( length > Network::Frame::PAYLOAD_SIZE )
    {
      return Chimera::CommonStatusCodes::MEMORY;
    }

    /*------------------------------------------------
    Build the data frame and ship it
    ------------------------------------------------*/
    Network::Frame::FrameType frame;
    frame.clear();

    frame.setDst( dst );
    frame.setSrc( mNetworkDriver->thisNode() );
    frame.setPayload( data, length );
    frame.setType( Network::MSG_APP_DATA );

    if ( mNetworkDriver->write( frame, Network::ROUTE_NORMALLY ) )
    {
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
  }


  Chimera::Status_t Device::read( void *const data, const size_t length )
  {
    auto result = Chimera::CommonStatusCodes::FAIL;

    mNetworkDriver->lock();

    size_t readLength = 0;
    RF24::Network::Frame::Payload payload;
    RF24::Network::Frame::FrameType tmp;

    payload.fill( 0 );
    tmp.clear();

    if ( mNetworkDriver->available() && mNetworkDriver->read( tmp ) )
    {
      tmp.getPayload( payload );
      auto size    = tmp.getPayloadLength();

      readLength = ( size > length ) ? length : size;

      memcpy( data, payload.data(), readLength );
      result = Chimera::CommonStatusCodes::OK;
    }

    mNetworkDriver->unlock();

    return result;
  }


  bool Device::packetAvailable()
  {
    mNetworkDriver->lock();
    auto isAvailable =  mNetworkDriver->available();
    mNetworkDriver->unlock();

    return isAvailable;
  }


  size_t Device::nextPacketLength()
  {
    size_t length = 0;

    /*-------------------------------------------------
    Safely check the length of the next packet
    -------------------------------------------------*/
    mNetworkDriver->lock();

    RF24::Network::Frame::FrameType tmp;
    tmp.clear();

    if ( mNetworkDriver->available() && mNetworkDriver->peek( tmp ) )
    {
      length = tmp.getPayloadLength();
    }

    mNetworkDriver->unlock();
    return length;
  }

}