/********************************************************************************
 *   File Name:
 *    frame.cpp
 *
 *   Description:
 *    Implements the frame interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>
#include <cstddef>

/* CRC Includes */
#include "CRC.h"

/* RF24 Includes */
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/network/frame/definitions.hpp>
#include <RF24Node/src/network/frame/types.hpp>

namespace RF24::Network::Frame
{
  FrameType::FrameType() : mStaleData( true )
  {
    memset( &mData, 0, sizeof( PackedData ) );
  }

  FrameType::FrameType( const FrameType &frame ) : mStaleData( true )
  {
    memcpy( &mData, &frame.mData, sizeof( PackedData ) );
  }

  FrameType::FrameType( const Buffer &buffer ) : mStaleData( true )
  {
    memcpy( &mData, buffer.data(), sizeof( PackedData ) );
  }

  FrameType::FrameType( const PackedData &rawFrame ) : mStaleData( true )
  {
    memcpy( &mData, &rawFrame, sizeof( PackedData ) );
  }

  FrameType::~FrameType(){};

  void FrameType::operator=( const Buffer &buffer )
  {
    mStaleData = true;
    memcpy( &mData, buffer.data(), sizeof( PackedData ) );
  }

  void FrameType::operator=( const FrameType &frame )
  {
    mStaleData = true;
    memcpy( &mData, &frame.mData, sizeof( PackedData ) );
  }

  void FrameType::operator=( const PackedData &rawFrame )
  {
    mStaleData = true;
    memcpy( &mData, &rawFrame, sizeof( PackedData ) );
  }

  bool FrameType::operator==( const FrameType &rhs )
  {
    return ( this->getCRC() == rhs.getCRC() );
  }

  void FrameType::clear()
  {
    mStaleData = true;
    memset( &mData, 0, sizeof( mData ) );
  }

  bool FrameType::valid()
  {
    return ( ( mData.crc - calculateCRC() ) == 0 );
  }

  /*------------------------------------------------
  Data Getters
  ------------------------------------------------*/
  CRC16_t FrameType::getCRC() const
  {
    CRC16_t temp;
    memcpy( &temp, &mData + CRC_OFFSET, sizeof( PackedData::crc ) );
    return temp;
  }

  Header FrameType::getHeader() const
  {
    Header temp;
    memcpy( &temp, &mData + HEADER_OFFSET, sizeof( PackedData::header ) );
    return temp;
  }

  Length FrameType::getLength() const
  {
    static_assert( sizeof( PackedData::length ) == sizeof( Length ), "Invalid length size" );
    Length temp;
    memcpy( &temp, &mData.length, sizeof( Length ) );
    return temp;
  }

  Length FrameType::getPayloadLength() const
  {
    return getLength() - static_cast<Length>( EMPTY_PAYLOAD_SIZE );
  }

  Payload FrameType::getPayload() const
  {
    Payload temp;
    memcpy( &temp, &mData + PAYLOAD_OFFSET, sizeof( PackedData::payload ) );
    return temp;
  }

  RF24::LogicalAddress FrameType::getDst() const
  {
    RF24::LogicalAddress temp;
    memcpy( &temp, &mData.header.dstNode, sizeof( PackedHeader::dstNode ) );
    return temp;
  }

  RF24::LogicalAddress FrameType::getSrc() const
  {
    RF24::LogicalAddress temp;
    memcpy( &temp, &mData.header.srcNode, sizeof( PackedHeader::srcNode ) );
    return temp;
  }

  RF24::Network::HeaderMessage FrameType::getType() const
  {
    RF24::Network::HeaderMessage temp;
    memcpy( &temp, &mData.header.type, sizeof( PackedHeader::type ) );
    return temp;
  }

  /*------------------------------------------------
  Data Setters
  ------------------------------------------------*/
  void FrameType::updateCRC()
  {
    if ( mStaleData )
    {
      mData.crc  = calculateCRC();
      mStaleData = false;
    }
  }

  void FrameType::setDst( const RF24::LogicalAddress dst )
  {
    memcpy( &mData.header.dstNode, &dst, sizeof( RF24::LogicalAddress ) );
  }

  void FrameType::setSrc( const RF24::LogicalAddress src )
  {
    memcpy( &mData.header.srcNode, &src, sizeof( RF24::LogicalAddress ) );
  }

  void FrameType::setType( const RF24::Network::HeaderMessage type )
  {
    memcpy( &mData.header.type, &type, sizeof( RF24::Network::HeaderMessage ) );
  }

  void FrameType::setLength( const Length len )
  {
    memcpy( &mData.length, &len, sizeof( Length ) );
  }

  /*------------------------------------------------
  Private Functions
  ------------------------------------------------*/
  uint16_t FrameType::calculateCRC()
  {
    static_assert( sizeof( PackedData::crc ) == sizeof( uint16_t ), "Invalid CRC field length" );
    return CRC::Calculate( &mData.header, ( sizeof( PackedData ) - sizeof( PackedData::crc ) ), CRC::CRC_16_ARC() );
  }

  Buffer FrameType::toBuffer() const
  {
    Buffer tmp;
    memcpy( tmp.data(), &mData, sizeof( PackedData ) );
    return tmp;
  }

  const RF24::Network::Frame::PackedData *const FrameType::getPackedData()
  {
    return &mData;
  }

  ::RF24::Network::HeaderMessage getHeaderTypeFromBuffer( const Buffer &buffer )
  {
    using namespace ::RF24::Network;

    HeaderMessage msg         = MSG_NO_MESSAGE;
    constexpr auto hdrOffset  = offsetof( PackedData, header );
    constexpr auto typeOffset = offsetof( PackedHeader, type );
    auto readPtr              = buffer.data() + hdrOffset + typeOffset;

    memcpy( &msg, readPtr, sizeof( msg ) );
    return msg;
  }

  Length getFrameLengthFromBuffer( const Buffer &buffer )
  {
    Length len               = 0;
    constexpr auto lenOffset = offsetof( PackedData, length );
    auto readPtr             = buffer.data() + lenOffset;

    memcpy( &len, readPtr, sizeof( len ) );
    return len;
  }

  CRC16_t getCRCFromBuffer( const Buffer &buffer )
  {
    CRC16_t temp = 0;
    auto readPtr = buffer.data() + offsetof( PackedData, crc );

    memcpy( &temp, readPtr, sizeof( temp ) );
    return temp;
  }

  CRC16_t calculateCRCFromBuffer( const Buffer &buffer )
  {
    constexpr auto crcLen = sizeof( PackedData ) - sizeof( PackedData::crc );
    auto readPtr          = buffer.data() + offsetof( PackedData, header );

    return CRC::Calculate( readPtr, crcLen, CRC::CRC_16_ARC() );
  }

  RF24::LogicalAddress getDestinationFromBuffer( const Buffer &buffer )
  {
    RF24::LogicalAddress temp = 0;
    auto readPtr              = buffer.data() + offsetof( PackedData, header ) + offsetof( PackedHeader, dstNode );
    
    memcpy( &temp, readPtr, sizeof( temp ) );
    return temp;
  }

  RF24::LogicalAddress getSourceFromBuffer( const Buffer &buffer )
  {
    RF24::LogicalAddress temp = 0;
    auto readPtr              = buffer.data() + offsetof( PackedData, header ) + offsetof( PackedHeader, srcNode );

    memcpy( &temp, readPtr, sizeof( temp ) );
    return temp;
  }

}    // namespace RF24::Network::Frame
