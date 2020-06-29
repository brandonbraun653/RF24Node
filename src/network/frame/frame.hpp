/********************************************************************************
 *   File Name:
 *
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_FRAME_HPP
#define NRF24L01_NETWORK_LAYER_FRAME_HPP

/* Driver Includes */
#include <RF24Node/src/network/frame/definitions.hpp>
#include <RF24Node/src/network/frame/types.hpp>

namespace RF24::Network::Frame
{
  /**
   *  Copies out the header message type found in the frame buffer
   *
   *  @param[in]  buffer      The raw frame buffer received from the hardware
   *  @return HeaderMessage
   */
  ::RF24::Network::HeaderMessage getHeaderTypeFromBuffer( const Buffer &buffer );

  /**
   *  Returns the destination node address from the buffer
   *
   *	@param[in]	buffer      The raw frame buffer received from the hardware
   *	@return RF24::LogicalAddress
   */
  LogicalAddress getDestinationFromBuffer( const Buffer &buffer );

  /**
   *  Returns the source node address from the buffer
   *
   *	@param[in]	buffer      The raw frame buffer received from the hardware
   *	@return RF24::LogicalAddress
   */
  LogicalAddress getSourceFromBuffer( const Buffer &buffer );

  /**
   *  Returns the actual length of the frame stored in the buffer.
   *
   *  Due to dynamic payloads, the total length of the frame may not be the full
   *  supported 32 bytes. Some pieces of data, like the header, are fixed in sized
   *  and will always be present, but the payload can vary widely.
   *
   *  @param[in]  buffer      The raw frame buffer received from the hardware
   *  @return Length
   */
  Length getFrameLengthFromBuffer( const Buffer &buffer );

  /**
   *	Returns the CRC field of the buffer (does not calculate it)
   *
   *	@param[in]	buffer      The raw frame buffer received from the hardware
   *	@return RF24::Network::Frame::CRC16_t
   */
  CRC16_t getCRCFromBuffer( const Buffer &buffer );

  /**
   *	Returns the current calculated CRC of the buffer. Note that this is not the
   *  same as reading the CRC field from this buffer.
   *
   *	@param[in]	buffer      The raw frame buffer received from the hardware
   *	@return RF24::Network::Frame::CRC16_t
   */
  CRC16_t calculateCRCFromBuffer( const Buffer &buffer );


  /**
   *   Base frame structure for sending messages around.
   *
   *
   *                   FRAME STRUCTURE
   *  |-CRC-|------Header------|-Size-|-------Payload------|
   *  |     | dst | src | type |      |      0-24 bytes    |
   *
   */
  class FrameType
  {
  public:
    FrameType();
    ~FrameType();

    FrameType( const Buffer &buffer );
    FrameType( const FrameType &frame );
    FrameType( const PackedData &rawFrame );

    void operator=( const FrameType &&frame ) noexcept;
    void operator=( const Buffer &buffer );
    void operator=( const FrameType &frame );
    void operator=( const PackedData &rawFrame );

    bool operator==( const FrameType &rhs );

    /**
     *  Empties the frame and fills it with zeros
     *  @return void
     */
    void clear();

    /**
     *  Calculates the CRC of the current data and compares it to the
     *  actual CRC given in the Frame. If the two match, the data is
     *  assumed to be valid.
     *
     *  @return bool
     */
    bool valid();

    /*------------------------------------------------
    Data Getters
    ------------------------------------------------*/
    CRC16_t getCRC() const;
    Header getHeader() const;
    void getPayload( Payload &pl ) const;
    const uint8_t * const peekPayload();
    Length getPayloadLength() const;
    RF24::LogicalAddress getDst() const;
    RF24::LogicalAddress getSrc() const;
    RF24::Network::HeaderMessage getType() const;

    Buffer toBuffer() const;
    const PackedData *const getPackedData();

    /*------------------------------------------------
    Data Setters
    ------------------------------------------------*/
    void updateCRC();
    void setDst( const RF24::LogicalAddress dst );
    void setSrc( const RF24::LogicalAddress src );
    void setType( const RF24::Network::HeaderMessage type );
    void setPayload( const void *const data, const Length length );

  private:
    uint16_t calculateCRC();

    bool mStaleCRC;
    PackedData mData;
  };
}    // namespace RF24::Network::Frame

#endif /* NRF24L01_NETWORK_LAYER_FRAME_HPP */
