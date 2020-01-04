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
#include <RF24Node/network/frame/definitions.hpp>
#include <RF24Node/network/frame/types.hpp>

namespace RF24::Network::Frame
{
  /**
   *   Base frame structure for sending messages around
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

    void operator=( const Buffer &buffer );
    void operator=( const FrameType &frame );
    void operator=( const PackedData &rawFrame );

    bool operator==( const FrameType &rhs);

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
    Payload getPayload() const;
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

  private:
    uint16_t calculateCRC();

    bool mStaleData;
    PackedData mData;
  };
}    // namespace RF24::Network::Frame

#endif /* NRF24L01_NETWORK_LAYER_FRAME_HPP */
