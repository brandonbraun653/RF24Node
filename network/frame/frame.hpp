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
#include <RF24Node/network/frame/frame_definitions.hpp>
#include <RF24Node/network/frame/frame_types.hpp>
#include <RF24Node/network/header/header.hpp>

namespace RF24::Network
{
  /**
   *   Frame structure for internal message handling, and for use by external applications
   *
   *   The actual frame put over the air consists of a header (8-bytes) and a message payload
   *   (Up to 24-bytes). When data is received, it is stored using the Frame structure,
   *   which includes:
   *       1. The header
   *       2. The size of the included message
   *       3. The data being received
   */
  class FrameHelper
  {
  public:
    
    constexpr size_t size()
    {
      return mBuffer.size();
    }
    
    /**
     *   Constructor to build a Frame from discrete parts
     */
    FrameHelper( const FrameHeaderField &header, const FrameLengthField &msgLen, const void *const message );

    /**
     *   Constructor to build a Frame from a frame buffer
     *
     *   @param[in]  buffer      The new frame data
     *   @return void
     */
    FrameHelper( const FrameBuffer &buffer );

    FrameHelper();
    ~FrameHelper();

    /**
     *   Allows updating a Frame object from a new set of data
     *
     *   @param[in]  buffer      The new frame data
     *   @return void
     */
    void operator()( const FrameBuffer &buffer );

    /**
     *  Empties the frame and fills it with zeros
     *  
     *  @return void
     */
    void clear();

    void build( const FrameHeaderField &header, const FrameLengthField &msgLen, const void *const message );

    void updateCRC();

    bool validateCRC();

    uint8_t *getBuffer();

    void commitBuffer();

    HeaderHelper getHeader();

    FrameLengthField getPayloadLength();

    FramePayloadField getPayload();

  private:
    uint16_t calculateCRC();

    bool staleData;
    FrameBuffer mBuffer;
    FrameData data;
  };
}    // namespace RF24::Network

#endif  /* NRF24L01_NETWORK_LAYER_FRAME_HPP */
