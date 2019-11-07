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
  class Frame
  {
  public:
    Frame_t data;

    /**
     *   Constructor to build a Frame from discrete parts
     */
    Frame( const Header_t &header, const FrameLength_t &msgLen, const void *const message );

    /**
     *   Constructor to build a Frame from a frame buffer
     *
     *   @param[in]  buffer      The new frame data
     *   @return void
     */
    Frame( const FrameBuffer_t &buffer );

    Frame();
    ~Frame();

    /**
     *   Allows updating a Frame object from a new set of data
     *
     *   @param[in]  buffer      The new frame data
     *   @return void
     */
    void operator()( const FrameBuffer_t &buffer );
  };
}    // namespace RF24::Network

#endif  /* NRF24L01_NETWORK_LAYER_FRAME_HPP */
