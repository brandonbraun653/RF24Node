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
#ifndef NRF24L01_NETWORK_LAYER_FRAME_TYPES_HPP
#define NRF24L01_NETWORK_LAYER_FRAME_TYPES_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Boost Includes */
#include <boost/circular_buffer_fwd.hpp>
#include <boost/circular_buffer.hpp>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/network/frame/frame_definitions.hpp>
#include <RF24Node/network/header/header_types.hpp>

namespace RF24::Network
{
  /**
   *   Defines enough memory to store a full frame of data from the NRF24 radio. The size
   *   of this array is limited by hardware and should not be changed.
   */
  using FrameBuffer_t = std::array<uint8_t, FRAME_TOTAL_SIZE>;
  static_assert( sizeof( FrameBuffer_t ) == FRAME_TOTAL_SIZE, "Incorrect frame size" );

  /**
   *   Defines enough memory to store the length of the user payload of a frame
   */
  using FrameLength_t = uint16_t;
  static_assert( sizeof( FrameLength_t ) == FRAME_MSG_LEN_SIZE, "Icorrect frame length size" );

  /**
   *   Defines enough memory to store the user payload of a frame
   */
  using FramePayload_t = std::array<uint8_t, MAX_FRAME_PAYLOAD_SIZE>;
  static_assert( sizeof( FramePayload_t ) == MAX_FRAME_PAYLOAD_SIZE, "Incorrect frame payload size" );

#pragma pack( push )
#pragma pack( 1 )
  struct Frame_t
  {
    Header_t header;
    FrameLength_t messageLength;
    FramePayload_t message;
  };
#pragma pack( pop )
  static_assert( sizeof( Frame_t ) == FRAME_TOTAL_SIZE, "Frame data structure is the wrong size" );

  /**
   *   Defines enough memory for storing multiple frames of data. This is intended
   *   to serve as the primary user accessible cache for incoming data.
   */
  using FrameCache_t = boost::circular_buffer<Frame_t>;

}    // namespace RF24::Network

#endif /* NRF24L01_NETWORK_LAYER_FRAME_TYPES_HPP */