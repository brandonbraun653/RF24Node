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
#ifndef NRF24L01_NETWORK_LAYER_FRAME_DEFINITIONS_HPP
#define NRF24L01_NETWORK_LAYER_FRAME_DEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/network/header/header_types.hpp>

namespace RF24::Network
{
  /**
   *   Max frame size, limited by the hardware TX/RX FIFO
   */
  constexpr uint8_t FRAME_TOTAL_SIZE = RF24::Hardware::MAX_PAYLOAD_WIDTH;

  /**
   *   Number of bytes contained within our Network Layer header. This is
   *   a sub-component of our Frame preamble.
   */
  constexpr uint8_t FRAME_HEADER_SIZE = HEADER_SIZE;

  /**
   *   How many bytes into FrameBuffer_t is the header data
   */
  constexpr uint8_t FRAME_HEADER_OFFSET = 0;

  /**
   *   Number of bytes used to indicate the total length of the message. Since
   *   messages could be fragmented, it must account for a much larger number than
   *   what a single byte could represent.
   */
  constexpr uint8_t FRAME_MSG_LEN_SIZE = 2;

  /**
   *   How many bytes into FrameBuffer_t is the message length data
   */
  constexpr uint8_t FRAME_MSG_LEN_OFFSET = FRAME_HEADER_SIZE;

  /**
   *   Number of bytes contained in the Frame preamble, which consists of a header
   *   and the message length information.
   */
  constexpr uint8_t FRAME_PREAMBLE_SIZE = FRAME_HEADER_SIZE + FRAME_MSG_LEN_SIZE;

  /**
   *   How many bytes into FrameBuffer_t is the message data
   */
  constexpr uint8_t FRAME_MESSAGE_OFFSET = FRAME_PREAMBLE_SIZE;

  /**
   *   The total amount (in bytes) of user defined data that can be attached to
   *   a single frame after the preamble data is accounted for.
   */
  constexpr uint8_t MAX_FRAME_PAYLOAD_SIZE = FRAME_TOTAL_SIZE - FRAME_PREAMBLE_SIZE;

}    // namespace RF24::Network

#endif /* NRF24L01_NETWORK_LAYER_FRAME_DEFINITIONS_HPP */