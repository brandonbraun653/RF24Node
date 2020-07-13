/********************************************************************************
 *  File Name:
 *    definitions.hpp
 *
 *  Description:
 *    Frame definitions
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_FRAME_DEFINITIONS_HPP
#define NRF24L01_FRAME_DEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <RF24Node/src/hardware/definitions.hpp>

namespace RF24::Network::Frame
{
  /*------------------------------------------------
  Frame data structure expected sizing/offsets. Useful for
  sanity checking the structures elsewhere.
  ------------------------------------------------*/
  constexpr size_t CRC_SIZE          = 2u;
  constexpr size_t CRC_OFFSET        = 0u;
  constexpr size_t HEADER_SIZE       = 5u;
  constexpr size_t HEADER_OFFSET     = CRC_SIZE;
  constexpr size_t HEADER_DST_SIZE   = 2u;
  constexpr size_t HEADER_DST_OFFSET = 0u;
  constexpr size_t HEADER_SRC_SIZE   = 2u;
  constexpr size_t HEADER_SRC_OFFSET = HEADER_DST_SIZE;
  constexpr size_t HEADER_MSG_SIZE   = 1u;
  constexpr size_t HEADER_MSG_OFFSET = HEADER_SRC_SIZE + HEADER_DST_SIZE;
  constexpr size_t MSG_LEN_SIZE      = 1;
  constexpr size_t MSG_LEN_OFFSET    = HEADER_OFFSET + HEADER_SIZE;
  constexpr size_t PAYLOAD_SIZE      = RF24::Hardware::MAX_PAYLOAD_WIDTH - ( CRC_SIZE + HEADER_SIZE + MSG_LEN_SIZE );
  constexpr size_t PAYLOAD_OFFSET    = MSG_LEN_OFFSET + MSG_LEN_SIZE;


  constexpr size_t size()
  {
    constexpr size_t FRAME_SIZE = CRC_SIZE + HEADER_SIZE + MSG_LEN_SIZE + PAYLOAD_SIZE;
    static_assert( FRAME_SIZE == RF24::Hardware::MAX_PAYLOAD_WIDTH, "Frame sizing is incorrect" );

    return FRAME_SIZE;
  }

  /*------------------------------------------------
  Size of a frame of data when there is no payload attached.
  Usually this is for frames that are used as a signaling mechanism.
  ------------------------------------------------*/
  constexpr size_t EMPTY_PAYLOAD_SIZE = size() - RF24::Network::Frame::PAYLOAD_SIZE;

}    // namespace RF24::Network::Frame

#endif /* NRF24L01_NETWORK_LAYER_DEFINITIONS_HPP */