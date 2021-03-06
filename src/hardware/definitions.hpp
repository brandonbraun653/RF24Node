/********************************************************************************
*   File Name:
*     definitions.hpp
*
*   Description:
*     NRF24L01(+) definitions
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef NRF24L01_HARDWARE_DEFINITIONS_HPP
#define NRF24L01_HARDWARE_DEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>
#include <limits>

/* Chimera Includes */
#include <Chimera/spi>

namespace RF24::Hardware
{
  /*----------------------------------------------
  General Definitions
  ----------------------------------------------*/
  static constexpr uint8_t MAX_NUM_PIPES    = 6;  /**< Hardware limit for number of pipes we can use */
  static constexpr size_t MAX_ADDRESS_WIDTH = 5;  /**< Hardware limit for how many bytes can represent a device's address */
  static constexpr size_t MAX_PAYLOAD_WIDTH = 32; /**< Hardware limit for RF max payload */
  static constexpr size_t MIN_PAYLOAD_WIDTH = 0;  /**< Hardware limit for RF min payload */
  static constexpr size_t COMMAND_WIDTH     = 1;  /**< Number of bytes for an SPI command */
  static constexpr size_t PACKET_WIDTH      = COMMAND_WIDTH + MAX_PAYLOAD_WIDTH;
  static constexpr size_t SPI_BUFFER_LEN  = PACKET_WIDTH; /**< Accounts for max payload of 32 bytes + 1 byte for the command */
  static constexpr size_t MIN_TIMEOUT_MS  = 1;            /**< The absolute lowest resolution timeout we want to achieve */
  static constexpr size_t DFLT_TIMEOUT_MS = 100;          /**< Default timeout for general operations */
  static constexpr size_t MAX_DELAY_AUTO_RETRY = 60; /**< Max number of milliseconds that a packet might spend in auto-retry */

  /*------------------------------------------------
  Hardware Configuration
  ------------------------------------------------*/
  static constexpr Chimera::SPI::BitOrder SPI_BIT_ORDER = Chimera::SPI::BitOrder::MSB_FIRST;
  static constexpr Chimera::SPI::ClockFreq SPI_MAX_CLOCK = 8000000;
  static constexpr Chimera::SPI::ClockMode SPI_CLOCK_MODE = Chimera::SPI::ClockMode::MODE0;
  
}

#endif  /* NRF24L01_HARDWARE_DEFINITIONS_HPP */