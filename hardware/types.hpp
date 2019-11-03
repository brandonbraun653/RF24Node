/********************************************************************************
*   File Name:
*     types.hpp
*
*   Description:
*     NRF24L01(+) hardware layer types
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef NRF24L01_HARDWARE_TYPES_HPP
#define NRF24L01_HARDWARE_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

namespace RF24::Hardware
{
  

  /**
   *   Definitions for tracking the hardware state machine mode
   */
  enum class Mode : uint8_t
  {
    POWER_DOWN = 0,
    STANDBY_I,
    STANDBY_II,
    RX,
    TX,

    MAX_MODES,
    UNKNOWN_MODE
  };

  /**
   *   Definitions for allowed TX power levels
   */
  enum class PowerAmplitude : uint8_t
  {
    MIN  = 0u, /**< -18 dBm */
    LOW  = 2u, /**< -12 dBm */
    HIGH = 4u, /**<  -6 dBm */
    MAX  = 6u  /**<   0 dBm */
  };

  /**
   *   Definitions for allowed data rates
   */
  enum class DataRate : uint8_t
  {
    DR_1MBPS,  /**< 1 MBPS */
    DR_2MBPS,  /**< 2 MBPS */
    DR_250KBPS /**< 250 KBPS */
  };

  /**
   *   Definitions for CRC settings
   */
  enum class CRCLength : uint8_t
  {
    CRC_DISABLED, /**< No CRC */
    CRC_8,        /**< 8 Bit CRC */
    CRC_16        /**< 16 Bit CRC */
  };

  /**
   *   Definitions for how many address bytes to use. The
   *   numerical value here is NOT the number of bytes. This is the
   *   register level definition.
   */
  enum class AddressWidth : uint8_t
  {
    AW_3Byte = 0x01,
    AW_4Byte = 0x02,
    AW_5Byte = 0x03
  };

  /**
   *   Definitions for the auto retransmit delay register field
   */
  enum class AutoRetransmitDelay : uint8_t
  {
    w250uS  = 0,
    w500uS  = 1,
    w750uS  = 2,
    w1000uS = 3,
    w1250uS = 4,
    w1500uS = 5,
    w1750uS = 6,
    w2000uS = 7,
    w2250uS = 8,
    w2500uS = 9,
    w2750uS = 10,
    w3000uS = 11,
    w3250uS = 12,
    w3500uS = 13,
    w3750uS = 14,
    w4000uS = 15,

    MIN = w250uS,
    MED = w2250uS,
    MAX = w4000uS
  };

  /**
   *   Provide reasons for why something has failed.
   */
  enum class FailureCode : uint8_t
  {
    NO_FAILURE = 0,
    CLEARED    = NO_FAILURE,
    MAX_RETRY_TIMEOUT,
    TX_FIFO_FULL_TIMEOUT,
    TX_FIFO_EMPTY_TIMEOUT,
    RADIO_IN_TX_MODE,
    RADIO_IN_RX_MODE,
    INVALID_PIPE,
    NOT_CONNECTED,
    REGISTER_WRITE_FAILURE,
    COULD_NOT_ERASE,
  };

  
} // namespace RF24::Hardware

#endif  /* NRF24L01_HARDWARE_TYPES_HPP */