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
  
  enum PipeBitField_t : size_t
  {
    PIPE_BF_0 = ( 1u << 0 ),
    PIPE_BF_1 = ( 1u << 1 ),
    PIPE_BF_2 = ( 1u << 2 ),
    PIPE_BF_3 = ( 1u << 3 ),
    PIPE_BF_4 = ( 1u << 4 ),
    PIPE_BF_5 = ( 1u << 5 ),

    PIPE_BF_MASK = 0x3Fu
  };

  enum PipeNumber : size_t
  {
    PIPE_NUM_0   = 0u,
    PIPE_NUM_1   = 1u,
    PIPE_NUM_2   = 2u,
    PIPE_NUM_3   = 3u,
    PIPE_NUM_4   = 4u,
    PIPE_NUM_5   = 5u,
    PIPE_NUM_MAX = 6u,
    PIPE_NUM_ALL = PIPE_NUM_MAX,
    PIPE_INVALID = std::numeric_limits<size_t>::max()
  };


  /**
   *   Definitions for tracking the hardware state machine mode
   */
  enum Mode : size_t
  {
    MODE_POWER_DOWN = 0,
    MODE_STANDBY_I,
    MODE_STANDBY_II,
    MODE_RX,
    MODE_TX,

    MAX_MODES,
    UNKNOWN_MODE
  };

  /**
   *   Definitions for allowed TX power levels
   */
  enum PowerAmplitude : size_t
  {
    PA_MIN  = 0u, /**< -18 dBm */
    PA_LOW  = 2u, /**< -12 dBm */
    PA_HIGH = 4u, /**<  -6 dBm */
    PA_MAX  = 6u  /**<   0 dBm */
  };

  /**
   *   Definitions for allowed data rates
   */
  enum DataRate : size_t
  {
    DR_1MBPS,  /**< 1 MBPS */
    DR_2MBPS,  /**< 2 MBPS */
    DR_250KBPS /**< 250 KBPS */
  };

  /**
   *   Definitions for CRC settings
   */
  enum CRCLength : size_t
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
  enum AddressWidth : size_t
  {
    AW_3Byte = 0x01,
    AW_4Byte = 0x02,
    AW_5Byte = 0x03
  };

  /**
   *   Definitions for the auto retransmit delay register field
   */
  enum AutoRetransmitDelay : size_t
  {
    ART_DELAY_250uS  = 0,
    ART_DELAY_500uS  = 1,
    ART_DELAY_750uS  = 2,
    ART_DELAY_1000uS = 3,
    ART_DELAY_1250uS = 4,
    ART_DELAY_1500uS = 5,
    ART_DELAY_1750uS = 6,
    ART_DELAY_2000uS = 7,
    ART_DELAY_2250uS = 8,
    ART_DELAY_2500uS = 9,
    ART_DELAY_2750uS = 10,
    ART_DELAY_3000uS = 11,
    ART_DELAY_3250uS = 12,
    ART_DELAY_3500uS = 13,
    ART_DELAY_3750uS = 14,
    ART_DELAY_4000uS = 15,

    ART_DELAY_MIN = ART_DELAY_250uS,
    ART_DELAY_MED = ART_DELAY_2250uS,
    ART_DELAY_MAX = ART_DELAY_4000uS
  };

  
} // namespace RF24::Hardware

#endif  /* NRF24L01_HARDWARE_TYPES_HPP */