/********************************************************************************
 *  File Name:
 *    types.hpp
 *
 *  Description:
 *    Types used in implementing the network driver for the NRF24 radios.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_PHYSICAL_LAYER_TYPES_HPP
#define NRF24L01_PHYSICAL_LAYER_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>
#include <string>

/* Chimera Includes */
#include <Chimera/gpio>
#include <Chimera/spi>

/* RF24 Includes */
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/hardware/types.hpp>

namespace RF24::Physical
{
  /*-------------------------------------------------------------------------------
  Typedefs
  -------------------------------------------------------------------------------*/
  using RFChannel = uint8_t;

  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum StatusFlag : uint32_t
  {
    SF_RX_DATA_READY = ( 1u << 0),
    SF_TX_DATA_SENT  = ( 1u << 1 ),
    SF_TX_MAX_RETRY  = ( 1u << 2 ),
    SF_TX_FIFO_FULL  = ( 1u << 3 ),
    SF_TX_FIFO_EMPTY = ( 1u << 4 ),
    SF_RX_FIFO_FULL  = ( 1u << 5 ),
    SF_RX_FIFO_EMPTY = ( 1u << 6 )

  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct Config
  {
    RFChannel rfChannel; /**< Radio's RF channel used for communication */

    Hardware::PowerAmplitude powerAmplitude;
    Hardware::DataRate dataRate;

    Chimera::SPI::DriverConfig spiConfig;
    Chimera::GPIO::PinInit chipEnableConfig;
    std::string deviceName;

    void clear()
    {
      rfChannel      = 0;
      powerAmplitude = RF24::Hardware::PA_MIN;
      dataRate       = RF24::Hardware::DR_1MBPS;
      deviceName     = {};
      spiConfig.clear();
      chipEnableConfig.clear();
    }
  };


  /**
   * Status information about the radio
   */
  struct Status
  {
    bool validity;
    uint32_t flags;

    uint8_t lostPacketCount;  /**< Reg OBSERVE_TX: PLOS_CNT */
    uint8_t retransmitCount;  /**< Reg OBSERVE_TX: ARC_CNT */
  };

}    // namespace RF24::Physical


#endif /* !NRF24L01_PHYSICAL_LAYER_TYPES_HPP */