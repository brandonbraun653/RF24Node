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
  using RFChannel = uint8_t;

  struct Config
  {
    RFChannel rfChannel; /**< Radio's RF channel used for communication */

    Hardware::PowerAmplitude powerAmplitude;
    Hardware::DataRate dataRate;

    Chimera::SPI::DriverConfig spiConfig;
    Chimera::GPIO::PinInit chipEnableConfig;
    std::string deviceName;
  };

}    // namespace RF24::Physical


#endif /* !NRF24L01_PHYSICAL_LAYER_TYPES_HPP */