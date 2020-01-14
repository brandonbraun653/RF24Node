/********************************************************************************
 *   File Name:
 *    types.hpp
 *
 *   Description:
 *    Types used in implementing the network driver for the NRF24 radios.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_PHYSICAL_LAYER_TYPES_HPP
#define NRF24L01_PHYSICAL_LAYER_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>
#include <string>

/* Chimera Includes */
#include <Chimera/types/spi_types.hpp>

/* RF24 Includes */
#include <RF24Node/hardware/types.hpp>

namespace RF24::Physical
{
  using RFChannel = uint8_t;

  struct Config
  {
    RFChannel rfChannel; /**< Radio's RF channel used for communication */

    Hardware::PowerAmplitude powerAmplitude;
    Hardware::DataRate dataRate;

#if !defined( RF24API )
    Chimera::SPI::DriverConfig spiConfig;
#endif

#if defined( RF24_SIMULATOR )
    std::string deviceName;
#endif
  };

}    // namespace RF24::Physical


#endif /* !NRF24L01_PHYSICAL_LAYER_TYPES_HPP */