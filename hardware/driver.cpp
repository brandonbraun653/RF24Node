/********************************************************************************
*   File Name:
*     driver.cpp
*
*   Description:
*     NRF24L01(+) low level hardware driver implementation
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/hardware/driver.hpp>
#include <RF24Node/hardware/register.hpp>
#include <RF24Node/hardware/types.hpp>

namespace RF24::Hardware
{
  /*-------------------------------------------------
  Constructors/Destructors
  -------------------------------------------------*/
  Driver::Driver()
  {
    spi = nullptr;
    memset( &spiConfig, 0, sizeof( spiConfig ) );
  }

  Driver::~Driver()
  {

  }

  /*-------------------------------------------------
  SPI Acceptor Functions
  -------------------------------------------------*/
  Chimera::Status_t Driver::attachSPI( Chimera::SPI::SPIClass_sPtr &spi )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::attachSPI( Chimera::SPI::SPIClass_sPtr &spi, Chimera::SPI::DriverConfig &setup )
  {
    auto result = Chimera::CommonStatusCodes::OK;
    this->spi   = spi;
    memcpy( &spiConfig, &setup, sizeof( Chimera::SPI::DriverConfig ) );

    /*------------------------------------------------
    Ensure that the configuration options are ok
    ------------------------------------------------*/
    if ( spiConfig.HWInit.bitOrder != SPI_BIT_ORDER )
    {
      spiConfig.HWInit.bitOrder = SPI_BIT_ORDER;
    }

    if ( spiConfig.HWInit.clockFreq > SPI_MAX_CLOCK ) 
    {
      spiConfig.HWInit.clockFreq = SPI_MAX_CLOCK;
    }

    if ( spiConfig.HWInit.clockMode != SPI_CLOCK_MODE )
    {
      spiConfig.HWInit.clockMode = SPI_CLOCK_MODE;
    }

    /*------------------------------------------------
    Perform the final initialization steps
    ------------------------------------------------*/
    result |= this->spi->init( spiConfig );
    result |= this->spi->setChipSelectControlMode( Chimera::SPI::CSMode::MANUAL );

    return result;
  }

  Chimera::Status_t Driver::attachSPI( Chimera::SPI::SPIClass_uPtr spi )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*-------------------------------------------------
  Driver Functions
  -------------------------------------------------*/
  Chimera::Status_t Driver::initialize()
  {
    //Try and talk to the dadgum thing.

    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t resetDevice()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t selfTest( const bool rpd )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t toggleRXDataPipe( const size_t pipe, const bool state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}
