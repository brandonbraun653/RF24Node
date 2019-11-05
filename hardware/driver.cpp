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
    spiConfig.externalCS = true;

    result |= this->spi->init( spiConfig );
    result |= this->spi->setChipSelectControlMode( Chimera::SPI::CSMode::MANUAL );

    return result;
  }

  Chimera::Status_t Driver::attachSPI( Chimera::SPI::SPIClass_uPtr spi )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::attachCS( Chimera::GPIO::PinInit &CSConfig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::attachCS( Chimera::GPIO::GPIOClass_sPtr &CSPin )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::attachCS( Chimera::GPIO::GPIOClass_uPtr CSPin )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*-------------------------------------------------
  Driver Functions
  -------------------------------------------------*/
  Chimera::Status_t Driver::initialize( const Chimera::GPIO::PinInit &CE, const Chimera::GPIO::PinInit &CS )
  {
    /*------------------------------------------------
    Input Protection
    ------------------------------------------------*/
    if ( !CE.validity || !CS.validity )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else if ( !spi )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Configure the GPIO pins. SPI should already be initialized.
    ------------------------------------------------*/
    auto result = Chimera::CommonStatusCodes::OK;

    CEPin = std::make_unique<Chimera::GPIO::GPIOClass>();
    result |= CEPin->init( CE );
    CEPin->setState( Chimera::GPIO::State::HIGH );

    CSPin = std::make_unique<Chimera::GPIO::GPIOClass>();
    result |= CSPin->init( CS );
    CSPin->setState( Chimera::GPIO::State::HIGH );

    return result;
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
