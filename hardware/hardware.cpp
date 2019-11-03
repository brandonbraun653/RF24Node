/********************************************************************************
*   File Name:
*     hardware.cpp
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
#include <RF24Node/hardware/hardware.hpp>
#include <RF24Node/hardware/register.hpp>
#include <RF24Node/hardware/types.hpp>

namespace RF24::Hardware
{
  /*-------------------------------------------------
  Constructors/Destructors
  -------------------------------------------------*/
  Driver::Driver()
  {

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
    this->spi = spi;
    memcpy( &spiConfig, &setup, sizeof( Chimera::SPI::DriverConfig ) );
    return Chimera::CommonStatusCodes::OK;
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
