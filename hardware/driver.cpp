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
#include <Chimera/types/event_types.hpp>

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
    this->spi = spi;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::attachSPI( Chimera::SPI::SPIClass_sPtr &spi, Chimera::SPI::DriverConfig &setup )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
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

    /*------------------------------------------------
    Power down the device and make sure we actually achieve it
    ------------------------------------------------*/
    Reg8_t configState = 0xFF;

    toggleRFPower( false );
    readRegister( REG_CONFIG, &configState, 1 );

    if ( configState & CONFIG_PWR_UP )
    {
      return Chimera::CommonStatusCodes::FAILED_INIT;
    }

    return result;
  }

  Reg8_t Driver::readRegister( const Reg8_t reg )
  {
    Reg8_t tempBuffer = std::numeric_limits<Reg8_t>::max();
    readRegister( reg, &tempBuffer, 1 );
    return tempBuffer;
  }

  Reg8_t Driver::readRegister( const Reg8_t reg, uint8_t *const buf, size_t len )
  {
    if ( len > MAX_PAYLOAD_WIDTH )
    {
      len = MAX_PAYLOAD_WIDTH;
    }

    spi_txbuff[ 0 ] = ( CMD_R_REGISTER | ( reg & CMD_REGISTER_MASK ) );
    memset( &spi_txbuff[ 1 ], CMD_NOP, len );

    /*------------------------------------------------
    Read the data out, adding 1 byte for the command instruction
    ------------------------------------------------*/
    if ( spi->lock( 100 ) == Chimera::CommonStatusCodes::OK )
    {
      CSPin->setState( Chimera::GPIO::State::LOW );
      spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), len + 1, 100 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );
      CSPin->setState( Chimera::GPIO::State::HIGH );

      memcpy( buf, &spi_rxbuff[ 1 ], len );

      /* Return only the status code of the chip. The register values will be in the rx buff */
      return spi_rxbuff[ 0 ];
    }
  }

  Chimera::Status_t Driver::toggleRFPower( const bool state )
  {
    do this thing 
  }

  Chimera::Status_t Driver::resetDevice()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::selfTest( const bool rpd )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::toggleRXDataPipe( const size_t pipe, const bool state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}
