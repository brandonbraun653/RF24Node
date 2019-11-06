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

  Reg8_t Driver::readRegister( const Reg8_t addr )
  {
    Reg8_t tempBuffer = std::numeric_limits<Reg8_t>::max();
    readRegister( addr, &tempBuffer, 1 );
    return tempBuffer;
  }

  Reg8_t Driver::readRegister( const Reg8_t addr, uint8_t *const buf, size_t len )
  {
    /*------------------------------------------------
    Input protection
    ------------------------------------------------*/
    if ( len > MAX_PAYLOAD_WIDTH )
    {
      len = MAX_PAYLOAD_WIDTH;
    }

    /*------------------------------------------------
    Populate the read command
    ------------------------------------------------*/
    spi_txbuff[ 0 ] = ( CMD_R_REGISTER | ( addr & CMD_REGISTER_MASK ) );
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

    return std::numeric_limits<Reg8_t>::max();
  }

  Reg8_t Driver::writeRegister( const Reg8_t addr, const Reg8_t value )
  {
    return writeRegister( addr, &value, 1u );
  }

  Reg8_t Driver::writeRegister( const Reg8_t addr, const Reg8_t *const buffer, size_t len )
  {
    /*------------------------------------------------
    Input protection
    ------------------------------------------------*/
    if ( len > MAX_PAYLOAD_WIDTH )
    {
      len = MAX_PAYLOAD_WIDTH;
    }

    /*------------------------------------------------
    Prepare the write command
    ------------------------------------------------*/
    spi_txbuff[ 0 ] = ( CMD_W_REGISTER | ( addr & CMD_REGISTER_MASK ) );
    memcpy( &spi_txbuff[ 1 ], buffer, len );

    /*------------------------------------------------
    Write the data out, adding 1 byte for the command instruction
    ------------------------------------------------*/
    if ( spi->lock( 100 ) == Chimera::CommonStatusCodes::OK )
    {
      CSPin->setState( Chimera::GPIO::State::LOW );
      spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), len + 1, 100 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );
      CSPin->setState( Chimera::GPIO::State::HIGH );

      /* Return only the status code of the chip. The register values will be in the rx buff */
      return spi_rxbuff[ 0 ];
    }
    
    return std::numeric_limits<Reg8_t>::max();
  }

  Reg8_t Driver::setRegisterBits( const Reg8_t addr, const Reg8_t mask )
  {
    Reg8_t current = readRegister( addr );
    current |= mask;
    return writeRegister( addr, current );
  }

  Reg8_t Driver::clrRegisterBits( const Reg8_t addr, const Reg8_t mask )
  {
    Reg8_t current = readRegister( addr );
    current &= ~mask;
    return writeRegister( addr, current );
  }

  void Driver::toggleRFPower( const bool state )
  {
    if ( state )
    {
      setRegisterBits( REG_CONFIG, CONFIG_PWR_UP );
    }
    else
    {
      clrRegisterBits( REG_CONFIG, CONFIG_PWR_UP );
    }
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
