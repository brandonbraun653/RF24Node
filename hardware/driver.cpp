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
  const std::array<Reg8_t, MAX_NUM_PIPES> rxPipeAddressRegister = { REG_RX_ADDR_P0, REG_RX_ADDR_P1, REG_RX_ADDR_P2,
                                                                    REG_RX_ADDR_P3, REG_RX_ADDR_P4, REG_RX_ADDR_P5 };

  const std::array<Reg8_t, MAX_NUM_PIPES> rxPipePayloadWidthRegister = { REG_RX_PW_P0, REG_RX_PW_P1, REG_RX_PW_P2,
                                                                         REG_RX_PW_P3, REG_RX_PW_P4, REG_RX_PW_P5 };

  const std::array<Reg8_t, MAX_NUM_PIPES> rxPipeEnableBitField = { EN_RXADDR_P0, EN_RXADDR_P1, EN_RXADDR_P2,
                                                                   EN_RXADDR_P3, EN_RXADDR_P4, EN_RXADDR_P5 };

  size_t pipeResourceIndex( const PipeBitField_t pipe )
  {
    switch ( pipe )
    {
      case PIPE_0:

        break;

      case PIPE_1:

        break;
      case PIPE_2:

        break;

      case PIPE_3:

        break;
      case PIPE_4:

        break;
      case PIPE_5:

        break;

      default:
        return 0u;
        break;
    }
  }

  /*-------------------------------------------------
  Constructors/Destructors
  -------------------------------------------------*/
  Driver::Driver()
  {
    spi = nullptr;
    memset( &spiConfig, 0, sizeof( spiConfig ) );

    spi_txbuff.fill( 0u );
    spi_rxbuff.fill( 0u );
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
    Reg8_t configState = CONFIG_PWR_UP;

    resetDevice();
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
    if ( state && !( readRegister( REG_CONFIG ) & CONFIG_PWR_UP ) )
    {
      /*-------------------------------------------------
      If not powered up already, do it. The worst startup
      delay is about 5mS, so just wait that amount.
      -------------------------------------------------*/
      setRegisterBits( REG_CONFIG, CONFIG_PWR_UP );
      Chimera::delayMilliseconds( 5 );
    }
    else
    {
      /*-------------------------------------------------
      Force standby mode and power down the chip
      -------------------------------------------------*/
      CEPin->setState( Chimera::GPIO::State::LOW );
      clrRegisterBits( REG_CONFIG, CONFIG_PWR_UP );
    }
  }

  Chimera::Status_t Driver::resetDevice()
  {
    toggleRFPower( false );
    writeRegister( REG_CONFIG, CONFIG_Reset );
    writeRegister( REG_EN_AA, EN_AA_Reset );
    writeRegister( REG_EN_RXADDR, EN_RXADDR_Reset );
    writeRegister( REG_SETUP_AW, SETUP_AW_Reset );
    writeRegister( REG_SETUP_RETR, SETUP_RETR_Reset );
    writeRegister( REG_RF_CH, RF_CH_Reset );
    writeRegister( REG_RF_SETUP, RF_SETUP_Reset );
    writeRegister( REG_OBSERVE_TX, OBSERVE_TX_Reset );
    writeRegister( REG_CD, RPD_Reset );
    writeRegister( REG_RX_ADDR_P0, reinterpret_cast<const uint8_t *>( &RX_ADDR_P0_Reset ), RX_ADDR_P0_byteWidth );
    writeRegister( REG_RX_ADDR_P1, reinterpret_cast<const uint8_t *>( &RX_ADDR_P1_Reset ), RX_ADDR_P1_byteWidth );
    writeRegister( REG_RX_ADDR_P2, RX_ADDR_P2_Reset );
    writeRegister( REG_RX_ADDR_P3, RX_ADDR_P3_Reset );
    writeRegister( REG_RX_ADDR_P4, RX_ADDR_P4_Reset );
    writeRegister( REG_RX_ADDR_P5, RX_ADDR_P5_Reset );
    writeRegister( REG_TX_ADDR, reinterpret_cast<const uint8_t *>( &TX_ADDR_Reset ), TX_ADDR_byteWidth );
    writeRegister( REG_RX_PW_P0, RX_PW_P0_Reset );
    writeRegister( REG_RX_PW_P1, RX_PW_P1_Reset );
    writeRegister( REG_RX_PW_P2, RX_PW_P2_Reset );
    writeRegister( REG_RX_PW_P3, RX_PW_P3_Reset );
    writeRegister( REG_RX_PW_P4, RX_PW_P4_Reset );
    writeRegister( REG_RX_PW_P5, RX_PW_P5_Reset );
    writeRegister( REG_FIFO_STATUS, FIFO_STATUS_Reset );
    writeRegister( REG_DYNPD, DYNPD_Reset );
    writeRegister( REG_FEATURE, FEATURE_Reset );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::selfTest( const bool rpd )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::toggleRXDataPipe( const size_t pipe, const bool state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  void Driver::toggleCE( const bool state )
  {
    if ( state )
    {
      CEPin->setState( Chimera::GPIO::State::HIGH );
    }
    else
    {
      CEPin->setState( Chimera::GPIO::State::LOW );
    }
  }

  uint8_t Driver::writePayload( const uint8_t *const buf, size_t len, const uint8_t writeType )
  {
    if ( ( writeType != RF24::Hardware::CMD_W_TX_PAYLOAD_NO_ACK ) && ( writeType != RF24::Hardware::CMD_W_TX_PAYLOAD ) )
    {
      return 0u;
    }

    /*-------------------------------------------------
    Calculate the number of bytes that do nothing. When dynamic payloads are enabled, the length
    doesn't matter as long as it is less than the max payload width (32).
    -------------------------------------------------*/
    len               = std::min( len, payloadSize );
    uint8_t blank_len = static_cast<uint8_t>( dynamicPayloadsEnabled ? 0 : ( payloadSize - len ) );
    size_t size       = len + blank_len + 1;

    /*-------------------------------------------------
    Format the write command and fill the rest with zeros
    -------------------------------------------------*/
    memset( spi_txbuff.data(), 0xff, spi_txbuff.size() );

    spi_txbuff[ 0 ] = writeType;                    /* Write command type */
    memcpy( &spi_txbuff[ 1 ], buf, len );           /* Payload information */
    memset( &spi_txbuff[ len + 1 ], 0, blank_len ); /* Null out the remaining buffer space */

    beginTransaction();
    spiWriteRead( spi_txbuff.data(), spi_rxbuff.data(), size );
    endTransaction();

    return spi_rxbuff[ 0 ];
  }

  

  uint8_t Driver::writeCMD( const uint8_t cmd )
  {
    size_t txLength = 1;
    spi_txbuff[ 0 ] = cmd;

    /*------------------------------------------------
    Write the data out, adding 1 byte for the command instruction
    ------------------------------------------------*/
    if ( spi->lock( 100 ) == Chimera::CommonStatusCodes::OK )
    {
      CSPin->setState( Chimera::GPIO::State::LOW );
      spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), txLength, 100 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );
      CSPin->setState( Chimera::GPIO::State::HIGH );

      /* Return only the status code of the chip */
      return spi_rxbuff[ 0 ];
    }

    return std::numeric_limits<Reg8_t>::max();
  }

  uint8_t Driver::getStatus()
  {
    return writeCMD( CMD_NOP );
  }

  void Driver::setCRCLength( const CRCLength length )
  {
    uint8_t config =
        readRegister( RF24::Hardware::REG_CONFIG ) & ~( RF24::Hardware::CONFIG_CRCO | RF24::Hardware::CONFIG_EN_CRC );

    switch ( length )
    {
      case CRCLength::CRC_8:
        config |= RF24::Hardware::CONFIG_EN_CRC;
        config &= ~RF24::Hardware::CONFIG_CRCO;
        break;

      case CRCLength::CRC_16:
        config |= RF24::Hardware::CONFIG_EN_CRC | RF24::Hardware::CONFIG_CRCO;
        break;

      default:
        break;
    }

    writeRegister( RF24::Hardware::REG_CONFIG, config );
  }

  CRCLength Driver::getCRCLength()
  {
    CRCLength result = CRCLength::CRC_DISABLED;

    uint8_t config =
        readRegister( RF24::Hardware::REG_CONFIG ) & ( RF24::Hardware::CONFIG_CRCO | RF24::Hardware::CONFIG_EN_CRC );
    uint8_t en_aa = readRegister( RF24::Hardware::REG_EN_AA );

    if ( ( config & RF24::Hardware::CONFIG_EN_CRC ) || en_aa )
    {
      if ( config & RF24::Hardware::CONFIG_CRCO )
      {
        result = CRCLength::CRC_16;
      }
      else
      {
        result = CRCLength::CRC_8;
      }
    }

    return result;
  }

  void Driver::disableCRC()
  {
    uint8_t disable = readRegister( RF24::Hardware::REG_CONFIG ) & ~RF24::Hardware::CONFIG_EN_CRC;
    writeRegister( RF24::Hardware::REG_CONFIG, disable );
  }
}    // namespace RF24::Hardware
