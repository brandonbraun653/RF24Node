/********************************************************************************
 *   File Name:
 *     driver.cpp
 *
 *   Description:
 *     NRF24L01(+) low level hardware driver implementation
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>

/* Aurora Includes */
#include <Aurora/utility>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/event>

/* Driver Includes */
#include <RF24Node/src/hardware/definitions.hpp>
#include <RF24Node/src/hardware/driver.hpp>
#include <RF24Node/src/hardware/register.hpp>
#include <RF24Node/src/hardware/types.hpp>

namespace RF24::Hardware
{
  /*------------------------------------------------
  Static Data
  ------------------------------------------------*/
  static std::array<uint8_t, 25> sValidationBuffer;

  /*------------------------------------------------
  External Data
  ------------------------------------------------*/
  const std::array<Reg8_t, MAX_NUM_PIPES> rxPipeAddressRegister = { REG_RX_ADDR_P0, REG_RX_ADDR_P1, REG_RX_ADDR_P2,
                                                                    REG_RX_ADDR_P3, REG_RX_ADDR_P4, REG_RX_ADDR_P5 };

  const std::array<Reg8_t, MAX_NUM_PIPES> rxPipePayloadWidthRegister = { REG_RX_PW_P0, REG_RX_PW_P1, REG_RX_PW_P2,
                                                                         REG_RX_PW_P3, REG_RX_PW_P4, REG_RX_PW_P5 };

  const std::array<Reg8_t, MAX_NUM_PIPES> rxPipeEnableBitField = { EN_RXADDR_P0, EN_RXADDR_P1, EN_RXADDR_P2,
                                                                   EN_RXADDR_P3, EN_RXADDR_P4, EN_RXADDR_P5 };

  /*-------------------------------------------------
  Constructors/Destructors
  -------------------------------------------------*/
  /**
   *
   */
  Driver::Driver()
  {
    spi = nullptr;

    spi_txbuff.fill( 0u );
    spi_rxbuff.fill( 0u );

    mDynamicPayloadsEnabled = false;
    mFeaturesActivated      = false;
    mAddressBytes           = 0u;
  }


  /**
   *
   */
  Driver::~Driver()
  {
  }

  /*-------------------------------------------------
  Driver Functions
  -------------------------------------------------*/
  /**
   *
   */
  Chimera::Status_t Driver::initialize( const Chimera::SPI::DriverConfig &setup, const Chimera::GPIO::PinInit &CE )
  {
    auto result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Input Protection
    ------------------------------------------------*/
    if ( !CE.validity )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    /*------------------------------------------------
    Create and configure the SPI driver
    ------------------------------------------------*/
    if ( !spi )
    {
      spi = Chimera::SPI::create_shared_ptr();
    }

    result |= spi->init( setup );

    /*------------------------------------------------
    Configure radio chip enable & spi chip select pins
    ------------------------------------------------*/
    CEPin = Chimera::GPIO::create_unique_ptr();
    result |= CEPin->init( CE, 100 );
    CEPin->setState( Chimera::GPIO::State::HIGH, 100 );

    CSPin = Chimera::GPIO::create_unique_ptr();
    result |= CSPin->init( setup.CSInit, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

    /*------------------------------------------------
    Power down the device and make sure we actually achieve it
    ------------------------------------------------*/
    Reg8_t configState = CONFIG_PWR_UP;

    resetDevice();
    readRegister( REG_CONFIG, &configState, 1 );

    if ( ( ( configState & CONFIG_PWR_UP_Msk ) != CONFIG_PWR_UP ) || ( result != Chimera::CommonStatusCodes::OK ) )
    {
      Aurora::Utility::insertBreakpoint();
      return Chimera::CommonStatusCodes::FAILED_INIT;
    }

    return result;
  }


  /**
   *
   */
  Reg8_t Driver::readRegister( const Reg8_t addr )
  {
    Reg8_t tempBuffer = std::numeric_limits<Reg8_t>::max();
    readRegister( addr, &tempBuffer, 1 );
    return tempBuffer;
  }


  /**
   *
   */
  Reg8_t Driver::readRegister( const Reg8_t addr, void *const buf, size_t len )
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
    if ( spi->try_lock_for( 100 ) )
    {
      CSPin->setState( Chimera::GPIO::State::LOW, 100 );
      spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), len + 1, 100 );
      spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
      CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

      memcpy( buf, &spi_rxbuff[ 1 ], len );

      /* Return only the status code of the chip. The register values will be in the rx buff */
      return spi_rxbuff[ 0 ];
    }

    return std::numeric_limits<Reg8_t>::max();
  }


  /**
   *
   */
  Reg8_t Driver::writeRegister( const Reg8_t addr, const Reg8_t value, const bool check )
  {
    return writeRegister( addr, &value, 1u, check );
  }


  /**
   *
   */
  Reg8_t Driver::writeRegister( const Reg8_t addr, const void *const buffer, size_t len, const bool check )
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
    if ( spi->try_lock_for( 100 ) )
    {
      CSPin->setState( Chimera::GPIO::State::LOW, 100 );
      spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), len + 1, 100 );
      spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
      CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

      /* Save off the return code because validation will overwrite the buffer */
      auto returnCode = spi_rxbuff[ 0 ];

      /*------------------------------------------------
      Optionally validate whatever was just written
      ------------------------------------------------*/
#if defined( DEBUG ) && defined( _EMBEDDED )
      if ( check )
      {
        sValidationBuffer.fill( 0 );
        readRegister( addr, sValidationBuffer.data(), len );

        if ( memcmp( buffer, sValidationBuffer.data(), len ) != 0 )
        {
          asm( "bkpt 255" );
        }
      }
#endif /* DEBUG */

      return returnCode;
    }

    return std::numeric_limits<Reg8_t>::max();
  }


  /**
   *
   */
  Reg8_t Driver::setRegisterBits( const Reg8_t addr, const Reg8_t mask, const bool check )
  {
    Reg8_t current = readRegister( addr );
    current |= mask;
    return writeRegister( addr, current, check );
  }


  /**
   *
   */
  Reg8_t Driver::clrRegisterBits( const Reg8_t addr, const Reg8_t mask )
  {
    Reg8_t current = readRegister( addr );
    current &= ~mask;
    return writeRegister( addr, current );
  }


  /**
   *
   */
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
      CEPin->setState( Chimera::GPIO::State::LOW, 100 );
      clrRegisterBits( REG_CONFIG, CONFIG_PWR_UP );
    }
  }


  /**
   *
   */
  void Driver::toggleFeatures( const bool state )
  {
    if ( !mFeaturesActivated )
    {
      spi_txbuff[ 0 ] = RF24::Hardware::CMD_ACTIVATE;
      spi_txbuff[ 1 ] = 0x73;

      CSPin->setState( Chimera::GPIO::State::LOW, 100 );
      spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), 2, 100 );
      spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
      CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

      mFeaturesActivated = true;
    }
    else if ( mFeaturesActivated && !state )
    {
      /*-------------------------------------------------
      Sending the activation command sequence again also disables the features
      -------------------------------------------------*/
      toggleFeatures( true );
      mFeaturesActivated = false;
    }
  }


  /**
   *
   */
  void Driver::toggleDynamicPayloads( const bool state )
  {
    if ( state )
    {
      /*-------------------------------------------------
      Send the activate command to enable selection of features
      -------------------------------------------------*/
      toggleFeatures( true );

      /*-------------------------------------------------
      Enable the dynamic payload feature bit
      -------------------------------------------------*/
      setRegisterBits( REG_FEATURE, FEATURE_EN_DPL );

      /*-------------------------------------------------
      Enable dynamic payload on all pipes. This requires that
      auto-acknowledge be enabled.
      -------------------------------------------------*/
      setRegisterBits( REG_EN_AA, EN_AA_Mask );
      setRegisterBits( REG_DYNPD, DYNPD_Mask );

      mDynamicPayloadsEnabled = true;
    }
    else if ( mFeaturesActivated && !state )
    {
      /*-------------------------------------------------
      Disable for all pipes
      -------------------------------------------------*/
      clrRegisterBits( REG_DYNPD, DYNPD_Mask );
      clrRegisterBits( REG_EN_AA, EN_AA_Mask );
      clrRegisterBits( REG_FEATURE, FEATURE_EN_DPL );

      mDynamicPayloadsEnabled = false;
    }
  }


  /**
   *
   */
  void Driver::toggleDynamicAck( const bool state )
  {
    if ( state )
    {
      toggleFeatures( true );
      setRegisterBits( REG_FEATURE, FEATURE_EN_DYN_ACK );
    }
    else if ( mFeaturesActivated )
    {
      clrRegisterBits( REG_FEATURE, FEATURE_EN_DYN_ACK );
    }
  }


  /**
   *
   */
  void Driver::toggleAutoAck( const bool state, const PipeNumber pipe )
  {
    if ( ( pipe == PIPE_NUM_ALL ) && state )
    {
      setRegisterBits( REG_EN_AA, EN_AA_Mask );
    }
    else if ( ( pipe == PIPE_NUM_ALL ) && !state )
    {
      clrRegisterBits( REG_EN_AA, EN_AA_Mask );
    }
    else if ( pipe < MAX_NUM_PIPES )
    {
      Reg8_t en_aa = readRegister( REG_EN_AA );

      if ( state )
      {
        en_aa |= 1u << pipe;
      }
      else
      {
        en_aa &= ~( 1u << pipe );
      }

      writeRegister( REG_EN_AA, en_aa );
    }
    /* Else invalid pipe input parameters */
  }


  /**
   *
   */
  void Driver::toggleAckPayloads( const bool state )
  {
    if ( !mDynamicPayloadsEnabled && state )
    {
      toggleFeatures( true );
      setRegisterBits( REG_FEATURE, FEATURE_EN_ACK_PAY | FEATURE_EN_DPL );
      setRegisterBits( REG_DYNPD, DYNPD_DPL_P0 | DYNPD_DPL_P1 );

      mDynamicPayloadsEnabled = true;
    }
    else if ( mDynamicPayloadsEnabled && !state )
    {
      clrRegisterBits( REG_FEATURE, FEATURE_EN_ACK_PAY | FEATURE_EN_DPL );
      clrRegisterBits( REG_DYNPD, DYNPD_DPL_P0 | DYNPD_DPL_P1 );

      mDynamicPayloadsEnabled = false;
    }
  }


  /**
   *
   */
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
    writeRegister( REG_FIFO_STATUS, FIFO_STATUS_Reset, false );
    writeRegister( REG_DYNPD, DYNPD_Reset );
    writeRegister( REG_FEATURE, FEATURE_Reset );

    return Chimera::CommonStatusCodes::OK;
  }


  /**
   *
   */
  Chimera::Status_t Driver::selfTest( const bool rpd )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }


  /**
   *
   */
  Chimera::Status_t Driver::toggleRXDataPipe( const size_t pipe, const bool state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }


  /**
   *
   */
  void Driver::toggleCE( const bool state )
  {
    if ( state )
    {
      CEPin->setState( Chimera::GPIO::State::HIGH, 100 );
    }
    else
    {
      CEPin->setState( Chimera::GPIO::State::LOW, 100 );
    }
  }


  /**
   *
   */
  Reg8_t Driver::writePayload( const void *const buf, const size_t len, const uint8_t writeType )
  {
    if ( ( writeType != CMD_W_TX_PAYLOAD_NO_ACK ) && ( writeType != CMD_W_TX_PAYLOAD ) )
    {
      return 0u;
    }

    /*-------------------------------------------------
    Calculate the number of bytes that do nothing. When dynamic payloads are enabled, the length
    doesn't matter as long as it is less than the max payload width (32).
    -------------------------------------------------*/
    size_t payload_len = std::min( len, MAX_PAYLOAD_WIDTH );
    size_t blank_len   = static_cast<uint8_t>( mDynamicPayloadsEnabled ? 0u : ( MAX_PAYLOAD_WIDTH - payload_len ) );
    size_t size        = len + blank_len + 1;

    /*-------------------------------------------------
    Format the write command and fill the rest with zeros
    -------------------------------------------------*/
    memset( spi_txbuff.data(), 0xff, spi_txbuff.size() );

    spi_txbuff[ 0 ] = writeType;                    /* Write command type */
    memcpy( &spi_txbuff[ 1 ], buf, len );           /* Payload information */
    memset( &spi_txbuff[ len + 1 ], 0, blank_len ); /* Null out the remaining buffer space */

    CSPin->setState( Chimera::GPIO::State::LOW, 100 );
    spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), size, 100 );
    spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

    return spi_rxbuff[ 0 ];
  }


  /**
   *
   */
  Reg8_t Driver::readPayload( void *const buffer, const size_t bufferLength, const size_t payloadLength )
  {
    using namespace RF24::Hardware;

    uint8_t status = 0u;

    /*-------------------------------------------------
    The chip enable pin must be low to read out data
    -------------------------------------------------*/
    toggleCE( false );

    /*-------------------------------------------------
    Cap the data length
    -------------------------------------------------*/
    auto readLength = std::min( bufferLength, payloadLength );
    readLength      = std::min( readLength, MAX_PAYLOAD_WIDTH );

    /*-------------------------------------------------
    Calculate the number of bytes that do nothing. This is important for
    fixed payload widths as the full width must be read out each time.
    -------------------------------------------------*/
    uint8_t blank_len = static_cast<uint8_t>( mDynamicPayloadsEnabled ? 0 : ( payloadLength - bufferLength ) );
    size_t size       = bufferLength + blank_len;

    /*-------------------------------------------------
    Format the read command and fill the rest with NOPs
    -------------------------------------------------*/
    spi_txbuff[ 0 ] = RF24::Hardware::CMD_R_RX_PAYLOAD;
    memset( &spi_txbuff[ 1 ], RF24::Hardware::CMD_NOP, size );
    memset( spi_rxbuff.data(), 0, spi_rxbuff.size() );

    /*-------------------------------------------------
    Read out the payload. The +1 is for the read command.
    -------------------------------------------------*/
    CSPin->setState( Chimera::GPIO::State::LOW, 100 );
    spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), size + 1u, 100 );
    spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

    status = spi_rxbuff[ 0 ];
    memcpy( buffer, &spi_rxbuff[ 1 ], readLength );

    /*------------------------------------------------
    Clear (by setting) the RX_DR flag to signal we've read data
    ------------------------------------------------*/
    setRegisterBits( REG_STATUS, STATUS_RX_DR, false );

    /*-------------------------------------------------
    Reset the chip enable back to the initial RX state
    -------------------------------------------------*/
    CEPin->setState( Chimera::GPIO::State::HIGH, 100 );

    return status;
  }


  /**
   *
   */
  Chimera::Status_t Driver::writeAckPayload( const PipeNumber pipe, const void *const buffer, const size_t len )
  {
    // TODO: Magic numbers abound in this function. Get rid of them.
    size_t size = std::min( len, static_cast<size_t>( 32 ) ) + 1u;

    spi_txbuff[ 0 ] = RF24::Hardware::CMD_W_ACK_PAYLOAD | ( pipe & 0x07 );
    memcpy( &spi_txbuff[ 1 ], buffer, size );

    CSPin->setState( Chimera::GPIO::State::LOW, 100 );
    spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), size, 100 );
    spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

    return Chimera::CommonStatusCodes::OK;
  }


  /**
   *
   */
  Reg8_t Driver::writeCMD( const uint8_t cmd )
  {
    size_t txLength = 1;
    spi_txbuff[ 0 ] = cmd;

    /*------------------------------------------------
    Write the data out, adding 1 byte for the command instruction
    ------------------------------------------------*/
    CSPin->setState( Chimera::GPIO::State::LOW, 100 );
    spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), txLength, 100 );
    spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

    /* Return only the status code of the chip */
    return spi_rxbuff[ 0 ];
  }


  /**
   *
   */
  void Driver::writeCMD( const Reg8_t cmd, const void *const buffer, const size_t length )
  {
    size_t size = std::min( length, ( spi_txbuff.size() - 1u ) );

    spi_txbuff[ 0 ] = cmd;
    memcpy( &spi_txbuff[ 1 ], buffer, size );

    CSPin->setState( Chimera::GPIO::State::LOW, 100 );
    spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), size, 100 );
    spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );
  }


  /**
   *
   */
  void Driver::readCMD( const Reg8_t cmd, void *const buffer, const size_t length )
  {
    size_t size = std::min( length, ( spi_rxbuff.size() - 1u ) );

    spi_txbuff[ 0 ] = cmd;
    memset( &spi_txbuff[ 1 ], 0, size );

    CSPin->setState( Chimera::GPIO::State::LOW, 100 );
    spi->readWriteBytes( spi_txbuff.data(), spi_rxbuff.data(), size, 100 );
    spi->await( Chimera::Event::TRIGGER_TRANSFER_COMPLETE, 100 );
    CSPin->setState( Chimera::GPIO::State::HIGH, 100 );

    memcpy( buffer, &spi_rxbuff[ 1 ], size );
  }


  /**
   *
   */
  uint8_t Driver::getStatus()
  {
    return writeCMD( CMD_NOP );
  }


  /**
   *
   */
  void Driver::setCRCLength( const CRCLength length )
  {
    uint8_t config = readRegister( REG_CONFIG ) & ~( CONFIG_CRCO | CONFIG_EN_CRC );

    switch ( length )
    {
      case CRCLength::CRC_8:
        config |= CONFIG_EN_CRC;
        config &= ~CONFIG_CRCO;
        break;

      case CRCLength::CRC_16:
        config |= CONFIG_EN_CRC | CONFIG_CRCO;
        break;

      default:
        break;
    }

    writeRegister( REG_CONFIG, config );
  }


  /**
   *
   */
  CRCLength Driver::getCRCLength()
  {
    CRCLength result = CRCLength::CRC_DISABLED;

    uint8_t config = readRegister( REG_CONFIG ) & ( CONFIG_CRCO | CONFIG_EN_CRC );
    uint8_t en_aa  = readRegister( REG_EN_AA );

    if ( ( config & CONFIG_EN_CRC ) || en_aa )
    {
      if ( config & CONFIG_CRCO )
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


  /**
   *
   */
  void Driver::disableCRC()
  {
    uint8_t disable = readRegister( REG_CONFIG ) & ~CONFIG_EN_CRC;
    writeRegister( REG_CONFIG, disable );
  }


  /**
   *
   */
  void Driver::maskIRQ( const bool tx_ok, const bool tx_fail, const bool rx_ready )
  {
    uint8_t config = readRegister( REG_CONFIG );

    config &= ~( CONFIG_MASK_MAX_RT | CONFIG_MASK_TX_DS | CONFIG_MASK_RX_DR );
    config |=
        ( tx_fail << CONFIG_MASK_MAX_RT_Pos ) | ( tx_ok << CONFIG_MASK_TX_DS_Pos ) | ( rx_ready << CONFIG_MASK_RX_DR_Pos );

    writeRegister( REG_CONFIG, config );
  }


  /**
   *
   */
  bool Driver::rxFifoFull()
  {
    Reg8_t status = readRegister( REG_FIFO_STATUS );
    return status & FIFO_STATUS_RX_FULL;
  }


  /**
   *
   */
  bool Driver::rxFifoEmpty()
  {
    Reg8_t status = readRegister( REG_FIFO_STATUS );
    return status & FIFO_STATUS_RX_EMPTY;
  }


  /**
   *
   */
  bool Driver::txFifoFull()
  {
    Reg8_t status = readRegister( REG_FIFO_STATUS );
    return status & FIFO_STATUS_TX_FULL;
  }


  /**
   *
   */
  bool Driver::txFifoEmpty()
  {
    Reg8_t status = readRegister( REG_FIFO_STATUS );
    return status & FIFO_STATUS_TX_EMPTY;
  }


  /**
   *  Function:
   *    setAddressWidth
   *
   *  Notes:
   *    
   */
  void Driver::setAddressWidth( const AddressWidth address_width )
  {
    writeRegister( REG_SETUP_AW, static_cast<Reg8_t>( address_width ) );
    mAddressBytes = getAddressWidthAsBytes();
  }

  void Driver::setStaticPayloadWidth( const PipeNumber pipe, const size_t bytes )
  {
    if ( pipe < rxPipePayloadWidthRegister.size() )
    {
      writeRegister( rxPipePayloadWidthRegister[ pipe ], static_cast<Reg8_t>( bytes ) );
    }
  }

  size_t Driver::getStaticPayloadWidth( const PipeNumber pipe )
  {
    if ( pipe < rxPipePayloadWidthRegister.size() )
    {
      return readRegister( rxPipePayloadWidthRegister[ pipe ] );
    }
    else
    {
      return 0;
    }
  }


  /**
   *
   */
  size_t Driver::getAddressWidthAsBytes()
  {
    auto reg = readRegister( REG_SETUP_AW );

    switch ( static_cast<AddressWidth>( reg ) )
    {
      case AddressWidth::AW_3Byte:
        return 3;
        break;

      case AddressWidth::AW_4Byte:
        return 4;
        break;

      case AddressWidth::AW_5Byte:
        return 5;
        break;

      default:
        return 0;
        break;
    }
  } /* getAddressWidthAsBytes() */


  /**
   *
   */
  size_t Driver::getDynamicPayloadSize()
  {
    auto result = MIN_PAYLOAD_WIDTH;

    if ( mDynamicPayloadsEnabled )
    {
      uint8_t temp = 0u;
      readCMD( CMD_R_RX_PL_WID, &temp, 1u );
      result = static_cast<size_t>( temp );

      /*-------------------------------------------------
      The dynamic payload size should never be greater than
      the max payload width. If it is, reset the RX queues.
      -------------------------------------------------*/
      if ( result > MAX_PAYLOAD_WIDTH )
      {
        writeCMD( CMD_FLUSH_RX );
        Chimera::delayMilliseconds( 2 );
        return Chimera::CommonStatusCodes::FAILED_READ;
      }
    }

    return result;
  } /* getDynamicPayloadSize() */

}    // namespace RF24::Hardware
