/********************************************************************************
 * File Name:
 *	  RF24Phy.cpp
 *
 * Description:
 *	  Implementation of the RF24 Physical layer
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>
#include <algorithm>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/hardware/driver.hpp>
#include <RF24Node/hardware/register.hpp>
#include <RF24Node/physical/physical.hpp>

namespace RF24::Physical
{
  Driver::Driver() : currentMode( RF24::Hardware::Mode::POWER_DOWN )
  {
    
  }

  Driver::~Driver()
  {

  }


//  bool Driver::erase()
//  {
//    bool eraseStatus = true;
//
//    cachedPipe0RXAddress = 0u;
//
//    toggleChipEnablePin( false );
//
//    flushTX();
//    flushRX();
//
//    
//  }

  bool Driver::begin()
  {
    initialized = false;

    /*-------------------------------------------------
    Check we can talk to the device and reset it back to
    some known initial condition.
    -------------------------------------------------*/
    if ( !isConnected() )
    {
      oopsies = RF24::Hardware::FailureCode::NOT_CONNECTED;
    }
//
//    else if ( !erase() )
//    {
//      oopsies = RF24::Hardware::RF24::Hardware::FailureCode::COULD_NOT_ERASE;
//    }
    else
    {
      /*-------------------------------------------------
      Enable 16-bit CRC
      -------------------------------------------------*/
      hwDriver->setCRCLength( RF24::Hardware::CRCLength::CRC_16 );

      /*-------------------------------------------------
      Set 1500uS timeout and 3 retry attempts. Don't lower
      or the 250KBS mode will break.
      -------------------------------------------------*/
      setRetries( RF24::Hardware::AutoRetransmitDelay::w1500uS, 3 );

      /*-------------------------------------------------
      Check whether or not we have a P variant of the chip
      -------------------------------------------------*/
      pVariant = setDataRate( RF24::Hardware::DataRate::DR_250KBPS );

      /*-------------------------------------------------
      Set data rate to the slowest, most reliable speed supported by all hardware
      -------------------------------------------------*/
      setDataRate( RF24::Hardware::DataRate::DR_1MBPS );

      /*------------------------------------------------
      Default to the 5 byte wide addressing scheme
      ------------------------------------------------*/
      hwDriver->setAddressWidth( RF24::Hardware::AddressWidth::AW_5Byte );

      /*-------------------------------------------------
      Set the default channel to a value that likely won't congest the spectrum
      -------------------------------------------------*/
      setChannel( 76 );

      /*-------------------------------------------------
      Clear the buffers to start with a clean slate
      -------------------------------------------------*/
      flushTX();
      flushRX();

      /*-------------------------------------------------
      Power up the module and enable PTX. Stay in standby mode by not writing CE high.
      Delay a few milliseconds to let things settle.
      -------------------------------------------------*/
      toggleChipEnablePin( false );
      hwDriver->toggleRFPower( true );

      initialized = true;
    }

    return initialized;
  }

  bool Driver::isInitialized()
  {
    return initialized;
  }

  void Driver::startListening()
  {
    if ( !listening )
    {
      /*-------------------------------------------------
      If we are auto-acknowledging RX packets with a payload, make sure the TX
      FIFO is clean so we don't accidently transmit data.
      -------------------------------------------------*/
      if ( _registerIsBitmaskSet( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_ACK_PAY ) )
      {
        flushTX();
      }

      /*-------------------------------------------------
      Clear interrupt flags and transition to RX mode
      -------------------------------------------------*/
      hwDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_RX_DR | RF24::Hardware::STATUS_TX_DS | RF24::Hardware::STATUS_MAX_RT );
      hwDriver->setRegisterBits( RF24::Hardware::REG_CONFIG, RF24::Hardware::CONFIG_PRIM_RX );
      toggleChipEnablePin( true);
      currentMode = RF24::Hardware::Mode::RX;

      /*------------------------------------------------
      If we clobbered the old pipe 0 listening address so we could transmit
      something, restore it back.
      ------------------------------------------------*/
      if ( cachedPipe0RXAddress )
      {
        openReadPipe( 0, cachedPipe0RXAddress, true );
      }

      listening       = true;
      listeningPaused = false;
    }
  }

  void Driver::pauseListening()
  {
    if ( listening && !listeningPaused )
    {
      toggleChipEnablePin( false );
      listeningPaused = true;
    }
  }

  void Driver::resumeListening()
  {
    if ( listeningPaused )
    {
      toggleChipEnablePin( true);
      listeningPaused = false;
    }
  }

  void Driver::stopListening()
  {
    if ( listening || listeningPaused )
    {
      /*-------------------------------------------------
      Set the chip into standby mode I
      -------------------------------------------------*/
      toggleChipEnablePin( false );
      currentMode = RF24::Hardware::Mode::STANDBY_I;
      Chimera::delayMilliseconds( 1 );

      /*-------------------------------------------------
      If we are auto-acknowledging RX packets with a payload, make sure the TX FIFO is clean so
      we don't accidentally transmit data the next time we write chipEnable high.
      -------------------------------------------------*/
      if ( _registerIsBitmaskSet( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_ACK_PAY ) )
      {
        flushTX();
      }

      /*-------------------------------------------------
      Disable RX/Enable TX
      -------------------------------------------------*/
      hwDriver->clrRegisterBits( RF24::Hardware::REG_CONFIG, RF24::Hardware::CONFIG_PRIM_RX );

      /*-------------------------------------------------
      Ensure RX Pipe 0 can listen (TX only transmits/receives on pipe 0)
      -------------------------------------------------*/
      hwDriver->setRegisterBits( RF24::Hardware::REG_EN_RXADDR, pipeEnableRXAddressReg[ 0 ] );

      listening = false;
    }
  }

  void Driver::toggleChipEnablePin( const bool state )
  {
    hwDriver->toggleCE( state );
  }

  bool Driver::available()
  {
    return !rxFifoEmpty();
  }

  bool Driver::available( uint8_t &pipe_num )
  {
    pipe_num = 0xFF;

    /*-------------------------------------------------
    Figure out which pipe has data available
    -------------------------------------------------*/
    if ( available() )
    {
      pipe_num = ( getStatus() >> RF24::Hardware::STATUS_RX_P_NO_Pos ) & RF24::Hardware::STATUS_RX_P_NO_Wid;
      return true;
    }

    return false;
  }

  void Driver::read( uint8_t *const buffer, size_t len )
  {
    readPayload( buffer, len );

    /*------------------------------------------------
    Clear the ISR flag bits by setting them to 1
    ------------------------------------------------*/
    uint8_t statusVal = RF24::Hardware::STATUS_RX_DR | RF24::Hardware::STATUS_MAX_RT | RF24::Hardware::STATUS_TX_DS;
    hwDriver->writeRegister( RF24::Hardware::REG_STATUS, statusVal );
  }

  bool Driver::writeFast( const uint8_t *const buffer, uint8_t len, const bool multicast )
  {
    /*------------------------------------------------
    Don't clobber the RX if we are listening
    ------------------------------------------------*/
    if ( listening )
    {
      oopsies = RF24::Hardware::FailureCode::RADIO_IN_RX_MODE;
      return false;
    }

    /*-------------------------------------------------
    Wait for the FIFO to have room for one more packet
    -------------------------------------------------*/
    uint32_t startTime = Chimera::millis();

    while ( txFifoFull() )
    {
      /*-------------------------------------------------
      If max retries hit from a previous transmission, we screwed up
      -------------------------------------------------*/
      if ( _registerIsBitmaskSet( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT ) )
      {
        hwDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT );
        oopsies = RF24::Hardware::FailureCode::MAX_RETRY_TIMEOUT;
        return false;
      }

      /*------------------------------------------------
      Make sure we aren't waiting around forever
      ------------------------------------------------*/
      if ( ( Chimera::millis() - startTime ) > DFLT_TIMEOUT_MS )
      {
        oopsies = RF24::Hardware::FailureCode::TX_FIFO_FULL_TIMEOUT;
        return false;
      }

      delayMilliseconds( MIN_TIMEOUT_MS );
    }

    /*------------------------------------------------
    We're free! Load the data into the FIFO and kick off the transfer
    ------------------------------------------------*/
    startFastWrite( buffer, len, multicast, true );
    return true;
  }

  void Driver::openWritePipe( const uint64_t address )
  {
    /*-------------------------------------------------
    Set the receive address for pipe 0 to be equal to the transmit address. This is to allow
    reception of an ACK packet should it be sent from the receiver.
    -------------------------------------------------*/
    hwDriver->writeRegister( RF24::Hardware::REG_RX_ADDR_P0, reinterpret_cast<const uint8_t *>( &address ), addressWidth );

    /*-------------------------------------------------
    Make sure we transmit back to the same address we expect receive from
    -------------------------------------------------*/
    hwDriver->writeRegister( RF24::Hardware::REG_TX_ADDR, reinterpret_cast<const uint8_t *>( &address ), addressWidth );

    /*-------------------------------------------------
    Set a static payload length for all receptions on pipe 0. There must also be
    an equal number of bytes clocked into the TX_FIFO when data is transmitted out.
    -------------------------------------------------*/
    hwDriver->writeRegister( RF24::Hardware::REG_RX_PW_P0, static_cast<uint8_t>( payloadSize ) );

#if defined( TRACK_REGISTER_STATES )
    registers.tx_addr.update( this );
    registers.rx_pw_p0.update( this );
    registers.rx_addr_p0.update( this );
#endif
  }

  bool Driver::openReadPipe( const uint8_t pipe, const uint64_t address, const bool validate )
  {
    /*------------------------------------------------
    Make sure the pipe is ok
    ------------------------------------------------*/
    if ( pipe >= MAX_NUM_PIPES )
    {
      oopsies = RF24::Hardware::FailureCode::INVALID_PIPE;
      return false;
    }

    /*-------------------------------------------------
    Assign the address for the pipe to listen against
    -------------------------------------------------*/
    if ( pipe < 2 )
    {
      /*-------------------------------------------------
      Write only as many address bytes as were set in SETUP_AW
      -------------------------------------------------*/
      uint8_t addressWidth = getAddressBytes();
      hwDriver->writeRegister( pipeRXAddressReg[ pipe ], reinterpret_cast<const uint8_t *>( &address ), addressWidth );

      /*------------------------------------------------
      Save the pipe 0 address because it can be clobbered by openWritePipe() and
      will need to be restored later when we start listening again.
      ------------------------------------------------*/
      if ( pipe == 0 )
      {
        memcpy( &cachedPipe0RXAddress, &address, addressWidth );
      }

      /*------------------------------------------------
      Optionally validate the write
      ------------------------------------------------*/
      if ( validate )
      {
        uint64_t setVal = 0u;
        hwDriver->readRegister( pipeRXAddressReg[ pipe ], reinterpret_cast<uint8_t *>( &setVal ), addressWidth );

        if ( setVal != ( address & 0xFFFFFFFFFF ) )
        {
          oopsies = RF24::Hardware::FailureCode::REGISTER_WRITE_FAILURE;
          return false;
        }
      }
    }
    else
    {
      /*------------------------------------------------
      These pipes only need their LSB set
      ------------------------------------------------*/
      hwDriver->writeRegister( pipeRXAddressReg[ pipe ], reinterpret_cast<const uint8_t *>( &address ), 1 );

      /*------------------------------------------------
      Optionally validate the write
      ------------------------------------------------*/
      if ( validate )
      {
        uint8_t setVal = 0u;
        hwDriver->readRegister( pipeRXAddressReg[ pipe ], &setVal, 1 );

        if ( setVal != ( address & 0xFF ) )
        {
          oopsies = RF24::Hardware::FailureCode::REGISTER_WRITE_FAILURE;
          return false;
        }
      }
    }

    /*-------------------------------------------------
    Let the pipe know how wide the payload will be, then turn it on
    -------------------------------------------------*/
    hwDriver->writeRegister( pipeRXPayloadWidthReg[ pipe ], static_cast<uint8_t>( payloadSize ) );
    hwDriver->setRegisterBits( RF24::Hardware::REG_EN_RXADDR, pipeEnableRXAddressReg[ pipe ] );

#if defined( TRACK_REGISTER_STATES )
    registers.en_rxaddr.update( this );
    registers.rx_addr_p0.update( this );
    registers.rx_addr_p1.update( this );
    registers.rx_addr_p2.update( this );
    registers.rx_addr_p3.update( this );
    registers.rx_addr_p4.update( this );
    registers.rx_addr_p5.update( this );
#endif

    return true;
  }

  bool Driver::isConnected()
  {
    /*------------------------------------------------
    Grab the old settings for the register
    ------------------------------------------------*/
    uint8_t old_setup     = hwDriver->readRegister( RF24::Hardware::REG_SETUP_AW );
    uint8_t illegal_setup = 0u;

    /*------------------------------------------------
    Write some new settings and record their value
    ------------------------------------------------*/
    hwDriver->writeRegister( RF24::Hardware::REG_SETUP_AW, illegal_setup );
    uint8_t new_setup = hwDriver->readRegister( RF24::Hardware::REG_SETUP_AW );

    /*------------------------------------------------
    We are connected if the recorded settings match what we thought
    was set. Reset back to the old settings before exiting.
    ------------------------------------------------*/
    if ( new_setup == illegal_setup )
    {
      hwDriver->writeRegister( RF24::Hardware::REG_SETUP_AW, old_setup );
      return true;
    }
    else
    {
      oopsies = RF24::Hardware::FailureCode::NOT_CONNECTED;
      return false;
    }
  }

  bool Driver::rxFifoFull()
  {
    uint8_t reg = hwDriver->readRegister( RF24::Hardware::REG_FIFO_STATUS );

#if defined( TRACK_REGISTER_STATES )
    registers.fifo_status = reg;
#endif

    return reg & FIFO_STATUS::RX_FULL;
  }

  bool Driver::rxFifoEmpty()
  {
    uint8_t reg = hwDriver->readRegister( RF24::Hardware::REG_FIFO_STATUS );

#if defined( TRACK_REGISTER_STATES )
    registers.fifo_status = reg;
#endif

    return reg & FIFO_STATUS::RX_EMPTY;
  }

  bool Driver::txFifoFull()
  {
    uint8_t reg = hwDriver->readRegister( RF24::Hardware::REG_FIFO_STATUS );

#if defined( TRACK_REGISTER_STATES )
    registers.fifo_status = reg;
#endif

    return reg & FIFO_STATUS::TX_FULL;
  }

  bool Driver::txFifoEmpty()
  {
    uint8_t reg = hwDriver->readRegister( RF24::Hardware::REG_FIFO_STATUS );

#if defined( TRACK_REGISTER_STATES )
    registers.fifo_status = reg;
#endif

    return reg & FIFO_STATUS::TX_EMPTY;
  }

  void Driver::startFastWrite( const uint8_t *const buffer, size_t len, const bool multicast, const bool startTX )
  {
    uint8_t payloadType = 0u;

    if ( multicast )
    {
      /*-------------------------------------------------
      Transmit one packet without waiting for an ACK from the RX device. In order for
      this to work, the Features register has to be enabled and EN_DYN_ACK set.
      -------------------------------------------------*/
      enableDynamicAck();
      payloadType = RF24::Hardware::CMD_W_TX_PAYLOAD_NO_ACK;
    }
    else
    {
      /*-------------------------------------------------
      Force waiting for the RX device to send an ACK packet. Don't bother disabling Dynamic Ack
      (should it even be enabled) as this command overrides it.
      -------------------------------------------------*/
      payloadType = RF24::Hardware::CMD_W_TX_PAYLOAD;
    }

    /*-------------------------------------------------
    Write the payload to the TX FIFO and optionally start the transfer
    -------------------------------------------------*/
    writePayload( buffer, len, payloadType );

    if ( startTX )
    {
      toggleChipEnablePin( true);
      currentMode = Mode::TX;
    }
  }

  bool Driver::txStandBy()
  {
    /*-------------------------------------------------
    Wait for the TX FIFO to signal it's empty
    -------------------------------------------------*/
    while ( !txFifoEmpty() )
    {
      /*-------------------------------------------------
      If we hit the Max Retries, we have a problem and the whole TX FIFO is screwed.
      Go back to standby mode and clear out the FIFO.
      -------------------------------------------------*/
      if ( _registerIsBitmaskSet( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT ) )
      {
        hwDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT );
        toggleChipEnablePin( false );
        flushTX();

        currentMode = Mode::STANDBY_I;
        return false;
      }
    }

    /*------------------------------------------------
    Sends the chip back to standby mode now that the FIFO is empty
    ------------------------------------------------*/
    toggleChipEnablePin( false );
    currentMode = Mode::STANDBY_I;
    return true;
  }

  bool Driver::txStandBy( const uint32_t timeout, const bool startTx )
  {
    /*------------------------------------------------
    Optionally start a new transfer
    ------------------------------------------------*/
    if ( startTx )
    {
      stopListening();
      toggleChipEnablePin( true);
    }

    /*------------------------------------------------
    Prevent the user from executing the function if they haven't told
    the device to quit listening yet.
    ------------------------------------------------*/
    if ( listening )
    {
      oopsies = RF24::Hardware::FailureCode::RADIO_IN_RX_MODE;
      return false;
    }

    /*------------------------------------------------
    Wait for the TX FIFO to empty, retrying packet transmissions as needed.
    ------------------------------------------------*/
    uint32_t start = Chimera::millis();

    while ( !txFifoEmpty() )
    {
      /*------------------------------------------------
      If max retries interrupt occurs, retry transmission. The data is
      automatically kept in the TX FIFO.
      ------------------------------------------------*/
      if ( _registerIsBitmaskSet( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT ) )
      {
        toggleChipEnablePin( false );
        hwDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT );

        delayMilliseconds( 1 );
        toggleChipEnablePin( true);
      }

      /*------------------------------------------------
      Automatic timeout failure
      ------------------------------------------------*/
      if ( ( Chimera::millis() - start ) > timeout )
      {
        toggleChipEnablePin( false );
        flushTX();
        oopsies     = RF24::Hardware::FailureCode::TX_FIFO_EMPTY_TIMEOUT;
        currentMode = Mode::STANDBY_I;
        return false;
      }
    }

    /*------------------------------------------------
    Transition back to Standby Mode I
    ------------------------------------------------*/
    toggleChipEnablePin( false );
    currentMode = Mode::STANDBY_I;
    return true;
  }

  void Driver::writeAckPayload( const uint8_t pipe, const uint8_t *const buffer, size_t len )
  {
    // TODO: Magic numbers abound in this function. Get rid of them.
    size_t size = std::min( len, static_cast<size_t>( 32 ) ) + 1u;

    spi_txbuff[ 0 ] = RF24::Hardware::CMD_W_ACK_PAYLOAD | ( pipe & 0x07 );
    memcpy( &spi_txbuff[ 1 ], buffer, size );

    beginTransaction();
    spiWrite( spi_txbuff.data(), size );
    endTransaction();
  }

  bool Driver::isAckPayloadAvailable()
  {
    uint8_t reg = hwDriver->readRegister( RF24::Hardware::REG_FIFO_STATUS );

#if defined( TRACK_REGISTER_STATES )
    registers.fifo_status = reg;
#endif

    return !( reg & FIFO_STATUS::RX_EMPTY );
  }

  void Driver::whatHappened( bool &tx_ok, bool &tx_fail, bool &rx_ready )
  {
    uint8_t statusActual  = hwDriver->readRegister( RF24::Hardware::REG_STATUS );
    uint8_t statusCleared = statusActual | RF24::Hardware::STATUS_RX_DR | RF24::Hardware::STATUS_TX_DS | RF24::Hardware::STATUS_MAX_RT;
    hwDriver->writeRegister( RF24::Hardware::REG_STATUS, statusCleared );

    tx_ok    = statusActual & RF24::Hardware::STATUS_TX_DS;
    tx_fail  = statusActual & RF24::Hardware::STATUS_MAX_RT;
    rx_ready = statusActual & RF24::Hardware::STATUS_RX_DR;
  }

  void Driver::closeReadPipe( const uint8_t pipe )
  {
    if ( pipe < MAX_NUM_PIPES )
    {
      uint8_t rxaddrVal = hwDriver->readRegister( RF24::Hardware::REG_EN_RXADDR ) & ~pipeEnableRXAddressReg[ pipe ];
      hwDriver->writeRegister( RF24::Hardware::REG_EN_RXADDR, rxaddrVal );

#if defined( TRACK_REGISTER_STATES )
      registers.en_rxaddr = rxaddrVal;
#endif
    }
  }

  void Driver::setAddressWidth( const AddressWidth address_width )
  {
    hwDriver->writeRegister( RF24::Hardware::REG_SETUP_AW, static_cast<uint8_t>( address_width ) );

    switch ( address_width )
    {
      case RF24Phy::AddressWidth::AW_3Byte:
        addressWidth = 3;
        break;

      case RF24Phy::AddressWidth::AW_4Byte:
        addressWidth = 4;
        break;

      case RF24Phy::AddressWidth::AW_5Byte:
        addressWidth = 5;
        break;
    }

#if defined( TRACK_REGISTER_STATES )
    registers.setup_aw.update( this );
#endif
  }

  AddressWidth Driver::getAddressWidth()
  {
    auto reg = hwDriver->readRegister( RF24::Hardware::REG_SETUP_AW );

#if defined( TRACK_REGISTER_STATES )
    registers.setup_aw = reg;
#endif

    return static_cast<AddressWidth>( reg );
  }

  uint8_t Driver::getAddressBytes()
  {
    switch ( getAddressWidth() )
    {
      case RF24Phy::AddressWidth::AW_3Byte:
        return 3;
        break;

      case RF24Phy::AddressWidth::AW_4Byte:
        return 4;
        break;

      case RF24Phy::AddressWidth::AW_5Byte:
        return 5;
        break;

      default:
        return 0;
        break;
    }
  }

  bool Driver::setRetries( const RF24::Hardware::AutoRetransmitDelay:: delay, const uint8_t count, const bool validate )
  {
    bool returnVal     = true;
    uint8_t ard        = ( static_cast<uint8_t>( delay ) & 0x0F ) << SETUP_RETR::ARD_Pos;
    uint8_t arc        = ( count & 0x0F ) << SETUP_RETR::ARC_Pos;
    uint8_t setup_retr = ard | arc;

    hwDriver->writeRegister( RF24::Hardware::REG_SETUP_RETR, setup_retr );

    if ( validate )
    {
      returnVal = ( hwDriver->readRegister( RF24::Hardware::REG_SETUP_RETR ) == setup_retr );
    }

#if defined( TRACK_REGISTER_STATES )
    if ( returnVal )
    {
      registers.setup_retr = setup_retr;
    }
#endif

    return returnVal;
  }

  void Driver::setStaticPayloadSize( const uint8_t size )
  {
    payloadSize = std::min( size, static_cast<uint8_t>( 32 ) );
  }

  bool Driver::setChannel( const uint8_t channel, const bool validate )
  {
    hwDriver->writeRegister( RF24::Hardware::REG_RF_CH, channel & RF_CH::Mask );

#if defined( TRACK_REGISTER_STATES )
    registers.rf_ch.update( this );
#endif

    if ( validate )
    {
      return ( hwDriver->readRegister( RF24::Hardware::REG_RF_CH ) == ( channel & RF_CH::Mask ) );
    }

    return true;
  }

  uint8_t Driver::getChannel()
  {
    uint8_t ch = hwDriver->readRegister( RF24::Hardware::REG_RF_CH );

#if defined( TRACK_REGISTER_STATES )
    registers.rf_ch = ch;
#endif

    return ch;
  }

  uint8_t Driver::getStaticPayloadSize()
  {
    return static_cast<uint8_t>( payloadSize );
  }

  uint8_t Driver::getDynamicPayloadSize()
  {
    uint8_t result = MAX_PAYLOAD_WIDTH;

    if ( dynamicPayloadsEnabled )
    {
      spi_txbuff[ 0 ] = RF24::Hardware::CMD_R_RX_PL_WID;
      spi_rxbuff[ 1 ] = RF24::Hardware::CMD_NOP;

      beginTransaction();
      spiWriteRead( spi_txbuff.data(), spi_rxbuff.data(), 2 );
      endTransaction();

      result = spi_rxbuff[ 1 ];

      if ( result > 32 )
      {
        flushRX();
        delayMilliseconds( 2 );
        return 0;
      }
    }

    return result;
  }

  uint8_t Driver::flushTX()
  {
    return _writeCMD( RF24::Hardware::CMD_FLUSH_TX );
  }

  uint8_t Driver::flushRX()
  {
    return _writeCMD( RF24::Hardware::CMD_FLUSH_RX );
  }

  void Driver::activateFeatures()
  {
    if ( !featuresActivated )
    {
      spi_txbuff[ 0 ] = RF24::Hardware::CMD_ACTIVATE;
      spi_txbuff[ 1 ] = 0x73;

      spiWrite( spi_txbuff.data(), 2 );
      featuresActivated = true;
    }
  }

  void Driver::deactivateFeatures()
  {
    if ( featuresActivated )
    {
      /*-------------------------------------------------
      Sending the activation command sequence again also disables the features
      -------------------------------------------------*/
      activateFeatures();
      featuresActivated = false;
    }
  }

  void Driver::enableAckPayload()
  {
    activateFeatures();
    hwDriver->setRegisterBits( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_ACK_PAY | RF24::Hardware::FEATURE_EN_DPL );
    hwDriver->setRegisterBits( RF24::Hardware::REG_DYNPD, DYNPD::DPL_P0 | DYNPD::DPL_P1 );

    dynamicPayloadsEnabled = true;

#if defined( TRACK_REGISTER_STATES )
    registers.dynpd.update( this );
    registers.feature.update( this );
#endif
  }

  void Driver::disableAckPayload()
  {
    if ( featuresActivated )
    {
      hwDriver->clrRegisterBits( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_ACK_PAY | RF24::Hardware::FEATURE_EN_DPL );
      hwDriver->clrRegisterBits( RF24::Hardware::REG_DYNPD, DYNPD::DPL_P0 | DYNPD::DPL_P1 );

      dynamicPayloadsEnabled = false;

#if defined( TRACK_REGISTER_STATES )
      registers.dynpd.update( this );
      registers.feature.update( this );
#endif
    }
  }

  void Driver::enableDynamicPayloads()
  {
    /*-------------------------------------------------
    Send the activate command to enable selection of features
    -------------------------------------------------*/
    activateFeatures();

    /*-------------------------------------------------
    Enable the dynamic payload feature bit
    -------------------------------------------------*/
    hwDriver->setRegisterBits( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_DPL );

    /*-------------------------------------------------
    Enable dynamic payload on all pipes. This requires that
    auto-acknowledge be enabled.
    -------------------------------------------------*/
    hwDriver->setRegisterBits( RF24::Hardware::REG_EN_AA, EN_AA::Mask );
    hwDriver->setRegisterBits( RF24::Hardware::REG_DYNPD, DYNPD::Mask );

    dynamicPayloadsEnabled = true;

#if defined( TRACK_REGISTER_STATES )
    registers.dynpd.update( this );
    registers.en_aa.update( this );
    registers.feature.update( this );
#endif
  }

  void Driver::disableDynamicPayloads()
  {
    /*-------------------------------------------------
    Disable for all pipes
    -------------------------------------------------*/
    if ( featuresActivated )
    {
      hwDriver->clrRegisterBits( RF24::Hardware::REG_DYNPD, DYNPD::Mask );
      hwDriver->clrRegisterBits( RF24::Hardware::REG_EN_AA, EN_AA::Mask );
      hwDriver->clrRegisterBits( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_DPL );

      dynamicPayloadsEnabled = false;

#if defined( TRACK_REGISTER_STATES )
      registers.dynpd.update( this );
      registers.en_aa.update( this );
      registers.feature.update( this );
#endif
    }
  }

  void Driver::enableDynamicAck()
  {
    activateFeatures();
    hwDriver->setRegisterBits( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_DYN_ACK );

#if defined( TRACK_REGISTER_STATES )
    registers.feature.update( this );
#endif
  }

  void Driver::disableDynamicAck()
  {
    if ( featuresActivated )
    {
      hwDriver->clrRegisterBits( RF24::Hardware::REG_FEATURE, RF24::Hardware::FEATURE_EN_DYN_ACK );

#if defined( TRACK_REGISTER_STATES )
      registers.feature.update( this );
#endif
    }
  }

  bool Driver::isPVariant()
  {
    return pVariant;
  }

  void Driver::setAutoAckAll( const bool enable )
  {
    if ( enable )
    {
      hwDriver->setRegisterBits( RF24::Hardware::REG_EN_AA, EN_AA::Mask );
    }
    else
    {
      hwDriver->clrRegisterBits( RF24::Hardware::REG_EN_AA, EN_AA::Mask );
    }

#if defined( TRACK_REGISTER_STATES )
    registers.en_aa.update( this );
#endif
  }

  bool Driver::setAutoAck( const uint8_t pipe, const bool enable, const bool validate )
  {
    bool returnVal = true;

    if ( pipe < MAX_NUM_PIPES )
    {
      uint8_t en_aa = hwDriver->readRegister( RF24::Hardware::REG_EN_AA );

      if ( enable )
      {
        en_aa |= 1u << pipe;
      }
      else
      {
        en_aa &= ~( 1u << pipe );
      }

      hwDriver->writeRegister( RF24::Hardware::REG_EN_AA, en_aa );

      if ( validate )
      {
        returnVal = ( hwDriver->readRegister( RF24::Hardware::REG_EN_AA ) == en_aa );
      }

#if defined( TRACK_REGISTER_STATES )
      if ( returnVal )
      {
        registers.en_aa = en_aa;
      }
#endif
    }
    else
    {
      returnVal = false;
    }

    return returnVal;
  }

  bool Driver::setPALevel( const PowerAmplitude level, const bool validate )
  {
    /*-------------------------------------------------
    Merge bits from level into setup according to a mask
    https://graphics.stanford.edu/~seander/bithacks.html#MaskedMerge
    -------------------------------------------------*/
    uint8_t setup = hwDriver->readRegister( RF24::Hardware::REG_RF_SETUP );
    setup ^= ( setup ^ static_cast<uint8_t>( level ) ) & RF_SETUP::RF_PWR_Msk;

    hwDriver->writeRegister( RF24::Hardware::REG_RF_SETUP, setup );

#if defined( TRACK_REGISTER_STATES )
    registers.rf_setup = setup;
#endif

    if ( validate )
    {
      return ( hwDriver->readRegister( RF24::Hardware::REG_RF_SETUP ) == setup );
    }

    return true;
  }

  PowerAmplitude Driver::getPALevel()
  {
    uint8_t setup = hwDriver->readRegister( RF24::Hardware::REG_RF_SETUP );

#if defined( TRACK_REGISTER_STATES )
    registers.rf_setup = setup;
#endif

    return static_cast<PowerAmplitude>( ( setup & RF_SETUP::RF_PWR ) >> 1 );
  }

  bool Driver::setDataRate( const DataRate speed )
  {
    uint8_t setup = hwDriver->readRegister( RF24::Hardware::REG_RF_SETUP );

    switch ( speed )
    {
      case RF24::Hardware::DataRate::DR_250KBPS:
        if ( pVariant )
        {
          setup |= RF_SETUP::RF_DR_LOW;
          setup &= ~RF_SETUP::RF_DR_HIGH;
        }
        else
        {
          return false;
        }
        break;

      case RF24::Hardware::DataRate::DR_1MBPS:
        setup &= ~( RF_SETUP::RF_DR_HIGH | RF_SETUP::RF_DR_LOW );
        break;

      case RF24::Hardware::DataRate::DR_2MBPS:
        setup &= ~RF_SETUP::RF_DR_LOW;
        setup |= RF_SETUP::RF_DR_HIGH;
        break;

      default:
        break;
    }

    hwDriver->writeRegister( RF24::Hardware::REG_RF_SETUP, setup );
    auto result = hwDriver->readRegister( RF24::Hardware::REG_RF_SETUP );

#if defined( TRACK_REGISTER_STATES )
    registers.rf_setup = result;
#endif

    return ( result == setup );
  }

  DataRate Driver::getDataRate()
  {
    uint8_t reg = hwDriver->readRegister( RF24::Hardware::REG_RF_SETUP );

#if defined( TRACK_REGISTER_STATES )
    registers.rf_setup = reg;
#endif

    return static_cast<DataRate>( reg & ( RF_SETUP::RF_DR_HIGH | RF_SETUP::RF_DR_LOW ) );
  }

  

  void Driver::maskIRQ( const bool tx_ok, const bool tx_fail, const bool rx_ready )
  {
    uint8_t config = hwDriver->readRegister( RF24::Hardware::REG_CONFIG );

    config &= ~( RF24::Hardware::CONFIG_MASK_MAX_RT | RF24::Hardware::CONFIG_MASK_TX_DS | RF24::Hardware::CONFIG_MASK_RX_DR );
    config |= ( tx_fail << RF24::Hardware::CONFIG_MASK_MAX_RT_Pos ) | ( tx_ok << RF24::Hardware::CONFIG_MASK_TX_DS_Pos )
              | ( rx_ready << RF24::Hardware::CONFIG_MASK_RX_DR_Pos );

    hwDriver->writeRegister( RF24::Hardware::REG_CONFIG, config );

#if defined( TRACK_REGISTER_STATES )
    registers.config = config;
#endif
  }

 
  bool Driver::_registerIsBitmaskSet( const uint8_t reg, const uint8_t bitmask )
  {
    return ( hwDriver->readRegister( reg ) & bitmask ) == bitmask;
  }

  bool Driver::_registerIsAnySet( const uint8_t reg, const uint8_t bitmask )
  {
    return hwDriver->readRegister( reg ) & bitmask;
  }
}  // namespace RF24Phy
