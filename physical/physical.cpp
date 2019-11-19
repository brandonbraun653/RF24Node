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
#include <algorithm>
#include <cstring>
#include <limits>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/hardware/driver.hpp>
#include <RF24Node/hardware/register.hpp>
#include <RF24Node/physical/physical.hpp>

namespace RF24::Physical
{
  Driver::Driver()
  {
    /*-------------------------------------------------
    Initialize class variables
    -------------------------------------------------*/
    mInitialized            = false;
    mPlusVariant            = false;
    mDynamicPayloadsEnabled = false;
    mCurrentlyListening     = false;
    mListeningPaused        = false;
    mAddressWidth           = std::numeric_limits<size_t>::min();
    mPayloadSize            = std::numeric_limits<size_t>::min();
    mCachedPipe0RXAddress   = std::numeric_limits<uint64_t>::max();
    mCurrentMode            = RF24::Hardware::Mode::MAX_MODES;
    mHWDriver               = nullptr;
    mFailureCode            = Chimera::CommonStatusCodes::NOT_INITIALIZED;
  }

  Driver::~Driver()
  {

  }

  Chimera::Status_t Driver::initialize()
  {
    mInitialized = false;

    /*-------------------------------------------------
    Check we can talk to the device and reset it back to
    some known initial condition.
    -------------------------------------------------*/
    if ( isConnected() != Chimera::CommonStatusCodes::OK )
    {
      mFailureCode = Chimera::CommonStatusCodes::NOT_FOUND;
      return mFailureCode;
    }

    /*-------------------------------------------------
    Enable 16-bit CRC
    -------------------------------------------------*/
    mHWDriver->setCRCLength( RF24::Hardware::CRCLength::CRC_16 );

    /*-------------------------------------------------
    Set 1500uS timeout and 3 retry attempts. Don't lower
    or the 250KBS mode will break.
    -------------------------------------------------*/
    setRetries( RF24::Hardware::AutoRetransmitDelay::w1500uS, 3 );

    /*-------------------------------------------------
    If we can set the data rate to 250kbps, we have a 
    plus variant of the chip
    -------------------------------------------------*/
    mPlusVariant = ( setDataRate( RF24::Hardware::DataRate::DR_250KBPS ) == Chimera::CommonStatusCodes::OK );

    /*-------------------------------------------------
    Set data rate to the slowest, most reliable speed 
    supported by all NRF24L01 chip variants
    -------------------------------------------------*/
    setDataRate( RF24::Hardware::DataRate::DR_1MBPS );

    /*------------------------------------------------
    Default to the 5 byte wide addressing scheme
    ------------------------------------------------*/
    mHWDriver->setAddressWidth( RF24::Hardware::AddressWidth::AW_5Byte );

    /*-------------------------------------------------
    Set the default channel to a value that likely won't 
    congest the spectrum.
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
    mHWDriver->toggleRFPower( true );

    mInitialized = true;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::isInitialized()
  {
    return mInitialized;
  }

  Chimera::Status_t Driver::isConnected()
  {
    /*------------------------------------------------
    Grab the old settings for the register
    ------------------------------------------------*/
    Reg8_t old_setup     = mHWDriver->readRegister( RF24::Hardware::REG_SETUP_AW );
    Reg8_t illegal_setup = 0u;

    /*------------------------------------------------
    Write some new settings and record their value
    ------------------------------------------------*/
    mHWDriver->writeRegister( RF24::Hardware::REG_SETUP_AW, illegal_setup );
    Reg8_t new_setup = mHWDriver->readRegister( RF24::Hardware::REG_SETUP_AW );

    /*------------------------------------------------
    We are connected if the recorded settings match what we thought
    was set. Reset back to the old settings before exiting.
    ------------------------------------------------*/
    if ( new_setup == illegal_setup )
    {
      mHWDriver->writeRegister( RF24::Hardware::REG_SETUP_AW, old_setup );
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      mFailureCode = Chimera::CommonStatusCodes::NOT_FOUND;
      return mFailureCode;
    }
  }

  Chimera::Status_t Driver::setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count, const bool validate )
  {
    bool returnVal    = true;
    Reg8_t ard        = ( static_cast<Reg8_t>( delay ) & 0x0F ) << RF24::Hardware::SETUP_RETR_ARD_Pos;
    Reg8_t arc        = ( count & 0x0F ) << RF24::Hardware::SETUP_RETR_ARC_Pos;
    Reg8_t setup_retr = ard | arc;

    mHWDriver->writeRegister( RF24::Hardware::REG_SETUP_RETR, setup_retr );

    if ( validate && ( mHWDriver->readRegister( RF24::Hardware::REG_SETUP_RETR ) != setup_retr ))
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::setChannel( const size_t channel, const bool validate )
  {
    auto maskedChannel = channel & RF24::Hardware::RF_CH_Mask;
    mHWDriver->writeRegister( RF24::Hardware::REG_RF_CH, maskedChannel );

    if ( validate && ( mHWDriver->readRegister( RF24::Hardware::REG_RF_CH ) != maskedChannel ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  size_t Driver::getChannel()
  {
    return static_cast<size_t>( mHWDriver->readRegister( RF24::Hardware::REG_RF_CH ) );
  }

  Chimera::Status_t Driver::setStaticPayloadSize( const size_t size )
  {
    mPayloadSize = std::min( size, static_cast<size_t>( 32u ) );
    return Chimera::CommonStatusCodes::OK;
  }

  size_t Driver::getStaticPayloadSize()
  {
    return mPayloadSize;
  }

  size_t Driver::getDynamicPayloadSize()
  {
    auto result = RF24::Hardware::MIN_PAYLOAD_WIDTH;

    if ( mDynamicPayloadsEnabled )
    {
      uint8_t temp = 0u;
      mHWDriver->readCMD( RF24::Hardware::CMD_R_RX_PL_WID, &temp, 1u );
      result = static_cast<size_t>( temp );

      /*-------------------------------------------------
      The dynamic payload size should never be greater than
      the max payload width. If it is, reset the RX queues.
      -------------------------------------------------*/
      if ( result > RF24::Hardware::MAX_PAYLOAD_WIDTH )
      {
        flushRX();
        Chimera::delayMilliseconds( 2 );
        return Chimera::CommonStatusCodes::FAILED_READ;
      }
    }

    return result;
  }

  Chimera::Status_t Driver::startListening()
  {
    using namespace RF24::Hardware;

    if ( mCurrentlyListening )
    {
      return Chimera::CommonStatusCodes::OK;
    }

    /*-------------------------------------------------
    Transition the module back to Stanby-1 mode
    -------------------------------------------------*/
    toggleChipEnablePin( false );

    /*-------------------------------------------------
    If we are auto-acknowledging RX packets with a payload,
    make sure the TX FIFO is clean so we don't accidently 
    transmit data on the next transition back to TX mode.
    -------------------------------------------------*/
    if ( _registerIsBitmaskSet( REG_FEATURE, FEATURE_EN_ACK_PAY ) )
    {
      flushTX();
    }

    /*-------------------------------------------------
    Clear interrupt flags and transition to RX mode by 
    setting PRIM_RX=1 and CE=1. Wait the required ~130uS
    RX settling time needed to get into RX mode.
    -------------------------------------------------*/
    mHWDriver->setRegisterBits( REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT );
    mHWDriver->setRegisterBits( REG_CONFIG, CONFIG_PRIM_RX );
    toggleChipEnablePin( true );
    Chimera::delayMilliseconds( 1 );
    mCurrentMode = MODE_RX;

    /*------------------------------------------------
    If we clobbered the old pipe 0 listening address so 
    we could transmit something, restore it.
    ------------------------------------------------*/
    if ( mCachedPipe0RXAddress )
    {
      openReadPipe( PIPE_NUM_0, mCachedPipe0RXAddress );
    }

    mCurrentlyListening = true;
    mListeningPaused    = false;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::pauseListening()
  {
    /*-------------------------------------------------
    According to the state transition diagram, the module is
    in RX mode when PRIM_RX=1 and CE=1. By clearing CE=0, we
    can transition the module to Standby-1 mode.
    -------------------------------------------------*/
    if ( mCurrentlyListening && !mListeningPaused )
    {
      toggleChipEnablePin( false );
      mListeningPaused    = true;
      mCurrentlyListening = false;
      mCurrentMode        = RF24::Hardware::MODE_STANDBY_I;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::resumeListening()
  {
    /*-------------------------------------------------
    According to the state transition diagram, the module is
    in Standby-1 mode when PRIM_RX=1 and CE=0. By setting
    CE=1, we can transition the module back to RX mode.
    -------------------------------------------------*/
    if ( mListeningPaused && !mCurrentlyListening )
    {
      toggleChipEnablePin( true );
      mListeningPaused    = false;
      mCurrentlyListening = true;

      /* The transition requires an RX settling period of ~130uS */
      Chimera::delayMilliseconds( 1 );
      mCurrentMode = RF24::Hardware::MODE_RX;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::stopListening()
  {
    using namespace RF24::Hardware;

    if ( mCurrentlyListening || mListeningPaused )
    {
      /*-------------------------------------------------
      Set the chip into standby mode I
      -------------------------------------------------*/
      toggleChipEnablePin( false );
      mHWDriver->clrRegisterBits( REG_CONFIG, CONFIG_PRIM_RX );
      mCurrentMode = MODE_STANDBY_I;

      /*-------------------------------------------------
      If we are auto-acknowledging RX packets with a payload, make sure the TX FIFO is clean so
      we don't accidentally transmit data the next time we write chipEnable high.
      -------------------------------------------------*/
      if ( _registerIsBitmaskSet( REG_FEATURE, FEATURE_EN_ACK_PAY ) )
      {
        flushTX();
      }

      mCurrentlyListening = false;
      mListeningPaused    = false;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::openWritePipe( const uint64_t address )
  {
    using namespace RF24::Hardware;

    /*-------------------------------------------------
    Set pipe 0 RX address == TX address. This allows the 
    reception of an ACK packet from the node at the TX
    address. Cache the currently configured RX address.
    -------------------------------------------------*/
    mCachedPipe0RXAddress = 0u;
    mHWDriver->readRegister( REG_RX_ADDR_P0, reinterpret_cast<uint8_t *const>( mCachedPipe0RXAddress ), MAX_ADDRESS_WIDTH );
    mHWDriver->writeRegister( REG_RX_ADDR_P0, reinterpret_cast<const uint8_t *>( &address ), MAX_ADDRESS_WIDTH );

    /*-------------------------------------------------
    Make sure we transmit back to the same address we expect to receive from
    -------------------------------------------------*/
    mHWDriver->writeRegister( REG_TX_ADDR, reinterpret_cast<const uint8_t *>( &address ), MAX_ADDRESS_WIDTH );

    /*-------------------------------------------------
    Set a static payload length for all receptions on pipe 0. There must also be
    an equal number of bytes clocked into the TX_FIFO when data is transmitted out.

    This setting only takes effect if static payloads are configured. Otherwise 
    the dynamic payload setting ignores this.
    -------------------------------------------------*/
    mHWDriver->writeRegister( REG_RX_PW_P0, static_cast<uint8_t>( mPayloadSize ) );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::closeWritePipe()
  {
    using namespace RF24::Hardware;
    /*-------------------------------------------------
    Clober the current TX addressing data
    -------------------------------------------------*/
    uint64_t address = 0u;
    mHWDriver->writeRegister( REG_TX_ADDR, reinterpret_cast<const uint8_t *>( &address ), MAX_ADDRESS_WIDTH );

    /*-------------------------------------------------
    If it's safe, clobber the RX Pipe 0 address too
    -------------------------------------------------*/
    if ( ( mCurrentMode == MODE_TX ) || ( !mCurrentlyListening && !mListeningPaused ) )
    {
      mHWDriver->writeRegister( REG_RX_ADDR_P0, reinterpret_cast<const uint8_t *>( &address ), MAX_ADDRESS_WIDTH );
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::openReadPipe( const RF24::Hardware::PipeNumber_t pipe, const uint64_t address, const bool validate )
  {
    using namespace RF24::Hardware;

    /*------------------------------------------------
    Make sure the pipe is ok
    ------------------------------------------------*/
    if ( pipe < PIPE_NUM_ALL )
    {
      mFailureCode = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      return mFailureCode;
    }

    /*-------------------------------------------------
    Assign the address for the pipe to listen against
    -------------------------------------------------*/
    if ( ( pipe == PIPE_NUM_0 ) || ( pipe == PIPE_NUM_1 ) )
    {
      /*-------------------------------------------------
      Write only as many address bytes as were set in SETUP_AW
      -------------------------------------------------*/
      auto addressWidth = mHWDriver->getAddressWidth();
      mHWDriver->writeRegister( rxPipeAddressRegister[ pipe ], &address, addressWidth );

      /*------------------------------------------------
      Save the pipe 0 address because it can be clobbered by openWritePipe() and
      will need to be restored later when we start listening again.
      ------------------------------------------------*/
      if ( pipe == PIPE_NUM_0 )
      {
        memcpy( &mCachedPipe0RXAddress, &address, addressWidth );
      }

      /*------------------------------------------------
      Optionally validate the write
      ------------------------------------------------*/
      if ( validate )
      {
        uint64_t setVal = 0u;
        mHWDriver->readRegister( rxPipeAddressRegister[ pipe ], &setVal, addressWidth );

        if ( setVal != ( address & 0xFFFFFFFFFFu ) )
        {
          mFailureCode = Chimera::CommonStatusCodes::FAIL;
          return mFailureCode;
        }
      }
    }
    else
    {
      /*------------------------------------------------
      These pipes only need the first bytes assigned as 
      they take the rest of their address from PIPE_NUM_0.
      ------------------------------------------------*/
      mHWDriver->writeRegister( rxPipeAddressRegister[ pipe ], &address, 1u );

      /*------------------------------------------------
      Optionally validate the write
      ------------------------------------------------*/
      if ( validate )
      {
        uint8_t setVal = 0u;
        mHWDriver->readRegister( rxPipeAddressRegister[ pipe ], &setVal, 1u );

        if ( setVal != ( address & 0xFF ) )
        {
          mFailureCode = Chimera::CommonStatusCodes::FAIL;
          return mFailureCode;
        }
      }
    }

    /*-------------------------------------------------
    Let the pipe know how wide the payload will be, then turn it on
    -------------------------------------------------*/
    mHWDriver->writeRegister( rxPipePayloadWidthRegister[ pipe ], mPayloadSize );
    mHWDriver->setRegisterBits( REG_EN_RXADDR, rxPipeEnableBitField[ pipe ] );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::closeReadPipe( const RF24::Hardware::PipeNumber_t pipe )
  {
    using namespace RF24::Hardware;

    if ( pipe < MAX_NUM_PIPES )
    {
      Reg8_t rxaddrVal = mHWDriver->readRegister( REG_EN_RXADDR ) & ( ~rxPipeEnableBitField[ pipe ] );
      mHWDriver->writeRegister( REG_EN_RXADDR, rxaddrVal );

      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
  }

  RF24::Hardware::PipeNumber_t Driver::payloadAvailable()
  {
    using namespace RF24::Hardware;

    PipeNumber_t pipeWithPayload = PIPE_NUM_MAX;

    /*-------------------------------------------------
    Figure out which pipe has data available, as reported
    by the device's status register
    -------------------------------------------------*/
    if ( !mHWDriver->rxFifoEmpty() )
    {
      pipeWithPayload = static_cast<PipeNumber_t>( ( mHWDriver->getStatus() >> STATUS_RX_P_NO_Pos ) & STATUS_RX_P_NO_Wid );
    }

    return pipeWithPayload;
  }

  size_t Driver::getPayloadSize( const RF24::Hardware::PipeNumber_t pipe )
  {
    using namespace RF24::Hardware;

    if ( pipe < MAX_NUM_PIPES )
    {
      /* The mask is the same across all pipes */
      auto payloadWidth = mHWDriver->readRegister( rxPipePayloadWidthRegister[ pipe ] );
      return payloadWidth & RX_PW_P0_Mask;
    }

    /* A value of zero indicates the pipe is not used or no data is available */
    return 0u;
  }

  Chimera::Status_t Driver::readPayload( void *const buffer, size_t len )
  {
    auto status = mHWDriver->readPayload( buffer, len );
    return Chimera::CommonStatusCodes::OK;
  }






  void Driver::toggleChipEnablePin( const bool state )
  {
    mHWDriver->toggleCE( state );
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
    if ( mCurrentlyListening )
    {
      mFailureCode = RF24::Hardware::FailureCode::RADIO_IN_RX_MODE;
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
        mHWDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT );
        mFailureCode = RF24::Hardware::FailureCode::MAX_RETRY_TIMEOUT;
        return false;
      }

      /*------------------------------------------------
      Make sure we aren't waiting around forever
      ------------------------------------------------*/
      if ( ( Chimera::millis() - startTime ) > DFLT_TIMEOUT_MS )
      {
        mFailureCode = RF24::Hardware::FailureCode::TX_FIFO_FULL_TIMEOUT;
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
      mCurrentMode = Mode::TX;
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
        mHWDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT );
        toggleChipEnablePin( false );
        flushTX();

        mCurrentMode = Mode::STANDBY_I;
        return false;
      }
    }

    /*------------------------------------------------
    Sends the chip back to standby mode now that the FIFO is empty
    ------------------------------------------------*/
    toggleChipEnablePin( false );
    mCurrentMode = Mode::STANDBY_I;
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
    if ( mCurrentlyListening )
    {
      mFailureCode = RF24::Hardware::FailureCode::RADIO_IN_RX_MODE;
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
        mHWDriver->setRegisterBits( RF24::Hardware::REG_STATUS, RF24::Hardware::STATUS_MAX_RT );

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
        mFailureCode     = RF24::Hardware::FailureCode::TX_FIFO_EMPTY_TIMEOUT;
        mCurrentMode = Mode::STANDBY_I;
        return false;
      }
    }

    /*------------------------------------------------
    Transition back to Standby Mode I
    ------------------------------------------------*/
    toggleChipEnablePin( false );
    mCurrentMode = Mode::STANDBY_I;
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
    uint8_t reg = mHWDriver->readRegister( RF24::Hardware::REG_FIFO_STATUS );

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

  uint8_t Driver::flushTX()
  {
    return _writeCMD( RF24::Hardware::CMD_FLUSH_TX );
  }

  uint8_t Driver::flushRX()
  {
    return _writeCMD( RF24::Hardware::CMD_FLUSH_RX );
  }

  Chimera::Status_t Driver::setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate )
  {
    using namespace RF24::Hardware;

    /*-------------------------------------------------
    Merge bits from level into setup according to a mask
    https://graphics.stanford.edu/~seander/bithacks.html#MaskedMerge
    -------------------------------------------------*/
    uint8_t setup = mHWDriver->readRegister( REG_RF_SETUP );
    setup ^= ( setup ^ static_cast<uint8_t>( level ) ) & RF_SETUP_RF_PWR_Msk;

    mHWDriver->writeRegister( REG_RF_SETUP, setup );

    if ( validate )
    {
      return ( mHWDriver->readRegister( REG_RF_SETUP ) == setup );
    }

    return true;
  }

  RF24::Hardware::PowerAmplitude Driver::getPALevel()
  {
    using namespace RF24::Hardware;

    uint8_t setup = mHWDriver->readRegister( REG_RF_SETUP );
    return static_cast<PowerAmplitude>( ( setup & RF_SETUP_RF_PWR ) >> 1 );
  }

  Chimera::Status_t Driver::setDataRate( const RF24::Hardware::DataRate speed )
  {
    using namespace RF24::Hardware;

    uint8_t setup = mHWDriver->readRegister( REG_RF_SETUP );

    switch ( speed )
    {
      case DataRate::DR_250KBPS:
        if ( mPlusVariant )
        {
          setup |= RF_SETUP_RF_DR_LOW;
          setup &= ~RF_SETUP_RF_DR_HIGH;
        }
        else
        {
          return false;
        }
        break;

      case DataRate::DR_1MBPS:
        setup &= ~( RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW );
        break;

      case DataRate::DR_2MBPS:
        setup &= ~RF_SETUP_RF_DR_LOW;
        setup |= RF_SETUP_RF_DR_HIGH;
        break;

      default:
        break;
    }

    mHWDriver->writeRegister( REG_RF_SETUP, setup );
    auto result = mHWDriver->readRegister( REG_RF_SETUP );

    return Chimera::CommonStatusCodes::FAIL;
  }

  RF24::Hardware::DataRate Driver::getDataRate()
  {
    using namespace RF24::Hardware;

    uint8_t reg = mHWDriver->readRegister( REG_RF_SETUP );
    return static_cast<DataRate>( reg & ( RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW ) );
  }

  bool Driver::_registerIsBitmaskSet( const uint8_t reg, const uint8_t bitmask )
  {
    return ( mHWDriver->readRegister( reg ) & bitmask ) == bitmask;
  }

  bool Driver::_registerIsAnySet( const uint8_t reg, const uint8_t bitmask )
  {
    return mHWDriver->readRegister( reg ) & bitmask;
  }
}  // namespace RF24Phy
