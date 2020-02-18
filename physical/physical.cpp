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

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>
#include <uLog/sinks/sink_cout.hpp>

#if !defined( RF24_SIMULATOR )

namespace RF24::Physical
{
  HardwareDriver::HardwareDriver()
  {
    /*-------------------------------------------------
    Initialize class variables
    -------------------------------------------------*/
    mInitialized          = false;
    mPlusVariant          = false;
    mCurrentlyListening   = false;
    mListeningPaused      = false;
    mAddressWidth         = std::numeric_limits<uint8_t>::min();
    mPayloadSize          = std::numeric_limits<uint8_t>::min();
    mCachedPipe0RXAddress = std::numeric_limits<uint64_t>::min();
    mCurrentMode          = RF24::Hardware::Mode::MAX_MODES;
    mHWDriver             = nullptr;
    mFailureCode          = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    logger                = nullptr;
  }

  HardwareDriver::~HardwareDriver()
  {
  }

  Chimera::Status_t HardwareDriver::attachLogger( uLog::SinkHandle sink )
  {
    logger = sink;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::initialize( const RF24::Physical::Config &cfg )
  {
    mInitialized = false;

    /*------------------------------------------------
    Initialize the low level hardware driver
    ------------------------------------------------*/
    if ( !mHWDriver ) 
    {
      mHWDriver = std::make_shared<RF24::Hardware::Driver>();
    }

    if ( mHWDriver->initialize( cfg.spiConfig, cfg.chipEnableConfig ) != Chimera::CommonStatusCodes::OK )
    {
      mFailureCode = Chimera::CommonStatusCodes::FAILED_INIT;
      return mFailureCode;
    }

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
    setRetries( RF24::Hardware::AutoRetransmitDelay::ART_DELAY_1500uS, 3 );

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

    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_0, RF24::Hardware::MAX_PAYLOAD_WIDTH );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_1, RF24::Hardware::MAX_PAYLOAD_WIDTH );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_2, RF24::Hardware::MAX_PAYLOAD_WIDTH );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_3, RF24::Hardware::MAX_PAYLOAD_WIDTH );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_4, RF24::Hardware::MAX_PAYLOAD_WIDTH );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_5, RF24::Hardware::MAX_PAYLOAD_WIDTH );

    /*-------------------------------------------------
    Set the default channel to a value that likely won't
    congest the spectrum.
    -------------------------------------------------*/
    setChannel( 76 );

    /*------------------------------------------------
    Enable dynamic payloads so that each transfer can handle
    a variable packet length. No need for fixed lengths.
    ------------------------------------------------*/
    mHWDriver->toggleFeatures( true );
    mHWDriver->toggleDynamicPayloads( false );

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

  Chimera::Status_t HardwareDriver::isInitialized()
  {
    return mInitialized;
  }

  Chimera::Status_t HardwareDriver::isConnected()
  {
    using namespace RF24::Hardware;

    /*------------------------------------------------
    Grab the old settings for the register
    ------------------------------------------------*/
    Reg8_t old_setup     = mHWDriver->readRegister( REG_SETUP_AW );
    Reg8_t illegal_setup = 0u;

    /*------------------------------------------------
    Write some new settings and record their value
    ------------------------------------------------*/
    mHWDriver->writeRegister( REG_SETUP_AW, illegal_setup );
    Reg8_t new_setup = mHWDriver->readRegister( REG_SETUP_AW );

    /*------------------------------------------------
    We are connected if the recorded settings match what we thought
    was set. Reset back to the old settings before exiting.
    ------------------------------------------------*/
    if ( new_setup == illegal_setup )
    {
      mHWDriver->writeRegister( REG_SETUP_AW, old_setup );
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      mFailureCode = Chimera::CommonStatusCodes::NOT_FOUND;
      return mFailureCode;
    }
  }

  Chimera::Status_t HardwareDriver::setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count,
                                                const bool validate )
  {
    bool returnVal    = true;
    Reg8_t ard        = ( static_cast<Reg8_t>( delay ) & 0x0F ) << RF24::Hardware::SETUP_RETR_ARD_Pos;
    Reg8_t arc        = ( count & 0x0F ) << RF24::Hardware::SETUP_RETR_ARC_Pos;
    Reg8_t setup_retr = ard | arc;

    mHWDriver->writeRegister( RF24::Hardware::REG_SETUP_RETR, setup_retr );

    if ( validate && ( mHWDriver->readRegister( RF24::Hardware::REG_SETUP_RETR ) != setup_retr ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::setChannel( const size_t channel, const bool validate )
  {
    auto maskedChannel = static_cast<Reg8_t>( channel & RF24::Hardware::RF_CH_Mask );
    mHWDriver->writeRegister( RF24::Hardware::REG_RF_CH, maskedChannel );

    if ( validate && ( mHWDriver->readRegister( RF24::Hardware::REG_RF_CH ) != maskedChannel ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  size_t HardwareDriver::getChannel()
  {
    return static_cast<size_t>( mHWDriver->readRegister( RF24::Hardware::REG_RF_CH ) );
  }

  Chimera::Status_t HardwareDriver::setStaticPayloadSize( const size_t size )
  {
    mPayloadSize = static_cast<Reg8_t>( std::min( size, RF24::Hardware::MAX_PAYLOAD_WIDTH ) );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_0, mPayloadSize );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_1, mPayloadSize );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_2, mPayloadSize );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_3, mPayloadSize );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_4, mPayloadSize );
    mHWDriver->setStaticPayloadWidth( RF24::Hardware::PIPE_NUM_5, mPayloadSize );
    return Chimera::CommonStatusCodes::OK;
  }

  size_t HardwareDriver::getStaticPayloadSize()
  {
    return mPayloadSize;
  }

  size_t HardwareDriver::getDynamicPayloadSize()
  {
    return mHWDriver->getDynamicPayloadSize();
  }

  Chimera::Status_t HardwareDriver::startListening()
  {
    using namespace RF24::Hardware;

    if ( mCurrentlyListening )
    {
      return Chimera::CommonStatusCodes::OK;
    }

    /*-------------------------------------------------
    Transition the module back to Standby-1 mode
    -------------------------------------------------*/
    toggleChipEnablePin( false );

    /*-------------------------------------------------
    If we are auto-acknowledging RX packets with a payload,
    make sure the TX FIFO is clean so we don't accidentally
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
    mHWDriver->setRegisterBits( REG_STATUS, ( STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT ), false );
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

  Chimera::Status_t HardwareDriver::pauseListening()
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

  Chimera::Status_t HardwareDriver::resumeListening()
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

  Chimera::Status_t HardwareDriver::stopListening()
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

  Chimera::Status_t HardwareDriver::openWritePipe( const uint64_t address )
  {
    using namespace RF24::Hardware;

    /*-------------------------------------------------
    Set pipe 0 RX address == TX address. This allows the
    reception of an ACK packet from the node at the TX
    address. Cache the currently configured RX address.
    -------------------------------------------------*/
    mCachedPipe0RXAddress = 0u;
    mHWDriver->readRegister( REG_RX_ADDR_P0, &mCachedPipe0RXAddress, MAX_ADDRESS_WIDTH );
    mHWDriver->writeRegister( REG_RX_ADDR_P0, &address, MAX_ADDRESS_WIDTH );

    /*-------------------------------------------------
    Make sure we transmit back to the same address we expect to receive from
    -------------------------------------------------*/
    mHWDriver->writeRegister( REG_TX_ADDR, &address, MAX_ADDRESS_WIDTH );

    /*-------------------------------------------------
    Set a static payload length for all receptions on pipe 0. There must also be
    an equal number of bytes clocked into the TX_FIFO when data is transmitted out.

    This setting only takes effect if static payloads are configured. Otherwise
    the dynamic payload setting ignores this.
    -------------------------------------------------*/
    mHWDriver->writeRegister( REG_RX_PW_P0, MAX_PAYLOAD_WIDTH );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::closeWritePipe()
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

  Chimera::Status_t HardwareDriver::openReadPipe( const RF24::Hardware::PipeNumber pipe, const uint64_t address,
                                                  const bool validate )
  {
    using namespace RF24::Hardware;

    /*------------------------------------------------
    Make sure the pipe is ok
    ------------------------------------------------*/
    if ( pipe > PIPE_NUM_ALL )
    {
      mFailureCode = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      return mFailureCode;
    }

    /*-------------------------------------------------
    Assign the address for the pipe to listen against
    -------------------------------------------------*/
    size_t addressBytes  = 0u;
    uint64_t addressMask = 0u;

    if ( ( pipe == PIPE_NUM_0 ) || ( pipe == PIPE_NUM_1 ) )
    {
      /*-------------------------------------------------
      Write only as many address bytes as were set in SETUP_AW
      -------------------------------------------------*/
      addressBytes = mHWDriver->getAddressWidthAsBytes();
      addressMask  = 0xFFFFFFFFFFu;
      mHWDriver->writeRegister( rxPipeAddressRegister[ pipe ], &address, addressBytes );

      /*------------------------------------------------
      Save the pipe 0 address because it can be clobbered by openWritePipe() and
      will need to be restored later when we start listening again.
      ------------------------------------------------*/
      if ( pipe == PIPE_NUM_0 )
      {
        memcpy( &mCachedPipe0RXAddress, &address, addressBytes );
      }
    }
    else
    {
      addressBytes = 1u;
      addressMask  = 0xFFu;

      /*------------------------------------------------
      These pipes only need the first bytes assigned as
      they take the rest of their address from PIPE_NUM_0.
      ------------------------------------------------*/
      mHWDriver->writeRegister( rxPipeAddressRegister[ pipe ], &address, addressBytes );
    }

    /*------------------------------------------------
    Optionally validate the write
    ------------------------------------------------*/
    if ( validate )
    {
      uint64_t writtenAddress = 0u;
      mHWDriver->readRegister( rxPipeAddressRegister[ pipe ], &writtenAddress, addressBytes );

      if ( writtenAddress != ( address & addressMask ) )
      {
        mFailureCode = Chimera::CommonStatusCodes::FAIL;
        return mFailureCode;
      }
    }


    /*-------------------------------------------------
    Let the pipe know how wide the payload will be, then turn it on
    -------------------------------------------------*/
    mHWDriver->writeRegister( rxPipePayloadWidthRegister[ pipe ], mPayloadSize );
    mHWDriver->setRegisterBits( REG_EN_RXADDR, rxPipeEnableBitField[ pipe ] );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::closeReadPipe( const RF24::Hardware::PipeNumber pipe )
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

  RF24::Hardware::PipeNumber HardwareDriver::payloadAvailable()
  {
    using namespace RF24::Hardware;

    PipeNumber pipeWithPayload = PIPE_NUM_MAX;

    /*-------------------------------------------------
    Figure out which pipe has data available, as reported
    by the device's status register
    -------------------------------------------------*/
    if ( !mHWDriver->rxFifoEmpty() )
    {
      pipeWithPayload = static_cast<PipeNumber>( ( mHWDriver->getStatus() >> STATUS_RX_P_NO_Pos ) & STATUS_RX_P_NO_Wid );
    }

    return pipeWithPayload;
  }

  size_t HardwareDriver::getPayloadSize( const RF24::Hardware::PipeNumber pipe )
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

  Chimera::Status_t HardwareDriver::readPayload( RF24::Network::Frame::Buffer &buffer, const size_t length )
  {
    using namespace RF24::Hardware;

    auto status = mHWDriver->readPayload( buffer.data(), buffer.size(), length );
    IF_SERIAL_DEBUG( logger->flog(uLog::Level::LVL_INFO, "%d-PHY: RX packet of length [%d]\n", Chimera::millis(), length ); );

    /*------------------------------------------------
    Clear the ISR flag bits by setting them to 1
    ------------------------------------------------*/
    mHWDriver->writeRegister( REG_STATUS, ( STATUS_RX_DR | STATUS_MAX_RT | STATUS_TX_DS ), false );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::immediateWrite( const RF24::Network::Frame::Buffer &buffer, const size_t length )
  {
    using namespace RF24::Hardware;

    /*------------------------------------------------
    Don't clobber the RX if we are listening
    ------------------------------------------------*/
    if ( mCurrentlyListening )
    {
      mFailureCode = Chimera::CommonStatusCodes::NOT_READY;
      return mFailureCode;
    }

    /*-------------------------------------------------
    Wait for the FIFO to have room for one more packet
    -------------------------------------------------*/
    auto startTime = Chimera::millis();

    while ( mHWDriver->txFifoFull() )
    {
      /*-------------------------------------------------
      If max retries hit from a previous transmission, we screwed up
      -------------------------------------------------*/
      if ( _registerIsBitmaskSet( REG_STATUS, STATUS_MAX_RT ) )
      {
        mHWDriver->setRegisterBits( REG_STATUS, STATUS_MAX_RT );
        mFailureCode = Chimera::CommonStatusCodes::TIMEOUT;
        return mFailureCode;
      }

      /*------------------------------------------------
      Make sure we aren't waiting around forever
      ------------------------------------------------*/
      if ( ( Chimera::millis() - startTime ) > DFLT_TIMEOUT_MS )
      {
        mFailureCode = Chimera::CommonStatusCodes::TIMEOUT;
        return mFailureCode;
      }

      Chimera::delayMilliseconds( MIN_TIMEOUT_MS );
    }

    /*------------------------------------------------
    We're free! Load the data into the FIFO and kick off the transfer
    ------------------------------------------------*/
    IF_SERIAL_DEBUG( logger->flog(uLog::Level::LVL_INFO, "%d-PHY: TX packet of length [%d]\n", Chimera::millis(), length ); );
    return startFastWrite( buffer.data(), length, false, false );
  }

  /******************************************************************************************************************** 
   *  @copydoc HardwareDriver::startFastWrite(const void*const, const size_t, const bool, const bool)
   *  
   *  Copies over the buffer of data into the TX FIFO of the RF24 device. IF desired, will optionally enable
   *  an ACK request from the receiver just for this transfer and then start the transfer. This is a pretty low level 
   *  function that requires a fair amount of setup before use.
   *  
   *******************************************************************************************************************/
  Chimera::Status_t HardwareDriver::startFastWrite( const void *const buffer, const size_t len, const bool multicast,
                                                    const bool startTX )
  {
    using namespace RF24::Hardware;

    uint8_t payloadType = 0u;

    if ( multicast )
    {
      /*-------------------------------------------------
      Transmit one packet without waiting for an ACK from the RX device. In order for
      this to work, the Features register has to be enabled and EN_DYN_ACK set.
      -------------------------------------------------*/
      mHWDriver->toggleDynamicAck( true );
      payloadType = CMD_W_TX_PAYLOAD_NO_ACK;
    }
    else
    {
      /*-------------------------------------------------
      Force waiting for the RX device to send an ACK packet. Don't bother disabling Dynamic Ack
      (should it even be enabled) as this command overrides it.
      -------------------------------------------------*/
      payloadType = CMD_W_TX_PAYLOAD;
    }

    /*-------------------------------------------------
    Write the payload to the TX FIFO and optionally start the transfer
    -------------------------------------------------*/
    mHWDriver->writePayload( buffer, len, payloadType );

    if ( startTX )
    {
      toggleChipEnablePin( true );
      mCurrentMode = MODE_TX;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  void HardwareDriver::toggleChipEnablePin( const bool state )
  {
    mHWDriver->toggleCE( state );
  }

  Chimera::Status_t HardwareDriver::txStandBy( const size_t timeout, const bool startTx )
  {
    using namespace RF24::Hardware;

    /*------------------------------------------------
    Optionally start a new transfer
    ------------------------------------------------*/
    if ( startTx )
    {
      stopListening();
      toggleChipEnablePin( true );
    }

    /*------------------------------------------------
    Prevent the user from executing the function if they haven't told
    the device to quit listening yet.
    ------------------------------------------------*/
    if ( mCurrentlyListening )
    {
      mFailureCode = Chimera::CommonStatusCodes::NOT_READY;
      return mFailureCode;
    }

    /*------------------------------------------------
    Wait for the TX FIFO to empty, retrying packet transmissions as needed.
    ------------------------------------------------*/
    auto startTime = Chimera::millis();

    while ( !mHWDriver->txFifoEmpty() )
    {
      /*------------------------------------------------
      If max retries interrupt occurs, retry transmission. The data is
      automatically kept in the TX FIFO.
      ------------------------------------------------*/
      if ( _registerIsBitmaskSet( REG_STATUS, STATUS_MAX_RT ) )
      {
        toggleChipEnablePin( false );
        mHWDriver->setRegisterBits( REG_STATUS, STATUS_MAX_RT );

        Chimera::delayMilliseconds( 1 );
        toggleChipEnablePin( true );
      }

      /*------------------------------------------------
      Automatic timeout failure
      ------------------------------------------------*/
      if ( ( Chimera::millis() - startTime ) > timeout )
      {
        toggleChipEnablePin( false );
        flushTX();
        mFailureCode = Chimera::CommonStatusCodes::TIMEOUT;
        mCurrentMode = MODE_STANDBY_I;
        return mFailureCode;
      }
    }

    /*------------------------------------------------
    Transition back to Standby Mode I
    ------------------------------------------------*/
    toggleChipEnablePin( false );
    mCurrentMode = MODE_STANDBY_I;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::stageAckPayload( const RF24::Hardware::PipeNumber pipe,
                                                     const RF24::Network::Frame::Buffer &buffer, size_t length )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  uint8_t HardwareDriver::flushTX()
  {
    return mHWDriver->writeCMD( RF24::Hardware::CMD_FLUSH_TX );
  }

  uint8_t HardwareDriver::flushRX()
  {
    return mHWDriver->writeCMD( RF24::Hardware::CMD_FLUSH_RX );
  }

  Chimera::Status_t HardwareDriver::toggleDynamicPayloads( const bool state )
  {
    mHWDriver->toggleDynamicPayloads( state );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t HardwareDriver::setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate )
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

  RF24::Hardware::PowerAmplitude HardwareDriver::getPALevel()
  {
    using namespace RF24::Hardware;

    uint8_t setup = mHWDriver->readRegister( REG_RF_SETUP );
    return static_cast<PowerAmplitude>( ( setup & RF_SETUP_RF_PWR ) >> 1 );
  }

  Chimera::Status_t HardwareDriver::setDataRate( const RF24::Hardware::DataRate speed )
  {
    using namespace RF24::Hardware;

    /*------------------------------------------------
    Cache the current setup so we don't blow away bits
    ------------------------------------------------*/
    Reg8_t setup = mHWDriver->readRegister( REG_RF_SETUP );

    /*------------------------------------------------
    Decide which bits need to be set/cleared
    ------------------------------------------------*/
    switch ( speed )
    {
      case DataRate::DR_250KBPS:
        setup |= RF_SETUP_RF_DR_LOW;
        setup &= ~RF_SETUP_RF_DR_HIGH;
        break;

      case DataRate::DR_1MBPS:
        setup &= ~( RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW );
        break;

      case DataRate::DR_2MBPS:
        setup &= ~RF_SETUP_RF_DR_LOW;
        setup |= RF_SETUP_RF_DR_HIGH;
        break;

      default:
        return Chimera::CommonStatusCodes::FAIL;
        break;
    }

    /*------------------------------------------------
    Write the configuration and verify it was set properly
    ------------------------------------------------*/
    mHWDriver->writeRegister( REG_RF_SETUP, setup );
    auto result = mHWDriver->readRegister( REG_RF_SETUP );

    return ( result == setup ) ? Chimera::CommonStatusCodes::OK : Chimera::CommonStatusCodes::FAIL;
  }

  RF24::Hardware::DataRate HardwareDriver::getDataRate()
  {
    using namespace RF24::Hardware;

    uint8_t reg = mHWDriver->readRegister( REG_RF_SETUP );
    return static_cast<DataRate>( reg & ( RF_SETUP_RF_DR_HIGH | RF_SETUP_RF_DR_LOW ) );
  }

  bool HardwareDriver::_registerIsBitmaskSet( const uint8_t reg, const uint8_t bitmask )
  {
    return ( mHWDriver->readRegister( reg ) & bitmask ) == bitmask;
  }

  bool HardwareDriver::_registerIsAnySet( const uint8_t reg, const uint8_t bitmask )
  {
    return mHWDriver->readRegister( reg ) & bitmask;
  }

  Chimera::Status_t HardwareDriver::toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber pipe )
  {
    mHWDriver->toggleAutoAck( state, pipe );
    return Chimera::CommonStatusCodes::OK;
  }
}    // namespace RF24::Physical

#endif /* !RF24_SIMULATOR */