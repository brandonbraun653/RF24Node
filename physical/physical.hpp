/********************************************************************************
 *  File Name:
 *    physical.hpp
 *
 *  Description:
 *    Interface to the NRF24L01 radio hardware driver. Originally based upon the work done by James Coliz
 *    (https://github.com/nRF24/RF24). This version expands on the original by adding more modern C++ features, reliability
 *    and safety checks, improved debugging notifications, and more helpful comments to understand how and why things are
 *    done.
 *
 *    The hardware SPI driver has been abstracted away and can be driven by either the Chimera HAL
 *    (https://github.com/brandonbraun653/Chimera) or by providing overriding functions of the SPI/GPIO interface as defined
 *    in the class below. This allows the driver to be platform agnostic.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_PHYSICAL_LAYER_HPP
#define NRF24L01_PHYSICAL_LAYER_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdio>
#include <array>
#include <memory>

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/types/common_types.hpp>

/* uLog Includes */
#include <uLog/types.hpp>

/* Driver Includes */
#include <RF24Node/hardware/driver.hpp>
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/interfaces/physical_intf.hpp>

#if !defined( RF24_SIMULATOR )

namespace RF24::Physical
{
  /**
   *  Physical Layer Driver:
   *    Implements the physical layer in the OSI model and is concerned with accurately transmitting
   *    data across the wireless medium. In practice this is a conglomerate of the very low level
   *    hardware driver for the NRF24 and the various logic to configure the device appropriately.
   */
  class HardwareDriver : public Interface
  {
  public:
    HardwareDriver();
    ~HardwareDriver();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink ) final override;

    /**
     *  Attaches a low level hardware driver for the Physical layer to consume.
     *
     *  @param[in]  driver    Hardware interface to the NRF24L01
     *  @return Chimera::Status_t
     */
    Chimera::Status_t attachHWDriver( RF24::Hardware::Driver_sPtr &driver );

    Chimera::Status_t initialize( const RF24::Physical::Config &cfg ) final override;
    Chimera::Status_t isInitialized() final override;
    Chimera::Status_t isConnected() final override;
    Chimera::Status_t setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count,
                                  const bool validate = false ) final override;
    Chimera::Status_t setChannel( const size_t channel, const bool validate = false ) final override;
    size_t getChannel() final override;
    Chimera::Status_t setStaticPayloadSize( const size_t size ) final override;
    size_t getStaticPayloadSize() final override;
    size_t getDynamicPayloadSize() final override;
    Chimera::Status_t startListening() final override;
    Chimera::Status_t stopListening() final override;
    Chimera::Status_t pauseListening() final override;
    Chimera::Status_t resumeListening() final override;
    Chimera::Status_t openWritePipe( const uint64_t address ) final override;
    Chimera::Status_t closeWritePipe() final override;
    Chimera::Status_t openReadPipe( const RF24::Hardware::PipeNumber pipe, const uint64_t address,
                                    const bool validate = false ) final override;
    Chimera::Status_t closeReadPipe( const RF24::Hardware::PipeNumber pipe ) final override;
    RF24::Hardware::PipeNumber payloadAvailable() final override;
    size_t getPayloadSize( const RF24::Hardware::PipeNumber pipe ) final override;
    Chimera::Status_t readPayload( RF24::Network::Frame::Buffer &buffer, const size_t length ) final override;
    Chimera::Status_t immediateWrite( const RF24::Network::Frame::Buffer &buffer, const size_t length ) final override;
    Chimera::Status_t txStandBy( const size_t timeout, const bool startTx = false ) final override;
    Chimera::Status_t stageAckPayload( const RF24::Hardware::PipeNumber pipe, const RF24::Network::Frame::Buffer &buffer,
                                       size_t length ) final override;

    Reg8_t flushTX() final override;
    Reg8_t flushRX() final override;
    Chimera::Status_t toggleDynamicPayloads( const bool state ) final override;
    Chimera::Status_t setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate = false ) final override;
    RF24::Hardware::PowerAmplitude getPALevel() final override;
    Chimera::Status_t setDataRate( const RF24::Hardware::DataRate speed ) final override;
    RF24::Hardware::DataRate getDataRate() final override;
    Chimera::Status_t toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber pipe ) final override;

  protected:
    /**
     *   Non-blocking write to an open TX pipe. If the TX FIFO is full when called, the data will simply be lost.
     *   By default, the transfer will immediately start.
     *
     *   @param[in] buffer       Array of data to be sent
     *   @param[in] len          Number of bytes to be sent
     *   @param[in] multicast    If false, requests the RX device to ACK the transmission for this packet
     *   @param[in] startTX      Starts the transfer immediately if true
     *   @return True if the payload was delivered successfully false if not
     */
    Chimera::Status_t startFastWrite( const void *const buffer, const size_t len, const bool multicast,
                                      const bool startTX = true );


    void toggleChipEnablePin( const bool state );

  private:
    bool mInitialized;                     /**< Track initialization state */
    bool mPlusVariant;                     /**< NRF24L01+ variant device? */
    bool mCurrentlyListening;              /**< Track if the radio is listening or not */
    bool mListeningPaused;                 /**< Track if the radio has paused listening */
    uint8_t mAddressWidth;                 /**< Keep track of the user's address width preference */
    uint8_t mPayloadSize;                  /**< Keep track of the user's payload width preference */
    uint64_t mCachedPipe0RXAddress;        /**< Remembers a previously set Pipe0 listening address */
    RF24::Hardware::Mode mCurrentMode;     /**< Keep track of which HW mode of the radio is likely to be in */
    RF24::Hardware::Driver_sPtr mHWDriver; /**< Low level radio module hardware driver */
    Chimera::Status_t mFailureCode;        /**< Latest reason why something failed. */

    uLog::SinkHandle logger;

    bool _registerIsBitmaskSet( const uint8_t reg, const uint8_t bitmask );
    bool _registerIsAnySet( const uint8_t reg, const uint8_t bitmask );
  };

  using Driver_sPtr = std::shared_ptr<HardwareDriver>;
  using Driver_uPtr = std::unique_ptr<HardwareDriver>;

}    // namespace RF24::Physical

#endif /* !RF24_SIMULATOR */
#endif /* NRF24L01_PHYSICAL_LAYER_HPP */
