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

/* Driver Includes */
#include <RF24Node/hardware/driver.hpp>
#include <RF24Node/hardware/types.hpp>

namespace RF24::Physical
{
  /**
   *   Base class for interacting with an NRF24L01 wireless module.
   *   Most of this was taken from https://github.com/nRF24/RF24 and modified to accommodate my
   *   particular flavor of hardware abstraction.
   */
  class Driver
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attachHWDriver( RF24::Hardware::Driver_sPtr &driver );

    /**
     *   Initialize the chip and verify correct setup
     *
     *   @return True if everything was properly set up, false if not
     */
    bool begin();

    /**
     *   Checks if the driver has been initialized properly
     *
     *   @return True if success, false if not
     */
    bool isInitialized();

    /**
     *   Start listening on the pipes opened for reading.
     *
     *   1. Be sure to call openReadPipe() first.
     *   2. Do not call write() while in this mode, without first calling stopListening().
     *   3. Call available() to check for incoming traffic, and read() to get it.
     *
     *   @return void
     */
    void startListening();

    /**
     *   Pauses a currently listening device. Does nothing if the device is not listening.
     *
     *   @return void
     */
    void pauseListening();

    /**
     *   Resumes listening a paused device. Does nothing if the device is not paused.
     *
     *   @return void
     */
    void resumeListening();

    /**
     *   Stop listening for RX messages and switch to transmit mode. Does nothing if already
     *   stopped listening.
     *
     *   @return void
     */
    void stopListening();

    /**
     *   Open pipe 0 to write to an address. This is the only pipe that can do this.
     *
     *   @param[in]  address     The address for pipe 0 to write to
     *   @return void
     */
    void openWritePipe( const uint64_t address );

    /**
     *   Open a pipe for reading
     *
     *   Up to 6 pipes can be open for reading at once.  Open all the required
     *   reading pipes, and then call startListening().
     *
     *   @see openWritingPipe
     *   @see setAddressWidth
     *
     *   @note Pipes 0 and 1 will store a full 5-byte address. Pipes 2-5 will technically
     *   only store a single byte, borrowing up to 4 additional bytes from pipe #1 per the
     *   assigned address width.
     *
     *   @warning Pipes 1-5 should share the same address, except the first byte. Only the first byte in the array should be
     * unique
     *
     *   @warning Pipe 0 is also used by the writing pipe.  So if you open pipe 0 for reading, and then startListening(), it
     * will overwrite the writing pipe.  Ergo, do an openWritingPipe() again before write().
     *
     *   @param[in]  number      Which pipe to open, 0-5.
     *   @param[in]  address     The address you want the pipe to listen to
     *   @param[in]  validate    Optionally validate the address was set properly
     *   @return True if the pipe was opened properly
     */
    bool openReadPipe( const uint8_t pipe, const uint64_t address, const bool validate = false );

    /**
     *   Close a pipe after it has been previously opened.
     *   Can be safely called without having previously opened a pipe.
     *
     *   @param[in]  pipe    Which pipe number to close, 0-5.
     *   @return void
     */
    void closeReadPipe( const uint8_t pipe );

    /**
     *   Check if data is available to be read on any pipe.
     *
     *   @return True if a payload is available, false if not
     */
    bool available();

    /**
     *   Check if data is available to be read on any pipe. If so, returns which pipe is ready. The payload
     *   returned from a consecutive call to read() then belongs to the pipe assigned in this function.
     *
     *   @param[out] pipeNum     Which pipe has the payload available
     *   @return True if there is a payload available, false if none is
     */
    bool available( uint8_t &pipeNum );

    /**
     *   Read the available payload into a buffer
     *
     *   The size of data read is the fixed payload size, see getPayloadSize()
     *
     *   @param[out] buffer      Pointer to a buffer where the data should be written
     *   @param[in]  len         Maximum number of bytes to read into the buffer
     *
     *   @return void
     */
    void read( uint8_t *const buffer, size_t len );

    /**
     *   Writes data onto a previously configured RF channel.
     *
     *   Prerequisite Calls:
     *       1. openWritePipe()
     *       2. setChannel()
     *       3. stopListening()
     *
     *   @param[in]  buffer          Array of data to be sent
     *   @param[in]  len             Number of bytes to be sent
     *   @param[in]  multicast       If true, disables Auto-Acknowledgment feature for just this packet
     *   @return True if the payload was delivered successfully false if not
     */
    bool writeFast( const uint8_t *const buffer, uint8_t len, const bool multicast = false );

    /**
     *   Checks if we can successfully talk with the radio over SPI
     *
     *   @return true if connected, false if not
     */
    bool isConnected();

    /**
     *   Set the number and delay of retries upon failed transfer
     *
     *   @param[in]  delay       How long to wait between each retry
     *   @param[in]  count       How many retries before giving up, max 15
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    bool setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const uint8_t count, const bool validate = false );

    /**
     *   Set RF communication channel
     *
     *   @param[in]  channel     Which RF channel to communicate on, 0-125
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    bool setChannel( const uint8_t channel, const bool validate = false );

    /**
     * This function allows extended blocking and auto-retries per a user defined timeout
     *
     * @return True if transmission is successful
     */
    bool txStandBy( const uint32_t timeout, const bool startTx = false );

    /**
     *   Write an ACK payload for the specified pipe
     *
     *   The next time a message is received on the given pipe, the buffer data will be
     *   be sent back in the ACK packet.
     *
     *   @warning Only three ACK payloads can be pending at any time as there are only 3 FIFO buffers.
     *   @note ACK payloads are dynamic payloads, which only works on pipes 0 and 1 by default. Call
     *   enableDynamicPayloads() to enable on all pipes.
     *
     *   @param[in] pipe     Which pipe will get this response
     *   @param[in] buffer   Data to be sent
     *   @param[in] len      Length of the data to send, up to 32 bytes max.  Not affected by the static payload set by
     * setPayloadSize().
     */
    void writeAckPayload( const uint8_t pipe, const uint8_t *const buffer, size_t len );

    /**
     *   Determine if an ACK payload was received in the most recent call to
     *   write(). The alternate function available() can also be used.
     *
     *   @return True if an ACK payload is available, false if not
     */
    bool isAckPayloadAvailable();

    /**
     *   Informs the caller what interrupts are currently active
     *
     *   Clears all interrupts before exiting.
     *
     *   @param[out] tx_ok       The send was successful (TX_DS)
     *   @param[out] tx_fail     The send failed, too many retries (MAX_RT)
     *   @param[out] rx_ready    There is a message waiting to be read (RX_DS)
     */
    void whatHappened( bool &tx_ok, bool &tx_fail, bool &rx_ready );

    /**
     *   Set static payload size
     *
     *   This implementation uses a pre-established fixed payload size for all transfers. If this method
     *   is never called, the driver will always transmit the maximum payload size (32 bytes), no matter how much
     *   was sent to write().
     *
     *   @todo Implement variable-sized payloads feature
     *
     *   @param[in]  size    The number of bytes in the payload
     *   @return void
     */
    void setStaticPayloadSize( const uint8_t size );

    /**
     *   Get the static payload size
     *
     *   @see setPayloadSize()
     *
     *   @return The number of bytes used in the payload
     */
    uint8_t getStaticPayloadSize();

    /**
     *   Get the dynamic payload length of the last received transfer
     *
     *   @return payload lenght
     */
    uint8_t getDynamicPayloadSize();

    /**
     *   Clears out the TX FIFO
     *
     *   @return Current value of Register::STATUS
     */
    uint8_t flushTX();

    /**
     *   Clears out the RX FIFO
     *
     *   @return Current value of Register::STATUS
     */
    uint8_t flushRX();

    

    /**
     *   Enable custom payloads on the RX acknowledge packets
     *
     *   ACK payloads are a handy way to return data back to senders without
     *   manually changing the radio modes on both units.
     *
     *   @note ACK payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
     *   enableDynamicPayloads() to enable on all pipes.
     *
     *   @return void
     */
    void enableAckPayload();

    /**
     *   Disables custom payloads on the RX acknowledge packets (all pipes)
     *
     *   @return void
     */
    void disableAckPayload();

    /**
     *   Enable dynamically-sized payloads (all pipes)
     *
     *  This disables dynamic payloads on ALL pipes. Since ACK Payloads requires Dynamic Payloads, ACK Payloads
     *   are also disabled. If dynamic payloads are later re-enabled and ACK payloads are desired then enableAckPayload()
     *   must be called again as well.
     *
     *   @return void
     */
    void toggleDynamicPayloads( const size_t pipe, const bool state);

    /**
     *   Enable the W_TX_PAYLOAD_NOACK command, which allows a packet to be transmitted
     *   without getting an ACK packet from the receiver. Only works when multicast==true in
     *   the class's write functions.
     *
     *    Disable the W_TX_PAYLOAD_NOACK command, effectively forcing all packets to obtain
     *   an ACK from the receiver.
     *
     *   @return void
     */
    void toggleDynamicAck( const bool state );

    /**
     *   Enable or disable auto-acknowledge packets on a per pipeline basis.
     *
     *   If enabled, the pipe will immediately go into RX mode after transmitting its payload
     *   so that it can listen for the receiver's ACK packet. If no ACK is received and the auto
     *   retransmit feature is enabled, it will retry until it either succeeds or it hits a retry
     *   limit (defined in SETUP_RETR::ARC).
     *
     *   @note The auto-acknowledge behavior can be temporarily disabled for one packet by enabling
     *           the feature register and using the W_TX_PAYLOAD_NO_ACK command. (ie multicast = true)
     *
     *   @param[in]  pipe        Which pipeline to modify
     *   @param[in]  enable      Whether to enable (true) or disable (false) auto-ACKs
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    bool setAutoAck( const uint8_t pipe, const bool enable, const bool validate = false );

    /**
     *   Set the power amplifier level
     *
     *   @param[in]  level       Desired power amplifier level
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    bool setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate = false );

    /**
     *   Get the current power amplitude level
     *
     *   @return Current power amplitude setting
     */
    RF24::Hardware::PowerAmplitude getPALevel();

    /**
     *   Set the TX/RX data rate
     *
     *   @warning setting RF24_250KBPS will fail for non-plus units
     *
     *   @param[in]  speed   Desired speed for the radio to TX/RX with
     *   @return true if the change was successful
     */
    bool setDataRate( const RF24::Hardware::DataRate speed );

    /**
     *   Get the transmission data rate
     *
     *   @return The current data rate
     */
    RF24::Hardware::DataRate getDataRate();

    

    


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
    void startFastWrite( const uint8_t *const buffer, size_t len, const bool multicast, const bool startTX = true );

    /**
     *   Write the transmit payload. If the TX FIFO is full when this is called, the data will simply be lost.
     *   The size of data written is capped at the max payload size.
     *
     *   @param[in]  buffer      Where to get the data
     *   @param[in]  len         Number of bytes to be sent
     *   @param[in]  writeType   Write using ACK (Command::W_TX_PAYLOAD) or NACK (Command::W_TX_PAYLOAD_NO_ACK)
     *   @return Current value of status register
     */
    uint8_t writePayload( const uint8_t *const buffer, size_t len, const uint8_t writeType );

    

    void toggleChipEnablePin( const bool state );


  private:
    RF24::Hardware::FailureCode oopsies; /**< Latest reason why something failed. */

    bool initialized            = false; /**< Track initialization state */
    bool pVariant               = false; /**< NRF24L01+ variant device? */
    bool featuresActivated      = false; /**< Features register functionality enabled? */
    bool dynamicPayloadsEnabled = false; /**< Are our payloads configured as variable width? */
    bool listening              = false; /**< Track if the radio is listening or not */
    bool listeningPaused        = false;

    size_t addressWidth = 0; /**< Keep track of the user's address width preference */
    size_t payloadSize  = 0; /**< Keep track of the user's payload width preference */

    uint64_t cachedPipe0RXAddress; /**< Remembers a previously set Pipe0 listening address */


    RF24::Hardware::Mode currentMode; /**< Keep track of which HW mode of the radio is likely to be in */
    RF24::Hardware::Driver_sPtr hwDriver;

    
    bool _registerIsBitmaskSet( const uint8_t reg, const uint8_t bitmask );
    bool _registerIsAnySet( const uint8_t reg, const uint8_t bitmask );
  };

  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

}    // namespace RF24::Physical

#endif /* NRF24L01_PHYSICAL_LAYER_HPP */
