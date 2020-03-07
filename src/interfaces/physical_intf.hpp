/********************************************************************************
 *  File Name:
 *    physical_intf.hpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_INTERFACE_HPP
#define RF24_NODE_PHYSICAL_INTERFACE_HPP

/* C++ Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/src/hardware/types.hpp>
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/physical/types.hpp>
#include <RF24Node/src/network/frame/types.hpp>


namespace RF24::Physical
{
  class Interface
  {
  public:
    virtual ~Interface() = default;

    /**
     *	Attaches a logging instance to the class so that we can log physical layer
     *  messages as needed for debugging.
     *
     *	@param[in]	sink    The log sink
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t attachLogger( uLog::SinkHandle sink ) = 0;

    /**
     *   Initialize the chip and verify correct setup
     *
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t initialize( const RF24::Physical::Config &cfg ) = 0;

    /**
     *   Checks if the driver has been initialized properly
     *
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t isInitialized() = 0;

    /**
     *   Checks if we can successfully talk with the radio over SPI
     *
     *   @return true if connected, false if not
     */
    virtual Chimera::Status_t isConnected() = 0;

    /**
     *   Set the number and delay of retries upon failed transfer
     *
     *   @param[in]  delay       How long to wait between each retry
     *   @param[in]  count       How many retries before giving up, max 15
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    virtual Chimera::Status_t setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count,
                                          const bool validate = false ) = 0;

    /**
     *   Set RF communication channel
     *
     *   @param[in]  channel     Which RF channel to communicate on, 0-125
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    virtual Chimera::Status_t setChannel( const size_t channel, const bool validate = false ) = 0;

    /**
     *  Get the current RF communication channel
     *
     *  @return The currently configured RF Channel
     */
    virtual size_t getChannel() = 0;

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
    virtual Chimera::Status_t setStaticPayloadSize( const size_t size ) = 0;

    /**
     *   Get the static payload size
     *
     *   @see setPayloadSize()
     *
     *   @return The number of bytes used in the payload
     */
    virtual size_t getStaticPayloadSize() = 0;

    /**
     *   Get the dynamic payload length of the last received transfer
     *
     *   @return payload length
     */
    virtual size_t getDynamicPayloadSize() = 0;

    /**
     *   Start listening on the pipes opened for reading.
     *
     *   1. Be sure to call openReadPipe() first.
     *   2. Do not call write() while in this mode, without first calling stopListening().
     *   3. Call available() to check for incoming traffic, and read() to get it.
     *
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t startListening() = 0;

    /**
     *   Stop listening for RX messages and switch to transmit mode. Does nothing if already
     *   stopped listening.
     *
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t stopListening() = 0;

    /**
     *   Pauses a currently listening device. Does nothing if the device is not listening.
     *
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t pauseListening() = 0;

    /**
     *   Resumes listening a paused device. Does nothing if the device is not paused.
     *
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t resumeListening() = 0;

    /**
     *   Open pipe 0 to write to an address. This is the only pipe that can do this.
     *
     *   @param[in]  address     The address for pipe 0 to write to
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t openWritePipe( const PhysicalAddress address ) = 0;

    /**
     *  Closes pipe 0 for writing. Can be safely called without previously calling open.
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t closeWritePipe() = 0;

    /**
     *   Open any pipe for reading. Up to 6 pipes can be open for reading at once.  Open all the required
     *   reading pipes, and then call startListening().
     *
     *   @see openWritingPipe
     *   @see setAddressWidth
     *
     *   @note
     *    Pipes 0 and 1 will store a full 5-byte address. Pipes 2-5 will technically
     *    only store a single byte, borrowing up to 4 additional bytes from pipe #1 per the
     *    assigned address width.
     *
     *   @warning
     *      Pipes 1-5 should share the same address, except the first byte. Only the first byte in the array should be
     *      unique
     *
     *   @warning
     *      Pipe 0 is also used by the writing pipe.  So if you open pipe 0 for reading, and then startListening(), it
     *      will overwrite the writing pipe.  Ergo, do an openWritingPipe() again before write().
     *
     *   @param[in]  number      Which pipe to open, 0-5.
     *   @param[in]  address     The address you want the pipe to listen to
     *   @param[in]  validate    Optionally validate the address was set properly
     *   @return Chimera::Status_t
     */
    virtual Chimera::Status_t openReadPipe( const RF24::Hardware::PipeNumber pipe, const PhysicalAddress address,
                                            const bool validate = false ) = 0;

    /**
     *  Close a pipe after it has been previously opened.
     *  Can be safely called without having previously opened a pipe.
     *
     *  @param[in]  pipe    Which pipe number to close, 0-5.
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t closeReadPipe( const RF24::Hardware::PipeNumber pipe ) = 0;

    /**
     *  Check if data is available to be read on any pipe. If so, returns a bitfield that indicates
     *  which pipe is ready.
     *
     *  @return RF24::Hardware::PipeNum_t
     */
    virtual RF24::Hardware::PipeNumber payloadAvailable() = 0;

    /**
     *  Checks how many bytes are available in the RX payload for the given pipe
     *
     *  @note   This function really isn't that useful unless used in conjunction with payloadAvailable()
     *  @see    payloadAvailable()
     *
     *  @param[in]  pipe    The pipe to check
     *  @return size_t      The number of bytes in the pipe's payload buffer
     */
    virtual size_t getPayloadSize( const RF24::Hardware::PipeNumber pipe ) = 0;

    /**
     *  Read the available payload into a buffer
     *
     *  @note   Call payloadAvailable() to know which pipe the payload data corresponds to
     *  @note   Call getPayloadSize() to know how much data is in the pipe payload
     *
     *  @see    payloadAvailable()
     *  @see    getPayloadSize()
     *
     *  @param[out] buffer      Pointer to a buffer where the data should be written
     *  @param[in]  length      Maximum number of bytes to read into the buffer
     *
     *  @return void
     */
    virtual Chimera::Status_t readPayload( RF24::Network::Frame::Buffer &buffer, const size_t length ) = 0;

    /**
     *  Immediately writes data to pipe 0 under the assumption that the hardware has already
     *  been configured for TX transfers. This prevents the software from going through the
     *  full TX configuration process each time a packet needs to be sent.
     *
     *  Prerequisite Calls:
     *    1. openWritePipe()
     *    2. setChannel()
     *    3. stopListening() [Pipe 0 only]
     *
     *   @param[in]  buffer          Array of data to be sent
     *   @param[in]  length          Number of bytes to be sent from the buffer
     *   @return True if the payload was delivered successfully false if not
     */
    virtual Chimera::Status_t immediateWrite( const RF24::Network::Frame::Buffer &buffer, const size_t length ) = 0;

    /**
     * This function allows extended blocking and auto-retries per a user defined timeout
     *
     * @return Chimera::Status_t
     */
    virtual Chimera::Status_t txStandBy( const size_t timeout, const bool startTx = false ) = 0;

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
     *   @param[in] length   Length of the data to send, up to 32 bytes max.  Not affected by the static payload set by
     * setPayloadSize().
     */
    virtual Chimera::Status_t stageAckPayload( const RF24::Hardware::PipeNumber pipe, const RF24::Network::Frame::Buffer &buffer, size_t length ) = 0;

    /**
     *   Clears out the TX FIFO
     *
     *   @return Current value of Register::STATUS
     */
    virtual Reg8_t flushTX() = 0;

    /**
     *   Clears out the RX FIFO
     *
     *   @return Current value of Register::STATUS
     */
    virtual Reg8_t flushRX() = 0;

    /**
     *   Enable dynamically-sized payloads for both TX and ACK packets
     *
     *  This disables dynamic payloads on ALL pipes. Since ACK Payloads requires Dynamic Payloads, ACK Payloads
     *   are also disabled. If dynamic payloads are later re-enabled and ACK payloads are desired then enableAckPayload()
     *   must be called again as well.
     *
     *   @return void
     */
    virtual Chimera::Status_t toggleDynamicPayloads( const bool state ) = 0;

    /**
     *   Set the power amplifier level
     *
     *   @param[in]  level       Desired power amplifier level
     *   @param[in]  validate    Check if the value was set correctly
     *   @return True if success, false if not
     */
    virtual Chimera::Status_t setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate = false ) = 0;

    /**
     *   Get the current power amplitude level
     *
     *   @return Current power amplitude setting
     */
    virtual RF24::Hardware::PowerAmplitude getPALevel() = 0;

    /**
     *   Set the TX/RX data rate
     *
     *   @warning setting RF24_250KBPS will fail for non-plus units
     *
     *   @param[in]  speed   Desired speed for the radio to TX/RX with
     *   @return true if the change was successful
     */
    virtual Chimera::Status_t setDataRate( const RF24::Hardware::DataRate speed ) = 0;

    /**
     *   Get the transmission data rate
     *
     *   @return The current data rate
     */
    virtual RF24::Hardware::DataRate getDataRate() = 0;

    virtual Chimera::Status_t toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber pipe ) = 0;
  };

  using Interface_sPtr = std::shared_ptr<Interface>;
  using Interface_uPtr = std::unique_ptr<Interface>;
}    // namespace RF24::Physical

#endif /* !RF24_NODE_PHYSICAL_INTERFACE_HPP */