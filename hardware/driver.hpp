/********************************************************************************
 *   File Name:
 *     driver.hpp
 *
 *   Description:
 *     Hardware driver for the NRF24L01(+)
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_HARDWARE_DRIVER_HPP
#define NRF24L01_HARDWARE_DRIVER_HPP

/* C++ Includes */
#include <array>
#include <memory>

/* Chimera Includes */
#include <Chimera/extensions/spi_ext.hpp>
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/threading.hpp>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/hardware/types.hpp>

namespace RF24::Hardware
{
  extern const std::array<Reg8_t, MAX_NUM_PIPES> rxPipeAddressRegister;
  extern const std::array<Reg8_t, MAX_NUM_PIPES> rxPipePayloadWidthRegister;
  extern const std::array<Reg8_t, MAX_NUM_PIPES> rxPipeEnableBitField;

  /**
   *  Register level interface to provide common functionality to the networking
   *  layer without having it mess with the SPI driver managment.
   *
   *  @note SPI Interface Requirements:
   *
   *        | Config Option | Expected Value |
   *        |:-------------:|:--------------:|
   *        |     Bit Order | MSB First      |
   *        |     Data Mode | MODE0          |
   *        |    Data Width | 8 Bits         |
   *        |     Max Clock | 8MHz           |
   *        |    CS Control | Manual         |
   */
  class Driver : public Chimera::Threading::Lockable
  {
  public:
    /*-------------------------------------------------
    Constructors/Destructors
    -------------------------------------------------*/
    Driver();
    ~Driver();

    /*-------------------------------------------------
    Driver Functions
    -------------------------------------------------*/
    /**
     *  Initializes the driver and any preliminary configurations for the NRF24 chip. Upon
     *  exiting, the driver will be ready for use and sitting in a powered off state.
     *
     *  @note If the SPI configuration specifies a CS config, it will be overwritten here
     *
     *  @param[in]  CE      The pin configuration for the Chip Enable pin (sets Rx/Tx modes)
     *  @param[in]  CS      The pin configuration for the Chip Select pin (SPI)
     *
     *  @return Chimera::Status_t
     *
     *  |   Return Value  |                     Explanation                    |
     *  |:---------------:|:--------------------------------------------------:|
     *  |              OK | The initialization sequence succeeded              |
     *  | NOT_INITIALIZED | The initialization sequence failed for some reason |
     *  |          LOCKED | The hardware is currently unavailabe               |
     */
    Chimera::Status_t initialize( const Chimera::SPI::DriverConfig &setup, const Chimera::GPIO::PinInit &CE );

    /**
     *  Wipes out all configuration from the hardware registers and
     *  resets them back to hardware defaults, as if the unit had
     *  just powered off and then back on again.
     *
     *  @return Chimera::Status_t
     *
     *  | Return Value |              Explanation             |
     *  |:------------:|:------------------------------------:|
     *  |           OK | The reset succeeded                  |
     *  |         FAIL | The reset failed                     |
     *  |       LOCKED | The hardware is currently unavailabe |
     */
    Chimera::Status_t resetDevice();

    /**
     *  Performs a self-test on the unit, including:
     *    1. Register read/write validation
     *    2. Received power detect (if enabled)
     *        a. Checks if any surrounding devices are transmitting
     *        b. Sets a flag indicating if other devices are detected
     *    3. TBD?
     *
     *  @param[in]  rpd     Received power detection check enable/disable
     *  @return Chimera::Status_t
     *
     *  | Return Value |              Explanation             |
     *  |:------------:|:------------------------------------:|
     *  |           OK | The self test passed                 |
     *  |         FAIL | The self test failed                 |
     *  |       LOCKED | The hardware is currently unavailabe |
     */
    Chimera::Status_t selfTest( const bool rpd );

    /**
     *  Reads a register on the device and returns the current value of that register
     *
     *  @param[in]  addr    The address of the register to read
     *  @return Reg8_t
     */
    Reg8_t readRegister( const Reg8_t addr );

    /**
     *  Reads a multibyte register into a buffer
     *
     *  @param[in]  addr    The address of the register to read
     *  @param[out] buf     The buffer to read into
     *  @param[in]  len     The total number of bytes to read from the register
     *  @return Reg8_t      The chip's status register
     */
    Reg8_t readRegister( const Reg8_t addr, void *const buf, size_t len );

    /**
     *  Writes a register on the device with a given value
     *
     *  @warning  This operation overwrites the entire register
     *  @note     Validation only works in debug builds
     *
     *  @param[in]  addr    The address of the register to write
     *  @param[in]  value   The data to write to that register
     *  @param[in]  check   Whether or not to validate the data was written correctly
     *  @return Reg8_t      The chip's status register
     */
    Reg8_t writeRegister( const Reg8_t addr, const Reg8_t value, const bool check = true );

    /**
     *  Writes a register on the device with multiple bytes
     *
     *  @warning  This operation overwrites the entire register
     *  @note     Validation only works in debug builds
     *
     *  @param[in]  addr    The address of the register to write
     *  @param[in]  buffer  The data to write to that register
     *  @param[in]  len     The number of bytes to write from the buffer
     *  @param[in]  check   Whether or not to validate the data was written correctly
     *  @return Reg8_t      The chip's status register
     */
    Reg8_t writeRegister( const Reg8_t addr, const void *const buffer, size_t len, const bool check = true );

    /**
     *  Performs a read/modify/write operation on a register to set specific bits
     *
     *  @note   Occasionally setting a register bit invokes the hardware to actually clear a field
     *          (I'm looking at you Status Register), so we allow disabling of the register checks
     *
     *  @param[in]  addr    The address of the register to write
     *  @param[in]  mask    Mask of which bits should be set
     *  @param[in]  check   Whether or not to validate the data was written correctly
     *  @return Reg8_t      The chip's status register
     */
    Reg8_t setRegisterBits( const Reg8_t addr, const Reg8_t mask, const bool check = true );

    /**
     *  Performs a read/modify/write operation on a register to clear specific bits
     *
     *  @param[in]  addr    The address of the register to write
     *  @param[in]  mask    Mask of which bits should be cleared
     *  @return Reg8_t      The chip's status register
     */
    Reg8_t clrRegisterBits( const Reg8_t addr, const Reg8_t mask );

    /**
     *  Toggles the PWRUP bit in the CONFIG register to enable/disable the chip
     *
     *  @param[in]  state   Turn the chip on (true) or off (false)
     *  @return void
     */
    void toggleRFPower( const bool state );

    void toggleFeatures( const bool state );

    void toggleDynamicPayloads( const bool state );

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
    void toggleAutoAck( const bool state, const PipeNumber pipe );

    void toggleAckPayloads( const bool state );

    /**
     *  Enables/Disables the given RX pipe for listening on the currently
     *  configured address for that pipe.
     *
     *  @note To act on all pipes, pass the function RX_PIPE_ALL
     *
     *  @param[in]  pipe    The pipe to act upon
     *  @param[in]  state   The state to set the pipe(s) to. True==Enabled, False==Disabled
     *  @return Chimera::Status_t
     *
     *  |   Return Value   |              Explanation             |
     *  |:----------------:|:------------------------------------:|
     *  |               OK | The setting applied successfully     |
     *  |             FAIL | The setting could not be applied     |
     *  | INVAL_FUNC_PARAM | Function input parameter was invalid |
     *  |           LOCKED | The hardware is currently unavailabe |
     */
    Chimera::Status_t toggleRXDataPipe( const size_t pipe, const bool state );

    void toggleCE( const bool state );

    /**
     *   Write the transmit payload. If the TX FIFO is full when this is called, the data will simply be lost.
     *   The size of data written is capped at the max payload size.
     *
     *   @param[in]  buffer      Where to get the data
     *   @param[in]  len         Number of bytes to be sent
     *   @param[in]  writeType   Write using ACK (Command::W_TX_PAYLOAD) or NACK (Command::W_TX_PAYLOAD_NO_ACK)
     *   @return Current value of status register
     */
    Reg8_t writePayload( const void *const buf, const size_t len, const uint8_t writeType );

    /**
     *   Read the receive payload
     *
     *   The size of data read is the fixed payload size, see getPayloadSize()
     *
     *   @param[in]  buffer  Where to put the data
     *   @param[in]  len     Maximum number of bytes to read
     *   @return Current value of status register
     */
    Reg8_t readPayload( void *const buffer, const size_t bufferLength, const size_t payloadLength );

    Reg8_t writeCMD( const uint8_t cmd );

    void writeCMD( const Reg8_t cmd, const void *const buffer, const size_t length );

    void readCMD( const Reg8_t cmd, void *const buffer, const size_t length );

    uint8_t getStatus();


    void setCRCLength( const CRCLength length );
    CRCLength getCRCLength();
    void disableCRC();

    /**
     *   Set the device's address width from 3 to 5 bytes (24, 32 or 40 bit)
     *
     *   @param[in]  address_width   The address width to use
     *   @return void
     */
    void setAddressWidth( const AddressWidth address_width );

    void setStaticPayloadWidth( const PipeNumber pipe, const size_t bytes );

    size_t getStaticPayloadWidth( const PipeNumber pipe );

    /**
     *   Get the number of bytes used in the device address width
     *
     *   @return The current address width byte size
     */
    size_t getAddressWidthAsBytes();

    
    bool setAutoAck( const uint8_t pipe, const bool enable, const bool validate = false );


    /**
     *   Determine whether the hardware is an nRF24L01+ or not.
     *
     *   @return true if the hardware is an NRF24L01+
     */
    bool isPVariant();

    /**
     *   Mask interrupt generation for various signals. (true==disabled, false==enabled)
     *
     *   @param[in]  tx_ok       Mask transmission complete interrupts
     *   @param[in]  tx_fail     Mask transmit failure interrupts
     *   @param[in]  rx_ready    Mask payload received interrupts
     *   @return void
     */
    void maskIRQ( const bool tx_ok, const bool tx_fail, const bool rx_ready );

    /**
     *   Check if the RX FIFO is full
     *
     *   @return true if full, false if not
     */
    bool rxFifoFull();

    /**
     *   Check if the RX FIFO is empty
     *
     *   @return true if empty, false if data is available
     */
    bool rxFifoEmpty();

    /**
     *   Check if the TX FIFO is full
     *
     *   @return true if full, false if not
     */
    bool txFifoFull();

    /**
     *   Check if the TX FIFO is empty
     *
     *   @return true if empty, false if not
     */
    bool txFifoEmpty();

    /**
     *   Place the radio into Standby-I mode
     *
     *   Waits for all TX transfers to complete (or for max retries interrupt)
     *   before actually transitioning to the standby mode.
     *
     *   @return True if waiting transfers were successful, false if not
     */
    bool txStandBy();

    Chimera::Status_t writeAckPayload( const PipeNumber pipe, const void *const buffer, const size_t len );

    size_t getDynamicPayloadSize();

  protected:
    void writeCommand();


  private:
    Chimera::GPIO::GPIOClass_uPtr CSPin;
    Chimera::GPIO::GPIOClass_uPtr CEPin;
    Chimera::SPI::SPI_sPtr spi;

    std::array<uint8_t, SPI_BUFFER_LEN> spi_txbuff; /**< Internal transmit buffer */
    std::array<uint8_t, SPI_BUFFER_LEN> spi_rxbuff; /**< Internal receive buffer */

    bool mDynamicPayloadsEnabled;
    bool mFeaturesActivated;
    size_t mAddressBytes;
  };

  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

}    // namespace RF24::Hardware


#endif /* NRF24L01_HARDWARE_DRIVER_HPP */