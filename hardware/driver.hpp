/********************************************************************************
*   File Name:
*     driver.hpp
*
*   Description:
*     Hardware driver for the NRF24L01(+)
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef NRF24L01_HARDWARE_DRIVER_HPP
#define NRF24L01_HARDWARE_DRIVER_HPP

/* C++ Includes */

/* Chimera Includes */
#include <Chimera/extensions/spi_ext.hpp>
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/threading.hpp>

/* Driver Includes */
#include <RF24Node/hardware/definitions.hpp>

namespace RF24::Hardware
{
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
  class Driver : public Chimera::SPI::SPIAcceptor,
                 public Chimera::Threading::Lockable
  {
  public:
    /*-------------------------------------------------
    Constructors/Destructors
    -------------------------------------------------*/
    Driver();
    ~Driver();

    /*-------------------------------------------------
    SPI Acceptor Functions
    -------------------------------------------------*/
    Chimera::Status_t attachSPI( Chimera::SPI::SPIClass_sPtr &spi ) final override;
    Chimera::Status_t attachSPI( Chimera::SPI::SPIClass_sPtr &spi, Chimera::SPI::DriverConfig &setup ) final override;
    Chimera::Status_t attachSPI( Chimera::SPI::SPIClass_uPtr spi ) final override;
    Chimera::Status_t attachCS( Chimera::GPIO::PinInit &CSConfig ) final override;
    Chimera::Status_t attachCS( Chimera::GPIO::GPIOClass_sPtr &CSPin ) final override;
    Chimera::Status_t attachCS( Chimera::GPIO::GPIOClass_uPtr CSPin ) final override;

    /*-------------------------------------------------
    Driver Functions
    -------------------------------------------------*/
    /**
     *  Initializes the driver and any preliminary configurations for the NRF24 chip. Upon
     *  exiting, the driver will be ready for use and sitting in an idle state.
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
    Chimera::Status_t initialize( const Chimera::GPIO::PinInit &CE, const Chimera::GPIO::PinInit &CS );

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

    Reg8_t readRegister( const Reg8_t addr );
    Reg8_t readRegister( const Reg8_t addr, uint8_t *const buf, size_t len );

    Reg8_t writeRegister( const Reg8_t addr, const Reg8_t value );
    Reg8_t writeRegister( const Reg8_t addr, const Reg8_t *const buffer, size_t len );

    Reg8_t setRegisterBits( const Reg8_t addr, const Reg8_t mask );
    Reg8_t clrRegisterBits( const Reg8_t addr, const Reg8_t mask );

    void toggleRFPower( const bool state );

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


  private:
    Chimera::GPIO::GPIOClass_uPtr CSPin;
    Chimera::GPIO::GPIOClass_uPtr CEPin;
    Chimera::SPI::SPIClass_sPtr spi;
    Chimera::SPI::DriverConfig spiConfig;

    std::array<uint8_t, SPI_BUFFER_LEN> spi_txbuff; /**< Internal transmit buffer */
    std::array<uint8_t, SPI_BUFFER_LEN> spi_rxbuff; /**< Internal receive buffer */
  };


} // namespace RF24::Hardware



#endif  /* NRF24L01_HARDWARE_DRIVER_HPP */