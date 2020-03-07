/********************************************************************************
 *  File Name:
 *    shockburst.hpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP

#if defined( _WIN32 ) || defined( _WIN64 )

/* C++ Includes */
#include <array>
#include <memory>
#include <mutex>
#include <string_view>

/* Boost Includes */
#include <boost/asio.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>

/* RF24 Includes */
#include <RF24Node/src/hardware/definitions.hpp>
#include <RF24Node/src/hardware/types.hpp>
#include <RF24Node/src/network/frame/types.hpp>
#include <RF24Node/src/physical/simulator/pipe.hpp>
#include <RF24Node/src/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Shockburst
{
  /**
   *  Models a physical transceiver channel on the NRF24L01. Ideally this behaves as
   *  functionally close as possible to the real hardware.
   */
  class Socket
  {
  public:
    Socket( const RF24::Hardware::PipeNumber pipe, boost::asio::io_service &io_service, const std::string name );
    ~Socket();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink );

    /**
     *  If this channel has a write pipe available in hardware, the tx
     *  pipe will be configured.
     *
     *  @param[in]  address   The address containing the IP and port configuration
     *  @return bool
     */
    bool openWritePipe( const RF24::PhysicalAddress address );

    /**
     *  Closes the TX pipe if available in hardware
     *
     *  @return bool
     */
    bool closeWritePipe();

    /**
     *  Writes the buffer to the previously opened pipe
     *
     *  @param[in]  buffer    The data to be written
     *  @return void
     */
    void write( const RF24::Network::Frame::Buffer &buffer );

    /**
     *  Opens the RX pipe to listen to the given address
     *
     *  @param[in]  address   The address containing the IP and port configuration
     *  @return bool
     */
    bool openReadPipe( const RF24::PhysicalAddress address );

    /**
     *  Closes the RX pipe
     *
     *  @return bool
     */
    bool closeReadPipe();

    /**
     *	Reads some data out from the pipe
     *
     *	@param[in]	buffer    The buffer to be read into
     *	@return bool
     */
    bool read( RF24::Network::Frame::Buffer &buffer );

    /**
     *  Checks if any data is available to be read out of the RX pipe
     *
     *	@return bool
     */
    bool available();

    /**
     *  Sets the number of retries allowed for a packet TX before giving up
     *
     *  @param[in]  limit   The number of attempts
     *  @return void
     */
    void setRetryLimit( const size_t limit );

    /**
     *  Sets the delay between retries (in uS)
     *
     *  @param[in]  delay   The delay in microseconds
     *  @return void
     */
    void setRetryDelay( const size_t delay );

    /**
     *  Enables the receiver to listen to incoming data
     *
     *  @return void
     */
    void startListening();

    /**
     *  Disables the receiver from listening to incoming data
     *
     *  @return void
     */
    void stopListening();

    bool isListening();

    /**
     *  Flushes the TX pipe
     *
     *	@return void
     */
    void flushTX();

    /**
     *  Flushes the RX pipe
     *
     *	@return void
     */
    void flushRX();

    /**
     *  Enables/disables the dynamic payload functionality
     *
     *	@param[in]	state   Enable/disable
     *	@return void
     */
    void toggleDynamicPayloads( const bool state );

    /**
     *  Enables/disables the auto-acknowledgment functionality
     *
     *	@param[in]	state   Enable/disable
     *	@return void
     */
    void toggleAutoAck( const bool state );

  private:
    std::recursive_mutex mSettingsLock;

    bool mHasUserTxPipe; /**< Does this channel allow the user to write to the TX pipe? */
    bool mUseDynamicPayloads;
    bool mUseAutoAcknowledge;
    size_t mRetryLimit; /**< Max number of retries before quitting TX */
    size_t mRetryDelay; /**< Microsecond delay between TX retries */

    Pipe::RX rxPipe;
    Pipe::TX txPipe;

    RF24::PhysicalAddress cachedTXAddress;
    const RF24::Hardware::PipeNumber pipeID;

    uLog::SinkHandle logger;
  };

  using DataPipe_sPtr = std::shared_ptr<Socket>;
  using DataPipe_uPtr = std::unique_ptr<Socket>;
}    // namespace RF24::Physical::Shockburst


#endif /* _WIN32 || _WIN64 */
#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP */
