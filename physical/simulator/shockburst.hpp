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

/* C++ Includes */
#include <array>
#include <memory>
#include <mutex>
#include <string_view>

/* Boost Includes */
#include <boost/asio.hpp>

/* RF24 Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/physical/simulator/pipe.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Shockburst
{
  
  /**
   *  Models a physical transceiver channel on the NRF24L01. Ideally this behaves as 
   *  functionally close as possible to the real hardware.
   */
  class DataPipe
  {
  public:
    DataPipe( const RF24::Hardware::PipeNumber_t pipe, boost::asio::io_service &io_service );
    ~DataPipe();

    /**
     *  If this channel has a write pipe available in hardware, the tx
     *  pipe will be configured.
     *  
     *  @param[in]  address   The address containing the IP and port configuration
     *  @return bool
     */
    bool openWritePipe( const uint64_t address );

    /** 
     *  Closes the TX pipe if available in hardware
     *  
     *  @return bool
     */
    bool closeWritePipe();

    /** 
     *  Opens the RX pipe to listen to the given address
     *  
     *  @param[in]  address   The address containing the IP and port configuration
     *  @return bool
     */
    bool openReadPipe( const uint64_t address );

    /** 
     *  Closes the RX pipe
     *  
     *  @return bool
     */
    bool closeReadPipe();

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

    void flushTX();

    void flushRX();

    void toggleDynamicPayloads( const bool state );

    void toggleAutoAck( const bool state );

  private:
    std::recursive_mutex mSettingsLock;

    bool mHasUserTxPipe; /**< Does this channel allow the user to write to the TX pipe? */
    bool mUseDynamicPayloads;
    bool mUseAutoAcknowledge;
    size_t mRetryLimit;  /**< Max number of retries before quitting TX */
    size_t mRetryDelay;  /**< Microsecond delay between TX retries */

    Pipe::RX rxPipe;
    Pipe::TX txPipe;

    uint64_t cachedTXAddress;
    const RF24::Hardware::PipeNumber_t pipeID;
  };

  using DataPipe_sPtr = std::shared_ptr<DataPipe>;
  using DataPipe_uPtr = std::unique_ptr<DataPipe>;
}

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_SHOCKBURST_HPP */
