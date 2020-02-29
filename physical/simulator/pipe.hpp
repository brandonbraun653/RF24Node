/********************************************************************************
 *  File Name:
 *    pipe.hpp
 *
 *  Description:
 *    Models the hardware pipes on the NRF24L01 using UDP sockets
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP

#if defined( RF24_SIMULATOR )

/* C++ Includes */
#include <atomic>
#include <string>
#include <memory>
#include <mutex>
#include <queue>

/* Boost Includes */
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/physical/simulator/pipe_types.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Pipe
{
  class TX
  {
  public:
    TX(boost::asio::io_service &io_service, const std::string name, const size_t pipe );
    ~TX();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink );

    /**
     *  Opens the pipe
     *
     *  @return void
     */
    void openPipe();

    /**
     *  Closes the pipe
     *
     *  @return void
     */
    void closePipe();

    /**
     *  Checks if the pipe is currently open
     *
     *	@return bool
     */
    bool isOpen();

    /**
     *  Writes raw data to the configured RX pipe
     *
     *  @param[in]  data    The data to be written
     */
    void write( const Shockburst::PacketBuffer &data );

    /**
     *  Clears the FIFO queues of data
     *
     *  @return void
     */
    void flush();

    /**
     *  User callback to be executed when an asynchronous TX event occurs
     *
     *  @param[in]  callback    The user's callback
     *  @return void
     */
    void onTransmit( TXPipeCallback &callback );

  private:
    void updateThread();
    void onAsyncTransmit( const boost::system::error_code &error, size_t bytes_transferred );

    boost::asio::io_service &mIOService;    /**< IoService needed to run event handling */
    boost::asio::ip::udp::socket mTXSocket; /**< UDP socket that models the TX pipe */
    boost::thread mTXThread;                /**< Event handler thread */
    std::recursive_mutex mBufferLock;       /**< Lock for the FIFO message queue */
    Shockburst::PacketBuffer mBuffer;       /**< Raw buffer for outgoing messages */
    std::atomic<bool> mTXEventProcessed;    /**< Flag indicating when the RX event was processed */
    TXPipeCallback mUserCallback;           /**< User callback for RX event */

    uLog::SinkHandle mLogger;
    const std::string mName;
    const size_t mPipeNumber;
  };

  class RX
  {
  public:
    RX( boost::asio::io_service &io_service, const std::string name, const size_t pipe  );
    ~RX();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink );

    /**
     *  Opens the RX pipe for listening
     *
     *  @param[in]  ip      The IPv4 address of the pipe
     *  @param[in]  port    The port the pipe listens on
     *  @return void
     */
    void openPipe( const std::string &ip, const uint16_t port );

    /**
     *  Closes the pipe
     *
     *  @return void
     */
    void closePipe();

    /**
     *  Enable the pipe's ability to receive new packets
     *
     *  @return void
     */
    void startListening();

    /**
     *  Disable the pipe's ability to receive new packets
     *
     *  @return void
     */
    void stopListening();

    /**
     *  Checks if the pipe is currently listening
     *
     *  @return bool
     */
    bool isListening();

    /**
     *  Checks if there is any data available to be read
     *
     *  @return bool
     */
    bool available();

    /**
     *  Reads a single packet out from the FIFO
     *
     *  @return RawBuffer
     */
    Shockburst::PacketBuffer read();

    /**
     *  Clears the FIFO queues of data
     *
     *  @return void
     */
    void flush();

    /**
     *  User callback to be executed when an asynchronous RX event occurs
     *
     *  @param[in]  callback    The user's callback
     *  @return void
     */
    void onReceive( RXPipeCallback &callback );

  private:
    /**
     *  Periodically processes the queued io_service work to handle RX events
     *
     *  @return void
     */
    void updateThread();

    /**
     *  Async callback to io_service receive event
     *
     *  @param[in]  error               RX error code
     *  @param[in]  bytes_transferred   How many bytes were actually transferred
     *  @return void
     */
    void onAsyncReceive( const boost::system::error_code &error, size_t bytes_transferred );


    boost::asio::io_service &mIOService;    /**< IoService needed to run event handling */
    boost::asio::ip::udp::socket mRXSocket; /**< UDP socket that models the RX pipe */
    boost::thread mRXThread;                /**< Event handler thread */
    RXPipeCallback mUserCallback;           /**< User callback for RX event */
    Shockburst::PacketBuffer mBuffer;       /**< Raw buffer for incoming messages */
    std::atomic<bool> mAllowListening;      /**< Flag to enable/disable listening for messages */
    std::atomic<bool> mRXEventProcessed;    /**< Flag indicating when the RX event was processed */
    std::recursive_mutex mBufferLock;       /**< Lock for the FIFO message queue */

    uLog::SinkHandle mLogger;
    const std::string mName;
    const size_t mPipeNumber;
  };

}    // namespace RF24::Physical::Pipe

#endif /* _WIN32 || _WIN64 */
#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP */
