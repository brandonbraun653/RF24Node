/********************************************************************************
 *  File Name:
 *    pipe.hpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP

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

/* RF24 Includes */
#include <RF24Node/hardware/definitions.hpp>
#include <RF24Node/physical/simulator/shockburst.hpp>

namespace RF24::Physical::Sim
{
  
  class TXPipe
  {
  public:
    
    TXPipe( boost::asio::io_service &io_service );
    ~TXPipe();

    
    void configure( const std::string &ip, const size_t port);

    void write( const void *const data, const size_t length );


    void __PrvUpdateThread();

  private:
    boost::asio::ip::tcp::socket txSocket;

  };

  class RXPipe
  {
  public:
    RXPipe( boost::asio::io_service &io_service );
    ~RXPipe();
    
    void configure( const std::string &ip, const size_t port);

    void startListening();

    void stopListening();

    size_t read( void *const data, const size_t length );

    bool available();

    void __PrvUpdateThread();

    void __PrvOnAccept(const boost::system::error_code& error);

  protected:
    void kill();

  private:
    boost::thread rxThread;

    boost::asio::io_service &ioService;
    boost::asio::ip::tcp::socket rxSocket;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor;

    std::atomic<bool> allowListening;
    std::atomic<bool> killFlag;
    std::atomic<bool> threadExecuting;

    std::mutex packetBufferLock;
    std::queue<RF24::Physical::Sim::ShockBurstPacket> rxFIFO;

    RF24::Physical::Sim::SBArray networkBuffer;
  };

}

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP */
