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
#include <string>

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
    
    void configure( const std::string &ip, const size_t port);

    void write( const void *const data, const size_t length );

  };

  class RXPipe
  {
  public:
    
    void configure( const std::string &ip, const size_t port);

    void read( void *const data, const size_t length );

    size_t nextPayloadSize();

    bool available();

  private:
    
  
  };

  class PipeConnectionHandler : public boost::enable_shared_from_this<PipeConnectionHandler>
  {
  public:
    typedef boost::shared_ptr<PipeConnectionHandler> pointer;


    static pointer create( boost::asio::io_service &io_service)
    {
      return pointer( new PipeConnectionHandler( io_service ) );
    }

    

    PipeConnectionHandler( boost::asio::io_service &io_service ) : sock( io_service )
    {
      rxData.fill( 0 );
      txData.fill( 0 );
    }

    void read()
    {
      boost::asio::async_read_until(sock, rxData, RF24::Physical::Sim::SBEndSequence );
    }


    void start()
    {
      sock.async_read_some( boost::asio::buffer( rxData, rxData.size() ),
                            boost::bind( &PipeConnectionHandler::handle_read, shared_from_this(),
                                         boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );

      sock.async_write_some( boost::asio::buffer( txData, txData.size() ),
                             boost::bind( &PipeConnectionHandler::handle_write, shared_from_this(),
                                          boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
    }

    void handle_read(const boost::system::error_code &err, size_t bytes_transferred )
    {
      
    }

    void handle_write(const boost::system::error_code &err, size_t bytes_transferred )
    {
    
    }


  private:
    boost::asio::ip::tcp::socket sock;
    RF24::Physical::Sim::SBArray rxData;
    RF24::Physical::Sim::SBArray txData;

  }
}

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_PIPE_HPP */