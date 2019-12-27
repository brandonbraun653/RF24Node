/********************************************************************************
 *   File Name:
 *    header.hpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_HEADER_HPP
#define NRF24L01_NETWORK_LAYER_HEADER_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <RF24Node/network/definitions.hpp>
#include <RF24Node/network/frame/frame_types.hpp>
#include <RF24Node/network/header/header_definitions.hpp>
#include <RF24Node/network/header/header_types.hpp>

namespace RF24::Network
{
  /**
   *   Header which is sent with each message. The frame put over the air consists of this header
   *   + message. Each are addressed to the appropriate node, and the network forwards them on to
   *   their final destination.
   */
  class HeaderHelper
  {
  public:
    static void initialize();

    /**
     *  Constructor to build from a memory buffer
     *
     *  @param[in]  buffer      Buffer containing byte data that can be converted to Payload_t
     */
    HeaderHelper( const FrameBuffer &buffer );

    /**
     *  Alternate constructor simply for ease of use
     *
     *  @param[in]  dstNode     The Octal format, logical node address where the message is going
     *  @param[in]  msgType     The type of message, as given by RF24Network::MessageType
     */
    HeaderHelper( const RF24::LogicalAddress dstNode, const HeaderMessage type = MSG_TX_NORMAL );


    HeaderHelper( const FrameHeaderField &rawData );

    HeaderHelper();
    ~HeaderHelper();

    /**
     *  Allows updating an existing Header object from memory array
     *
     *  @param[in]  buffer      Buffer containing byte data that can be converted to Payload_t
     *  @return void
     */
    void operator()( const FrameBuffer &buffer );

    /**
     *  Copy constructor
     *
     *  @param[in]  header      Header class object to copy
     *  @return void
     */
    void operator=( const HeaderHelper &headerClass );

    /**
     *   Copy constructor
     *
     *   @param[in]  headerData  Header data object to copy
     *   @return void
     */
    void operator=( const FrameHeaderField &headerData );

    /**
     *   Convert the header payload into a string. Uses internal memory shared across all
     *   header objects to create the string, which will be overridden on the next call.
     *
     *   @return String representation of the payload
     */
    const char *toString() const;

    void setDestinationNode( const LogicalAddress destination );
    void setSourceNode( const LogicalAddress source );
    void setType( const HeaderMessage type );

    LogicalAddress getDestinationNode();
    LogicalAddress getSourceNode();
    HeaderMessage getType();

    FrameHeaderField getField();

  private:    
    FrameHeaderField data;

  };
}    // namespace RF24::Network

#endif /* NRF24L01_NETWORK_LAYER_HEADER_HPP */