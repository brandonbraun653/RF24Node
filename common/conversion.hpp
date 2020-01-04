/********************************************************************************
 *  File Name:
 *    conversion.hpp
 *
 *  Description:
 *    Utility functions for data conversions
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_CONVERSION_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_CONVERSION_HPP

/* C++ Includes */
#include <cstdlib>
#include <string>

/* RF24 Includes */
#include <RF24Node/common/types.hpp>
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Conversion
{
  /**
   *  Encodes an IP string and port number into the 64-bit address transmitted
   *  over the RF24 network.
   *  
   *  @param[in]  ip      The IPv4 address to be encoded
   *  @param[in]  port    The port to be encoded 
   *  @return uint64_t
   */
  uint64_t encodeAddress( const std::string &ip, const uint16_t port );

  /**
   *  Decodes a 64-bit node address into an IP string
   *  
   *  @param[in]  address   The address to be decoded
   *  @return string
   */
  std::string decodeIP( const uint64_t &address );

  /**
   *  Pulls the IP address from raw Shockburst data
   *  
   *  @param[in]  pkt     The data to be decoded
   *  @return string
   */
  std::string decodeIP( const Shockburst::PacketBuffer &pkt );

  /**
   *  Decodes a 64-bit node address into a port number
   *  
   *  @param[in]  address   The address to be decoded
   *  @return uint16_t
   */
  uint16_t decodePort( const uint64_t &address );

  /**
   *  Pulls the port number from raw Shockburst data
   *  
   *  @param[in]  pkt     The data to be decoded
   *  @return string
   */
  uint16_t decodePort( const Shockburst::PacketBuffer &pkt );

  /**
   *   Calculates the the pipe address of a logical node in the tree network. For information on exactly
   *   how the addressing is calculated, see: http://tmrh20.blogspot.com/ (scroll down mid-way)
   *
   *   @param[in]  nodeID      Octal node address (00, 02125, 0444, etc)
   *   @param[in]  pipe        The pipe number on the given nodeID
   *   @return The address assigned to that node's pipe
   */
  PhysicalAddress getPhysicalAddress( const LogicalAddress nodeID, const Hardware::PipeNumber_t pipeNum );

  
  /**
   *   output is octal
   */
  LogicalAddress levelToLogicalAddress( uint8_t level );
}

#endif	/* !RF24_NODE_PHYSICAL_SIMULATOR_CONVERSION_HPP */
