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
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/hardware/types.hpp>
#include <RF24Node/src/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Conversion
{
#if defined( RF24_SIMULATOR )
  /**
   *  Encodes an IP string and port number into the 64-bit address transmitted
   *  over the RF24 network.
   *  
   *  @param[in]  ip      The IPv4 address to be encoded
   *  @param[in]  port    The port to be encoded 
   *  @return uint64_t
   */
  PhysicalAddress encodeAddress( const std::string &ip, const Port port );

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
   *  @param[in]  pkt       The data to be decoded
   *  @return string
   */
  std::string decodeIP( const Shockburst::PacketBuffer &pkt );

  /**
   *  Decodes a 64-bit node address into a port number
   *  
   *  @param[in]  address   The address to be decoded
   *  @return Port
   */
  Port decodePort( const uint64_t &address );

  /**
   *  Pulls the port number from raw Shockburst data
   *  
   *  @param[in]  pkt       The data to be decoded
   *  @return string
   */
  Port decodePort( const Shockburst::PacketBuffer &pkt );

#endif  /* RF24_SIMULATOR */

  /**
   *   Calculates the the pipe address of a logical node in the tree network. For information on exactly
   *   how the addressing is calculated, see: http://tmrh20.blogspot.com/ (scroll down mid-way)
   *
   *   @param[in]  address     Octal node address (00, 02125, 0444, etc)
   *   @param[in]  pipe        The pipe number on the given nodeID
   *   @return The address assigned to that node's pipe
   */
  PhysicalAddress getPhysicalAddress( const LogicalAddress address, const Hardware::PipeNumber pipeNum );

  /**
   *	Gets the pipe this node *could* be bound to on the network. This does not mean that the node is
   *	actually connected, just that if it were connected, this is the pipe it would be connected to.
   *	
   *	@param[in]	address       Octal address to get the pipe for
   *	@return RF24::Hardware::PipeNumber
   */
   RF24::Hardware::PipeNumber getExpectedPipe( const ::RF24::LogicalAddress address );
}

#endif	/* !RF24_NODE_PHYSICAL_SIMULATOR_CONVERSION_HPP */
