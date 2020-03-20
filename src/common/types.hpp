/********************************************************************************
 *  File Name:
 *     types.hpp
 *
 *  Description:
 *     Common types used across the RF24Node project
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_COMMON_TYPES_HPP
#define RF24_NODE_COMMON_TYPES_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* RF24 Includes */
#include <RF24Node/src/simulator/sim_definitions.hpp>

namespace RF24
{
  using PhysicalAddress = uint64_t; /**< Hardware address uniquely identifying a pipe in the network */
  using LogicalAddress  = uint16_t; /**< A node's fully qualified address in the network (octal) */
  using LogicalLevel    = uint16_t; /**< A level in the network tree structure */
  using LogicalID       = uint16_t; /**< Which logical channel the node is registered to */


  static constexpr size_t DEVICE_NAME_LEN = 16;
  using DeviceName = std::array<char, DEVICE_NAME_LEN + 1u>;

  /**
   *  Function pointer callback to be invoked on an event occurrence
   */
  using EventFuncPtr_t = void ( * )( void );

  enum class Event
  {
    CONNECT,
    DISCONECT,
    MSG_TX,
    MSG_RX
  };

  namespace Connection
  {
    enum class Result
    {
      CONNECTION_UNKNOWN, /**< Something happened but it was not known how to be handled */
      CONNECTION_SUCCESS, /**< The connection to the node succeeded */
      CONNECTION_FAILED,  /**< The connection to the node failed for some reason */
      CONNECTION_TIMEOUT, /**< The connection to the node timed out */
    };

    /**
     *	Defines a callback for the user to have invoked
     *	
     *	@param[in]  result    Whether or not the connection succeeded
     *	@return void
     */
    using Callback = void(*)(const Result result);
  }

}    // namespace RF24

#endif /* !RF24_NODE_COMMON_TYPES_HPP */