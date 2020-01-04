/********************************************************************************
 *   File Name:
 *     types.hpp
 *
 *   Description:
 *     Common types used across the RF24Node project
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_COMMON_TYPES_HPP
#define RF24_NODE_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* RF24 Includes */
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24
{
  using PhysicalAddress = uint64_t; /**< Hardware address uniquely identifying a pipe in the network */
  using LogicalAddress  = uint16_t; /**< A node's fully qualified address in the network (octal) */
  using LogicalLevel    = uint16_t; /**< A level in the network tree structure */
  using LogicalID       = uint16_t; /**< Which logical channel the node is registered to */

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


}    // namespace RF24

#endif /* !RF24_NODE_COMMON_TYPES_HPP */