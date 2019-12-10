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
  /**
   *  Represents a node's address in the network in octal format
   */
  using LogicalAddress = uint16_t;
  

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