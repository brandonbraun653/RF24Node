/********************************************************************************
 *  File Name:
 *    pipe_types.hpp
 *
 *  Description:
 *    Types used for modeling NRF24L01 hardware pipes
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_PIPE_TYPES_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_PIPE_TYPES_HPP

/* C++ Includes */
#include <functional>

/* RF24 Includes */
#include <RF24Node/physical/simulator/shockburst_types.hpp>

namespace RF24::Physical::Pipe
{
  class RX;
  class TX;

  using RXPipeCallback = std::function<void( RX *const pipe, Shockburst::PacketBuffer &data )>;
  using TXPipeCallback = std::function<void( TX *const pipe )>;
}

#endif	/* !RF24_NODE_PHYSICAL_SIMULATOR_PIPE_TYPES_HPP */
