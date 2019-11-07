/********************************************************************************
 *   File Name:
 *    path.hpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_PATH_HPP
#define NRF24L01_NETWORK_LAYER_PATH_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Driver Includes */
#include <RF24Node/network/definitions.hpp>

namespace RF24::Network
{
#pragma pack( push )
#pragma pack( 1 )
    struct MessagePath
    {
      uint8_t hop0 = INVALID_NODE_ID; /**< This node is the node that sent the message */
      uint8_t hop1 = INVALID_NODE_ID; /**< Potential parent node id */
      uint8_t hop2 = INVALID_NODE_ID; /**< Potential grandparent node id */
      uint8_t hop3 = INVALID_NODE_ID; /**< Potential great grandparent node id */
    };
#pragma pack( pop )

  /**
   *   Payload that can be used to track the path of a message through the network.
   *   In total there can be three hops + the original node id which when combined build up
   *   the full network tree path that was taken.
   *
   *   This is mostly useful for nodes that are just joining the network and do not have a
   *   fully defined network address. The message could be coming from anywhere and the master
   *   needs to know where to respond.
   */
  class Path
  {
  public:
    Path( std::array<uint8_t, MAX_FRAME_PAYLOAD_SIZE> messagePath );

    void process();

    MessagePath msgPath;
    Level networkLevel = Level::LEVEL0;
  };
}

#endif  /* NRF24L01_NETWORK_LAYER_PATH_HPP */