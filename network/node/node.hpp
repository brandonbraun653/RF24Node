/********************************************************************************
 *   File Name:
 *    node.hpp
 *
 *   Description:
 *
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_NODE_HPP
#define NRF24L01_NETWORK_LAYER_NODE_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <RF24Node/network/definitions.hpp>

namespace RF24::Network
{

  class Node
  {
  public:
    uint8_t childID            = INVALID_NODE_ID;
    uint8_t parentID           = INVALID_NODE_ID;
    uint8_t grandParentID      = INVALID_NODE_ID;
    uint8_t greatGrandParentID = INVALID_NODE_ID;

    /**
     *   Converts a decimal representation of a logical node address into the class properties
     *
     *   @param[in]  nodeID      Node id of a logical device (00, 043, 04444, etc)
     *   @return void
     */
    void fromDecimal( const uint16_t nodeID )
    {
      childID            = 0xFF & ( nodeID >> ( CHAR_BIT * 3 ) );
      parentID           = 0xFF & ( nodeID >> ( CHAR_BIT * 2 ) );
      grandParentID      = 0xFF & ( nodeID >> ( CHAR_BIT * 1 ) );
      greatGrandParentID = 0xFF & nodeID;
    }

    static uint8_t getLevel( const uint16_t address )
    {
      auto temp     = address;
      uint8_t level = 0u;

      while ( temp )
      {
        temp /= 8u;
        level++;
      }

      return level;
    }

    /**
     *   Converts an octal representation of a logical node address into the class properties
     *
     *   @param[in]  nodeID      Node id of a logical device (00, 043, 04444, etc)
     *   @return void
     */
    void fromOctal( const uint16_t nodeID )
    {
      auto temp = nodeID;
      nodeDepth = 0u;

      while ( temp )
      {
        temp /= 8;
        nodeDepth++;
      }

      switch ( nodeDepth )
      {
        case 1:
          childID            = ( nodeID & ADDR_LEVEL1_Msk ) >> ADDR_LEVEL1_Pos;
          parentID           = INVALID_NODE_ID;
          grandParentID      = INVALID_NODE_ID;
          greatGrandParentID = INVALID_NODE_ID;
          break;

        case 2:
          childID            = ( nodeID & ADDR_LEVEL2_Msk ) >> ADDR_LEVEL2_Pos;
          parentID           = ( nodeID & ADDR_LEVEL1_Msk ) >> ADDR_LEVEL1_Pos;
          grandParentID      = INVALID_NODE_ID;
          greatGrandParentID = INVALID_NODE_ID;
          break;

        case 3:
          childID            = ( nodeID & ADDR_LEVEL3_Msk ) >> ADDR_LEVEL3_Pos;
          parentID           = ( nodeID & ADDR_LEVEL2_Msk ) >> ADDR_LEVEL2_Pos;
          grandParentID      = ( nodeID & ADDR_LEVEL1_Msk ) >> ADDR_LEVEL1_Pos;
          greatGrandParentID = INVALID_NODE_ID;
          break;

        case 4:
          childID            = ( nodeID & ADDR_LEVEL4_Msk ) >> ADDR_LEVEL4_Pos;
          parentID           = ( nodeID & ADDR_LEVEL3_Msk ) >> ADDR_LEVEL3_Pos;
          grandParentID      = ( nodeID & ADDR_LEVEL2_Msk ) >> ADDR_LEVEL2_Pos;
          greatGrandParentID = ( nodeID & ADDR_LEVEL1_Msk ) >> ADDR_LEVEL1_Pos;
          break;

        default:
          childID            = INVALID_NODE_ID;
          parentID           = INVALID_NODE_ID;
          grandParentID      = INVALID_NODE_ID;
          greatGrandParentID = INVALID_NODE_ID;
          break;
      }
    }

    uint16_t asOctal()
    {
      return 0;
    }

    /**
     *   Adds a new child to the node at the lowest level of the tree
     */
    void addChild( const uint8_t id )
    {
    }

    bool childID_isValid()
    {
      return ( MIN_NODE_ID <= childID ) && ( childID <= MAX_NODE_ID );
    }

    bool parentID_isValid()
    {
      return ( MIN_NODE_ID <= parentID ) && ( parentID <= MAX_NODE_ID );
    }

    bool grandParentID_isValid()
    {
      return ( MIN_NODE_ID <= grandParentID ) && ( grandParentID <= MAX_NODE_ID );
    }

    bool greatGrandParentID_isValid()
    {
      return ( MIN_NODE_ID <= greatGrandParentID ) && ( greatGrandParentID <= MAX_NODE_ID );
    }

    Node()  = default;
    ~Node() = default;

  private:
    uint8_t nodeDepth = 0u;
  };

}    // namespace RF24::Network

#endif /* NRF24L01_NETWORK_LAYER_NODE_HPP */