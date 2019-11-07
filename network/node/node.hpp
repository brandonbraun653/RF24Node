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
          childID            = ( nodeID & OCTAL_level1BitMask ) >> OCTAL_level1BitShift;
          parentID           = INVALID_NODE_ID;
          grandParentID      = INVALID_NODE_ID;
          greatGrandParentID = INVALID_NODE_ID;
          break;

        case 2:
          childID            = ( nodeID & OCTAL_level2BitMask ) >> OCTAL_level2BitShift;
          parentID           = ( nodeID & OCTAL_level1BitMask ) >> OCTAL_level1BitShift;
          grandParentID      = INVALID_NODE_ID;
          greatGrandParentID = INVALID_NODE_ID;
          break;

        case 3:
          childID            = ( nodeID & OCTAL_level3BitMask ) >> OCTAL_level3BitShift;
          parentID           = ( nodeID & OCTAL_level2BitMask ) >> OCTAL_level2BitShift;
          grandParentID      = ( nodeID & OCTAL_level1BitMask ) >> OCTAL_level1BitShift;
          greatGrandParentID = INVALID_NODE_ID;
          break;

        case 4:
          childID            = ( nodeID & OCTAL_level4BitMask ) >> OCTAL_level4BitShift;
          parentID           = ( nodeID & OCTAL_level3BitMask ) >> OCTAL_level3BitShift;
          grandParentID      = ( nodeID & OCTAL_level2BitMask ) >> OCTAL_level2BitShift;
          greatGrandParentID = ( nodeID & OCTAL_level1BitMask ) >> OCTAL_level1BitShift;
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