/********************************************************************************
 *  File Name:
 *    utility.hpp
 *
 *  Description:
 *    Utility functions used across the NRF24L01 networking stack
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_COMMON_UTILITY_HPP
#define RF24_NODE_COMMON_UTILITY_HPP

/* C++ Includes */
#include <cstdint>

/* RF24 Includes */
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/hardware/types.hpp>

namespace RF24
{
  /**
   *  Checks if the given address is valid for a network child node
   *
   *  @param[in]  address     The address to be checked
   *  @return bool
   */
  bool isAddressChild( const LogicalAddress address );

  /**
   *	Checks if the given address is valid for a network root node
   *
   *	@param[in]	address     The address to be checked
   *	@return bool
   */
  bool isAddressRoot( const LogicalAddress address );

  /**
   *	Checks if the address is valid from the perspective of the question:
   *	"Can this node exist on the network?". This includes root nodes as
   *	well as parent/child nodes.
   *
   *	@note   See isAddressChild() and isAddressRoot() for more specific implementations
   *
   *	@param[in]	address     The address to be checked
   *	@return bool
   */
  bool isAddressValid( const LogicalAddress address );

  /**
   *	Checks if the given network address is one of the special
   *	system reserved addresses that cannot be used by users.
   *
   *	@param[in]	address     The address to be checked
   *	@return bool
   */
  constexpr bool isAddressReserved( const LogicalAddress address );

  /**
   *	Checks if the descendent node falls somewhere in the tree structure that the parent
   *	node is the root of. This searches through all levels.
   *
   *	@param[in]	parent        The parent of the tree
   *	@param[in]	descendent    The address to checked if in the parent tree
   *	@return bool
   */
  bool isDescendent( const LogicalAddress parent, const LogicalAddress descendent );

  /**
   *	Checks if the child node is a direct descendent of the parent
   *
   *	@param[in]	parent        The parent being checked
   *	@param[in]	child         The child being checked
   *	@return bool
   */
  bool isDirectDescendent( const LogicalAddress parent, const LogicalAddress child );

  /**
   *	Gets the parent address of the child
   *
   *	@param[in]	child
   *	@return RF24::LogicalAddress
   */
  LogicalAddress getParent( const LogicalAddress child );

  /**
   *	Gets the tree structure level associated with the given address
   *
   *	@warning  This function assumes the address is valid
   *
   *	@param[in]	address
   *	@return RF24::LogicalLevel
   */
  LogicalLevel getLevel( LogicalAddress address );

  /**
   *	Parses a node address to return which ID it is registered as at a given level
   *
   *	@param[in]	address
   *	@param[in]	level
   *	@return RF24::LogicalID
   */
  LogicalID getIdAtLevel( const LogicalAddress address, const LogicalLevel level );
}    // namespace RF24

#endif /* !RF24_NODE_COMMON_UTILITY_HPP */
