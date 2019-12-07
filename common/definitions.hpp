/********************************************************************************
*   File Name:
*     definitions.hpp
*
*   Description:
*     Common definitions used in the RF24Node library
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef RF24_NODE_COMMON_DEFINITIONS_HPP
#define RF24_NODE_COMMON_DEFINITIONS_HPP

/* RF24 Includes */
#include <RF24Node/common/types.hpp>

namespace RF24
{
  /*-------------------------------------------------
  Defines the root nodes that all RF24 nodes branch out 
  from when creating a network of devices in a tree like 
  structure. Very useful when there could be multiple end 
  devices that could sink data or provide network addresses.

  At a minimum, each network must have a single node
  that has the RootNode0 address.
  -------------------------------------------------*/
  static constexpr LogicalAddress RootNode0 = 0000;
  static constexpr LogicalAddress RootNode1 = 1000;
  static constexpr LogicalAddress RootNode2 = 2000;
  static constexpr LogicalAddress RootNode3 = 3000;


} // namespace RF24



#endif  /* !RF24_NODE_COMMON_DEFINITIONS_HPP */