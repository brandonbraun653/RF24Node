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

/* C++ Includes */
#include <string>

/* RF24 Includes */
#include <RF24Node/src/common/types.hpp>

namespace RF24
{
  static const std::string_view RF24_SW_VER = "1.0.0";

  /*-------------------------------------------------
  Defines the root nodes that all RF24 nodes branch out
  from when creating a network of devices in a tree like
  structure. Very useful when there could be multiple end
  devices that could sink data or provide network addresses.

  At a minimum, each network must have a single node
  that has the RootNode0 address.
  -------------------------------------------------*/
  static constexpr LogicalAddress RootNode0 = 00000;
  static constexpr LogicalAddress RootNode1 = 01000;
  static constexpr LogicalAddress RootNode2 = 02000;
  static constexpr LogicalAddress RootNode3 = 03000;

  static constexpr LogicalLevel MIN_OCTAL_NUMBER = 0u;
  static constexpr LogicalLevel MAX_OCTAL_NUMBER = 7u;

  static constexpr LogicalLevel NODE_LEVEL_0       = 0u; /**< Reserved for root nodes only */
  static constexpr LogicalLevel NODE_LEVEL_1       = 1u;
  static constexpr LogicalLevel NODE_LEVEL_2       = 2u;
  static constexpr LogicalLevel NODE_LEVEL_3       = 3u;
  static constexpr LogicalLevel NODE_LEVEL_4       = 4u;
  static constexpr LogicalLevel NODE_LEVEL_5       = 5u;
  static constexpr LogicalLevel NODE_LEVEL_MAX     = NODE_LEVEL_5;
  static constexpr LogicalLevel NODE_LEVEL_MIN     = NODE_LEVEL_1;
  static constexpr LogicalLevel NODE_LEVEL_ABS_MIN = NODE_LEVEL_0;
  static constexpr LogicalLevel NODE_LEVEL_INVALID = NODE_LEVEL_MAX + 1u;

  static constexpr LogicalID NODE_ID_ROOT    = 0u;
  static constexpr LogicalID NODE_ID_1       = 1u;
  static constexpr LogicalID NODE_ID_2       = 2u;
  static constexpr LogicalID NODE_ID_3       = 3u;
  static constexpr LogicalID NODE_ID_4       = 4u;
  static constexpr LogicalID NODE_ID_5       = 5u;
  static constexpr LogicalID NODE_ID_6       = 6u;
  static constexpr LogicalID NODE_ID_7       = 7u;
  static constexpr LogicalID NODE_ID_MAX     = NODE_ID_7;
  static constexpr LogicalID NODE_ID_INVALID = NODE_ID_MAX + 1u;

  /*-------------------------------------------------------------------------------
  Logging Filters:
    Filters named "DBG_LOG_XXX" are top level filters for that particular layer and
    are the most generic possible.

    Filters named "DBG_LOG_XXX_TRACE" are intended to provide short statements about
    where lines of code have been hit, to help trace execution flow at a fine level.

    Filters named "DBG_LOG_XXX_PROC_YYY" are meant to follow execution flow of a
    process running at that layer. These tend to be state machines of some sort.
  -------------------------------------------------------------------------------*/
  static constexpr bool ENABLE_DBG_LOG = true;  /**< Master logging on/off switch */

  /*-------------------------------------------------
  Application Layer
  -------------------------------------------------*/
  static constexpr bool DBG_LOG_APP = true && ENABLE_DBG_LOG;

  /*-------------------------------------------------
  Network Layer
  -------------------------------------------------*/
  static constexpr bool DBG_LOG_NET              = true && ENABLE_DBG_LOG;
  static constexpr bool DBG_LOG_NET_TRACE        = true && ENABLE_DBG_LOG;
  static constexpr bool DBG_LOG_NET_PROC_CONNECT = true && ENABLE_DBG_LOG;

  /*-------------------------------------------------
  Physical Layer
  -------------------------------------------------*/
  static constexpr bool DBG_LOG_PHY       = false && ENABLE_DBG_LOG;
  static constexpr bool DBG_LOG_PHY_TRACE = false && ENABLE_DBG_LOG;

}    // namespace RF24


#endif /* !RF24_NODE_COMMON_DEFINITIONS_HPP */