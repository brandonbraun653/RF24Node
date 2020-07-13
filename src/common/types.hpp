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
  using DeviceName                        = std::array<char, DEVICE_NAME_LEN + 1u>;


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
      CONNECT_PROC_UNKNOWN,     /**< Something happened but it was not known how to be handled */
      CONNECT_PROC_SUCCESS,     /**< The connection to the parent node succeeded */
      CONNECT_PROC_FAIL,        /**< The connection to the parent node failed for some reason */
      CONNECT_PROC_TIMEOUT,     /**< The connection to the parent node timed out */
      CONNECT_PROC_NO_RESPONSE, /**< The other node did not respond to a packet */
    };

    enum class BindSite : uint8_t
    {
      PARENT = 0,
      CHILD_1,
      CHILD_2,
      CHILD_3,
      CHILD_4,
      CHILD_5,
      INVALID,

      FIRST = PARENT,
      LAST  = CHILD_5,
      MAX   = LAST - FIRST + 1
    };

    enum class Direction : uint8_t
    {
      CONNECT,
      DISCONNECT
    };

    /**
     *	Defines an event callback for the user to have invoked when a connection
     *	to a parent node succeeds (child perspective) or a child is bound to the
     *	parent node (parent perspective).
     *
     *	@param[in]  result    Whether or not the connection succeeded
     *	@param[in]  id        The connection id describing which node the result applies to
     *	@return void
     */
    using OnCompleteCallback = void ( * )( const Result result, const BindSite id );

    using BindSiteList = std::array<LogicalAddress, static_cast<size_t>( BindSite::MAX )>;

  }    // namespace Connection



  /**
   *  Function pointer callback to be invoked on an event occurrence
   */
  using EventFuncPtr_t = void ( * )( void );


  using OnConnectCallback = void ( * )( const Connection::BindSite id );


  using OnDisconnectCallback = void ( * )( const Connection::BindSite id );
}    // namespace RF24

#endif /* !RF24_NODE_COMMON_TYPES_HPP */