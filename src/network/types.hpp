/********************************************************************************
 *   File Name:
 *    types.hpp
 *
 *   Description:
 *    Types used in implementing the network driver for the NRF24 radios.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_TYPES_HPP
#define NRF24L01_NETWORK_LAYER_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* RF24 Includes */
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/network/definitions.hpp>

namespace RF24::Network
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class Mode : size_t
  {
    NET_MODE_INVALID,
    NET_MODE_STATIC,
    NET_MODE_MESH
  };

  enum Services : uint8_t
  {
    NET_SERVICE_DHCP,
    NET_SERVICE_MSG_FORWARDING
  };

  enum class StatusFlag : uint8_t
  {
    SF_IDLE,
    SF_IN_PROGRESS,

    NUM_OPTIONS
  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct TransferControlBlock
  {
    StatusFlag flag;  /**< Flags indicating current status */
    size_t startTime; /**< When this packet was first tx'd */
    size_t lastTime;  /**< Last time this packet was touched */
    size_t timeout;   /**< Computed packet timeout value */
  };

  struct Config
  {
    void *rxQueueBuffer;                /**< (Optional) External buffer for the network rx queue */
    size_t rxQueueSize;                 /**< (Required) Size of the network rx queue in bytes */
    void *txQueueBuffer;                /**< (Optional) External buffer for the network tx queue */
    size_t txQueueSize;                 /**< (Required) Size of the network tx queue in bytes */
    Mode mode;                          /**< Networking mode to use */
    LogicalAddress nodeStaticAddress;   /**< Static address for the node (Only available in NET_MODE_STATIC) */
    LogicalAddress parentStaticAddress; /**< Static address of the parent (Only available in NET_MODE_STATIC) */
  };
  static_assert( sizeof( Config ) % sizeof( size_t ) == 0, "Struct not aligned properly" );

  /**
   *  Consolidates metrics about where the next destination of
   *  a frame should be transmitted and what routing style it should
   *  take.
   */
  struct JumpType
  {
    LogicalAddress hopAddress;  /**< The address of the next node in the transmit sequence */
    RoutingStyle routing;       /**< The type of routing that should be used for transmission */
  };

  /**
   *  Network object system control block to manage the runtime
   *  state of the network.
   */
  struct SystemCB
  {
    bool connectedToNet;      /**< Device has connected to the network through the parent node */
    size_t connectedToNetAt;  /**< Time stamp the network connection occurred at */

    void clear()
    {
      connectedToNet = false;
      connectedToNetAt = 0;
    }
  };


  /**
   *  Control block that describes a bind site. These objects will track the lifetime
   *  information about any existing connections present at a bind site.
   */
  struct BindSiteCB
  {
    /**
     *  Indicates if the data present in this struct should be trusted
     */
    bool valid;

    /**
     *  High level on/off switch to enable a particular bind site to maintain
     *  a connection pipe to other nodes.
     */
    bool enabled;

    /**
     *  Is the node described by this control block connected to the device?
     */
    bool connected;

    /**
     *  Tracks which bind site this control block is for
     */
    RF24::Connection::BindSite bindId;

    /**
     *  Marks the system time at which the site had any sort of activity
     */
    size_t lastRXActive;

    /**
     *  Number of milliseconds past lastPing which implies the network connection is
     *  stale and the node no longer is confident of its connection.
     */
    size_t staleExpirationDelta;

    /**
     *  The address of the node connected to this bind site
     */
    RF24::LogicalAddress address;

    /**
     *  Various event callbacks that can be executed
     */
    OnConnectCallback onConnect;       /**< Executes when the bind site connects */
    OnDisconnectCallback onDisconnect; /**< Executes when the bind site disconnects */

    BindSiteCB()
    {
      reset();
    }

    /**
     *  Clears the control block data back to defaults
     */
    void reset()
    {
      valid                = false;
      enabled              = false;
      connected            = false;
      bindId               = Connection::BindSite::INVALID;
      lastRXActive         = 0;
      staleExpirationDelta = RF24::Network::DFLT_CONNECTION_TIMEOUT;
      address              = RF24::Network::RSVD_ADDR_INVALID;

      onConnect    = nullptr;
      onDisconnect = nullptr;
    }
  };

  using BindSiteCBList = std::array<BindSiteCB, static_cast<size_t>( RF24::Connection::BindSite::MAX )>;

}    // namespace RF24::Network


#endif /* !NRF24L01_NETWORK_LAYER_TYPES_HPP */