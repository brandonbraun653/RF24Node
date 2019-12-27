/********************************************************************************
 *   File Name:
 *       network.hpp
 *
 *   Description:
 *       Implements the RF24 Network layer. Based on the work originally done by
 *       James Coliz on the popular RF24Network library:
 *       https://github.com/nRF24/RF24Network.
 *
 *       This version of the code attempts to make performance improvements, modernize
 *       the interface using modern C++, and abstract things further away from specific
 *       platforms. The common platform interface is from the Chimera library:
 *       https://github.com/brandonbraun653/Chimera
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24L01_NETWORK_LAYER_HPP
#define NRF24L01_NETWORK_LAYER_HPP

/* C++ Includes */
#include <cstddef>
#include <cstdint>
#include <climits>
#include <cmath>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* uLog Includes */
#include <uLog/types.hpp>

/* Driver Includes */
#include <RF24Node/common/types.hpp>
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/interfaces/network_intf.hpp>
#include <RF24Node/network/definitions.hpp>
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/header/header.hpp>
#include <RF24Node/network/queue/queue.hpp>
#include <RF24Node/physical/physical.hpp>


namespace RF24::Network
{
  /**
   *  Checks if the given node is a root node. 
   */
  bool isARootNode( const LogicalAddress );


  class Driver : public Interface
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink ) final override;
    Chimera::Status_t attachPhysicalDriver( RF24::Physical::Interface_sPtr &physicalLayer ) final override;
    Chimera::Status_t initRXQueue( void *buffer, const size_t size ) final override;
    Chimera::Status_t initTXQueue( void *buffer, const size_t size ) final override;
    Chimera::Status_t begin( const uint8_t channel, const uint16_t nodeAddress,
                const RF24::Hardware::DataRate dataRate = RF24::Hardware::DataRate::DR_1MBPS,
                const RF24::Hardware::PowerAmplitude pwr = RF24::Hardware::PowerAmplitude::PA_MAX ) final override;
    HeaderMessage update() final override;
    bool available() const final override;
    uint16_t peek( HeaderHelper &header ) final override;
    bool peek( HeaderHelper &header, void *const message, const uint16_t maxlen ) final override;
    bool read( HeaderHelper &header, void *const message, const uint16_t maxlen ) final override;
    bool write( HeaderHelper &header, const void *message, const uint16_t len ) final override;
    bool write( HeaderHelper &header, const void *message, uint16_t length, NodeAddressType writeDirect ) final override;
    bool multicast( HeaderHelper &header, const void *message, const uint16_t length, const uint8_t level ) final override;
    void setMulticastLevel( const uint8_t level ) final override;
    bool setAddress( const LogicalAddress address ) final override;
    LogicalAddress getLogicalAddress() final override;
    LogicalAddress parent() const final override;

    void setNetworkingMode( const Mode mode ) final override;

    /**
     * Determines whether update() will return after the radio buffers have been emptied (DEFAULT), or
     * whether to return immediately when (most) system types are received.
     *
     * As an example, this is used with RF24Mesh to catch and handle system messages without loading them into the user cache.
     *
     * The following reserved/system message types are handled automatically, and not returned.
     *
     * | System Message Types (Not Returned) |
     * |-----------------------|
     * | NETWORK_ADDR_RESPONSE |
     * | NETWORK_ACK           |
     * | NETWORK_PING          |
     * | NETWORK_POLL <br>(With multicast enabled) |
     * | NETWORK_REQ_ADDRESS   |
     *
     */
    void toggleSystemMessageReturn( const bool state );

    /**
     *   Enabling this will allow this node to automatically forward received multicast frames to the next highest
     *   multicast level. Duplicate frames are filtered out, so multiple forwarding nodes at the same level should
     *   not interfere. Forwarded payloads will also be received.
     *   @see multicastLevel
     */
    void toggleMulticastRelay( const bool state );

    /**
     *   @note: This value is automatically assigned based on the node address
     *   to reduce errors and increase throughput of the network.
     *
     *   Sets the timeout period for individual payloads in milliseconds at staggered intervals.
     *   Payloads will be retried automatically until success or timeout
     *   Set to 0 to use the normal auto retry period defined by radio.setRetries()
     */
    uint32_t txTimeout; /**< Network timeout value */

    /**
     *   This only affects payloads that are routed by one or more nodes.
     *   This specifies how long to wait for an ack from across the network.
     *   Radios sending directly to their parent or children nodes do not
     *   utilize this value.
     */
    uint16_t routeTimeout; /**< Timeout for routed payloads */

    /**
     * The raw system frame buffer of received data.
     */
    FrameBuffer frameBuffer;

    /**
     *   The frag_ptr is only used with Arduino (not RPi/Linux) and is mainly used for external data systems like RF24Ethernet.
     * When an EXTERNAL_DATA payload type is received, and returned from network.update(), the frag_ptr will always point to the
     * starting memory location of the received frame. <br>This is used by external data systems (RF24Ethernet) to immediately
     * copy the received data to a buffer, without using the user-cache.
     */
    FrameHelper *frag_ptr;



    /**
     *   Network Flags allow control of data flow
     *
     *   Incoming Blocking: If the network user-cache is full, lets radio cache fill up. Radio ACKs are not sent when radio
     * internal cache is full.<br> This behavior may seem to result in more failed sends, but the payloads would have otherwise
     * been dropped due to the cache being full.<br>
     *
     *   |       FLAGS       |   Value  | Description |
     *   |-------------------|----------|------------------------------------------------------------------------------------------------------------|
     *   |FLAG_HOLD_INCOMING | 1(bit_1) | INTERNAL: Set automatically when a fragmented payload will exceed the available cache
     *   |FLAG_BYPASS_HOLDS  | 2(bit_2) | EXTERNAL: Can be used to prevent holds from blocking. Note: Holds are disabled &
     * re-enabled by RF24Mesh when renewing addresses. This will cause data loss if incoming data exceeds the available cache
     * space. |FLAG_FAST_FRAG     | 4(bit_3) | INTERNAL: Replaces the fastFragTransfer variable, and allows for faster transfers
     * between directly connected nodes. |FLAG_NO_POLL       | 8(bit_4) | EXTERNAL/USER: Disables NETWORK_POLL responses on a
     * node-by-node basis.
     *
     */
    uint8_t networkFlags;

    std::array<bool, MAX_CHILD_NODE_ID> childAttached;
    std::array<uint16_t, MAX_CHILD_NODE_ID> children;
    bool childrenAvailable()
    {
      bool result = true;
      for ( bool &val : childAttached )
      {
        result &= val;
      }

      /*------------------------------------------------
      If result is true, this means there is a child attached
      at all pipes, meaning we don't have room for another ie == false
      ------------------------------------------------*/
      return !result;
    }

    uint8_t childBitField()
    {
      uint8_t bf = 0u;
      for ( uint8_t i = 0; i < children.size(); i++ )
      {
        if ( children[ i ] != EMPTY_LOGICAL_ADDRESS )
        {
          bf |= 1u << i;
        }
      }
      return bf;
    }

  private:
    ErrorType oopsies = ErrorType::OK;

    bool initialized = false;
    bool returnSysMsgs = false;
    bool multicastRelay = false;

    uint32_t txTime;
    uint16_t logicalNodeAddress; /**< Logical node address of this unit, 1 .. UINT_MAX */

    Mode mNetMode = Mode::NET_MODE_INVALID;

    void enqueue( FrameHelper &frame );

    bool writeDirect( uint16_t toNode, LogicalAddress directTo );

    bool writeFrameBufferToPipeAtNodeID( const LogicalAddress node, const RF24::Hardware::PipeNumber_t pipe, const bool multicast );

    uint16_t directChildRouteTo( uint16_t node );
    void setupAddress( void );
    bool _write( HeaderHelper &header, const void *const message, const uint16_t len, const NodeAddressType directTo );

    struct logicalToPhysicalStruct
    {
      LogicalAddress send_node;
      RF24::Hardware::PipeNumber_t send_pipe;
      bool multicast;
    };

    bool logicalToPhysicalAddress( logicalToPhysicalStruct *conversionInfo );

    RF24::Physical::Interface_sPtr radio; /**< Underlying radio driver, provides link/physical layers */

    uint8_t multicastLevel;

    FrameHelper txFrame;

    FrameCache_t frameQueue; /**< Space for a small set of frames that need to be delivered to the app layer */

    uint16_t parentNode; /**< Our parent's node address */
    RF24::Hardware::PipeNumber_t parentPipe;  /**< The pipe our parent uses to listen to us */
    uint16_t nodeMask;   /**< The bits which contain significant node address information */


    RF24::Network::Queue::ManagedFIFO txQueue;
    RF24::Network::Queue::ManagedFIFO rxQueue;

    uLog::SinkHandle logger;
  };
}    // namespace RF24::Network

#endif /* ! RF24_NETWORK_HPP */
