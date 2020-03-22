/********************************************************************************
 *  File Name:
 *    network_intf.hpp
 *
 *  Description:
 *    Describes the interface used for the networking layer
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24_NETWORK_INTERFACE_HPP
#define NRF24_NETWORK_INTERFACE_HPP

/* C++ Includes */
#include <memory>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/hardware/types.hpp>
#include <RF24Node/src/network/definitions.hpp>
#include <RF24Node/src/network/types.hpp>
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/physical/physical.hpp>
#include <RF24Node/src/simulator/sim_definitions.hpp>

namespace RF24::Network
{
  namespace Internal::Processes::Connection
  {
    enum class State
    {
      CONNECT_IDLE,
      CONNECT_REQUEST,
      CONNECT_REQUEST_RETRY,
      CONNECT_WAIT_FOR_PARENT_RESPONSE,
      CONNECT_WAIT_FOR_CHILD_ACK,
      CONNECT_SEND_RESPONSE,
      CONNECT_RESPONSE_ACK,
      CONNECT_SUCCESS_DIRECT,
      CONNECT_SUCCESS_ASYNC,
      CONNECT_TERMINATE,
      CONNECT_TIMEOUT,
      CONNECT_EXIT_LOOP
    };

    struct ControlBlock
    {
      /**
       *  Identifies which leaf in the connection tree is being handled by
       *  this control block (parent, child 1, child 2, etc)
       */
      RF24::Connection::BindSite bindId;

      /**
       *  The address of the node that is being connected to. From the parent's
       *  perspective, this is the child node. From the child's perspective, this
       *  is the parent node.
       */
      RF24::LogicalAddress connectToAddress;

      /**
       *  The "other" node in the connection process, opposite of the connectToAddress.
       *
       *  This simply ends up being the address of the node that is actually processing
       *  the control block locally.
       */
      RF24::LogicalAddress connectFromAddress;

      /**
       *  Simply keeps track of the current state of the connection process
       */
      State currentState;

      /**
       *  System time (in milliseconds) when the bind process started
       *  
       *  @note Child parameter only
       */
      size_t startTime;

      /**
       *  System time (in milliseconds) when the last event "X" started
       */
      size_t lastEventTime;

      /**
       *  How many attempts have been made to make a connection to another node
       *  
       *  @note Child parameter only
       */
      uint8_t connectAttempts;

      /**
       *  Maximum number of attempts that can be made to connect before exiting
       *  
       *  @note Child parameter only
       */
      uint8_t maxAttempts;

      /**
       *  User requested timeout on the connection operation. Typically this
       *  is from the child node's perspective as the parent's functionality
       *  tends to be more temporally fluid.
       */
      size_t processTimeout;

      /**
       *  Internal timeout of how long to wait for a response from the other node 
       *  before either exiting the process or retrying a transmission.
       */
      size_t netTimeout;

      /**
       *  Result of the connection attempt
       */
      RF24::Connection::Result result;

      /**
       *  A callback that is invoked on the child node when a direct connection to a
       *  parent node is attempted and a completion event occurs (ie success/fail/timeout).
       *
       *  OR
       *
       *  A callback that is invoked on the parent node when a connection from a
       *  child node has been requested and successfully registered on the expected
       *  pipe. Used to notify the network (or higher) layer that a node connected.
       *
       *  Because pipes can only either connect to a parent (from pipe 0) or be connected to
       *  from a child (on pipes 1-5), a single connection complete callback is sufficient
       *  to distinguish between the two modes/perspectives of operation.
       */
      RF24::Connection::OnCompleteCallback onConnectComplete;

      /**
       *  Stores the last transmitted frame in case it needs to be retransmitted
       *  due to some timeout
       */
      RF24::Network::Frame::FrameType frameCache;

      ControlBlock()
      {
        bindId             = RF24::Connection::BindSite::FIRST;
        connectToAddress   = RF24::Network::RSVD_ADDR_INVALID;
        connectFromAddress = RF24::Network::RSVD_ADDR_INVALID;
        currentState       = State::CONNECT_IDLE;
        onConnectComplete  = nullptr;
        startTime          = 0;
        lastEventTime      = 0;
        connectAttempts    = 0;
        maxAttempts        = 10;
        processTimeout     = 60000;    // 60 seconds
        netTimeout         = 500;      // 100 milliseconds
        result             = RF24::Connection::Result::CONNECTION_UNKNOWN;
        frameCache         = RF24::Network::Frame::FrameType();
      }
    };

    using ControlBlockList = std::array<ControlBlock, MAX_CONNECTIONS>;
  }    // namespace Internal::Processes::Connection

  class Interface;
  using Interface_sPtr = std::shared_ptr<Interface>;
  using Interface_uPtr = std::unique_ptr<Interface>;

  class Interface
  {
  public:
    virtual ~Interface() = default;

    /**
     *	Attaches a logging instance to the class so that we can log network
     *  messages as needed for debugging.
     *
     *	@param[in]	sink    The log sink
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t attachLogger( uLog::SinkHandle sink ) = 0;

    /**
     *  Attaches a managed instance of a physical layer driver to the network driver.
     *
     *  @warning  Not supported in simulator builds
     *
     *  @param[in]  physicalLayer     The driver to attach
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t attachPhysicalDriver( RF24::Physical::Interface_sPtr physicalLayer ) = 0;

    /**
     *  Assigns the memory pool used for dynamic RX allocation
     *
     *  @param[in]  buffer    The memory pool
     *  @param[in]  size      Number of bytes in the pool
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t initRXQueue( void *buffer, const size_t size ) = 0;

    /**
     *	Assigns the memory pool used for dynamic TX allocation
     *
     *	@param[in]	buffer    The memory pool
     *	@param[in]	size      Number of bytes in the pool
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t initTXQueue( void *buffer, const size_t size ) = 0;

    /**
     *  Runs the RX half of the network processing stack
     *
     *  @warning  Must be called regularly to handle data in a timely manner
     *  @return HeaderMessage
     */
    virtual HeaderMessage updateRX() = 0;

    /**
     *	Runs the TX half of the network processing stack
     *
     *  @warning  Must be called regularly to handle data in a timely manner
     *	@return void
     */
    virtual void updateTX() = 0;

    /**
     *	Similar to Boost AsyncIO's pollOnce(), this function will poll the network
     *  stack and do any work immediately available in the queue, then return.
     *
     *	@return void
     */
    virtual void pollNetStack() = 0;

    /**
     *  Check whether a frame is available for this node.
     *
     *  @return bool
     */
    virtual bool available() = 0;

    /*------------------------------------------------
    Actions
    ------------------------------------------------*/
    /**
     *  Peeks the next available frame if it exists.
     *
     *  @param[out] frame       The next frame of data
     *  @return bool            Validity of the data
     */
    virtual bool peek( Frame::FrameType &frame ) = 0;

    /**
     *  Read a frame from the network queue
     *
     *  @param[out] frame       The next frame of data
     *  @return bool            Validity of the data
     */
    virtual bool read( Frame::FrameType &frame ) = 0;

    /**
     *	Transfers a frame on the network
     *
     *	@param[in]	frame       The frame of data to be transferred
     *  @param[in]  route       The routing strategy to be used
     *	@return bool
     */
    virtual bool write( Frame::FrameType &frame, const RoutingStyle route ) = 0;

    /**
     *	Removes the last received frame off the RX queue
     *
     *	@return void
     */
    virtual void removeRXFrame() = 0;

    /**
     *	Uses the internal routing table to decide which node in the tree hierarchy
     *	is the next hop for a packet.
     *
     *	@param[in]	dst         The desired end destination node
     *	@return RF24::LogicalAddress
     */
    virtual LogicalAddress nextHop( const LogicalAddress dst ) = 0;

    /**
     *	Instructs the network driver to update it's table of devices that are immediately
     *	connected to this node with the given address. It will automatically deduce where
     *	it should be placed (parent/child).
     *
     *	@param[in]	address     The address to update the route table with
     *	@return bool
     */
    virtual bool updateRouteTable( const LogicalAddress address ) = 0;

    /**
     *	Let's the network driver know which node it is in the network.
     *
     *	@param[in]	address     The address the network layer will operate as
     *	@return void
     */
    virtual void setNodeAddress( const LogicalAddress address ) = 0;

    /**
     *	Gets the address of the central node represented by this network driver
     *
     *	@return RF24::LogicalAddress
     */
    virtual LogicalAddress thisNode() = 0;

    /**
     *	Checks if a given address is connected to this node as a parent/child
     *
     *	@param[in]	toCheck     The address to check
     *	@return bool
     */
    virtual bool isConnectedTo( const LogicalAddress toCheck ) = 0;


    /*------------------------------------------------
    Data Getters
    ------------------------------------------------*/
    virtual bool connectionsInProgress() = 0;

    virtual Internal::Processes::Connection::ControlBlock &getConnection( const RF24::Connection::BindSite id ) = 0;

    virtual Internal::Processes::Connection::ControlBlockList &getConnectionList() = 0;


    /*------------------------------------------------
    Data Setters
    ------------------------------------------------*/
    virtual void setConnectionInProgress( const RF24::Connection::BindSite id, const bool enabled ) = 0;


    /*------------------------------------------------
    Callbacks
    ------------------------------------------------*/
    /**
     *	Registers a callback to be invoked when an external node successfully
     *	binds to a particular site.
     *
     *	@note Only valid for nodes that can have children (pipes 1-5)
     *
     *	@param[in]	id          The bind site identifier to invoke the listener on
     *	@param[in]	listener    The callback to be invoked
     *	@return void
     */
    virtual void onNodeHasBound( const RF24::Connection::BindSite id, RF24::Connection::OnCompleteCallback listener ) = 0;

  protected:
    /**
     *	Performs the initialization sequence for the network layer
     *
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t initialize( const RF24::Network::Config &cfg ) = 0;
  };
}    // namespace RF24::Network

#endif /* !NRF24_NETWORK_INTERFACE_HPP */