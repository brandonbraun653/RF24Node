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

/* Chimera Includes */
#include <Chimera/thread>

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
       *  Defines whether or not a new connection is being created or destroyed
       */
      RF24::Connection::Direction direction;

      /**
       *  The address of the node that is being connected to. From the parent's
       *  perspective, this is the child node. From the child's perspective, this
       *  is the parent node.
       */
      RF24::LogicalAddress toAddress;

      /**
       *  The "other" node in the connection process, opposite of the connectToAddress.
       *
       *  This simply ends up being the address of the node that is actually processing
       *  the control block locally.
       */
      RF24::LogicalAddress fromAddress;

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
      uint8_t attempts;

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
        clear();
      }

      void clear()
      {
        bindId            = RF24::Connection::BindSite::FIRST;
        direction         = RF24::Connection::Direction::CONNECT;
        toAddress         = RF24::Network::RSVD_ADDR_INVALID;
        fromAddress       = RF24::Network::RSVD_ADDR_INVALID;
        currentState      = State::CONNECT_IDLE;
        onConnectComplete = nullptr;
        startTime         = 0;
        lastEventTime     = 0;
        attempts          = 0;
        maxAttempts       = 10;
        processTimeout    = 60000;    // 60 seconds
        netTimeout        = 1000;      // 100 milliseconds
        result            = RF24::Connection::Result::CONNECT_PROC_UNKNOWN;
        frameCache        = RF24::Network::Frame::FrameType();
      }
    };

    using ControlBlockList = std::array<ControlBlock, MAX_CONNECTIONS>;
  }    // namespace Internal::Processes::Connection

  class Interface;
  using Interface_sPtr = std::shared_ptr<Interface>;
  using Interface_uPtr = std::unique_ptr<Interface>;

  class Interface : virtual public Chimera::Threading::LockableInterface
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
     *  Assigns the memory pool used for queueing data packets that are destined
     *  for the application layer.
     *
     *  @param[in]  buffer    The memory pool
     *  @param[in]  size      Number of bytes in the pool
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t initAppRXQueue( void *buffer, const size_t size ) = 0;

    /**
     *	Assigns the memory pool used for queueing data packets that need to be transmitted.
     *
     *	@param[in]	buffer    The memory pool
     *	@param[in]	size      Number of bytes in the pool
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t initNetTXQueue( void *buffer, const size_t size ) = 0;

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
     *	@return RF24::JumpType
     */
    virtual JumpType nextHop( const LogicalAddress dst ) = 0;

    /**
     *	Instructs the network driver to update it's table of devices that are immediately
     *	connected to this node with the given address. It will automatically deduce where
     *	it should be placed (parent/child).
     *
     *	@param[in]	address     The address to update the route table with
     *	@param[in]  attach      Whether or not the address should be attached or removed
     *	@return bool
     */
    virtual bool updateRouteTable( const LogicalAddress address, const bool attach ) = 0;

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

    /**
     *  Resets connection information with the associated bind site. This will detach
     *  any information about a previous connection, forcing another process to reinitialize
     *  later if the bind site is to be used for a new connection.
     *
     *  @param[in]  id          The bind site to be reset
     *  @return void
     */
    virtual void resetConnection( const RF24::Connection::BindSite id ) = 0;


    /*------------------------------------------------
    Data Getters
    ------------------------------------------------*/
    /**
     *  Checks if a connection process in the given direction is ongoing for any node
     *  in the connection list.
     *
     *  @param[in]  dir         Which connect direction to look up
     *  @return bool
     */
    virtual bool connectionsInProgress( const RF24::Connection::Direction dir ) = 0;

    /**
     *  Copies out the latest bind site control block data
     *
     *  @param[in]  id          The bind site to get the data for
     *  @param[in]  cb          The control block structure to copy into
     *  @return void
     */
    virtual void getBindSiteStatus( const RF24::Connection::BindSite id, BindSiteCB &cb ) = 0;

    /**
     *  Gets the current system control block
     *
     *  @warning This should only be used with the appropriate lock on the
     *            network object. Otherwise the SCB could get corrupted.
     *
     *  @param[in]  scb         Variable to copy the internal SCB state into
     *  @return void
     */
    virtual void getSCBUnsafe( SystemCB &scb ) = 0;

    /**
     *  Makes a copy of the internal system control block in a thread-safe manner
     *  @return SystemControlBlock
     */
    virtual SystemCB getSCBSafe() = 0;

    virtual RF24::Network::BindSiteCB getBindSiteCBSafe( const RF24::Connection::BindSite site ) = 0;

    /**
     *	Gets the logger registered with the network
     *
     *	@return uLog::SinkHandle
     */
    virtual uLog::SinkHandle getLogger() = 0;


    /*------------------------------------------------
    Data Setters
    ------------------------------------------------*/
    /**
     *  Updates the connection control blocks for the given bind site so
     *  that it knows a new process has begun.
     *
     *  @param[in]  id          Which bindsite to set up
     *  @param[in]  dir         The direction the connection process is going
     *  @param[in]  enabled     If enabled, marks the process as beginning. If not, marks as completed.
     *  @return void
     */
    virtual void setConnectionInProgress( const RF24::Connection::BindSite id, const RF24::Connection::Direction dir,
                                          const bool enabled ) = 0;

    /**
     *  Updates the system control block with new data.
     *
     *  @warning This should only be used in conjunction with getSCB and the appropriate lock on the
     *            network object. Otherwise the SCB could get corrupted.
     *
     *  @param[in]  scb         Data used to update the internal SCB state
     *  @return void
     */
    virtual void setSCBUnsafe( const SystemCB &scb ) = 0;


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


    /*-------------------------------------------------------------------------------
    Unsafe Data: The network driver is not meant to used directly and this class
      exposes lots of data that can screw up the network layer if handled frivolously.
      Due to the memory overhead of adding thread-safe getters/setters for all complex
      class metadata, access to the below variables is left up to the programmer to
      maintain thread safety. When in doubt, wrap accesses to these data types with
      proper lock/unlock protocol.
    -------------------------------------------------------------------------------*/

    /**
     *  Tracks ongoing connections and their runtime status
     */
    Internal::Processes::Connection::ControlBlockList unsafe_ConnectionList;

    /**
     *  Tracks the state of all bind-sites
     */
    BindSiteCBList unsafe_BindSiteList;

    SystemCB unsafe_DriverCB;

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