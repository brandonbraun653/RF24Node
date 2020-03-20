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
      CONNECT_WAIT_FOR_BIND_PARENT_RESPONSE,
      CONNECT_WAIT_FOR_CHILD_NODE_ACK,
      CONNECT_SEND_RESPONSE,
      CONNECT_RESPONSE_ACK,
      CONNECT_SUCCESS,
      CONNECT_TERMINATE,
      CONNECT_TIMEOUT,
      CONNECT_EXIT_LOOP
    };

    struct ControlBlock
    {
      ConnectionId connectId;
      RF24::LogicalAddress connectTo;
      RF24::LogicalAddress connectFrom;
      State currentState;
      State previousState;
      RF24::Connection::Callback onEventCallback;
      size_t startTime;
      size_t timeout;

      ControlBlock()
      {
        connectId       = CONNECTION_FIRST;
        connectTo       = RF24::Network::RSVD_ADDR_INVALID;
        connectFrom     = RF24::Network::RSVD_ADDR_INVALID;
        currentState    = State::CONNECT_IDLE;
        previousState   = State::CONNECT_IDLE;
        onEventCallback = nullptr;
        startTime       = 0;
        timeout         = 0;
      }
    };

    using ControlBlockList = std::array<ControlBlock, MAX_CONNECTIONS>;
  }

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

    virtual Internal::Processes::Connection::ControlBlock &getConnection( const ConnectionId id ) = 0;

    virtual Internal::Processes::Connection::ControlBlockList &getConnectionList() = 0;


    /*------------------------------------------------
    Data Setters
    ------------------------------------------------*/
    virtual void setConnectionInProgress( const ConnectionId id, const bool enabled ) = 0;

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