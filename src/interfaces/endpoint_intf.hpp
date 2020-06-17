/********************************************************************************
 *  File Name:
 *    endpoint.hpp
 *
 *  Description:
 *    Describes a high level interface to configuring and using the NRF24L01
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_INTERFACE_HPP
#define RF24_NODE_ENDPOINT_INTERFACE_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/endpoint/types.hpp>
#include <RF24Node/src/interfaces/network_intf.hpp>
#include <RF24Node/src/network/types.hpp>
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/simulator/sim_definitions.hpp>

namespace RF24::Endpoint
{
  class Interface;
  using Interface_sPtr = std::shared_ptr<Interface>;
  using Interface_uPtr = std::unique_ptr<Interface>;

  class Interface : public Chimera::Threading::Lockable
  {
  public:
    virtual ~Interface() = default;

    virtual Chimera::Status_t attachLogger( uLog::SinkHandle sink ) = 0;

    /*-------------------------------------------------
    Initialization and Configuration
    -------------------------------------------------*/
    /**
     *  Initializes the endpoint with critical settings needed to operate
     *
     *  @param[in]  cfg       The configuration details to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t configure( const SystemInit &cfg ) = 0;

    /**
     *	Assigns a human friendly name to the node. This makes it easy to
     *  identify both in the field and in debugging sessions.
     *
     *	@param[in]	name      The name for this device
     *	@return void
     */
    virtual void setName( const std::string &name ) = 0;

    /**
     *  Change how the endpoint makes connections to the network
     *
     *  @param[in]  mode      The desired networking strategy
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setNetworkingMode( const ::RF24::Network::Mode mode ) = 0;

    /**
     *  Manually sets the endpoint's logical address. This is essentially
     *  the same as setting a static IP.
     *
     *  @warning  This only works if the networking mode is configured as NetworkMode::NET_MODE_STATIC
     *
     *  @param[in]  address   The new address to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setEnpointStaticAddress( const ::RF24::LogicalAddress address ) = 0;

    /**
     *  Manually sets the endpoint's logical address. This is essentially
     *  the same as setting a static IP.
     *
     *  @warning  This only works if the networking mode is configured as NetworkMode::NET_MODE_STATIC
     *
     *  @param[in]  address   The new address to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setParentStaticAddress( const ::RF24::LogicalAddress address ) = 0;

    /*-------------------------------------------------
    Networking Operations
    -------------------------------------------------*/
    /**
     *  Requests an address for this node from the network's DHCP server.
     *
     *  @warning This only works if the networking mode is configured as NetworkMode::NET_MODE_MESH
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t requestAddress() = 0;

    /**
     *  Talks with the DHCP server to renew our address allocation. Periodically the address
     *  list will be pruned of stale nodes to free up addresses for new devices.
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t renewAddressReservation() = 0;

    /**
     *  Talks with the DHCP server to remove this device from the network.
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t releaseAddress() = 0;

    /**
     *  Attempts to connect to the network asynchronously. Immediately returns before
     *  the connection has fully completed.
     *
     *  @warning  Node must have previously had settings configured correctly
     *
     *  @param[in]  callback    The callback to be invoked upon success/fail/timeout
     *  @param[in]  timeout     Timeout in milliseconds to wait for success
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t connectAsync( RF24::Connection::OnCompleteCallback callback, const size_t timeout ) = 0;

    /**
     *  Attempts to connect to the network, blocking until either a connection has
     *  been made or the specified timeout occurs.
     *
     *  @warning  Node must have previously had settings configured correctly
     *
     *  @param[in]  timeout     Timeout in milliseconds to wait for the response
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t connectBlocking( const size_t timeout ) = 0;

    /**
     *  Gracefully detaches from the network
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disconnect() = 0;

    /**
     *  Reconnects the node to the network. If configured as a static network, will use the last
     *  known network settings. If a mesh network, will try and dynamically obtain a new address
     *  from the DHCP server.
     *
     *  @note Only reconnects this device to the parent node. Child node connections are handled by
     *        the connect() protocol.
     *
     *  @param[in]  callback    The callback to be invoked upon success/fail/timeout
     *  @param[in]  timeout     Timeout in milliseconds to wait for success
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reconnect( RF24::Connection::OnCompleteCallback callback, const size_t timeout ) = 0;

    /**
     *  Pings a given node to see if it's on the network
     *
     *  @param[in]  node      The node to ping
     *  @param[in]  timeout   How long to wait for the ping to succeed (mS) before error-ing out
     *  @return bool
     */
    virtual bool ping( const ::RF24::LogicalAddress node, const size_t timeout ) = 0;

    /**
     *  Refreshes an existing connection on the network
     *  
     *  @param[in]  site      The bind site to refresh
     *  @return void
     */
    virtual void refreshConnection( const RF24::Connection::BindSite site ) = 0;


    /*-------------------------------------------------
    Asynchronous Processing
    -------------------------------------------------*/
    /**
     *	High level function to process the whole networking stack:
     *
     *  1. Transmits queued TX packets from the application layer
     *  2. Enqueues RX packets destined for the application layer
     *  3. Processes through all the lower layers to ensure the stack doesn't stall
     *
     *  @note   The frequency at which this should be called is dependent upon
     *          system hardware. Generally faster is better, but don't get crazy.
     *          Start at around 10mS and work backwards.
     *
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t doAsyncProcessing() = 0;

    /**
     *  Attaches a handler to be called whenever some kind of event occurs
     *
     *  @note   Currently only a single handler per event is supported
     *
     *  @param[in]  event       The event to attach the handler to
     *  @param[in]  function    The handler to be invoked on event occurrence
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t onEvent( const Event event, const EventFuncPtr_t function ) = 0;

    virtual Chimera::Status_t processDHCPServer( ::RF24::Network::Frame::FrameType &frame ) = 0;

    /**
     *	Interacts with the network layer to handle any complex messages that
     *  should be dealt with internally.
     *
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t processMessageRequests( ::RF24::Network::Frame::FrameType &frame ) = 0;

    virtual Chimera::Status_t processEventHandlers( ::RF24::Network::Frame::FrameType &frame ) = 0;

    /**
     *	Polls through the networking layer a single time in order to
     *  keep data flowing in the TX and RX directions.
     *
     *	@return Chimera::Status_t
     */
    virtual Chimera::Status_t processNetworking() = 0;

    virtual RF24::Network::Interface_sPtr getNetworkingDriver() = 0;

    /*-------------------------------------------------
    Data Transfer IO
    -------------------------------------------------*/
    /**
     *  Writes some data to a particular node in the network
     *
     *  @param[in]  dst       The logical address of the destination node
     *  @param[in]  data      The data to be sent
     *  @param[in]  length    How long the data is in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t write( const ::RF24::LogicalAddress dst, const void *const data, const size_t length ) = 0;

    /**
     *  Reads the next available data packet into the buffer
     *
     *  @see packetAvailable()
     *  @see nextPacketLength()
     *
     *  @param[out] data      Buffer to read into
     *  @param[in]  length    How many bytes the data buffer can hold
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t read( void *const data, const size_t length ) = 0;

    /**
     *  Checks if there is any data waiting to be read from the RX queue
     *
     *  @return bool
     */
    virtual bool packetAvailable() = 0;

    /**
     *  If there is data ready to be consumed, returns the size in bytes
     *
     *  @note   Use read() to pull the next queued RX packet out
     *
     *  @see packetAvailable()
     *  @return size_t
     */
    virtual size_t nextPacketLength() = 0;


    /*-------------------------------------------------
    Information Getters
    -------------------------------------------------*/
    /**
     *  Gets the latest system status of the endpoint node
     *
     *  @return EndpointStatus
     */
    virtual Status getStatus() = 0;

    /**
     *  Gets the latest configuration information of the endpoint node
     *
     *  @return EndpointConfig
     */
    virtual SystemInit &getConfig() = 0;

    /**
     *	Gets the currently assigned logical address of the node
     *
     *	@return RF24::LogicalAddress
     */
    virtual ::RF24::LogicalAddress getLogicalAddress() = 0;

    /**
     *  Checks if the node is still connected to the network
     *
     *  @param[in]  site      The bind site to check
     *  @return bool
     */
    virtual bool isConnected( const RF24::Connection::BindSite site ) = 0;

    virtual SystemState getCurrentState() = 0;
  };
}    // namespace RF24::Endpoint

#endif /* !RF24_NODE_ENDPOINT_INTERFACE_HPP*/