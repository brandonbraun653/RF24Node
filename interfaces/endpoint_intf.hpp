/********************************************************************************
 *  File Name:
 *    endpoint.hpp
 *
 *  Description:
 *    Describes a high level interface to configuring and using the NRF24L01
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_INTERFACE_HPP
#define RF24_NODE_ENDPOINT_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* RF24 Includes */
#include <RF24Node/common/types.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24
{
  
  class RF24API EndpointInterface
  {
  public:
    virtual ~EndpointInterface() = default;

    /*-------------------------------------------------
    Initialization and Configuration
    -------------------------------------------------*/
    /**
     *  Initializes the endpoint with critical settings needed to operate
     * 
     *  @param[in]  cfg       The configuration details to be set
     *  @return Chimera::Status_t 
     */
    virtual Chimera::Status_t configure( const EndpointConfig &cfg ) = 0;
    
    /**
     *  Change how the endpoint makes connections to the network
     * 
     *  @param[in]  mode      The desired networking strategy
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setNetworkingMode( const NetworkMode mode ) = 0;

    /**
     *  Manually sets the endpoint's logical address. This is essentially
     *  the same as setting a static IP.
     * 
     *  @warning  This only works if the networking mode is configured as NetworkMode::NET_MODE_STATIC
     * 
     *  @param[in]  address   The new address to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setEnpointStaticAddress( const LogicalAddress address ) = 0;

    /**
     *  Manually sets the endpoint's logical address. This is essentially
     *  the same as setting a static IP.
     * 
     *  @warning  This only works if the networking mode is configured as NetworkMode::NET_MODE_STATIC
     * 
     *  @param[in]  address   The new address to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setParentStaticAddress( const LogicalAddress address ) = 0;

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
     *  Attempts to connect to the network using previously configured settings
     * 
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t connect() = 0;

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
     *  @return Chimera::Status_t 
     */
    virtual Chimera::Status_t reconnect() = 0;


    /*-------------------------------------------------
    Asynchronous Processing
    -------------------------------------------------*/
    /**
     *  Attaches a handler to be called whenever some kind of event occurs
     * 
     *  @note   Currently only a single handler per event is supported
     * 
     *  @param[in]  event       The event to attach the handler to
     *  @param[in]  function    The handler to be invoked on event occurance
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t onEvent( const Event event, const EventFuncPtr_t function ) = 0;

    virtual Chimera::Status_t processMessageBuffers() = 0;

    virtual Chimera::Status_t processDHCPServer() = 0;

    virtual Chimera::Status_t processMessageRequests() = 0;

    virtual Chimera::Status_t processEventHandlers() = 0;


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
    virtual Chimera::Status_t write( const LogicalAddress dst, const void *const data, const size_t length ) = 0;
    
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
    virtual EndpointStatus getEndpointStatus() = 0;

    
    /**
     *  
     */
    virtual Chimera::Status_t isConnected() = 0;
  };
}

#endif /* !RF24_NODE_ENDPOINT_INTERFACE_HPP*/