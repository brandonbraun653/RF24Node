/********************************************************************************
 *  File Name:
 *    endpoint.hpp
 *
 *  Description:
 *    Describes a node endpoint implementation. The goal is for the user instantiate
 *    only this class and be able to create/connect to an NRF24L01 network without
 *    too much hassle and start sending data around.
 *
 *  Notes:
 *    Is thread-safe for all function calls.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_ENDPOINT_HPP
#define RF24_NODE_ENDPOINT_HPP

/* C++ Includes */
#include <string_view>

/* Chimera Includes */
#include <Chimera/thread>
#include <Chimera/common>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/src/endpoint/config.hpp>
#include <RF24Node/src/endpoint/processes/rf24_endpoint_connection.hpp>
#include <RF24Node/src/interfaces/endpoint_intf.hpp>
#include <RF24Node/src/interfaces/network_intf.hpp>
#include <RF24Node/src/interfaces/physical_intf.hpp>
#include <RF24Node/src/network/network.hpp>
#include <RF24Node/src/simulator/sim_definitions.hpp>

namespace RF24::Endpoint
{
  class Device;
  using Device_sPtr = std::shared_ptr<Device>;
  using Device_uPtr = std::unique_ptr<Device>;

  Interface_sPtr createShared( const SystemInit &cfg );
  Interface_uPtr createUnique( const SystemInit &cfg );

  class Device : public Interface
  {
  public:
    Device();
    ~Device();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink ) final override;
    Chimera::Status_t configure( const SystemInit &cfg ) final override;
    Chimera::Status_t setNetworkingMode( const Network::Mode mode ) final override;
    Chimera::Status_t setEnpointStaticAddress( const LogicalAddress address ) final override;
    Chimera::Status_t setParentStaticAddress( const LogicalAddress address ) final override;
    Chimera::Status_t requestAddress() final override;
    Chimera::Status_t renewAddressReservation() final override;
    Chimera::Status_t releaseAddress() final override;
    Chimera::Status_t connectAsync( RF24::Connection::OnCompleteCallback callback, const size_t timeout ) override;
    Chimera::Status_t connectBlocking( const size_t timeout ) final override;
    Chimera::Status_t disconnect() final override;
    Chimera::Status_t reconnect( RF24::Connection::OnCompleteCallback callback, const size_t timeout ) final override;
    Chimera::Status_t onEvent( const Event event, const EventFuncPtr_t function ) final override;
    Chimera::Status_t doAsyncProcessing() final override;
    Chimera::Status_t processDHCPServer( RF24::Network::Frame::FrameType &frame ) final override;
    Chimera::Status_t processMessageRequests( RF24::Network::Frame::FrameType &frame ) final override;
    Chimera::Status_t processEventHandlers( RF24::Network::Frame::FrameType &frame ) final override;
    Chimera::Status_t processNetworking() final override;
    Chimera::Status_t write( const LogicalAddress dst, const void *const data, const size_t length ) final override;
    Chimera::Status_t read( void *const data, const size_t length ) final override;
    bool packetAvailable() final override;
    size_t nextPacketLength() final override;
    Status getStatus() final override;
    SystemInit &getConfig() final override;
    LogicalAddress getLogicalAddress() final override;
    bool isConnected( const bool check ) override;
    bool ping( const ::RF24::LogicalAddress node, const size_t timeout ) final override;

    void setName( const std::string &name ) final override;


    RF24::Network::Interface_sPtr getNetworkingDriver() final override;

    SystemState getCurrentState() final override;

  protected:
    friend Interface_sPtr createShared( const SystemInit &cfg );
    friend Interface_uPtr createUnique( const SystemInit &cfg );

    RF24::Physical::Interface_sPtr mPhysicalDriver; /**< Physical layer object */
    RF24::Network::Interface_sPtr mNetworkDriver;   /**< Network layer object */
    uLog::SinkHandle mLogger;                       /**< Logger object */


    SystemInit mEndpointInit; /**< User endpoint initial configuration for this node */
    SystemState mState;       /**< Current state of this node */


    void initNetworkingStack(::RF24::Network::Interface_sPtr net, ::RF24::Physical::Interface_sPtr phy );

  private:
    Chimera::Status_t initHardwareLayer();
    Chimera::Status_t initPhysicalLayer();
    Chimera::Status_t initNetworkLayer();
    Chimera::Status_t initMeshLayer();
  };
}    // namespace RF24::Endpoint

#endif /* !RF24_NODE_ENDPOINT_HPP*/