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
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/endpoint/connection.hpp>
#include <RF24Node/endpoint/config.hpp>
#include <RF24Node/endpoint/registration.hpp>
#include <RF24Node/interfaces/endpoint_intf.hpp>
#include <RF24Node/interfaces/network_intf.hpp>
#include <RF24Node/interfaces/physical_intf.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24::Endpoint
{
  /**
   *  Primary interface that describes interaction with an RF24L01 device node.
   */
  class Device : public Interface, public Chimera::Threading::Lockable
  {
  public:
    Device();
    ~Device();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink );
    Chimera::Status_t configure( const Config &cfg ) override;
    Chimera::Status_t setNetworkingMode( const Network::Mode mode ) final override;
    Chimera::Status_t setEnpointStaticAddress( const LogicalAddress address ) final override;
    Chimera::Status_t setParentStaticAddress( const LogicalAddress address ) final override;
    Chimera::Status_t requestAddress() final override;
    Chimera::Status_t renewAddressReservation() final override;
    Chimera::Status_t releaseAddress() final override;
    Chimera::Status_t connect( const size_t timeout ) final override;
    Chimera::Status_t disconnect() final override;
    Chimera::Status_t reconnect() final override;
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
    Config &getConfig() final override;
    LogicalAddress getLogicalAddress() final override;
    Chimera::Status_t isConnected() final override;

    void setName( const std::string_view &name ) final override;

  protected:
    friend Internal::RegistrationManager;
    friend Internal::ConnectionManager;

    Config mConfig;                    /**< User endpoint configuration for this node */
    Physical::Interface_sPtr physical; /**< Physical layer object */
    Network::Interface_uPtr network;   /**< Network layer object */
    uLog::SinkHandle logger;           /**< Logger object */

    LogicalAddress mDeviceAddress;
    LogicalAddress mParentAddress;

  private:
    char mDeviceName[ MAX_CHARS_IN_DEVICE_NAME + 1u ];

    Internal::RegistrationManager mRegistrationManager;
    Internal::ConnectionManager mConectionManager;

    Chimera::Status_t initHardwareLayer();
    Chimera::Status_t initPhysicalLayer();
    Chimera::Status_t initNetworkLayer();
    Chimera::Status_t initMeshLayer();
  };
}    // namespace RF24::Endpoint

/*------------------------------------------------
Exported functions for construction/destruction of Endpoints
------------------------------------------------*/
extern "C" RF24API RF24::Endpoint::Device *new__Endpoint();
extern "C" RF24API void delete__Endpoint( RF24::Endpoint::Device *obj );

#endif /* !RF24_NODE_ENDPOINT_HPP*/