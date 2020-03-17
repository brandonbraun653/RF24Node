/********************************************************************************
 *  File Name:
 *    network.hpp
 *
 *  Description:
 *    Implements the RF24 Network layer
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Chimera/common>

/* uLog Includes */
#include <uLog/types.hpp>

/* Driver Includes */
#include <RF24Node/src/common/types.hpp>
#include <RF24Node/src/hardware/types.hpp>
#include <RF24Node/src/interfaces/endpoint_intf.hpp>
#include <RF24Node/src/interfaces/network_intf.hpp>
#include <RF24Node/src/network/connections/rf24_network_route_table.hpp>
#include <RF24Node/src/network/definitions.hpp>
#include <RF24Node/src/network/frame/frame.hpp>
#include <RF24Node/src/network/frame/types.hpp>
#include <RF24Node/src/network/queue/queue.hpp>
#include <RF24Node/src/physical/physical.hpp>


namespace RF24::Network
{
  Interface_sPtr createShared( const RF24::Network::Config &cfg );
  Interface_uPtr createUnique( const RF24::Network::Config &cfg );


  class Driver;
  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

  class Driver : virtual public Interface
  {
  public:
    Driver();
    ~Driver();

    bool requestAccessKey( size_t &key ) final override;
    void releaseAccessKey( const size_t key ) final override;

    Chimera::Status_t attachLogger( uLog::SinkHandle sink ) final override;
    Chimera::Status_t attachPhysicalDriver( RF24::Physical::Interface_sPtr physicalLayer ) override;
    Chimera::Status_t initRXQueue( void *buffer, const size_t size ) final override;
    Chimera::Status_t initTXQueue( void *buffer, const size_t size ) final override;
    HeaderMessage updateRX( const size_t key ) override;
    void updateTX( const size_t key ) final override;
    bool available() final override;
    bool peek( Frame::FrameType &frame ) final override;
    bool read( Frame::FrameType &frame ) final override;
    bool write( Frame::FrameType &frame, const RoutingStyle route ) final override;
    void removeRXFrame() final override;

    LogicalAddress nextHop( const LogicalAddress dst ) final override;

    bool updateRouteTable( const LogicalAddress address ) final override;
    void setNodeAddress( const LogicalAddress address ) final override;
    LogicalAddress thisNode() final override;

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

  protected:
    friend Interface_sPtr createShared( const RF24::Network::Config &cfg );
    friend Interface_uPtr createUnique( const RF24::Network::Config &cfg );

    Chimera::Status_t initialize( const RF24::Network::Config &cfg ) final override;

    /**
     *	Checks if the given address is registered directly with this network. This could be as
     *  either a parent or a child
     *
     *	@param[in]	toCheck   The address to check
     *	@return bool
     */
    bool isRegisteredDirectly( const LogicalAddress toCheck );

    /**
     *  Checks if the given node is a descendant of any children registered with the current
     *  node at any point in the tree branches.
     *
     *	@param[in]	toCheck   The destination node address to check
     *	@param[in]	which     Which child address is the ancestor (populated on return true)
     *	@return bool
     */
    bool isDescendantOfRegisteredChild( const LogicalAddress toCheck, LogicalAddress &which );

    /**
     *	Computes the destination pipe for a given transfer
     *	
     *	@param[in]	destination     The node receiving the data
     *	@param[in]	source          The node sending the data
     *	@return RF24::Hardware::PipeNumber
     */
    Hardware::PipeNumber getDestinationRXPipe( const LogicalAddress destination, const LogicalAddress source );

  private:
    size_t mAccessKey;

    bool mInitialized;
    bool mReturnSystemMessages;
    bool mMulticastRelay;
    size_t mLastTxTime;
    RF24::Physical::Interface_sPtr mPhysicalDriver; 
    RF24::Network::Queue::ManagedFIFO mTXQueue;
    RF24::Network::Queue::ManagedFIFO mRXQueue;
    Internal::NodeConnections mRouteTable;
    uLog::SinkHandle mLogger;

    void enqueueRXPacket( Frame::FrameType &frame );
    void enqueueTXPacket( Frame::FrameType &frame );
    bool transferToPipe( const PhysicalAddress address, const Frame::Buffer &buffer, const size_t length, const bool autoAck );

    HeaderMessage handleDestination( Frame::FrameType &frame );
    HeaderMessage handlePassthrough( Frame::FrameType &frame );

    bool writeDirect( Frame::FrameType &frame );
    bool writeRouted( Frame::FrameType &frame );
    bool writeMulticast( Frame::FrameType &frame );

    bool readWithPop( Frame::FrameType &frame, const bool pop );
  };
}    // namespace RF24::Network

#endif /* ! RF24_NETWORK_HPP */