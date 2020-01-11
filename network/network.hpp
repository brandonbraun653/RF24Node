/********************************************************************************
 *   File Name:
 *    network.hpp
 *
 *   Description:
 *    Implements the RF24 Network layer
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
#include <RF24Node/interfaces/endpoint_intf.hpp>
#include <RF24Node/interfaces/network_intf.hpp>
#include <RF24Node/network/definitions.hpp>
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/frame/types.hpp>
#include <RF24Node/network/queue/queue.hpp>
#include <RF24Node/physical/physical.hpp>


namespace RF24::Network
{
  class Driver : public Interface
  {
  public:
    Driver( ::RF24::Endpoint::Interface *const node );
    ~Driver();

    Chimera::Status_t attachLogger( uLog::SinkHandle sink ) final override;
    Chimera::Status_t attachPhysicalDriver( RF24::Physical::Interface_sPtr &physicalLayer ) final override;
    Chimera::Status_t initRXQueue( void *buffer, const size_t size ) final override;
    Chimera::Status_t initTXQueue( void *buffer, const size_t size ) final override;
    Chimera::Status_t initialize() final override;
    HeaderMessage updateRX() final override;
    void updateTX() final override;
    bool available() final override;
    bool peek( Frame::FrameType &frame ) final override;
    bool read( Frame::FrameType &frame ) final override;
    bool write( Frame::FrameType &frame, const RoutingStyle route ) final override;

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

  private:
    bool mInitialized;
    bool mReturnSystemMessages;
    bool mMulticastRelay;
    size_t mLastTxTime;
    ::RF24::Endpoint::Interface *const mNode;


    RF24::Physical::Interface_sPtr radio; /**< Underlying radio driver, provides link/physical layers */
    RF24::Network::Queue::ManagedFIFO txQueue;
    RF24::Network::Queue::ManagedFIFO rxQueue;

    uLog::SinkHandle logger;

    void enqueueRXPacket( Frame::Buffer &buffer );
    bool transferToPipe( const PhysicalAddress address, const Frame::Buffer &buffer, const bool autoAck );

    HeaderMessage handleDestination( Frame::Buffer &buffer ); 
    HeaderMessage handlePassthrough( Frame::Buffer &frame );

    bool writeDirect( Frame::FrameType &frame );
    bool writeRouted( Frame::FrameType &frame );
    bool writeMulticast( Frame::FrameType &frame );

    bool readWithPop( Frame::FrameType &frame, const bool pop );
  };
}    // namespace RF24::Network

#endif /* ! RF24_NETWORK_HPP */
