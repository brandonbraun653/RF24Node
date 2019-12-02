/********************************************************************************
 *   File Name:
 *    network_sim.hpp
 *
 *   Description:
 *    Describes the interface used for exporting the networking layer into a DLL
 *    for hooking into simulators based on C# and Python.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24_NETWORK_SIMULATOR_HPP
#define NRF24_NETWORK_SIMULATOR_HPP


/* RF24 Includes */
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/network/definitions.hpp>
#include <RF24Node/network/network_types.hpp>
#include <RF24Node/network/header/header.hpp>
#include <RF24Node/physical/physical.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24::Network 
{
  class NetworkInterface
  {
  public:
    virtual ~NetworkInterface() = default;

    /**
     *  Attaches an unmanaged instance of a physical layer driver to the network driver.
     *  
     *  @warning  Only supported in simulator builds
     *  
     *  @param[in]  physicalLayer     The driver to attach
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t attachPhysicalDriver( RF24::Physical::Driver *physicalLayer ) = 0;

    /**
     *   Initializes the network and configures the address, which designates the location
     *   of the node within RF24Network topology.
     *
     *   @note Node addresses are specified in Octal format
     *
     *   @warning Be sure to 'begin' the radio first.
     *
     *   **Example 1:** Begin on channel 90 with address 0 (master node)
     *       network.begin(90,0);
     *
     *   **Example 2:** Begin on channel 90 with address 01 (child of master)
     *       network.begin(90,01);
     *
     *   **Example 3:** Begin on channel 90 with address 011 (child of 01, grandchild of master)
     *       network.begin(90,011);
     *
     *   @param[in]  channel         The radio channel to operate on
     *   @param[in]  node_address    The logical address of this node
     *   @return True if the setup was successful, false if not
     */
    virtual bool begin( const uint8_t channel, const uint16_t nodeAddress, const RF24::Hardware::DataRate dataRate,
                        const RF24::Hardware::PowerAmplitude pwr ) = 0;

    /**
     *   Updates the internal network processing stack. This function must be called regularly to
     *   keep the network layer going.  This is where payloads are re-routed, received, and all the
     *   action happens.
     *
     *   @return Returns the type of the last received payload.
     */
    virtual NetHdrMsgType update() = 0;

    /**
     *   Check whether a message available for this node.
     *
     *   @return True if a message is available, False if not
     */
    virtual bool available() const = 0;

    /**
     *   Reads the next available header without advancing to the next incoming message.  Useful
     *   for doing a switch on the message type. If there is no message available, the header is
     *   not touched.
     *
     *   @param[out] header      The header of the next message
     *   @return TODO?
     */
    virtual uint16_t peek( HeaderHelper &header ) = 0;

    /**
     *   Read the next available payload
     *
     *   Reads the next available payload without advancing to the next
     *   incoming message.  Useful for doing a transparent packet
     *   manipulation layer on top of RF24Network.
     *
     *   @param[out] header      The header (envelope) of this message
     *   @param[out] message     Pointer to memory where the message should be placed
     *   @param maxlen Amount of bytes to copy to message.
     *   @return void
     */
    virtual void peek( HeaderHelper &header, void *const message, const uint16_t maxlen ) = 0;

    /**
     *   Read a message
     *
     *   @param[out] header      The header (envelope) of this message
     *   @param[out] message     Pointer to memory where the message should be placed
     *   @param[in]  maxlen      The largest message size which can be held in message
     *   @return The total number of bytes copied into message
     */
    virtual uint16_t read( HeaderHelper &header, void *const message, const uint16_t maxlen ) = 0;

    /**
     *   Send a message
     *
     *   @note RF24Network now supports fragmentation for very long messages, send as normal.
     *   Default max payload size is 120 bytes.
     *
     *   @param[in,out] header   The header (envelope) of this message.  The critical
     *                           thing to fill in is the @p to_node field so we know where to send the
     *                           message.  It is then updated with the details of the actual header sent.
     *
     *   @param[in]  message     Pointer to memory where the message is located
     *   @param[in]  len         The size of the message
     *   @return True if the message sent successfully, false if not
     */
    virtual bool write( HeaderHelper &header, const void *message, const uint16_t len ) = 0;

    /**
     *   Writes a direct (unicast) payload. This allows routing or sending messages outside of the
     *   usual routing paths. The same as write, but a physical address is specified as the last option.
     *   The payload will be written to the physical address, and routed as necessary by the recipient
     *
     *   @param[in]  header      TODO
     *   @param[in]  message     TODO
     *   @param[in]  length      TODO
     *   @param[in]  writeDirect TODO
     *   @return True if the message sent successfully, false if not
     */
    virtual bool write( HeaderHelper &header, const void *message, uint16_t length, NodeAddressType writeDirect ) = 0;

    /**
     *   Allows messages to be rapidly broadcast through the network by seding to multiple nodes at once
     *
     *   Multicasting is arranged in levels, with all nodes on the same level listening to the same address
     *   Levels are assigned by network level ie: nodes 01-05: Level 1, nodes 011-055: Level 2
     *   @see multicastLevel
     *   @see multicastRelay
     *
     *   @param[in] header       TODO
     *   @param[in] message      Pointer to memory where the message is located
     *   @param[in] len          The size of the message
     *   @param[in] level        Multicast level to broadcast to
     *   @return True if the message sent successfully, false if not
     */
    virtual bool multicast( HeaderHelper &header, const void *message, const uint16_t length, const uint8_t level ) = 0;

    /**
    *   By default, multicast addresses are divided into levels.
    *
    *   Nodes 1-5 share a multicast address, nodes n1-n5 share a multicast address, and nodes n11-n55 share a multicast
    address.<br>
    *
    *   This option is used to override the defaults, and create custom multicast groups that all share a single
    *   address. The level should be specified in decimal format 1-6 <br>
    *   @see multicastRelay

    *   @param[in]  level       Levels 1 to 6 are available. All nodes at the same level will receive the same
    *                           messages if in range. Messages will be routed in order of level, low to high by default, with
    the
    *                           master node (00) at multicast Level 0
    *   @return void
    */
    virtual void setMulticastLevel( const uint8_t level ) = 0;

    /**
     *   Check if a network address is valid or not
     *
     *   @param[in]  node        The octal nodeID to validate
     *   @return True if a supplied address is valid
     */
    virtual bool isValidNetworkAddress( const uint16_t node ) = 0;

    /**
     *   Changes the network's address at runtime
     *
     *   @param[in]  address     The address to be set
     *   @return True if the address was valid and set correctly, false if not
     */
    virtual bool setAddress( const uint16_t address ) = 0;

    /**
     *   Retrieves the current network address
     *
     *   @return Current network address in octal format
     */
    virtual uint16_t getLogicalAddress() = 0;

    /**
     *   This node's parent address
     *
     *   @return This node's parent address, or -1 if this is the base
     */
    virtual uint16_t parent() const = 0;

    /**
     *   Provided a node address and a pipe number, will return the RF24Network address of that child pipe for that node
     */
    virtual uint16_t addressOfPipe( uint16_t node, uint8_t pipeNo ) = 0;
  
  };
}    // namespace RF24::Network


#if defined( RF24_SIMULATOR )

/**
 *  Initialize the dll memory
 *  
 *  @return void
 */
extern "C" RF24API void initializeNetworkAPI();

/**
 *  Factory method to create an unmanaged instance of the NetworkInterface
 *  
 *  @return NetworkInterface *
 */
extern "C" RF24API RF24::Network::NetworkInterface *createNetworkInterface();

/**
 *  Destroys a previously created instance of the NetworkInterface
 *  
 *  @return void
 */
extern "C" RF24API void releaseNetworkInterface( RF24::Network::NetworkInterface *object );

#endif /* RF24_SIMULATOR */


#endif  /* !NRF24_NETWORK_SIMULATOR_HPP */