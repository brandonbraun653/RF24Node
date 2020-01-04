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
#ifndef NRF24_NETWORK_INTERFACE_HPP
#define NRF24_NETWORK_INTERFACE_HPP

/* C++ Includes */
#include <memory>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/hardware/types.hpp>
#include <RF24Node/network/definitions.hpp>
#include <RF24Node/network/types.hpp>
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/physical/physical.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24::Network
{
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
    virtual Chimera::Status_t attachPhysicalDriver( RF24::Physical::Interface_sPtr &physicalLayer ) = 0;

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
     *	Performs the initialization sequence for the network layer
     *	
     *	@return Chimera::Status_t
     */
     virtual Chimera::Status_t initialize() = 0;

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
     *  Check whether a frame is available for this node.
     *
     *  @return bool
     */
    virtual bool available() const = 0;

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
  };

  using Interface_sPtr = std::shared_ptr<Interface>;
  using Interface_uPtr = std::unique_ptr<Interface>;
}    // namespace RF24::Network

#endif /* !NRF24_NETWORK_INTERFACE_HPP */