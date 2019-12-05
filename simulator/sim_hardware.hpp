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
#ifndef NRF24_HARDWARE_SIMULATOR_HPP
#define NRF24_HARDWARE_SIMULATOR_HPP

/* RF24 Includes */
#include <RF24Node/hardware/driver.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

namespace RF24::Hardware
{
  class DriverInterface
  {
  public:
    virtual ~DriverInterface() = default;

    virtual Chimera::Status_t resetDevice() = 0;
  };
}

#if defined( RF24_SIMULATOR )

/**
 *  Initialize the dll memory
 *  
 *  @return void
 */
extern "C" RF24API void initializeHardwareAPI();

/**
 *  Factory method to create an unmanaged instance of the DriverInterface
 *  
 *  @return DriverInterface *
 */
extern "C" RF24API RF24::Hardware::DriverInterface *createHardwareInterface();

/**
 *  Destroys a previously created instance of the NetworkInterface
 *  
 *  @return void
 */
extern "C" RF24API void releaseDriverInterface( RF24::Hardware::DriverInterface *object );



extern "C" RF24API Chimera::Status_t resetDevice( RF24::Hardware::Driver *driver );

#endif /* RF24_SIMULATOR */

#endif  /* NRF24_HARDWARE_SIMULATOR_HPP */
