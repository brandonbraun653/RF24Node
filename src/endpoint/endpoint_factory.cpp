/********************************************************************************
 *  File Name:
 *    endpoint_factory.cpp
 *
 *  Description:
 *    Declares factory methods for creating Endpoint objects
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* RF24 Includes */
#include <RF24Node/endpoint>
#include <RF24Node/src/network/network.hpp>
#include <RF24Node/src/physical/physical.hpp>
#include <RF24Node/src/physical/simulator/sim_physical.hpp>

namespace RF24::Endpoint
{

  Interface_sPtr createShared( const SystemInit &cfg )
  {
    auto physical = RF24::Physical::createShared( cfg.physical );
    auto network = RF24::Network::createShared( cfg.network );

    Device_sPtr device = std::make_shared<Device>();
    device->initNetworkingStack( network, physical );

    return device;
  }

  Interface_uPtr createUnique( const SystemInit &cfg )
  {
    return nullptr;
  }
}