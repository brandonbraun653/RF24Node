/********************************************************************************
 *  File Name:
 *    sim_network.cpp
 *
 *  Description:
 *    Implementation of simulator based network API functions
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <algorithm>
#include <vector>

/* Simulator Includes */
#include <RF24Node/network/network.hpp>
#include <RF24Node/simulator/sim_network.hpp>


static std::vector<RF24::Network::NetworkInterface*> networkInstances;

void initializeNetworkAPI()
{
  for ( size_t x = 0; x < networkInstances.size(); x++ )
  {
    releaseNetworkInterface( networkInstances[ x ] );
  }

  networkInstances.clear();
  networkInstances.reserve( 5 );
}

RF24::Network::NetworkInterface *createNetworkInterface()
{
  auto object = new RF24::Network::Network();
  networkInstances.push_back( object );

  return object;
}

void releaseNetworkInterface( RF24::Network::NetworkInterface *object )
{
  if ( object )
  {
    networkInstances.erase( std::remove( networkInstances.begin(), networkInstances.end(), object ), networkInstances.end() );
    delete object;
  }
}