/********************************************************************************
*   File Name:
*     queue.cpp
*
*   Description:
*     Implementation of the network message queue
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* C++ Includes */


/* RF24 Includes */
#include <RF24Node/network/queue/queue.hpp>

namespace RF24::Network::Queue
{

  ManagedFIFO::ManagedFIFO( const size_t depth ) : element_depth( depth )
  {

  }

  ManagedFIFO::~ManagedFIFO()
  {

  }
}