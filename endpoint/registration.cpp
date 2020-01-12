/********************************************************************************
 *  File Name:
 *    endpoint_registration.cpp
 *
 *  Description:
 *    Methods for interaction with the registration aspect of endpoints
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */


/* RF24 Includes */
#include <RF24Node/endpoint/registration.hpp>
#include <RF24Node/network/definitions.hpp>

namespace RF24::Endpoint::Internal
{
  
  RegistrationManager::RegistrationManager()
  {
    for ( size_t x = 0; x < mChildren.size(); x++ )
    {
      mChildren[ x ].logicalAddress  = RF24::Network::RSVD_ADDR_INVALID;
      mChildren[ x ].physicalAddress = RF24::Network::RSVD_ADDR_INVALID;
    }

  }

  RegistrationManager::~RegistrationManager()
  {
  
  }

  bool RegistrationManager::bind( const RF24::LogicalAddress address )
  {
    //need to work on this next 
    return true;
  }

  void RegistrationManager::detach( const RF24::LogicalAddress address )
  {
  
  }

  template<>
  bool RegistrationManager::detachable( const RF24::LogicalAddress address )
  {
    return true;
  }

  template<>
  bool RegistrationManager::detachable( const RF24::Hardware::PipeNumber pipe )
  {
    return true;
  }

  template<>
  bool RegistrationManager::isBound( const RF24::LogicalAddress address )
  {
    return true;
  }

  template<>
  bool RegistrationManager::isBound( const RF24::Hardware::PipeNumber pipe )
  {
    return true;
  }
}
