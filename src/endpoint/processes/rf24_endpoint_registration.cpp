/********************************************************************************
 *  File Name:
 *    endpoint_registration.cpp
 *
 *  Description:
 *    Methods for interaction with the registration aspect of endpoints
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* RF24 Includes */
#include <RF24Node/src/endpoint/processes/rf24_endpoint_registration.hpp>
#include <RF24Node/src/network/definitions.hpp>
#include <RF24Node/src/common/conversion.hpp>

namespace RF24::Endpoint::Internal
{
  RegistrationManager::RegistrationManager()
  {
    for ( size_t x = 0; x < mChildren.size(); x++ )
    {
      auto pipe = static_cast<RF24::Hardware::PipeNumber>( x );
      resetRegistration( pipe );
    }
  }

  RegistrationManager::~RegistrationManager()
  {
  }

  bool RegistrationManager::bind( const RF24::LogicalAddress address )
  {
    auto desiredPipe = RF24::Physical::Conversion::getPipeOnParent( address );
    auto pipeIsBound = isPipeBound( desiredPipe );
    auto boundNode   = getBoundAddress( desiredPipe );

    if ( pipeIsBound && ( boundNode != address ) )
    {
      return false;
    }
    else if ( pipeIsBound && ( boundNode == address ) )
    {
      return true;
    }
    else
    {
      return initRegistration( desiredPipe, address );
    }
  }

  void RegistrationManager::detach( const RF24::LogicalAddress address )
  {
    if ( isNodeBound( address ) )
    {
      resetRegistration( getBoundPipe( address ) );
    }
  }

  bool RegistrationManager::isNodeBound( const RF24::LogicalAddress address )
  {
    auto expectedPipe = RF24::Physical::Conversion::getPipeOnParent( address );
    return ( getBoundAddress( expectedPipe ) == address );
  }

  bool RegistrationManager::isPipeBound( const RF24::Hardware::PipeNumber pipe )
  {
    return ( getBoundAddress( pipe ) != RF24::Network::RSVD_ADDR_INVALID );
  }

  RF24::LogicalAddress RegistrationManager::getBoundAddress( const RF24::Hardware::PipeNumber pipe )
  {
    if ( pipe < RF24::Hardware::PIPE_NUM_MAX )
    {
      return mChildren[ static_cast<size_t>( pipe ) ].logicalAddress;
    }
    else
    {
      return RF24::Network::RSVD_ADDR_INVALID;
    }
  }

  RF24::Hardware::PipeNumber RegistrationManager::getBoundPipe( const RF24::LogicalAddress address )
  {
    for ( size_t x = 0; x < mChildren.size(); x++ )
    {
      if ( address == mChildren[ x ].logicalAddress )
      {
        return static_cast<RF24::Hardware::PipeNumber>( x );
      }
    }

    return RF24::Hardware::PipeNumber::PIPE_INVALID;
  }

  const RF24::Endpoint::Node *const RegistrationManager::getBoundNode( const RF24::LogicalAddress pipe )
  {
    const size_t lookupIndex = static_cast<size_t>( pipe );
    if ( lookupIndex < mChildren.size() )
    {
      return &mChildren[ lookupIndex ];
    }

    return nullptr;
  }

  void RegistrationManager::resetRegistration( const RF24::Hardware::PipeNumber pipe )
  {
    const size_t lookupIndex = static_cast<size_t>( pipe );
    if ( lookupIndex < mChildren.size() )
    {
      /* Allow the default constructor to handle re-initialization */
      mChildren[ lookupIndex ] = {};
    }
  }

  bool RegistrationManager::initRegistration( const RF24::Hardware::PipeNumber pipe, const RF24::LogicalAddress address )
  {
    const size_t lookupIndex = static_cast<size_t>( pipe );
    if ( lookupIndex < mChildren.size() )
    {
      mChildren[ lookupIndex ].logicalAddress  = address;
      mChildren[ lookupIndex ].physicalAddress = RF24::Physical::Conversion::getPhysicalAddress( address, pipe );
    
      return true;
    }

    return false;
  }
}    // namespace RF24::Endpoint::Internal
