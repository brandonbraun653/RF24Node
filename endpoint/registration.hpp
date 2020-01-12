/********************************************************************************
 *  File Name:
 *    registration.hpp
 *
 *  Description:
 *    Handles registration tasks of children nodes with an RF24 device
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef RF24_NODE_ENDPOINT_REGISTRATION_HPP
#define RF24_NODE_ENDPOINT_REGISTRATION_HPP

/* C++ Includes */
#include <array>

/* RF24 Includes */
#include <RF24Node/endpoint/fwd.hpp>
#include <RF24Node/endpoint/types.hpp>
#include <RF24Node/hardware/definitions.hpp>

namespace RF24::Endpoint::Internal
{
  class RegistrationManager
  {
  public:
    RegistrationManager();
    ~RegistrationManager();

    /**
     *  Attempts to bind the given address to this device. In order for this to
     *  succeed, the bind site (channel) must be free and the address should be
     *  a direct descendant of the current device address.
     *
     *	@param[in]	address       The address of the node to try and bind
     *	@return bool
     */
    bool bind( const RF24::LogicalAddress address );

    /**
     *  Removes a given node from the registration list if it exists
     *
     *	@param[in]	address       The address of the node to try and remove
     *	@return void
     */
    void detach( const RF24::LogicalAddress address );

    template<typename Class>
    bool detachable( const Class x );

    template<typename Class>
    bool isBound( const Class x );

  private:
    std::array<Node, RF24::Hardware::MAX_NUM_PIPES> mChildren;
  };

}    // namespace RF24::Endpoint::Internal

#endif /* !RF24_NODE_ENDPOINT_REGISTRATION_HPP */
