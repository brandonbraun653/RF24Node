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

    /**
     *  Checks if the device specified by 'address' has already been bound to a pipe
     *
     *	@param[in]	address       The address of the node to check
     *	@return bool
     */
    bool isNodeBound( const RF24::LogicalAddress address );

    /**
     *  Checks if the given pipe already has a child bound to it
     *
     *	@param[in]	pipe          The pipe to check for a bound child
     *	@return bool
     */
    bool isPipeBound( const RF24::Hardware::PipeNumber pipe );

    /**
     *	Looks up the logical address that has been bound to a given pipe
     *
     *	@param[in]	pipe          The pipe to return the bound address for
     *	@return RF24::LogicalAddress
     */
    RF24::LogicalAddress getBoundAddress( const RF24::Hardware::PipeNumber pipe );

    /**
     *  Gets the pipe that the address is bound to via lookup through all
     *  registered addresses.
     *
     *	@param[in]	address       The address of the node to check
     *	@return RF24::Hardware::PipeNumber
     */
    RF24::Hardware::PipeNumber getBoundPipe( const RF24::LogicalAddress address );

    /**
     *	Gets the Node structure that was bound to the given pipe
     *
     *	@param[in]	pipe          The pipe to return the bound node for
     *	@return RF24::Endpoint::Node
     */
    const RF24::Endpoint::Node *const getBoundNode( const RF24::LogicalAddress pipe );

  protected:
    /**
     *	Resets the registration structure for the given pipe, effectively
     *  detaching that node from the parent.
     *
     *	@param[in]	pipe          The pipe to be reset
     *	@return void
     */
    void resetRegistration( const RF24::Hardware::PipeNumber pipe );

    /**
     *	Binds a new address to the given pipe and performs any initialization
     *  steps needed to complete the binding process.
     *
     *	@param[in]	pipe          The pipe to bind the address to
     *	@param[in]	address       The address to be bound
     *	@return bool
     */
    bool initRegistration( const RF24::Hardware::PipeNumber pipe, const RF24::LogicalAddress address );

  private:
    std::array<Node, RF24::Hardware::MAX_NUM_PIPES> mChildren;
  };

}    // namespace RF24::Endpoint::Internal

#endif /* !RF24_NODE_ENDPOINT_REGISTRATION_HPP */
