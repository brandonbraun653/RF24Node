/********************************************************************************
 *  File Name:
 *    rf24_network_route_table.hpp
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
#include <RF24Node/common>
#include <RF24Node/src/endpoint/node.hpp>
#include <RF24Node/src/hardware/definitions.hpp>

namespace RF24::Network::Internal
{
  using PipeRegistrationList = std::array<::RF24::Endpoint::Node, RF24::Hardware::MAX_NUM_PIPES>;

  /**
   *  Models the connection structure for a single node in the network, which encompasses
   *  the direct parent node and all possible direct children. Graphically this looks like:
   *
   *                +-------------+
   *                | Parent Node |
   *                +------+------+
   *                       |
   *                       |
   *                +------+------+
   *       +--------+  This Node  +--------+
   *       |        +-+---------+-+        |
   *       |          |         |          |
   *       |          |         |          |
   *  +----+----------+---------+----------+-----+
   *  | Child 0    Child 1    Child 2    Child x |
   *  +------------------------------------------+
   */
  class NodeConnections
  {
  public:
    /*------------------------------------------------
    Ctors/Dtors
    ------------------------------------------------*/
    /**
     *  Constructs a NodeConnections object using the given address as the Central Node
     *
     *	@param[in]	address       The address to use as the Central Node
     */
    NodeConnections( ::RF24::LogicalAddress address );

    NodeConnections();
    ~NodeConnections();

    /*------------------------------------------------
    Actions
    ------------------------------------------------*/
    /**
     *  Initializes the tracking data for the parent/children
     *
     *	@return void
     */
    void initialize();

    /**
     *  Updates the central node from which the parent and children are referenced.
     *  If it is different than what is currently registered, the internal cache
     *  will be reset.
     *
     *	@param[in]	address       The address to set as the central node
     *	@return void
     */
    void updateCentralNode( ::RF24::LogicalAddress address );

    /**
     *  Attempts to bind the given address to this device. In order for this to
     *  succeed, the bind site (channel) must be free and the address should be
     *  a direct descendant of the current device address.
     *
     *  @note Automatically deduces the appropriate pipe/parent from the address
     *
     *	@param[in]	address       The address of the node to try and bind
     *	@return bool
     */
    bool attach( const ::RF24::LogicalAddress address );

    /**
     *  Removes a given node from the registration list if it exists
     *
     *	@param[in]	address       The address of the node to try and remove
     *	@return void
     */
    void detach( const ::RF24::LogicalAddress address );

    /*------------------------------------------------
    Questions
    ------------------------------------------------*/
    /**
     *  Checks if the device specified by 'address' has already been bound as a parent/child
     *
     *	@param[in]	address       The address of the node to check
     *	@return bool
     */
    bool isAddressBound( const ::RF24::LogicalAddress address );

    /**
     *  Checks if the given pipe already has a child bound to it
     *
     *	@param[in]	pipe          The pipe to check for a bound child
     *	@return bool
     */
    bool isPipeBound( const RF24::Hardware::PipeNumber pipe );

    /*------------------------------------------------
    Data Getters
    ------------------------------------------------*/
    /**
     *	Gets a reference to the currently registered list of children
     *  for each pipe in the node.
     *	
     *	@return const RF24::Network::Internal::PipeRegistrationList &
     */
    const PipeRegistrationList &getRegistrationList();

    /**
     *	Gets the Child Node that is bound to the given pipe
     *
     *	@param[in]	pipe          The pipe to return the bound node for
     *	@return ::RF24::Endpoint::Node
     */
    ::RF24::Endpoint::Node getChildNode( const RF24::Hardware::PipeNumber pipe );

    /**
     *  Gets the Node attached to the Central Node as a parent	
     *	
     *	@return ::RF24::Endpoint::Node
     */
    ::RF24::Endpoint::Node getParentNode();

    /**
     *	Gets the Central Node
     *	
     *	@return ::RF24::Endpoint::Node
     */
    ::RF24::Endpoint::Node getCentralNode();

  protected:
    enum class InitType
    {
      AS_PARENT,
      AS_CHILD
    };

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
     *	@param[in]	address       The address to be bound
     *  @param[in]  placement     Where to place the address binding
     *	@return bool
     */
    bool initRegistration( const ::RF24::LogicalAddress address, InitType placement );

  private:
    ::RF24::Endpoint::Node mParentNode;
    ::RF24::Endpoint::Node mCentralNode;
    PipeRegistrationList mChildrenNodes;
  };

}    // namespace RF24::Endpoint::Internal::Processor

#endif /* !RF24_NODE_ENDPOINT_REGISTRATION_HPP */
