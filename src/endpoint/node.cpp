/********************************************************************************
 *  File Name:
 *    node.cpp
 *
 *  Description:
 *    Declares the Node implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/endpoint>


namespace RF24::Endpoint
{
  Node::Node( const LogicalAddress address )
  {
    using namespace ::RF24::Physical;

    if ( isAddressValid( address ) )
    {
      mValidity        = true;
      mLogicalAddress  = address;
      mPipe            = Conversion::getExpectedPipe( address );
      mPhysicalAddress = Conversion::getPhysicalAddress( address, mPipe );
      mNetworkLevel    = ::RF24::getLevel( address );
    }
    else
    {
      mValidity        = false;
      mLogicalAddress  = RF24::Network::RSVD_ADDR_INVALID;
      mPhysicalAddress = 0;
      mPipe            = Hardware::PipeNumber::PIPE_INVALID;
      mNetworkLevel    = NODE_LEVEL_INVALID;
    }
  }

  Node::Node() :
      mValidity( false ), mLogicalAddress( RF24::Network::RSVD_ADDR_INVALID ), mPhysicalAddress( 0 ),
      mPipe( Hardware::PipeNumber::PIPE_INVALID ), mNetworkLevel( NODE_LEVEL_INVALID )
  {
  }

  Node::~Node()
  {
  }

  bool Node::isValid() const
  {
    return mValidity;
  }

  LogicalAddress Node::getLogicalAddress() const
  {
    return mLogicalAddress;
  }

  PhysicalAddress Node::getPhysicalAddress() const
  {
    return mPhysicalAddress;
  }

  Hardware::PipeNumber Node::getPipe() const
  {
    return mPipe;
  }

  LogicalLevel Node::getLevel() const
  {
    return mNetworkLevel;
  }

}    // namespace RF24::Endpoint
