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
      mValidity = true;
      mLogicalAddress = address;
      mPipe = Conversion::getExpectedPipe( address );
      mPhysicalAddress = Conversion::getPhysicalAddress( address, mPipe );
    }
    else
    {
      mValidity = false;
      mLogicalAddress = RF24::Network::RSVD_ADDR_INVALID;
      mPhysicalAddress = 0;
      mPipe = Hardware::PipeNumber::PIPE_INVALID;
    }
  }

  Node::Node() :
      mValidity( false ), mLogicalAddress( RF24::Network::RSVD_ADDR_INVALID ), mPhysicalAddress( 0 ),
      mPipe( Hardware::PipeNumber::PIPE_INVALID )
  {
  }

  Node::~Node()
  {
  }

  bool Node::isValid()
  {
    return mValidity;
  }

  LogicalAddress Node::getLogicalAddress()
  {
    return mLogicalAddress;
  }

  PhysicalAddress Node::getPhysicalAddress()
  {
    return mPhysicalAddress;
  }

  Hardware::PipeNumber Node::getPipe()
  {
    return mPipe;
  }

}    // namespace RF24::Endpoint
