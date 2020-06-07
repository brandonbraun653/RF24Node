/********************************************************************************
 *  File Name:
 *    rf24_network_route_table.cpp
 *
 *  Description:
 *    Methods for interaction with the registration aspect of endpoints
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/
 
/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_intf.hpp>

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/endpoint>
#include <RF24Node/src/network/connections/rf24_network_route_table.hpp>
#include <RF24Node/src/network/definitions.hpp>

namespace RF24::Network::Internal
{
  NodeConnections::NodeConnections( ::RF24::LogicalAddress address ) : mCentralNode( address )
  {
  }

  NodeConnections::NodeConnections()
  {
  }
  
  NodeConnections::~NodeConnections()
  {
  }

  void NodeConnections::initialize()
  {
    mParentNode = {};

    for ( size_t x = 0; x < mChildrenNodes.size(); x++ )
    {
      auto pipe = static_cast<RF24::Hardware::PipeNumber>( x );
      resetRegistration( pipe );
    }
  }

  void NodeConnections::updateCentralNode( ::RF24::LogicalAddress address )
  {
    if ( address != mCentralNode.getLogicalAddress() )
    {
      mCentralNode = ::RF24::Endpoint::Node( address );
      initialize();
    }
  }

  bool NodeConnections::attach( const RF24::LogicalAddress address )
  {
    if ( !isAddressValid( address ) )
    {
      return false;
    }
    else if ( isDirectDescendent( mCentralNode.getLogicalAddress(), address ) )
    {
      return initRegistration( address, InitType::AS_CHILD );
    }
    else if ( isDirectDescendent( address, mCentralNode.getLogicalAddress() ) )
    {
      return initRegistration( address, InitType::AS_PARENT );
    }
    else
    {
      return false;
    }
  }

  void NodeConnections::detach( const RF24::LogicalAddress address )
  {
    using namespace ::RF24::Physical;

    if ( address == mParentNode.getLogicalAddress() )
    {
      mParentNode = {};
    }
    else if ( address == getChildNode( Conversion::getExpectedPipe( address ) ).getLogicalAddress() )
    {
      resetRegistration( Conversion::getExpectedPipe( address ) );
    }
  }

  bool NodeConnections::isAddressBound( const RF24::LogicalAddress address )
  {
    using namespace ::RF24::Physical;

    if ( address == mParentNode.getLogicalAddress() )
    {
      return true;
    }
    else if ( address == getChildNode( Conversion::getExpectedPipe( address ) ).getLogicalAddress() )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool NodeConnections::isPipeBound( const RF24::Hardware::PipeNumber pipe )
  {
    const size_t lookupIndex = static_cast<size_t>( pipe );
    if ( lookupIndex < mChildrenNodes.size() )
    {
      return mChildrenNodes[ lookupIndex ].isValid();
    }

    return false;
  }

  const PipeRegistrationList &NodeConnections::getRegistrationList()
  {
    return mChildrenNodes;
  }

  RF24::Endpoint::Node NodeConnections::getChildNode( const RF24::Hardware::PipeNumber pipe )
  {
    const size_t lookupIndex = static_cast<size_t>( pipe );
    if ( lookupIndex < mChildrenNodes.size() )
    {
      return mChildrenNodes[ lookupIndex ];
    }

    return ::RF24::Endpoint::Node();
  }

  RF24::Endpoint::Node NodeConnections::getParentNode()
  {
    return mParentNode;
  }

  RF24::Endpoint::Node NodeConnections::getCentralNode()
  {
    return mCentralNode;
  }

  void NodeConnections::resetRegistration( const RF24::Hardware::PipeNumber pipe )
  {
    const size_t lookupIndex = static_cast<size_t>( pipe );
    if ( lookupIndex < mChildrenNodes.size() )
    {
      mChildrenNodes[ lookupIndex ] = {};
    }
  }

  bool NodeConnections::initRegistration( const RF24::LogicalAddress address, InitType placement )
  {
    size_t lookupIndex;
    auto logger = uLog::getRootSink();

    switch ( placement )
    {
      case InitType::AS_CHILD:
        lookupIndex = static_cast<size_t>( ::RF24::Physical::Conversion::getExpectedPipe( address ) );
        if ( lookupIndex < mChildrenNodes.size() )
        {
          logger->flog( uLog::Level::LVL_INFO, "%d-NET: Attached node 0%o as child\n", Chimera::millis(), address );
          mChildrenNodes[ lookupIndex ] = ::RF24::Endpoint::Node( address );
          return true;
        }
        return false;
        break;

      case InitType::AS_PARENT:
        logger->flog( uLog::Level::LVL_INFO, "%d-NET: Attached node 0%o as parent\n", Chimera::millis(), address );
        mParentNode = ::RF24::Endpoint::Node( address );
        return true;
        break;

      default:
        return false;
        break;
    }
  }
}    // namespace RF24::Endpoint::Internal
