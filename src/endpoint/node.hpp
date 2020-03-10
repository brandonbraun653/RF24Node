/********************************************************************************
 *  File Name:
 *    node.hpp
 *
 *  Description:
 *    Defines the Node interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_ENDPOINT_NODE_HPP
#define RF24_ENDPOINT_NODE_HPP

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/hardware/definitions.hpp>

namespace RF24::Endpoint
{
  class Node
  {
  public:
    /*------------------------------------------------
    Ctors/Dtors
    ------------------------------------------------*/
    /**
     *	Constructs a Node object from the given address
     *
     *	@param[in]	address
     *	@return
     */
    Node( const LogicalAddress address );

    Node();
    ~Node();

    /*------------------------------------------------
    Data Getters
    ------------------------------------------------*/
    bool isValid() const ;
    LogicalAddress getLogicalAddress() const;
    PhysicalAddress getPhysicalAddress() const;
    Hardware::PipeNumber getPipe() const;
    LogicalLevel getLevel() const;

  private:
    bool mValidity;
    LogicalLevel mNetworkLevel;
    LogicalAddress mLogicalAddress;
    PhysicalAddress mPhysicalAddress;
    Hardware::PipeNumber mPipe;
  };
}    // namespace RF24::Endpoint


#endif /* !RF24_ENDPOINT_NODE_HPP */
