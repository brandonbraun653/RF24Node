/********************************************************************************
 *   File Name:
 *     queue.hpp
 *
 *   Description:
 *     Implements a variable element size circular buffer style queue
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_NETWORK_QUEUE_HPP
#define RF24_NODE_NETWORK_QUEUE_HPP

/* C++ Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

namespace RF24::Network::Queue
{
  struct Element
  {
    size_t size;    /**< Number of bytes in the queue element */
    void *payload;  /**< Pointer to the data */
  };

  class ManagedFIFO
  {
  public:
    ManagedFIFO( const size_t depth );
    ~ManagedFIFO();

    Chimera::Status_t attachStaticHeap( void * buffer, const size_t size );

    Chimera::Status_t attachDynamicHeap( const size_t size );

    Chimera::Status_t push( const void *const data, const size_t size );

    Chimera::Status_t pop( void *const data, const size_t size );

    Element peekNextElement();

  private:
    const size_t element_depth;
  };

}    // namespace RF24::Network

#endif /* !RF24_NODE_NETWORK_QUEUE_HPP */