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

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* RF24 Includes */
#include <RF24Node/network/memory/heap.hpp>

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
    ManagedFIFO();
    ~ManagedFIFO();

    Chimera::Status_t setDepth( const size_t depth );

    Chimera::Status_t attachHeap( void * buffer, const size_t size );

    Chimera::Status_t push( const void *const data, const size_t size );

    Chimera::Status_t pop( void *const data, const size_t size );

    Chimera::Status_t clear();

    Element peekNextElement();

  private:
    size_t element_depth;

    boost::circular_buffer<Element> ringBuffer;
    RF24::Network::Memory::Heap heap;

  };

}    // namespace RF24::Network

#endif /* !RF24_NODE_NETWORK_QUEUE_HPP */