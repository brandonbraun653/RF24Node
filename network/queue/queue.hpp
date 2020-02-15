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
#include <Chimera/common>
#include <Chimera/thread>

/* Aurora Includes */
#include <Aurora/memory/heap/heap.hpp>

namespace RF24::Network::Queue
{
  struct Element
  {
    size_t size;    /**< Number of bytes in the queue element */
    void *payload;  /**< Pointer to the data */
  };

  class ManagedFIFO : public Chimera::Threading::Lockable
  {
  public:
    ManagedFIFO();
    ~ManagedFIFO();

    Chimera::Status_t setDepth( const size_t depth );

    Chimera::Status_t attachHeap( void * buffer, const size_t size );

    Chimera::Status_t push( const void *const data, const size_t size );

    Chimera::Status_t pop( void *const data, const size_t size );

    Chimera::Status_t clear();

    Element peek();

    bool empty();

    bool full();

  private:
    size_t element_depth;

    boost::circular_buffer<Element> ringBuffer;
    Chimera::Modules::Memory::Heap heap;

  };

}    // namespace RF24::Network

#endif /* !RF24_NODE_NETWORK_QUEUE_HPP */