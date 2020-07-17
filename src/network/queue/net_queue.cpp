/********************************************************************************
 *   File Name:
 *     queue.cpp
 *
 *   Description:
 *     Implementation of the network message queue
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Chimera Includes */
#include <Chimera/thread>

/* RF24 Includes */
#include <RF24Node/src/network/queue/queue.hpp>

namespace RF24::Network::Queue
{
  ManagedFIFO::ManagedFIFO() : ringBuffer( 10 )
  {
    element_depth = 10;
  }

  ManagedFIFO::~ManagedFIFO()
  {
  }

  Chimera::Status_t ManagedFIFO::setDepth( const size_t depth )
  {
    ringBuffer.set_capacity( depth );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t ManagedFIFO::attachHeap( void *buffer, const size_t size )
  {
    if ( buffer && size )
    {
      return heap.attachStaticBuffer( buffer, size );
    }
    else if ( size )
    {
      return heap.attachDynamicBuffer( size );
    }
    else
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
  }

  Chimera::Status_t ManagedFIFO::push( const void *const data, const size_t size )
  {
    /*------------------------------------------------
    Input protection and early exit
    ------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else if ( ringBuffer.full() )
    {
      return Chimera::CommonStatusCodes::FULL;
    }

    /*------------------------------------------------
    Allocate new memory from the heap and copy in the data
    ------------------------------------------------*/
    Element temp;
    temp.payload = heap.malloc( size );
    temp.size    = size;

    if ( !temp.payload )
    {
      return Chimera::CommonStatusCodes::MEMORY;
    }
    else
    {
      memcpy( temp.payload, data, temp.size );

      ringBuffer.push_back( temp );
      return Chimera::CommonStatusCodes::OK;
    }
  }

  Chimera::Status_t ManagedFIFO::pop( void *const data, const size_t size )
  {
    /*------------------------------------------------
    Input protection and early exit
    ------------------------------------------------*/
    if ( !data || !size )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else if ( ringBuffer.empty() )
    {
      return Chimera::CommonStatusCodes::EMPTY;
    }

    /*------------------------------------------------
    Once we know the element exists, copy out the data
    and then free the allocated memory.
    ------------------------------------------------*/
    auto element = peek();

    if ( element.payload && element.size )
    {
      size_t bytesToCopy = std::min( size, element.size );
      memcpy( data, element.payload, bytesToCopy );

      heap.free( element.payload );
      ringBuffer.pop_front();
    }

    return Chimera::CommonStatusCodes::OK;
  }

  RF24::Network::Queue::Element &ManagedFIFO::peek()
  {
    if ( ringBuffer.empty() )
    {
      Element x;
      x.size    = 0;
      x.payload = nullptr;

      return x;
    }

    return ringBuffer.front();
  }

  Chimera::Status_t ManagedFIFO::clear()
  {
    while ( !ringBuffer.empty() )
    {
      heap.free( ringBuffer.front().payload );
      ringBuffer.pop_front();
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t ManagedFIFO::removeFront()
  {
    if ( !ringBuffer.empty() )
    {
      auto element = peek();
      if ( element.payload && element.size )
      {
        heap.free( element.payload );
        ringBuffer.pop_front();
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  bool ManagedFIFO::empty()
  {
    return ringBuffer.empty();
  }

  bool ManagedFIFO::full()
  {
    return ringBuffer.full();
  }

}    // namespace RF24::Network::Queue