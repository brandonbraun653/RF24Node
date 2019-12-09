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


/* RF24 Includes */
#include <RF24Node/network/queue/queue.hpp>

namespace RF24::Network::Queue
{
  ManagedFIFO::ManagedFIFO()
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
    auto element = peekNextElement();

    if ( element.payload && element.size )
    {
      size_t bytesToCopy = std::min( size, element.size );
      memcpy( data, element.payload, bytesToCopy );

      heap.free( element.payload );
    }

    return Chimera::CommonStatusCodes::OK;
  }

  RF24::Network::Queue::Element ManagedFIFO::peekNextElement()
  {
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
}    // namespace RF24::Network::Queue