/********************************************************************************
 *   File Name:
 *    network.cpp
 *
 *   Description:
 *    Implements the NRF24L01 network layer driver.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cmath>

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/modules/ulog/serial_sink.hpp>

/* RF24 Includes */
#include <RF24Node/common/conversion.hpp>
#include <RF24Node/common/types.hpp>
#include <RF24Node/common/utility.hpp>
#include <RF24Node/network/network.hpp>
#include <RF24Node/network/types.hpp>
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/header/header.hpp>
#include <RF24Node/network/node/node.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>
#include <uLog/sinks/sink_cout.hpp>

namespace RF24::Network
{
  static constexpr uint16_t max_frame_payload_size = FRAME_TOTAL_SIZE - sizeof( FrameHeaderField );


  // TODO: I don't actually know the purpose of this...
  static uint16_t levelToAddress( uint8_t level );

  bool isARootNode( const LogicalAddress address )
  { 
    /*------------------------------------------------
    All variants of root nodes have their first level bits set to zero
    ------------------------------------------------*/
    return ( ( address & ADDR_LEVEL0 ) == 0 );
  }

  Driver::Driver() : frameQueue( FrameCache_t( 3 ) )
  {
    txTime         = 0;
    networkFlags   = 0;
    returnSysMsgs  = 0;
    multicastRelay = 0;

    children.fill( EMPTY_LOGICAL_ADDRESS );
    childAttached.fill( false );

    HeaderHelper::initialize();

    txFrame.clear();
    logger = nullptr;
  }

  Driver::~Driver()
  {
  }

  
  Chimera::Status_t Driver::attachLogger( uLog::SinkHandle sink )
  {
    logger = sink;
    return Chimera::CommonStatusCodes::OK;
  }


  Chimera::Status_t Driver::attachPhysicalDriver( RF24::Physical::Interface_sPtr &physicalLayer )
  {
    /*------------------------------------------------
    Shared_ptr could be passed in but still empty
    ------------------------------------------------*/
    if ( !physicalLayer )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    radio = physicalLayer;
    return Chimera::CommonStatusCodes::OK;
  }

  
  Chimera::Status_t Driver::initRXQueue( void *buffer, const size_t size )
  {
    return rxQueue.attachHeap( buffer, size );
  }

  Chimera::Status_t Driver::initTXQueue( void *buffer, const size_t size )
  {
    return txQueue.attachHeap( buffer, size );
  }


  Chimera::Status_t Driver::begin( const uint8_t channel, const uint16_t nodeAddress, const RF24::Hardware::DataRate dataRate,
                                    const RF24::Hardware::PowerAmplitude pwr )
  {
    using namespace RF24::Hardware;
    using namespace RF24::Physical::Conversion;

    auto result = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Check error conditions that would prevent a solid startup.
    ------------------------------------------------*/
    if ( !isAddressChild( nodeAddress ) )
    {
      oopsies = ErrorType::INVALID_ADDRESS;
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_ERROR, "ERR: Invalid node address\n" ); );
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Turn on the radio. By default, this wipes all pre-existing settings.
    ------------------------------------------------*/
    if ( !radio->isInitialized() )
    {
      /*------------------------------------------------
      The system model is to interact with the network layer, not the
      physical layer. Let this function initialize the radio->
      ------------------------------------------------*/
      oopsies = ErrorType::RADIO_FAILED_INIT;
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_ERROR, "ERR: Radio not initialized\n" ); );
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Reconfigure the radio
    ------------------------------------------------*/
    txTimeout          = 25;
    routeTimeout       = txTimeout * 3;    // Adjust for max delay per node within a single chain
    logicalNodeAddress = nodeAddress;

    result |= radio->setChannel( channel );
    result |= radio->setPALevel( pwr );
    result |= radio->setDataRate( dataRate );
    result |= radio->toggleAutoAck( false, RF24::Hardware::PIPE_NUM_0 );
    result |= radio->toggleDynamicPayloads( true );

    /*------------------------------------------------
    Use different retry periods to reduce data collisions
    ------------------------------------------------*/
    auto retryVar = static_cast<RF24::Hardware::AutoRetransmitDelay>( ( ( ( nodeAddress % 6 ) + 1 ) * 2 ) + 3 );
    result |= radio->setRetries( retryVar, 5 );

    /*------------------------------------------------
    Set up the address helper cache
    ------------------------------------------------*/
    setupAddress();

    /*------------------------------------------------
    Open all pipes in listening mode
    ------------------------------------------------*/
    IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_DEBUG, "%d: NET: Opening all pipes for listening\n", Chimera::millis() ); );

    result |= radio->setStaticPayloadSize( RF24::Hardware::MAX_PAYLOAD_WIDTH );

    for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
    {
      auto pipe = static_cast<PipeNumber_t>( i );
      auto addr = getPipeAddress( nodeAddress, pipe );

      result |= radio->openReadPipe( pipe, addr, true );


      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: NET Pipe %i on node 0%o has IP[%s] and Port[%d]\n",
                                     Chimera::millis(), pipe, nodeAddress, decodeIP( addr ).c_str(), decodePort( addr ) ); );
    }

    result |= radio->startListening();

    /*------------------------------------------------
    Make sure we passed the init sequence
    ------------------------------------------------*/
    if ( result != Chimera::CommonStatusCodes::OK )
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_ERROR, "ERR: NET init failed\n" ); );
    }

    return result;
  }

  NetHdrMsgType Driver::update()
  {
    if ( !initialized )
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_DEBUG, "%d: NET Not initialized\n", Chimera::millis() ); );
      oopsies = ErrorType::NOT_INITIALIZED;
      return MSG_NETWORK_ERR;
    }

    uint8_t pipeNum         = 0u;
    NetHdrMsgType returnVal = MSG_TX_NORMAL;

    /*------------------------------------------------
    If BYPASS_HOLDS is enabled, incoming user data may be dropped. This allows for
    system payloads to be read while the user cache is full. HOLD_INCOMING prevents data from
    being read from the radios, thereby preventing incoming payloads from being acknowledged.
    ------------------------------------------------*/
    if ( !( networkFlags & FLAG_BYPASS_HOLDS ) && ( ( networkFlags & FLAG_HOLD_INCOMING ) || frameQueue.full() ) )
    {
      if ( !available() )
      {
        networkFlags &= ~FLAG_HOLD_INCOMING;
      }
      else
      {
        return returnVal;
      }
    }

    /*------------------------------------------------
    Process the incoming/outgoing data
    ------------------------------------------------*/
    while ( radio->payloadAvailable() < RF24::Hardware::PIPE_NUM_MAX )
    {
      radioPayloadSize = radio->getDynamicPayloadSize();

      /*------------------------------------------------
      The data received is garbage, go on to the next pipe.
      ------------------------------------------------*/
      if ( radioPayloadSize < FRAME_PREAMBLE_SIZE )
      {
        Chimera::delayMilliseconds( 10 );
        continue;
      }

      /*------------------------------------------------
      Dump the payloads until we've gotten everything.
      Fetch the payload, and see if this was the last one.
      ------------------------------------------------*/
      radio->readPayload( frameBuffer.data(), frameBuffer.size(), radioPayloadSize );

      /*------------------------------------------------
      Read the beginning of the frame as the header
      ------------------------------------------------*/
      FrameHelper frame( frameBuffer );

      FrameHeaderField *header = &frame.data.header;

      IF_SERIAL_DEBUG( HeaderHelper headerClass; headerClass = frame.data.header;
                       logger->flog( uLog::Level::LVL_INFO, "%d: MAC Received on pipe %u, Header: [%s]\n\r", Chimera::millis(),
                                   pipeNum, headerClass.toString() ); );

      /*------------------------------------------------
      Throw it away if it's not a valid address
      ------------------------------------------------*/
      if ( !isAddressChild( header->dstNode ) )
      {
        continue;
      }

      returnVal = static_cast<NetHdrMsgType>( header->msgType );

      /*------------------------------------------------
      Is this message for us?
      ------------------------------------------------*/
      if ( header->dstNode == this->logicalNodeAddress )
      {
        /*------------------------------------------------
        No action required for this one
        ------------------------------------------------*/
        if ( header->msgType == MSG_NETWORK_PING )
        {
          continue;
        }

        /*------------------------------------------------
        Allow the Mesh layer to process the address response
        ------------------------------------------------*/
        if ( header->msgType == MSG_MESH_ADDR_RESPONSE )
        {
          return MSG_MESH_ADDR_RESPONSE;
        }

        /*------------------------------------------------
        Allow the Mesh layer to process the address requesting
        ------------------------------------------------*/
        if ( header->msgType == MSG_MESH_REQ_ADDRESS )
        {
          return MSG_MESH_REQ_ADDRESS;
        }

        /*------------------------------------------------

        ------------------------------------------------*/
        if ( ( returnSysMsgs && ( header->msgType > MSG_MAX_USER_DEFINED_HEADER_TYPE ) ) || header->msgType == MSG_NETWORK_ACK )
        {
          IF_SERIAL_DEBUG_ROUTING(
              logger->flog( uLog::Level::LVL_INFO, "%d MAC: System payload rcvd %d\n", Chimera::millis(), returnVal ); );

          if ( header->msgType != MSG_NETWORK_FIRST_FRAGMENT && header->msgType != MSG_NETWORK_MORE_FRAGMENTS &&
               header->msgType != MSG_NETWORK_MORE_FRAGMENTS_NACK && header->msgType != MSG_EXTERNAL_DATA_TYPE &&
               header->msgType != MSG_NETWORK_LAST_FRAGMENT )
          {
            return returnVal;
          }
        }

        enqueue( frame );
      }
      else
      {
        /*------------------------------------------------
        Handle a multicast scenario
        ------------------------------------------------*/
        if ( header->dstNode == RSVD_ADDR_MULTICAST )
        {
          if ( header->msgType == MSG_NETWORK_POLL )
          {
            /*------------------------------------------------
            Assuming we:
                1. Have a slot available for another child to attach
                2. Are initialized and connected to the network
                3. Are not blocked by a flag

            Send an empty message directly back to the polling node to indicate they can attach
            ------------------------------------------------*/
            if ( childrenAvailable() && ( this->logicalNodeAddress != DEFAULT_LOGICAL_ADDRESS ) &&
                 !( networkFlags & FLAG_NO_POLL ) )
            {
              header->dstNode = header->srcNode;
              header->srcNode = this->logicalNodeAddress;

              Chimera::delayMilliseconds( parentPipe );

              memcpy( frameBuffer.data(), header, sizeof( FrameHeaderField ) );
              writeDirect( header->dstNode, MSG_USER_TX_TO_PHYSICAL_ADDRESS );
            }

            continue;
          }

          if ( multicastRelay )
          {
            IF_SERIAL_DEBUG_ROUTING( logger->flog( uLog::Level::LVL_INFO, "%u: MAC FWD multicast frame from 0%o to level %u\n",
                                                 Chimera::millis(), header->srcNode, multicastLevel + 1 ); );

            /*------------------------------------------------
            For all but the first level of nodes (those not directly
            connected to the master) we add the total delay per level.
            ------------------------------------------------*/
            if ( ( this->logicalNodeAddress >> 3 ) != 0 )
            {
              Chimera::delayMilliseconds( 1 );
            }

            Chimera::delayMilliseconds( this->logicalNodeAddress % 4 );
            writeDirect( levelToAddress( multicastLevel ) << 3, MSG_USER_TX_MULTICAST );
          }
        }
        else
        {
          /*------------------------------------------------
          Send it on, indicate it is a routed payload
          ------------------------------------------------*/
          writeDirect( header->dstNode, MSG_TX_ROUTED );
        }
      }
    }

    return returnVal;
  }

  bool Driver::available() const
  {
    if ( !initialized )
    {
      return false;
    }

    return !frameQueue.empty();
  }

  uint16_t Driver::parent() const
  {
    if ( logicalNodeAddress == 0 )
    {
      return -1;
    }
    else
    {
      return parentNode;
    }
  }

  uint16_t Driver::peek( HeaderHelper &header )
  {
    uint16_t msgSize = 0u;

    if ( available() )
    {
      auto frame = frameQueue.front();

      header  = frame.header;
      msgSize = frame.messageLength;
    }

    return msgSize;
  }

  bool Driver::peek( HeaderHelper &header, void *message, uint16_t maxlen )
  {
    if ( available() )
    {
      auto frame = frameQueue.front();
      /*------------------------------------------------
      Copy the header and message
      ------------------------------------------------*/
      header = frame.header;

      if ( maxlen <= frame.message.max_size() )
      {
        memcpy( message, frame.message.data(), maxlen );
      }
    }
  }

  bool Driver::read( HeaderHelper &header, void *message, uint16_t maxlen )
  {
    uint16_t msgLen     = 0; /* How large the payload is */
    uint16_t readLength = 0; /* How many bytes were copied out */

    if ( available() )
    {
      auto frame = frameQueue.front();

      /*------------------------------------------------
      Copy the header over to the user's variable
      ------------------------------------------------*/
      header = frame.header;
      msgLen = frame.messageLength;

      /*------------------------------------------------
      Ensure the buffer is large enough to handle the message
      ------------------------------------------------*/
      if ( maxlen >= msgLen )
      {
        /*------------------------------------------------
        Copy over the message data
        ------------------------------------------------*/
        readLength = std::min( maxlen, msgLen );
        memcpy( message, frame.message.data(), readLength );


        /*------------------------------------------------
        If enabled, print out the message data
        ------------------------------------------------*/
        IF_SERIAL_DEBUG( uint16_t len = maxlen;
                         logger->flog( uLog::Level::LVL_INFO, "%d: NET message size %d\n", Chimera::millis(), msgLen );
                         logger->flog( uLog::Level::LVL_INFO, "%d: NET r message ", Chimera::millis() );
                         const uint8_t *charPtr = reinterpret_cast<const uint8_t *>( message );
                         while ( len-- ) { logger->flog( uLog::Level::LVL_INFO, "%02x ", charPtr[ len ] ); }

                         logger->flog( uLog::Level::LVL_INFO, "\n\r" ); );

        /*------------------------------------------------
        Remove the packet that was just read out
        ------------------------------------------------*/
        frameQueue.pop_front();
        IF_SERIAL_DEBUG(
            logger->flog( uLog::Level::LVL_INFO, "%d: NET Received %s\n\r", Chimera::millis(), header.toString() ); );
      }
    }

    return readLength;
  }

  bool Driver::multicast( HeaderHelper &header, const void *const message, const uint16_t len, const uint8_t level )
  {
    header.data.dstNode = RSVD_ADDR_MULTICAST;
    header.data.srcNode = this->logicalNodeAddress;

    return write( header, message, len, levelToAddress( level ) );
  }

  bool Driver::write( HeaderHelper &header, const void *const message, const uint16_t len )
  {
    return write( header, message, len, RSVD_ADDR_ROUTED );
  }

  bool Driver::write( HeaderHelper &header, const void *const message, const uint16_t len, const NodeAddressType writeDirect )
  {
    /*------------------------------------------------
    Protect against invalid inputs
    ------------------------------------------------*/
    if ( !initialized )
    {
      oopsies = ErrorType::NOT_INITIALIZED;
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: ERR Not initialized\n", Chimera::millis() ); );
      return false;
    }

    /*------------------------------------------------
    Allows time for requests (RF24Mesh) to get through between failed writes on busy nodes
    ------------------------------------------------*/
    while ( Chimera::millis() - txTime < 25 )
    {
      if ( update() > MSG_MAX_USER_DEFINED_HEADER_TYPE )
      {
        break;
      }
    }

    /*------------------------------------------------
    Do a normal, unfragmented write
    ------------------------------------------------*/
    if ( len <= max_frame_payload_size )
    {
      radioPayloadSize = len + FRAME_PREAMBLE_SIZE;
      if ( _write( header, message, len, writeDirect ) )
      {
        return true;
      }
      txTime = Chimera::millis();
      return false;
    }

    return true;
  }

  bool Driver::_write( HeaderHelper &header, const void *const message, const uint16_t len, const uint16_t directTo )
  {
    /*------------------------------------------------
    Fill out the header
    ------------------------------------------------*/
    header.data.srcNode = logicalNodeAddress;

    /*------------------------------------------------
    Build the full frame to send
    ------------------------------------------------*/
    txFrame.build( header.data, len, message );

    IF_SERIAL_DEBUG(
        logger->flog( uLog::Level::LVL_INFO, "%d: NET Sending Header [%s]\n\r", Chimera::millis(), header.toString() );

        if ( len ) {
          uint16_t tmpLen        = len;
          const uint8_t *charPtr = reinterpret_cast<const uint8_t *>( message );

          logger->flog( uLog::Level::LVL_INFO, "%d: NET message ", Chimera::millis() );
          while ( tmpLen-- )
          {
            logger->flog( uLog::Level::LVL_INFO, "%02x ", charPtr[ tmpLen ] );
          }

          logger->flog( uLog::Level::LVL_INFO, ( "\n\r" ) );
        } );

    /*------------------------------------------------
    Decide where that frame is going to be sent
    ------------------------------------------------*/
    if ( directTo == RSVD_ADDR_ROUTED )
    {
      return writeDirect( header.data.dstNode, MSG_TX_NORMAL );
    }
    else
    {
      /*------------------------------------------------
      Payload is multicast to the first node, and routed normally to the next
      ------------------------------------------------*/
      NetHdrMsgType sendType = MSG_USER_TX_TO_LOGICAL_ADDRESS;

      if ( header.data.dstNode == RSVD_ADDR_MULTICAST )
      {
        sendType = MSG_USER_TX_MULTICAST;
      }

      if ( header.data.dstNode == directTo )
      {
        sendType = MSG_USER_TX_TO_PHYSICAL_ADDRESS;    // Payload is multicast to the first node, which is the recipient
      }

      return writeDirect( directTo, sendType );
    }
  }

  bool Driver::writeDirect( uint16_t toNode, LogicalAddress directTo )
  {
    // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route
    /*------------------------------------------------
    Throw it away if it's not a valid address
    ------------------------------------------------*/
    if ( !isAddressChild( toNode ) )
    {
      oopsies = ErrorType::INVALID_ADDRESS;
      return false;
    }

    /*------------------------------------------------
    Load info into our conversion structure, and get the converted address info
    ------------------------------------------------*/
    logicalToPhysicalStruct conversion = { toNode, static_cast<RF24::Hardware::PipeNumber_t>( directTo ), 0 };
    logicalToPhysicalAddress( &conversion );

    /*------------------------------------------------
    Write it
    ------------------------------------------------*/
    IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: MAC Sending to node [0%o] via node [0%o] on pipe %x\n\r",
                                 Chimera::millis(), toNode, conversion.send_node, conversion.send_pipe ); );


    memcpy( frameBuffer.data(), &txFrame.data, sizeof( FrameData ) );

    bool writeSucceeded = writeFrameBufferToPipeAtNodeID( conversion.send_node, conversion.send_pipe, conversion.multicast );
    bool isAckType =
        ( txFrame.data.header.msgType >= MSG_USER_MIN_ACK ) && ( txFrame.data.header.msgType <= MSG_NETWORK_MAX_ACK );

    if ( !writeSucceeded )
    {
      IF_SERIAL_DEBUG_ROUTING( logger->flog( uLog::Level::LVL_INFO, "%d: MAC Send fail to 0%o via 0%o on pipe %x\n\r",
                                           Chimera::millis(), toNode, conversion.send_node, conversion.send_pipe ); );
    }


    /*------------------------------------------------
    Handle writes that have an ACK coming back
    ------------------------------------------------*/
    if ( writeSucceeded && isAckType )
    {
      /*------------------------------------------------

      ------------------------------------------------*/
      if ( directTo == MSG_TX_ROUTED && conversion.send_node == toNode )
      {
        txFrame.data.header.msgType = MSG_NETWORK_ACK;                // Set the payload type to NETWORK_ACK
        txFrame.data.header.dstNode = txFrame.data.header.srcNode;    // Change the 'to' address to the 'from' address

        conversion.send_node = txFrame.data.header.srcNode;
        conversion.send_pipe = static_cast<RF24::Hardware::PipeNumber_t>( MSG_TX_ROUTED );
        conversion.multicast = 0;
        logicalToPhysicalAddress( &conversion );

        // TODO: When you figure out what this section does, re-evaluate radioPayloadSize...likely to be wrong
        // TODO: BUG!
        radioPayloadSize = sizeof( FrameHeaderField );
        writeFrameBufferToPipeAtNodeID( conversion.send_node, conversion.send_pipe, conversion.multicast );

        IF_SERIAL_DEBUG_ROUTING( logger->flog( uLog::Level::LVL_INFO, "%d MAC: Route OK to 0%o ACK sent to 0%o\n",
                                             Chimera::millis(), toNode, txFrame.data.header.srcNode ); );
      }

      /*------------------------------------------------

      ------------------------------------------------*/
      if ( conversion.send_node != toNode && ( directTo == MSG_TX_NORMAL || directTo == MSG_USER_TX_TO_LOGICAL_ADDRESS ) )
      {
        // Now, continue listening
        if ( networkFlags & FLAG_FAST_FRAG )
        {
          radio->txStandBy( txTimeout );
          networkFlags &= ~FLAG_FAST_FRAG;
          radio->toggleAutoAck( false, RF24::Hardware::PIPE_NUM_0 );
        }
        radio->startListening();

        uint32_t reply_time = Chimera::millis();

        while ( update() != MSG_NETWORK_ACK )
        {
          if ( Chimera::millis() - reply_time > routeTimeout )
          {
            IF_SERIAL_DEBUG_ROUTING( logger->flog( uLog::Level::LVL_INFO,
                                                 "%d: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r", Chimera::millis(),
                                                 toNode, conversion.send_node, conversion.send_pipe ); );
            writeSucceeded = false;
            break;
          }
        }
      }
    }


    radio->startListening();
    return writeSucceeded;
  }

  bool Driver::logicalToPhysicalAddress( logicalToPhysicalStruct *conversionInfo )
  {
    // Create pointers so this makes sense.. kind of
    // We take in the toNode(logical) now, at the end of the function, output the send_node(physical) address, etc.
    // back to the original memory address that held the logical information.
    auto *toNode  = &conversionInfo->send_node;
    auto *directTo = &conversionInfo->send_pipe;
    bool *multicast   = &conversionInfo->multicast;

    // Where do we send this?  By default, to our parent
    auto pre_conversion_send_node = parentNode;

    // On which pipe
    auto pre_conversion_send_pipe = parentPipe;

    if ( *directTo > static_cast<uint8_t>( MSG_TX_ROUTED ) )
    {
      pre_conversion_send_node = *toNode;
      *multicast               = 1;
      pre_conversion_send_pipe = RF24::Hardware::PIPE_NUM_0;
    }
    else if ( isDirectChild( *toNode ) )
    {
      // Send directly
      pre_conversion_send_node = *toNode;
      // To its listening pipe
      pre_conversion_send_pipe = RF24::Hardware::PIPE_NUM_5;
    }
    // If the node is a child of a child
    // talk on our child's listening pipe,
    // and let the direct child relay it.
    else if ( isDescendant( *toNode ) )
    {
      pre_conversion_send_node = directChildRouteTo( *toNode );
      pre_conversion_send_pipe = RF24::Hardware::PIPE_NUM_5;
    }

    *toNode   = pre_conversion_send_node;
    *directTo = pre_conversion_send_pipe;

    return 1;
  }

  bool Driver::writeFrameBufferToPipeAtNodeID( const LogicalAddress node, const RF24::Hardware::PipeNumber_t pipe, const bool multicast )
  {
    Chimera::Status_t result  = Chimera::CommonStatusCodes::OK;
    uint64_t writePipeAddress = RF24::Physical::Conversion::getPipeAddress( node, pipe );

    /*------------------------------------------------
    If we are multi casting, turn off auto acknowledgment. 
    We don't care how the message gets out as long as it does.
    ------------------------------------------------*/
    result |= radio->stopListening();
    result |= radio->toggleAutoAck( !multicast, RF24::Hardware::PIPE_NUM_0 );
    result |= radio->openWritePipe( writePipeAddress );
    result |= radio->immediateWrite( frameBuffer.data(), radioPayloadSize, false );
    result |= radio->txStandBy( txTimeout );

    return result == Chimera::CommonStatusCodes::OK;
  }

  bool Driver::isDirectChild( uint16_t node )
  {
    bool result = false;

    // A direct child of ours has the same low numbers as us, and only
    // one higher number.
    //
    // e.g. node 0234 is a direct child of 034, and node 01234 is a
    // descendant but not a direct child

    // First, is it even a descendant?
    if ( isDescendant( node ) )
    {
      // Does it only have ONE more level than us?
      uint16_t child_node_mask = ( ~nodeMask ) << 3;
      result                   = ( node & child_node_mask ) == 0;
    }
    return result;
  }

  bool Driver::isDescendant( uint16_t node )
  {
    return ( node & nodeMask ) == logicalNodeAddress;
  }

  void Driver::setupAddress()
  {
    /*------------------------------------------------
    First, establish the node mask
    ------------------------------------------------*/
    uint16_t node_mask_check = 0xFFFF;
    uint8_t count            = 0;

    while ( this->logicalNodeAddress & node_mask_check )
    {
      node_mask_check <<= OCTAL_TO_BIN_BITSHIFT;
      count++;
    }
    multicastLevel = count;

    nodeMask = ~node_mask_check;

    /*------------------------------------------------
    Parent mask is the next level down
    ------------------------------------------------*/
    uint16_t parent_mask = nodeMask >> OCTAL_TO_BIN_BITSHIFT;

    /*------------------------------------------------
    Parent node is the part IN the mask
    ------------------------------------------------*/
    parentNode = this->logicalNodeAddress & parent_mask;

    /*------------------------------------------------
    Parent pipe is the part OUT of the mask
    ------------------------------------------------*/
    uint16_t i = this->logicalNodeAddress;
    uint16_t m = parent_mask;

    while ( m )
    {
      i >>= OCTAL_TO_BIN_BITSHIFT;
      m >>= OCTAL_TO_BIN_BITSHIFT;
    }
    parentPipe = static_cast<RF24::Hardware::PipeNumber_t>( i );

    IF_SERIAL_DEBUG_MINIMAL( logger->flog( uLog::Level::LVL_INFO, "setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r",
                                         this->logicalNodeAddress, nodeMask, parentNode, parentPipe ); );
  }

  void Driver::enqueue( FrameHelper &frame )
  {
    if ( !frameQueue.full() )
    {
      frameQueue.push_back( frame.data );
    }
    else
    {
      IF_SERIAL_DEBUG( logger->flog( uLog::Level::LVL_INFO, "%d: NET **Drop Payload** Buffer Full\n", Chimera::millis() ); );
    }
  }

  uint16_t Driver::directChildRouteTo( uint16_t node )
  {
    // Presumes that this is in fact a child!!
    uint16_t child_mask = ( nodeMask << 3 ) | 0x07;
    return node & child_mask;
  }

  bool Driver::setAddress( const LogicalAddress address )
  {
    if ( isAddressChild( address ) )
    {
      bool initialized         = true;
      this->logicalNodeAddress = address;

      /*------------------------------------------------
      Make sure we can't receive any data
      ------------------------------------------------*/
      radio->stopListening();

      /*------------------------------------------------
      Close all the previously opened pipes
      ------------------------------------------------*/
      for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
      {
        radio->closeReadPipe( static_cast<RF24::Hardware::PipeNumber_t>( i ) );
      }

      /*------------------------------------------------
      Re-setup the address helper cache
      ------------------------------------------------*/
      setupAddress();

      /*------------------------------------------------
      Open all the listening pipes
      ------------------------------------------------*/
      for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
      {
        auto _pipe = static_cast<RF24::Hardware::PipeNumber_t>( i );
        initialized &= radio->openReadPipe( _pipe, RF24::Physical::Conversion::getPipeAddress( address, _pipe ), true );
      }
      radio->startListening();

      return initialized;
    }
    else
    {
      oopsies = ErrorType::INVALID_ADDRESS;
      return false;
    }
  }

  LogicalAddress Driver::getLogicalAddress()
  {
    return this->logicalNodeAddress;
  }

  void Driver::setMulticastLevel( uint8_t level )
  {
    multicastLevel = level;
    radio->openReadPipe( static_cast<RF24::Hardware::PipeNumber_t>( 0u ), RF24::Physical::Conversion::getPipeAddress( levelToAddress( level ), RF24::Hardware::PIPE_NUM_0 ), true );
  }


  static uint16_t levelToAddress( uint8_t level )
  {
    uint16_t levelAddr = 1;
    if ( level )
    {
      levelAddr = levelAddr << ( ( level - 1 ) * OCTAL_TO_BIN_BITSHIFT );
    }
    else
    {
      return 0;
    }
    return levelAddr;
  }


}    // namespace RF24::Network
