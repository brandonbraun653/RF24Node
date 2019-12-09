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

/* Driver Includes */
#include <RF24Node/network/network.hpp>
#include <RF24Node/network/network_types.hpp>
#include <RF24Node/network/frame/frame.hpp>
#include <RF24Node/network/header/header.hpp>
#include <RF24Node/network/node/node.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>

namespace RF24::Network
{
  static constexpr uint16_t max_frame_payload_size = FRAME_TOTAL_SIZE - sizeof( FrameHeaderField );

  static uLog::SinkType debugSink = nullptr;

  /**
   *  Create and register the serial output sink used for debug messages
   *
   *  @return void
   */
  static void initializeDebugSink()
  {
    /*------------------------------------------------
    Create the sink
    ------------------------------------------------*/
#if defined( MICRO_LOGGER_HAS_VGDB_SEMIHOSTING ) && ( MICRO_LOGGER_HAS_VGDB_SEMIHOSTING == 1 )
    debugSink = std::make_shared<uLog::VGDBSemihostingSink>();
#else
    debugSink = std::make_shared<Chimera::Modules::uLog::SerialSink>();
#endif

    debugSink->setLogLevel( uLog::Level::LVL_DEBUG );

    /*------------------------------------------------
    Register the sink with the logger
    ------------------------------------------------*/
    auto sinkHandle = uLog::registerSink( debugSink );
    uLog::enableSink( sinkHandle );
  }

  Network::Network() : frameQueue( FrameCache_t( 3 ) )
  {
    txTime         = 0;
    networkFlags   = 0;
    returnSysMsgs  = 0;
    multicastRelay = 0;

    children.fill( EMPTY_LOGICAL_ADDRESS );
    childAttached.fill( false );

    HeaderHelper::initialize();

    txFrame.clear();

#if defined( SERIAL_DEBUG )
    initializeDebugSink();
#endif
  }

  Network::~Network()
  {
  }

  Chimera::Status_t Network::attachPhysicalDriver( RF24::Physical::Interface_sPtr &physicalLayer )
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

  
  Chimera::Status_t Network::initRXQueue( void *buffer, const size_t size )
  {
    return rxQueue.attachHeap( buffer, size );
  }

  Chimera::Status_t Network::initTXQueue( void *buffer, const size_t size )
  {
    return txQueue.attachHeap( buffer, size );
  }


  bool Network::begin( const uint8_t channel, const uint16_t nodeAddress, const RF24::Hardware::DataRate dataRate,
                       const RF24::Hardware::PowerAmplitude pwr )
  {
    using namespace RF24::Hardware;

    auto result = Chimera::CommonStatusCodes::OK;
    initialized = true;

    /*------------------------------------------------
    Check error conditions that would prevent a solid startup.
    ------------------------------------------------*/
    if ( !isValidNetworkAddress( nodeAddress ) )
    {
      oopsies     = ErrorType::INVALID_ADDRESS;
      initialized = false;
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_ERROR, "ERR: Invalid node address\r\n" ); );
      return false;
    }

    /*------------------------------------------------
    Turn on the radio. By default, this wipes all pre-existing settings.
    ------------------------------------------------*/
    if ( radio->isInitialized() )
    {
      /*------------------------------------------------
      The system model is to interact with the network layer, not the
      physical layer. Let this function initialize the radio->
      ------------------------------------------------*/
      oopsies     = ErrorType::RADIO_PRE_INITIALIZED;
      initialized = false;
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_ERROR, "ERR: Radio pre-initialized\r\n" ); );
      return false;
    }
    else if ( radio->initialize() != Chimera::CommonStatusCodes::OK )
    {
      /*------------------------------------------------
      More than likely a register read/write failed.
      ------------------------------------------------*/
      oopsies     = ErrorType::RADIO_FAILED_INIT;
      initialized = false;
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_ERROR, "ERR: Radio HW failed init\r\n" ); );
      return false;
    }

    /*------------------------------------------------
    Initialize the radio
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
    IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_DEBUG, "%d: NET: Opening all pipes for listening\r\n", Chimera::millis() ); );

    result |= radio->setStaticPayloadSize( RF24::Hardware::MAX_PAYLOAD_WIDTH );

    for ( size_t i = 0; i < RF24::Hardware::MAX_NUM_PIPES; i++ )
    {
      auto pipe = static_cast<PipeNumber_t>( i );
      result |= radio->openReadPipe( pipe, pipeAddress( nodeAddress, pipe ), true );
    }

    result |= radio->startListening();

    /*------------------------------------------------
    Make sure we passed the init sequence
    ------------------------------------------------*/
    if ( result != Chimera::CommonStatusCodes::OK )
    {
      initialized = false;
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_ERROR, "ERR: NET init failed\r\n" ); );
    }

    return initialized;
  }

  NetHdrMsgType Network::update()
  {
    if ( !initialized )
    {
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_DEBUG, "%d: NET Not initialized\r\n", Chimera::millis() ); );
      oopsies = ErrorType::NOT_INITIALIZED;
      return MSG_NETWORK_ERR;
    }

    uint8_t pipeNum         = 0u;
    NetHdrMsgType returnVal = MSG_TX_NORMAL;

    /*------------------------------------------------
    If BYPASS_HOLDS is enabled, incoming user data may be dropped. This allows for
    system payloads to be read while the user cache is full. HOLD_INCOMING prevents data from
    being read from the radios, thereby preventing incoming payloads from being acked.
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
                       uLog::flog( uLog::Level::LVL_INFO, "%d: MAC Received on pipe %u, Header: [%s]\n\r", Chimera::millis(),
                                   pipeNum, headerClass.toString() ); );

      /*------------------------------------------------
      Throw it away if it's not a valid address
      ------------------------------------------------*/
      if ( !isValidNetworkAddress( header->dstNode ) )
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
              uLog::flog( uLog::Level::LVL_INFO, "%d MAC: System payload rcvd %d\n", Chimera::millis(), returnVal ); );

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
        if ( header->dstNode == MULTICAST_ADDRESS )
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
            IF_SERIAL_DEBUG_ROUTING( uLog::flog( uLog::Level::LVL_INFO, "%u: MAC FWD multicast frame from 0%o to level %u\n",
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

  bool Network::available() const
  {
    if ( !initialized )
    {
      return false;
    }

    return !frameQueue.empty();
  }

  uint16_t Network::parent() const
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

  uint16_t Network::peek( HeaderHelper &header )
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

  void Network::peek( HeaderHelper &header, void *message, uint16_t maxlen )
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

  uint16_t Network::read( HeaderHelper &header, void *message, uint16_t maxlen )
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
                         uLog::flog( uLog::Level::LVL_INFO, "%d: NET message size %d\n", Chimera::millis(), msgLen );
                         uLog::flog( uLog::Level::LVL_INFO, "%d: NET r message ", Chimera::millis() );
                         const uint8_t *charPtr = reinterpret_cast<const uint8_t *>( message );
                         while ( len-- ) { uLog::flog( uLog::Level::LVL_INFO, "%02x ", charPtr[ len ] ); }

                         uLog::flog( uLog::Level::LVL_INFO, "\n\r" ); );

        /*------------------------------------------------
        Remove the packet that was just read out
        ------------------------------------------------*/
        frameQueue.pop_front();
        IF_SERIAL_DEBUG(
            uLog::flog( uLog::Level::LVL_INFO, "%d: NET Received %s\n\r", Chimera::millis(), header.toString() ); );
      }
    }

    return readLength;
  }

  bool Network::multicast( HeaderHelper &header, const void *const message, const uint16_t len, const uint8_t level )
  {
    header.data.dstNode = MULTICAST_ADDRESS;
    header.data.srcNode = this->logicalNodeAddress;

    return write( header, message, len, levelToAddress( level ) );
  }

  bool Network::write( HeaderHelper &header, const void *const message, const uint16_t len )
  {
    return write( header, message, len, ROUTED_ADDRESS );
  }

  bool Network::write( HeaderHelper &header, const void *const message, const uint16_t len, const NodeAddressType writeDirect )
  {
    /*------------------------------------------------
    Protect against invalid inputs
    ------------------------------------------------*/
    if ( !initialized )
    {
      oopsies = ErrorType::NOT_INITIALIZED;
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_INFO, "%d: ERR Not initialized\r\n", Chimera::millis() ); );
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

  bool Network::_write( HeaderHelper &header, const void *const message, const uint16_t len, const uint16_t directTo )
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
        uLog::flog( uLog::Level::LVL_INFO, "%d: NET Sending Header [%s]\n\r", Chimera::millis(), header.toString() );

        if ( len ) {
          uint16_t tmpLen        = len;
          const uint8_t *charPtr = reinterpret_cast<const uint8_t *>( message );

          uLog::flog( uLog::Level::LVL_INFO, "%d: NET message ", Chimera::millis() );
          while ( tmpLen-- )
          {
            uLog::flog( uLog::Level::LVL_INFO, "%02x ", charPtr[ tmpLen ] );
          }

          uLog::flog( uLog::Level::LVL_INFO, ( "\n\r" ) );
        } );

    /*------------------------------------------------
    Decide where that frame is going to be sent
    ------------------------------------------------*/
    if ( directTo == ROUTED_ADDRESS )
    {
      return writeDirect( header.data.dstNode, MSG_TX_NORMAL );
    }
    else
    {
      /*------------------------------------------------
      Payload is multicast to the first node, and routed normally to the next
      ------------------------------------------------*/
      NetHdrMsgType sendType = MSG_USER_TX_TO_LOGICAL_ADDRESS;

      if ( header.data.dstNode == MULTICAST_ADDRESS )
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

  bool Network::writeDirect( uint16_t toNode, NodeAddressType directTo )
  {
    // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route
    /*------------------------------------------------
    Throw it away if it's not a valid address
    ------------------------------------------------*/
    if ( !isValidNetworkAddress( toNode ) )
    {
      oopsies = ErrorType::INVALID_ADDRESS;
      return false;
    }

    /*------------------------------------------------
    Load info into our conversion structure, and get the converted address info
    ------------------------------------------------*/
    logicalToPhysicalStruct conversion = { toNode, static_cast<uint8_t>( directTo ), 0 };
    logicalToPhysicalAddress( &conversion );

    /*------------------------------------------------
    Write it
    ------------------------------------------------*/
    IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_INFO, "%d: MAC Sending to node [0%o] via node [0%o] on pipe %x\n\r",
                                 Chimera::millis(), toNode, conversion.send_node, conversion.send_pipe ); );


    memcpy( frameBuffer.data(), &txFrame.data, sizeof( FrameData ) );

    bool writeSucceeded = writeFrameBufferToPipeAtNodeID( conversion.send_node, conversion.send_pipe, conversion.multicast );
    bool isAckType =
        ( txFrame.data.header.msgType >= MSG_USER_MIN_ACK ) && ( txFrame.data.header.msgType <= MSG_NETWORK_MAX_ACK );

    if ( !writeSucceeded )
    {
      IF_SERIAL_DEBUG_ROUTING( uLog::flog( uLog::Level::LVL_INFO, "%d: MAC Send fail to 0%o via 0%o on pipe %x\n\r",
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
        conversion.send_pipe = static_cast<uint8_t>( MSG_TX_ROUTED );
        conversion.multicast = 0;
        logicalToPhysicalAddress( &conversion );

        // TODO: When you figure out what this section does, re-evaluate radioPayloadSize...likely to be wrong
        // TODO: BUG!
        radioPayloadSize = sizeof( FrameHeaderField );
        writeFrameBufferToPipeAtNodeID( conversion.send_node, conversion.send_pipe, conversion.multicast );

        IF_SERIAL_DEBUG_ROUTING( uLog::flog( uLog::Level::LVL_INFO, "%d MAC: Route OK to 0%o ACK sent to 0%o\n",
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
            IF_SERIAL_DEBUG_ROUTING( uLog::flog( uLog::Level::LVL_INFO,
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

  bool Network::logicalToPhysicalAddress( logicalToPhysicalStruct *conversionInfo )
  {
    // Create pointers so this makes sense.. kind of
    // We take in the toNode(logical) now, at the end of the function, output the send_node(physical) address, etc.
    // back to the original memory address that held the logical information.
    uint16_t *toNode  = &conversionInfo->send_node;
    uint8_t *directTo = &conversionInfo->send_pipe;
    bool *multicast   = &conversionInfo->multicast;

    // Where do we send this?  By default, to our parent
    uint16_t pre_conversion_send_node = parentNode;

    // On which pipe
    uint8_t pre_conversion_send_pipe = parentPipe;

    if ( *directTo > static_cast<uint8_t>( MSG_TX_ROUTED ) )
    {
      pre_conversion_send_node = *toNode;
      *multicast               = 1;
      pre_conversion_send_pipe = 0;
    }
    else if ( isDirectChild( *toNode ) )
    {
      // Send directly
      pre_conversion_send_node = *toNode;
      // To its listening pipe
      pre_conversion_send_pipe = 5;
    }
    // If the node is a child of a child
    // talk on our child's listening pipe,
    // and let the direct child relay it.
    else if ( isDescendant( *toNode ) )
    {
      pre_conversion_send_node = directChildRouteTo( *toNode );
      pre_conversion_send_pipe = 5;
    }

    *toNode   = pre_conversion_send_node;
    *directTo = pre_conversion_send_pipe;

    return 1;
  }

  bool Network::writeFrameBufferToPipeAtNodeID( const uint16_t node, const uint8_t pipe, const bool multicast )
  {
    Chimera::Status_t result  = Chimera::CommonStatusCodes::OK;
    uint64_t writePipeAddress = pipeAddress( node, pipe );

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

  bool Network::isDirectChild( uint16_t node )
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

  bool Network::isDescendant( uint16_t node )
  {
    return ( node & nodeMask ) == logicalNodeAddress;
  }

  void Network::setupAddress()
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
    parentPipe = i;

    IF_SERIAL_DEBUG_MINIMAL( uLog::flog( uLog::Level::LVL_INFO, "setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r",
                                         this->logicalNodeAddress, nodeMask, parentNode, parentPipe ); );
  }

  void Network::enqueue( FrameHelper &frame )
  {
    if ( !frameQueue.full() )
    {
      frameQueue.push_back( frame.data );
    }
    else
    {
      IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_INFO, "%d: NET **Drop Payload** Buffer Full\r\n", Chimera::millis() ); );
    }
  }

  uint16_t Network::addressOfPipe( uint16_t node, uint8_t pipeNo )
  {
    // Say this node is 013 (1011), mask is 077 or (00111111)
    // Say we want to use pipe 3 (11)
    // 6 bits in node mask, so shift pipeNo 6 times left and | into address
    uint16_t m = nodeMask >> 3;
    uint8_t i  = 0;

    while ( m )
    {             // While there are bits left in the node mask
      m >>= 1;    // Shift to the right
      i++;        // Count the # of increments
    }
    return node | ( pipeNo << i );
  }

  uint16_t Network::directChildRouteTo( uint16_t node )
  {
    // Presumes that this is in fact a child!!
    uint16_t child_mask = ( nodeMask << 3 ) | 0x07;
    return node & child_mask;
  }

  bool Network::isValidNetworkAddress( const uint16_t node )
  {
    bool result    = true;
    uint8_t nodeID = 0;
    uint16_t n     = node;

    /*------------------------------------------------
    If this isn't one of our special (invalid) node destinations, manually
    check each level of the octal address.
    ------------------------------------------------*/
    if ( !( node == MULTICAST_ADDRESS ) && !( node == ROUTED_ADDRESS ) )
    {
      while ( n )
      {
        /*------------------------------------------------
        Grab the node id of the LSB octal number
        ------------------------------------------------*/
        nodeID = n & OCTAL_MASK;

        /*------------------------------------------------
        Test for the proper node id range
        ------------------------------------------------*/
        if ( ( nodeID < MIN_NODE_ID ) || ( nodeID > MAX_NODE_ID ) )
        {
          result = false;
          IF_SERIAL_DEBUG( uLog::flog( uLog::Level::LVL_INFO, "*** WARNING *** Invalid address 0%o\n\r", n ); );
          break;
        }

        /*------------------------------------------------
        Push the next octal number to the LSB
        ------------------------------------------------*/
        n >>= OCTAL_TO_BIN_BITSHIFT;
      }
    }

    return result;
  }

  bool Network::setAddress( const uint16_t address )
  {
    if ( isValidNetworkAddress( address ) )
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
        initialized &= radio->openReadPipe( static_cast<RF24::Hardware::PipeNumber_t>( i ), pipeAddress( address, i ), true );
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

  uint16_t Network::getLogicalAddress()
  {
    return this->logicalNodeAddress;
  }

  void Network::setMulticastLevel( uint8_t level )
  {
    multicastLevel = level;
    radio->openReadPipe( static_cast<RF24::Hardware::PipeNumber_t>( 0u ), pipeAddress( levelToAddress( level ), 0 ), true );
  }

  uint16_t Network::levelToAddress( uint8_t level )
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

  uint64_t Network::pipeAddress( const uint16_t nodeID, const uint8_t pipeNum )
  {
    /*------------------------------------------------
    These bytes take on many uses. They can represent pipe numbers, device numbers,
    parent/child numbers, etc. They are really just a way to identify where a device
    is inside the network. As needed, a byte can be used to compose an address.
    ------------------------------------------------*/
    static uint8_t addressBytePool[] = { 0xec, 0xc3, 0x3c, 0x33, 0xce, 0x3e };

    /*------------------------------------------------
    The base, 5 byte address we start off with. 0xCC is reserved to indicate
    the master node.
    ------------------------------------------------*/
    uint64_t result  = 0xCCCCCCCCCC;
    uint8_t *address = reinterpret_cast<uint8_t *>( &result );

    /*------------------------------------------------
    If we want the master node (00), calculating the pipe address is easy. Simply
    replace the first bytes of the base address with the appropriate addressByte[].
    ------------------------------------------------*/
    if ( !nodeID )
    {
      address[ 0 ] = addressBytePool[ pipeNum ];
    }
    else
    {
      Node node;
      node.fromOctal( nodeID );

      /*------------------------------------------------
      Calculate what byte goes at each level of the address scheme. The first byte
      will always be associated with the pipe number. The remaining bytes are determined
      like so (MSB -> LSB):

      Level 0: (Direct child of master)
          [ 0xCC, 0xCC, 0xCC, childID, pipeNum ]

      Level 1: (Has a parent, ie 011: 1st child of the 1st level 0 node)
          [ 0xCC, 0xCC, childID, parentID, pipeNum ]

      Level 2: (Has a parent and grandparent)
          [ 0xCC, childID, parentID, grandParentID, pipeNum ]

      Level 3: (You get the idea)
          [ childID, parentID, grandParentID, greatGrandParentID, pipeNum ]

      Each level is limited to the range indicated by MIN_NODE_ID and MAX_NODE_ID.
      ------------------------------------------------*/
      address[ 0 ] = addressBytePool[ pipeNum ];

      for ( uint8_t i = 1; i < RF24::Hardware::MAX_ADDRESS_WIDTH; i++ )
      {
        if ( node.greatGrandParentID_isValid() )
        {
          address[ i ]            = addressBytePool[ node.greatGrandParentID ];
          node.greatGrandParentID = INVALID_NODE_ID;
          continue;
        }
        else if ( node.grandParentID_isValid() )
        {
          address[ i ]       = addressBytePool[ node.grandParentID ];
          node.grandParentID = INVALID_NODE_ID;
          continue;
        }
        else if ( node.parentID_isValid() )
        {
          address[ i ]  = addressBytePool[ node.parentID ];
          node.parentID = INVALID_NODE_ID;
          continue;
        }
        else if ( node.childID_isValid() )
        {
          address[ i ] = addressBytePool[ node.childID ];
          node.childID = INVALID_NODE_ID;
          continue;
        }
        else
        {
          break;
        }
      }
    }

    IF_SERIAL_DEBUG( uint32_t *top = reinterpret_cast<uint32_t *>( address + 1 ); printf(
                         "%d: NET Pipe %i on node 0%o has address 0x%05x\n\r", Chimera::millis(), pipeNum, nodeID, result ); );

    return result;
  }


}    // namespace RF24::Network
