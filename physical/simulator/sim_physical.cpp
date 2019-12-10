/********************************************************************************
 *  File Name:
 *    sim_physical.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* RF24 Includes */
#include <RF24Node/physical/simulator/sim_physical.hpp>

namespace RF24::Physical
{
  SimulatorDriver::SimulatorDriver()
  {
  }

  SimulatorDriver::~SimulatorDriver()
  {
  }

  Chimera::Status_t SimulatorDriver::initialize( const RF24::Physical::Config &cfg )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::isInitialized()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::isConnected()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count,
                                                 const bool validate )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::setChannel( const size_t channel, const bool validate )
  {
    return Chimera::Status_t();
  }

  size_t SimulatorDriver::getChannel()
  {
    return size_t();
  }

  Chimera::Status_t SimulatorDriver::setStaticPayloadSize( const size_t size )
  {
    return Chimera::Status_t();
  }

  size_t SimulatorDriver::getStaticPayloadSize()
  {
    return size_t();
  }

  size_t SimulatorDriver::getDynamicPayloadSize()
  {

    return size_t();
  }

  Chimera::Status_t SimulatorDriver::startListening()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::stopListening()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::pauseListening()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::resumeListening()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::openWritePipe( const uint64_t address )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::closeWritePipe()
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::openReadPipe( const RF24::Hardware::PipeNumber_t pipe, const uint64_t address,
                                                   const bool validate )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::closeReadPipe( const RF24::Hardware::PipeNumber_t pipe )
  {
    return Chimera::Status_t();
  }

  RF24::Hardware::PipeNumber_t SimulatorDriver::payloadAvailable()
  {
    return RF24::Hardware::PipeNumber_t();
  }

  size_t SimulatorDriver::getPayloadSize( const RF24::Hardware::PipeNumber_t pipe )
  {
    return size_t();
  }

  Chimera::Status_t SimulatorDriver::readPayload( void *const buffer, const size_t bufferLength, const size_t payloadLength )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::immediateWrite( const void *const buffer, const size_t len, const bool multicast )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::txStandBy( const size_t timeout, const bool startTx )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::stageAckPayload( const uint8_t pipe, const uint8_t *const buffer, size_t len )
  {
    return Chimera::Status_t();
  }

  Reg8_t SimulatorDriver::flushTX()
  {
    return Reg8_t();
  }

  Reg8_t SimulatorDriver::flushRX()
  {
    return Reg8_t();
  }

  Chimera::Status_t SimulatorDriver::toggleDynamicPayloads( const bool state )
  {
    return Chimera::Status_t();
  }

  Chimera::Status_t SimulatorDriver::setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate )
  {
    return Chimera::Status_t();
  }

  RF24::Hardware::PowerAmplitude SimulatorDriver::getPALevel()
  {
    return RF24::Hardware::PowerAmplitude();
  }

  Chimera::Status_t SimulatorDriver::setDataRate( const RF24::Hardware::DataRate speed )
  {
    return Chimera::Status_t();
  }

  RF24::Hardware::DataRate SimulatorDriver::getDataRate()
  {
    return RF24::Hardware::DataRate();
  }

  Chimera::Status_t SimulatorDriver::toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber_t pipe )
  {
    return Chimera::Status_t();
  }
}    // namespace RF24::Physical