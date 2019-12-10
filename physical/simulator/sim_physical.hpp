/********************************************************************************
 *  File Name:
 *    sim_physical.hpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_SIMULATOR_HPP
#define RF24_NODE_PHYSICAL_SIMULATOR_HPP

/* C++ Includes */


/* RF24 Includes */
#include <RF24Node/interfaces/physical_intf.hpp>

namespace RF24::Physical
{
  class SimulatorDriver : public Interface
  {
  public:
    SimulatorDriver();
    ~SimulatorDriver();

    Chimera::Status_t initialize( const RF24::Physical::Config &cfg ) final override;
    Chimera::Status_t isInitialized() final override;
    Chimera::Status_t isConnected() final override;
    Chimera::Status_t setRetries( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count,
                                          const bool validate = false ) final override;
    Chimera::Status_t setChannel( const size_t channel, const bool validate = false ) final override;
    size_t getChannel() final override;
    Chimera::Status_t setStaticPayloadSize( const size_t size ) final override;
    size_t getStaticPayloadSize() final override;
    size_t getDynamicPayloadSize() final override;
    Chimera::Status_t startListening() final override;
    Chimera::Status_t stopListening() final override;
    Chimera::Status_t pauseListening() final override;
    Chimera::Status_t resumeListening() final override;
    Chimera::Status_t openWritePipe( const uint64_t address ) final override;
    Chimera::Status_t closeWritePipe() final override;
    Chimera::Status_t openReadPipe( const RF24::Hardware::PipeNumber_t pipe, const uint64_t address,
                                            const bool validate = false ) final override;
    Chimera::Status_t closeReadPipe( const RF24::Hardware::PipeNumber_t pipe ) final override;
    RF24::Hardware::PipeNumber_t payloadAvailable() final override;
    size_t getPayloadSize( const RF24::Hardware::PipeNumber_t pipe ) final override;
    Chimera::Status_t readPayload( void *const buffer, const size_t bufferLength, const size_t payloadLength ) final override;
    Chimera::Status_t immediateWrite( const void *const buffer, const size_t len,
                                              const bool multicast = false ) final override;
    Chimera::Status_t txStandBy( const size_t timeout, const bool startTx = false ) final override;
    Chimera::Status_t stageAckPayload( const uint8_t pipe, const uint8_t *const buffer, size_t len ) final override;
    Reg8_t flushTX() final override;
    Reg8_t flushRX() final override;
    Chimera::Status_t toggleDynamicPayloads( const bool state ) final override;
    Chimera::Status_t setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate = false ) final override;
    RF24::Hardware::PowerAmplitude getPALevel() final override;
    Chimera::Status_t setDataRate( const RF24::Hardware::DataRate speed ) final override;
    RF24::Hardware::DataRate getDataRate() final override;
    Chimera::Status_t toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber_t pipe ) final override;
  };
}    // namespace RF24::Physical

#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_HPP */