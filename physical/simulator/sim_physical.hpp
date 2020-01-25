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

#if defined( RF24_SIMULATOR )

/* C++ Includes */
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>

/* Boost Includes */
#include <boost/asio.hpp>

/* uLog Includes */
#include <uLog/types.hpp>

/* RF24 Includes */
#include <RF24Node/interfaces/physical_intf.hpp>
#include <RF24Node/physical/simulator/shockburst.hpp>

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
    size_t getPayloadSize( const RF24::Hardware::PipeNumber pipe ) final override;
    Chimera::Status_t startListening() final override;
    Chimera::Status_t stopListening() final override;
    Chimera::Status_t pauseListening() final override;
    Chimera::Status_t resumeListening() final override;
    Chimera::Status_t openWritePipe( const PhysicalAddress address ) final override;
    Chimera::Status_t closeWritePipe() final override;
    Chimera::Status_t openReadPipe( const RF24::Hardware::PipeNumber pipe, const PhysicalAddress address,
                                    const bool validate = false ) final override;
    Chimera::Status_t closeReadPipe( const RF24::Hardware::PipeNumber pipe ) final override;
    RF24::Hardware::PipeNumber payloadAvailable() final override;
    Chimera::Status_t readPayload( RF24::Network::Frame::Buffer &buffer, const size_t length ) final override;
    Chimera::Status_t immediateWrite( const RF24::Network::Frame::Buffer &buffer, const size_t length ) final override;
    Chimera::Status_t txStandBy( const size_t timeout, const bool startTx = false ) final override;
    Chimera::Status_t stageAckPayload( const RF24::Hardware::PipeNumber pipe, const RF24::Network::Frame::Buffer &buffer,
                                       size_t length ) final override;
    Reg8_t flushTX() final override;
    Reg8_t flushRX() final override;
    Chimera::Status_t toggleDynamicPayloads( const bool state ) final override;
    Chimera::Status_t setPALevel( const RF24::Hardware::PowerAmplitude level, const bool validate = false ) final override;
    RF24::Hardware::PowerAmplitude getPALevel() final override;
    Chimera::Status_t setDataRate( const RF24::Hardware::DataRate speed ) final override;
    RF24::Hardware::DataRate getDataRate() final override;
    Chimera::Status_t toggleAutoAck( const bool state, const RF24::Hardware::PipeNumber pipe ) final override;

  private:
    bool flagInitialized;

    std::string IPAddress;
    size_t mChannel;
    size_t mStaticPayloadSize;
    RF24::Hardware::PowerAmplitude mPALevel;
    RF24::Hardware::DataRate mDataRate;

    boost::asio::io_service ioService;
    std::array<RF24::Physical::Shockburst::DataPipe_uPtr, 6> mDataPipes;

    uLog::SinkHandle logger;

    struct FIFOElement
    {
      RF24::Hardware::PipeNumber pipe;
      RF24::Network::Frame::Buffer payload;
      size_t size;
    };

    std::queue<FIFOElement> TxFIFO;
    std::queue<FIFOElement> RxFIFO;
    std::thread FIFOProcessingThread;
    std::recursive_mutex FIFOLock;
    std::atomic<bool> killFlag;

    /**
     *	Implements a hardware feature in the NRF24L01 that manages the 
     *	TX/RX FIFOs for data flowing through the pipes.
     *	
     *	@note   This is a thread
     *	
     *	@return void
     */
     void QueueHandler();
  };
}    // namespace RF24::Physical

#endif /* RF24_SIMULATOR */
#endif /* !RF24_NODE_PHYSICAL_SIMULATOR_HPP */
