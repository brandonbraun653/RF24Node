/********************************************************************************
 *  File Name:
 *    physical_mock.hpp
 *
 *  Description:
 *    Mocks the physical layer for use in test drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_PHYSICAL_MOCK_HPP
#define RF24_NODE_PHYSICAL_MOCK_HPP

/* RF24 Includes */
#include <RF24Node/common>
#include <RF24Node/src/interfaces/physical_intf.hpp>
#include <RF24Node/src/physical/simulator/sim_physical.hpp>

#if defined( RF24_GTEST ) && defined( RF24_GMOCK )

/* Testing Includes */
#include "gtest/gtest.hpp"
#include "gmock/gmock.hpp"

namespace RF24::Physical::Mock
{
  
  class Driver : virtual RF24::Physical::Interface
  {
  public:
    MOCK_METHOD( Chimera::Status_t, isInitialized, ( void ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, isConnected, ( void ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, setRetries,
                 ( const RF24::Hardware::AutoRetransmitDelay delay, const size_t count, const bool validate = false ),
                 ( override ) );
    MOCK_METHOD( Chimera::Status_t, setChannel, ( const size_t channel, const bool validate = false ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, setStaticPayloadSize, ( const size_t size ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, startListening, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, stopListening, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, pauseListening, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, resumeListening, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, openWritePipe, ( const PhysicalAddress address ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, closeWritePipe, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, openReadPipe,
                 ( const RF24::Hardware::PipeNumber pipe, const PhysicalAddress address, const bool validate = false ),
                 ( override ) );
    MOCK_METHOD( Chimera::Status_t, closeReadPipe, ( const RF24::Hardware::PipeNumber pipe ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, readPayload, ( RF24::Network::Frame::Buffer & buffer, const size_t length ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, immediateWrite, ( const RF24::Network::Frame::Buffer &buffer, const size_t length ),
                 ( override ) );
    MOCK_METHOD( Chimera::Status_t, txStandBy, ( const size_t timeout, const bool startTx = false ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, stageAckPayload,
                 ( const RF24::Hardware::PipeNumber pipe, const RF24::Network::Frame::Buffer &buffer, size_t length ),
                 ( override ) );
    MOCK_METHOD( Chimera::Status_t, toggleDynamicPayloads, ( const bool state ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, setPALevel, ( const RF24::Hardware::PowerAmplitude level, const bool validate = false ),
                 ( override ) );
    MOCK_METHOD( Chimera::Status_t, setDataRate, ( const RF24::Hardware::DataRate speed ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, toggleAutoAck, ( const bool state, const RF24::Hardware::PipeNumber pipe ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, attachLogger, ( uLog::SinkHandle sink ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, initialize, ( const RF24::Physical::Config &cfg ), ( override ) );
    MOCK_METHOD( size_t, getChannel, ( void ), ( override ) );
    MOCK_METHOD( size_t, getStaticPayloadSize, ( void ), ( override ) );
    MOCK_METHOD( size_t, getDynamicPayloadSize, ( void ), ( override ) );
    MOCK_METHOD( size_t, getPayloadSize, ( const RF24::Hardware::PipeNumber pipe ), ( override ) );
    MOCK_METHOD( Reg8_t, flushTX, ( void ), ( override ) );
    MOCK_METHOD( Reg8_t, flushRX, ( void ), ( override ) );
    MOCK_METHOD( RF24::Hardware::DataRate, getDataRate, ( void ), ( override ) );
    MOCK_METHOD( RF24::Hardware::PowerAmplitude, getPALevel, ( void ), ( override ) );
    MOCK_METHOD( RF24::Hardware::PipeNumber, payloadAvailable, ( void ), ( override ) );

  private:
    RF24::Physical::SimulatorDriver_uPtr simDriver;
  };
}

#endif /* RF24_GTEST && RF24_GMOCK */

#endif /* !RF24_NODE_PHYSICAL_MOCK_HPP */
