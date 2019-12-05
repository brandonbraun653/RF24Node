/********************************************************************************
*  File Name:
*    sim_header.hpp
*
*  Description:
*    
*
*  2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef RF24_SIMULATOR_HEADER_HPP
#define RF24_SIMULATOR_HEADER_HPP

#include <RF24Node/network/header/header.hpp>
#include <RF24Node/network/header/header_types.hpp>
#include <RF24Node/simulator/sim_definitions.hpp>

#if defined( RF24_SIMULATOR )

extern "C" RF24API RF24::Network::HeaderHelper *new__HeaderHelper();
extern "C" RF24API RF24::Network::HeaderHelper *new__HeaderHelperFromFrameBuffer( const uint8_t *frameBuffer );
extern "C" RF24API RF24::Network::HeaderHelper *new__HeaderHelperFromSpec( const uint16_t dstNode, const uint8_t type );

extern "C" RF24API void delete__HeaderHelper( RF24::Network::HeaderHelper *const obj );

extern "C" RF24API void copyFromFrameBuffer( RF24::Network::HeaderHelper *const obj, const uint8_t *frameBuffer );
extern "C" RF24API void copyInstance( RF24::Network::HeaderHelper *const obj, const RF24::Network::HeaderHelper *const classInstance );
extern "C" RF24API void copyFrameHeader( RF24::Network::HeaderHelper *const obj, const RF24::Network::FrameHeaderField *const frameHeader );


#endif

#endif /* !RF24_SIMULATOR_HEADER_HPP */
