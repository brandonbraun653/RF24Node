/********************************************************************************
 *   File Name:
 *    sim_definitions.hpp
 *
 *   Description:
 *    Definitions for the simulator
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef NRF24_SIMULATOR_DEFINITIONS_HPP
#define NRF24_SIMULATOR_DEFINITIONS_HPP


#if defined( EMBEDDED )
#define RF24API
#elif defined( RF24_DLL )
#define RF24API __declspec
#else

#endif

#endif /* NRF24_SIMULATOR_DEFINITIONS_HPP */
