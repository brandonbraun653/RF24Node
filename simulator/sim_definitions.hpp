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


#if defined( _EMBEDDED )
#define RF24API
#elif defined( RF24DLL )
#define RF24API __declspec(dllexport)
#else
#define RF24API __declspec(dllimport)
#endif 

#endif /* NRF24_SIMULATOR_DEFINITIONS_HPP */
