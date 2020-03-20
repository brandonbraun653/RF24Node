/********************************************************************************
 *  File Name:
 *    environment.hpp
 *
 *  Description:
 *    Provides compile time constants used to indicate the environment the software
 *    is executing inside of.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_ENVIRONMENT_HPP
#define RF24_NODE_ENVIRONMENT_HPP

/*-------------------------------------------------
Does the GTest framework exist?
-------------------------------------------------*/
#if __has_include( "gtest/gtest.hpp" ) && !defined( RF24_GTEST )
#pragma message( "RF24 build utilizes GTest" )
#define RF24_GTEST
#endif

/*------------------------------------------------
Does the GMock framework exist?
------------------------------------------------*/
#if !defined( RF24_GMOCK ) && __has_include( <gmock/gmock.hpp> )
#pragma message( "RF24 build utilizes GMock" )
#define RF24_GMOCK
static constexpr bool RF24_GMOCK_ENABLED = true;
#else 
static constexpr bool RF24_GMOCK_ENABLED = false;
#endif

#endif  /* !RF24_NODE_ENVIRONMENT_HPP */
