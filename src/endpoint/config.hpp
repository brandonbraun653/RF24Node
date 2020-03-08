/********************************************************************************
*  File Name:
*    config.hpp
*
*  Description:
*    Configuration settings for the Endpoint Device driver
*
*  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#ifndef RF24_NODE_ENDPOINT_CONFIG_HPP
#define RF24_NODE_ENDPOINT_CONFIG_HPP

namespace RF24::Endpoint
{
  
  static constexpr size_t Default_LinkTimeout = 1000 * 60; /**< Time in which the network connection is considered expired */
  static constexpr size_t Minimum_LinkTimeout = 1000;      /**< Minimum connection expiration rate */
  
}

#endif /* !RF24_NODE_ENDPOINT_CONFIG_HPP */
