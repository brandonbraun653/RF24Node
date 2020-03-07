/********************************************************************************
 *  File Name:
 *    fwd.hpp
 *
 *  Description:
 *    Forward declarations for helping define class objects
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef RF24_NODE_ENDPOINT_FORWARD_DECLARE_HPP
#define RF24_NODE_ENDPOINT_FORWARD_DECLARE_HPP

namespace RF24::Endpoint
{
  class Device;

  namespace Internal
  {
    class RegistrationManager;
    class ConnectionManager;
  }    // namespace Internal
}

#endif /* !RF24_NODE_ENDPOINT_FORWARD_DECLARE_HPP */