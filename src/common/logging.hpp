/********************************************************************************
 *  File Name:
 *    logging.hpp
 *
 *  Description:
 *    Common logging helpers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef RF24_NODE_COMMON_LOGGING_HPP
#define RF24_NODE_COMMON_LOGGING_HPP

/* RF24 Includes */
#include <RF24Node/src/common/definitions.hpp>

/* uLog Includes */
#include <uLog/ulog.hpp>
#include <uLog/sinks/sink_vgdb_semihosting.hpp>
#include <uLog/sinks/sink_cout.hpp>

/*-------------------------------------------------------------------------------
Logging Helper Macros
The user must specify the message string formatter and any supporting arguments
in the variadic section of the macro. For example:

DBG_MSG_NET( logger, ... ) --> DBG_MSG_NET( myLogger, "%d: some string", 10 )
-------------------------------------------------------------------------------*/
#define DBG_MSG_NET( logger, ... )                       \
  if constexpr ( DBG_LOG_NET )                           \
  {                                                      \
    logger->flog( uLog::Level::LVL_DEBUG, __VA_ARGS__ ); \
  }

#define DBG_MSG_NET_CONNECT( logger, ... )               \
  if constexpr ( DBG_LOG_NET_PROC_CONNECT )              \
  {                                                      \
    logger->flog( uLog::Level::LVL_DEBUG, __VA_ARGS__ ); \
  }


#endif  /* !RF24_NODE_COMMON_LOGGING_HPP */
