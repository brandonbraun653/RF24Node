/********************************************************************************
 *  File Name:
 *    utility.cpp
 *
 *  Description:
 *    Implementation of NRF24L01 networking stack utility functions
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* RF24 Includes */
#include <RF24Node/src/common/definitions.hpp>
#include <RF24Node/src/common/utility.hpp>
#include <RF24Node/src/network/definitions.hpp>

namespace RF24
{
  bool isAddressChild( const LogicalAddress address )
  {
    if ( isAddressReserved( address ) )
    {
      return false;
    }

    LogicalAddress addressCopy = address;
    do
    {
      /*------------------------------------------------
      Check that the reported level falls within the expected child node range
      ------------------------------------------------*/
      LogicalLevel level = ( addressCopy & Network::BASE_LEVEL_MASK );
      if ( ( level < Network::MIN_CHILD_NODE_ID ) || ( level > Network::MAX_CHILD_NODE_ID ) )
      {
        return false;
        break;
      }

      /*------------------------------------------------
      Shift in the next octal number for checking
      ------------------------------------------------*/
      addressCopy >>= Network::OCTAL_TO_BIN_BITSHIFT;

    } while ( addressCopy );

    return true;
  }

  bool isAddressRoot( const LogicalAddress address )
  {
    if ( isAddressReserved( address ) )
    {
      return false;
    }

    /*------------------------------------------------
    If the first indicated level isn't expected, bail out now
    ------------------------------------------------*/
    LogicalAddress addressCopy = address;
    if ( ( addressCopy & Network::BASE_LEVEL_MASK ) != Network::ROOT_NODE_START_LEVEL )
    {
      return false;
    }
    addressCopy >>= Network::OCTAL_TO_BIN_BITSHIFT;

    /*------------------------------------------------
    If the min/max are equal to the octal number range, return true
    because we know any additional numbers at this point are ok.
    ------------------------------------------------*/
    if constexpr ( ( Network::MIN_ROOT_NODE_ID == MIN_OCTAL_NUMBER ) && ( Network::MAX_ROOT_NODE_ID == MAX_OCTAL_NUMBER ) )
    {
      return true;
    }

    /*------------------------------------------------
    Check any remaining octal numbers
    ------------------------------------------------*/
    while ( addressCopy )
    {
      /*------------------------------------------------
      Check that the reported level falls within the expected root node range
      ------------------------------------------------*/
      LogicalLevel level = ( addressCopy & Network::BASE_LEVEL_MASK );
      if ( ( level < Network::MIN_ROOT_NODE_ID ) || ( level > Network::MAX_ROOT_NODE_ID ) )
      {
        return false;
        break;
      }

      /*------------------------------------------------
      Shift in the next octal number for checking
      ------------------------------------------------*/
      addressCopy >>= Network::OCTAL_TO_BIN_BITSHIFT;
    }

    return true;
  }

  bool isAddressValid( const LogicalAddress address )
  {
    return ( isAddressChild( address ) || isAddressRoot( address ) );
  }

  constexpr bool isAddressReserved( const LogicalAddress address )
  {
    return ( ( address == Network::RSVD_ADDR_INVALID ) || ( address == Network::RSVD_ADDR_LOOKUP ) ||
             ( address == Network::RSVD_ADDR_MULTICAST ) || ( address == Network::RSVD_ADDR_ROUTED ) );
  }

  bool isDescendent( const LogicalAddress parent, const LogicalAddress descendent )
  {
    /*------------------------------------------------
    Only valid addresses can be checked for descendants
    ------------------------------------------------*/
    if ( !isAddressValid( parent ) || !isAddressValid( descendent ) )
    {
      return false;
    }

    /*------------------------------------------------
    Every node is a descendent of a root node, as long
    as its also not a root node
    ------------------------------------------------*/
    if ( isAddressRoot( parent ) )
    {
      return !isAddressRoot( descendent );
    }

    /*------------------------------------------------
    Simplest checks first: Is the parent at a higher level?
    If the parent happens to be a root node (level 0) then we don't know
    if the child actually is a descendant as there could be multiple roots.
    (Higher level == lower number)
    ------------------------------------------------*/
    auto parentLevel     = getLevel( parent );
    auto descendentLevel = getLevel( descendent );
    if ( ( parentLevel >= descendentLevel ) || ( parentLevel == NODE_LEVEL_0 ) )
    {
      return false;
    }

    /*------------------------------------------------
    More complex check: Is the descendent in the parent node tree?
    ------------------------------------------------*/
    auto currentLevel = parentLevel;

    do
    {
      /*------------------------------------------------
      Only true if the addresses match perfectly up until the level of the descendent.
      ------------------------------------------------*/
      if ( !( getIdAtLevel( parent, currentLevel ) == getIdAtLevel( descendent, currentLevel ) ) &&
           ( currentLevel == descendentLevel ) )
      {
        return true;
        break;
      }
      else
      {
        currentLevel++;
      }
    } while ( currentLevel <= NODE_LEVEL_MAX );

    return false;
  }

  bool isDirectDescendent( const LogicalAddress parent, const LogicalAddress child )
  {
    /*------------------------------------------------
    Only valid addresses can be checked for descendants
    ------------------------------------------------*/
    if ( !isAddressValid( parent ) || !isAddressValid( child ) )
    {
      return false;
    }

    return ( parent == getParent( child ) );
  }

  LogicalAddress getParent( const LogicalAddress child )
  {
    auto level = getLevel( child );

    if ( ( level == NODE_LEVEL_INVALID ) || ( level == NODE_LEVEL_0 ) )
    {
      /*------------------------------------------------
      A root node has no parent. Neither do invalid levels.
      ------------------------------------------------*/
      return Network::RSVD_ADDR_INVALID;
    }
    else if ( level == NODE_LEVEL_1 )
    {
      /*------------------------------------------------
      Eventually multiple roots will be supported, but for now,
      all level 1 nodes have root node 0 as their parent.
      ------------------------------------------------*/
      return RF24::RootNode0;
    }
    else
    {
      /*------------------------------------------------
      Create the parent address mask, which only erases
      the current level the child is on.
      ------------------------------------------------*/
      LogicalAddress mask = 0;
      --level; /* Subtract 1 so that we don't include the current level in the mask */

      while ( level )
      {
        /* Subtract 1 from level so we get back to a zero indexed scheme */
        mask |= ( Network::BASE_LEVEL_MASK << ( Network::BITS_PER_LEVEL * ( level - 1u ) ) );
        --level;
      }

      return mask & child;
    }
  }

  LogicalAddress getChild( const LogicalAddress parent, const Connection::BindSite which )
  {
    LogicalLevel level    = getLevel( parent );
    LogicalAddress retVal = RF24::Network::RSVD_ADDR_INVALID;

    /*------------------------------------------------
    If the device falls into the proper level move forward
    ------------------------------------------------*/
    if ( ( level >= NODE_LEVEL_0 ) && ( level <= NODE_LEVEL_MAX ) )
    {
      size_t bitShift = RF24::Network::BITS_PER_LEVEL * static_cast<size_t>( level );
      size_t childNum = std::numeric_limits<size_t>::max();

      switch ( which )
      {
        case Connection::BindSite::CHILD_1:
          childNum = 1u;
          break;

        case Connection::BindSite::CHILD_2:
          childNum = 2u;
          break;

        case Connection::BindSite::CHILD_3:
          childNum = 3u;
          break;

        case Connection::BindSite::CHILD_4:
          childNum = 4u;
          break;

        case Connection::BindSite::CHILD_5:
          childNum = 5u;
          break;

        default:
          // Nothing as the default value is sufficient
          break;
      };

      /*------------------------------------------------
      Build up the child address
      ------------------------------------------------*/
      if ( childNum != std::numeric_limits<size_t>::max() )
      {
        retVal = ( parent + ( childNum << bitShift ) ) & RF24::Network::FULL_LEVEL_MASK;
      }
    }

    return retVal;
  }

  LogicalLevel getLevel( LogicalAddress address )
  {
    /*------------------------------------------------
    Handle an invalid address
    ------------------------------------------------*/
    if ( !isAddressValid( address ) )
    {
      return NODE_LEVEL_INVALID;
    }

    /*------------------------------------------------
    Root nodes will be zero, other nodes will have some level
    ------------------------------------------------*/
    LogicalLevel level = 0;
    while ( address & Network::BASE_LEVEL_MASK )
    {
      level++;
      address >>= Network::BITS_PER_LEVEL;
    }

    return level;
  }

  LogicalID getIdAtLevel( const LogicalAddress address, const LogicalLevel level )
  {
    /*------------------------------------------------
    Root nodes don't have an ID at any level. They are (g)root.
    ------------------------------------------------*/
    if ( level == NODE_LEVEL_0 )
    {
      if ( ( address & Network::ADDR_LEVEL1_Msk ) == NODE_ID_ROOT )
      {
        return NODE_ID_ROOT;
      }
      else
      {
        return NODE_ID_INVALID;
      }
    }

    /*------------------------------------------------
    Otherwise, the ID is just a simple bit-mask
    ------------------------------------------------*/
    const auto shiftAmount = Network::BITS_PER_LEVEL * ( level - 1u );
    const auto levelMask   = Network::BASE_LEVEL_MASK << shiftAmount;
    const auto idAtLevel   = ( address & levelMask ) >> shiftAmount;

    /*------------------------------------------------
    Beyond level 0 we are in the realm of possible children nodes, which
    have a minimum level constraint on them.
    ------------------------------------------------*/
    if ( ( idAtLevel > NODE_LEVEL_MAX ) || ( idAtLevel < Network::MIN_CHILD_NODE_ID ) )
    {
      return NODE_ID_INVALID;
    }

    return idAtLevel;
  }


  LogicalAddress getAddressAtLevel( const LogicalAddress address, const LogicalLevel level )
  {
    using namespace RF24::Network;

    /*------------------------------------------------
    Handle edge cases
    ------------------------------------------------*/
    if ( !isAddressValid( address ) )
    {
      return RSVD_ADDR_INVALID;
    }
    else if ( level == NODE_LEVEL_0 )
    {
      if ( isAddressRoot( address ) )
      {
        return address;
      }
      else
      {
        return RSVD_ADDR_INVALID;
      }
    }

    /*------------------------------------------------
    Build the address mask for the appropriate level
    ------------------------------------------------*/
    LogicalAddress mask = 0;

    for ( size_t idxLvl = 1; idxLvl <= level; idxLvl++ )
    {
      const auto shiftAmount = Network::BITS_PER_LEVEL * ( idxLvl - 1u );
      const auto levelMask   = Network::BASE_LEVEL_MASK << shiftAmount;

      mask |= levelMask;
    }

    /*------------------------------------------------
    Apply the mask and if the resulting address has the
    same level as the requested level, it is valid.

    Don't need to check for valid resulting addresses
    because invalid base addresses are rejected at the
    start of the function.
    ------------------------------------------------*/
    LogicalAddress resultAddress = address & mask;
    if ( getLevel( resultAddress ) == level )
    {
      return resultAddress;
    }
    else
    {
      return RSVD_ADDR_INVALID;
    }

  }
}    // namespace RF24
