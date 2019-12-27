/********************************************************************************
 *  File Name:
 *    utility.cpp
 *
 *  Description:
 *    Implementation of NRF24L01 networking stack utility functions
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* RF24 Includes */
#include <RF24Node/common/definitions.hpp>
#include <RF24Node/common/utility.hpp>
#include <RF24Node/network/definitions.hpp>

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
    Simplest checks first: Is the parent at a higher level?
    If the parent happens to be a root node (level 0) then we don't know
    if the child actually is a descendant as there could be multiple roots.
    (Higher level == lower number)
    ------------------------------------------------*/
    auto parentLevel = getLevel( parent );
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
    return ( parent == getParent( child ) );
  }

  LogicalAddress getParent( const LogicalAddress child )
  {
    auto level = getLevel( child );
    auto copy = child;

    if ( level == NODE_LEVEL_INVALID )
    {
      /*------------------------------------------------
      The child address is invalid, so is the parent
      ------------------------------------------------*/
      return Network::RSVD_ADDR_INVALID;
    }
    else if ( level == NODE_LEVEL_0 )
    {
      /*------------------------------------------------
      If an address is at the root level, it's the parent
      ------------------------------------------------*/
      return child;
    }
    else if ( level == NODE_LEVEL_1 )
    {
      /*------------------------------------------------
      If we are at level 1, we have no clue which is the correct parent 
      as there could be multiple root nodes in the network. The user
      will have to ask that child specifically who is their parent node.
      ------------------------------------------------*/
      return Network::RSVD_ADDR_LOOKUP;
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

  constexpr LogicalID getIdAtLevel( const LogicalAddress address, const LogicalLevel level )
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
    const auto levelMask = Network::BASE_LEVEL_MASK << shiftAmount;
    const auto actLevel = ( address & levelMask ) >> shiftAmount;

    /*------------------------------------------------
    Beyond level 0 we are in the realm of possible children nodes, which
    have a minimum level constraint on them.
    ------------------------------------------------*/
    if ( ( actLevel > NODE_LEVEL_MAX ) || ( actLevel < Network::MIN_CHILD_NODE_ID) )
    {
      return NODE_ID_INVALID;
    }

    return actLevel;
  }

}
