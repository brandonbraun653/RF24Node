/********************************************************************************
*   nrf24l01Definitions.cpp
*       Implementation of register models for the NRF24L01 radio.
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* Driver Includes */
#include <RF24Phy/RF24Phy.hpp>
#include <RF24Phy/RF24PhyDef.hpp>

namespace RF24Phy
{
  namespace CONFIG
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::CONFIG));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::CONFIG));
    }
  }

  namespace EN_AA
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::EN_AA));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::EN_AA));
    }
  }

  namespace EN_RXADDR
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::EN_RXADDR));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::EN_RXADDR));
    }
  }

  namespace SETUP_AW
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::SETUP_AW));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::SETUP_AW));
    }
  }

  namespace SETUP_RETR
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::SETUP_RETR));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::SETUP_RETR));
    }
  }

  namespace RF_CH
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::STATUS));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::STATUS));
    }
  }

  namespace RF_SETUP
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RF_SETUP));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RF_SETUP));
    }
  }

  namespace STATUS
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::STATUS));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::STATUS));
    }
  }

  namespace OBSERVE_TX
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::OBSERVE_TX));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::OBSERVE_TX));
    }
  }

  namespace CD
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::CD));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::CD));
    }
  }

  namespace RX_ADDR_P0
  {
    void BitField::update(Phy *const radio)
    {
      uint64_t reg = 0u;
      radio->readRegister(Register::RX_ADDR_P0, reinterpret_cast<uint8_t *>(&reg), RX_ADDR_P0::byteWidth);
      this->operator=(reg);
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      uint64_t reg = 0u;
      radio->readRegister(Register::RX_ADDR_P0, reinterpret_cast<uint8_t *>(&reg), RX_ADDR_P0::byteWidth);
      this->operator=(reg);
    }
  }

  namespace RX_ADDR_P1
  {
    void BitField::update(Phy *const radio)
    {
      uint64_t reg = 0u;
      radio->readRegister(Register::RX_ADDR_P1, reinterpret_cast<uint8_t *>(&reg), RX_ADDR_P1::byteWidth);
      this->operator=(reg);
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      uint64_t reg = 0u;
      radio->readRegister(Register::RX_ADDR_P1, reinterpret_cast<uint8_t *>(&reg), RX_ADDR_P1::byteWidth);
      this->operator=(reg);
    }
  }

  namespace RX_ADDR_P2
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P2));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P2));
    }
  }

  namespace RX_ADDR_P3
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P3));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P3));
    }
  }

  namespace RX_ADDR_P4
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P4));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P4));
    }
  }

  namespace RX_ADDR_P5
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P5));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_ADDR_P5));
    }
  }

  namespace TX_ADDR
  {
    void BitField::update(Phy *const radio)
    {
      uint64_t reg = 0u;
      radio->readRegister(Register::TX_ADDR, reinterpret_cast<uint8_t *>(&reg), TX_ADDR::byteWidth);
      this->operator=(reg);
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      uint64_t reg = 0u;
      radio->readRegister(Register::TX_ADDR, reinterpret_cast<uint8_t *>(&reg), TX_ADDR::byteWidth);
      this->operator=(reg);
    }
  }

  namespace RX_PW_P0
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P0));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P0));
    }
  }

  namespace RX_PW_P1
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P1));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P1));
    }
  }

  namespace RX_PW_P2
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P2));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P2));
    }
  }

  namespace RX_PW_P3
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P3));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P3));
    }
  }

  namespace RX_PW_P4
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P4));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P4));
    }
  }

  namespace RX_PW_P5
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P5));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::RX_PW_P5));
    }
  }

  namespace FIFO_STATUS
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::FIFO_STATUS));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::FIFO_STATUS));
    }
  }

  namespace DYNPD
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::DYNPD));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::DYNPD));
    }
  }

  namespace FEATURE
  {
    void BitField::update(Phy *const radio)
    {
      this->operator=(radio->readRegister(Register::FEATURE));
    }

    void BitField::update(std::shared_ptr<Phy> &radio)
    {
      this->operator=(radio->readRegister(Register::FEATURE));
    }
  }
}
