/********************************************************************************
*   nrf24l01Definitions.hpp
*       Provides definitions and models of the NRF24L01 radio.
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef NRF24L01_DEFINITIONS_HPP
#define NRF24L01_DEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdio>
#include <memory>

namespace RF24Phy
{
    /*------------------------------------------------
    Configuration Constants
    ------------------------------------------------*/
    #define TRACK_REGISTER_STATES /* Keep track of the register states as they are set/read (Debugging Feature, lots of overhead) */

    /*----------------------------------------------
    General Definitions
    ----------------------------------------------*/
    class Phy; /* Forward declare the class so register Bitfields can auto-update */

    constexpr uint8_t MAX_NUM_PIPES = 6;                                 /**< Hardware limit for number of pipes we can use */
    constexpr size_t MAX_ADDRESS_WIDTH = 5;                              /**< Hardware limit for how many bytes can represent a device's address */
    constexpr size_t MAX_PAYLOAD_WIDTH = 32;                             /**< Hardware limit for RF payload */
    constexpr size_t COMMAND_WIDTH = 1;                                  /**< Number of bytes for an SPI command */
    constexpr size_t SPI_BUFFER_LEN = COMMAND_WIDTH + MAX_PAYLOAD_WIDTH; /**< Accounts for max payload of 32 bytes + 1 byte for the command */
    constexpr uint32_t MIN_TIMEOUT_MS = 1;                               /**< The absolute lowest resolution timeout we want to achieve */
    constexpr uint32_t DFLT_TIMEOUT_MS = 100;                            /**< Default timeout for general operations */

    /**
    *   Definitions for tracking the hardware state machine mode
    */
    enum class Mode : uint8_t
    {
        POWER_DOWN = 0,
        STANDBY_I,
        STANDBY_II,
        RX,
        TX,

        MAX_MODES,
        UNKNOWN_MODE
    };

    /**
    *   Definitions for allowed TX power levels
    */
    enum class PowerAmplitude : uint8_t
    {
        MIN = 0u,  /**< -18 dBm */
        LOW = 2u,  /**< -12 dBm */
        HIGH = 4u, /**<  -6 dBm */
        MAX = 6u   /**<   0 dBm */
    };

    /**
    *   Definitions for allowed data rates
    */
    enum class DataRate : uint8_t
    {
        DR_1MBPS,  /**< 1 MBPS */
        DR_2MBPS,  /**< 2 MBPS */
        DR_250KBPS /**< 250 KBPS */
    };

    /**
    *   Definitions for CRC settings
    */
    enum class CRCLength : uint8_t
    {
        CRC_DISABLED, /**< No CRC */
        CRC_8,        /**< 8 Bit CRC */
        CRC_16        /**< 16 Bit CRC */
    };

    /**
    *   Definitions for how many address bytes to use. The
    *   numerical value here is NOT the number of bytes. This is the
    *   register level definition.
    */
    enum class AddressWidth : uint8_t
    {
        AW_3Byte = 0x01,
        AW_4Byte = 0x02,
        AW_5Byte = 0x03
    };

    /**
    *   Definitions for the auto retransmit delay register field
    */
    enum class AutoRetransmitDelay : uint8_t
    {
        w250uS = 0,
        w500uS = 1,
        w750uS = 2,
        w1000uS = 3,
        w1250uS = 4,
        w1500uS = 5,
        w1750uS = 6,
        w2000uS = 7,
        w2250uS = 8,
        w2500uS = 9,
        w2750uS = 10,
        w3000uS = 11,
        w3250uS = 12,
        w3500uS = 13,
        w3750uS = 14,
        w4000uS = 15,

        MIN = w250uS,
        MED = w2250uS,
        MAX = w4000uS
    };

    /**
    *   Provide reasons for why something has failed.
    */
    enum class FailureCode : uint8_t
    {
        NO_FAILURE = 0,
        CLEARED = NO_FAILURE,
        MAX_RETRY_TIMEOUT,
        TX_FIFO_FULL_TIMEOUT,
        TX_FIFO_EMPTY_TIMEOUT,
        RADIO_IN_TX_MODE,
        RADIO_IN_RX_MODE,
        INVALID_PIPE,
        NOT_CONNECTED,
        REGISTER_WRITE_FAILURE,
        COULD_NOT_ERASE,
    };

    /*----------------------------------------------
    Command Instructions
    ----------------------------------------------*/
    namespace Command
    {
        constexpr uint8_t REGISTER_MASK = 0x1F;       /**<*< Masks off the largest available register address */
        constexpr uint8_t R_REGISTER = 0x00;          /**<*< Read command and status registers  */
        constexpr uint8_t W_REGISTER = 0x20;          /**<*< Write command and status registers  */
        constexpr uint8_t R_RX_PAYLOAD = 0x61;        /**<*< Read RX Payload (1-32 bytes) */
        constexpr uint8_t W_TX_PAYLOAD = 0xA0;        /**<*< Write TX Payload (1-32 bytes) */
        constexpr uint8_t FLUSH_TX = 0xE1;            /**<*< Flush TX FIFO, used in TX Mode */
        constexpr uint8_t FLUSH_RX = 0xE2;            /**<*< Flush RX FIFO, used in RX Mode */
        constexpr uint8_t REUSE_TX_PL = 0xE3;         /**<*< Reuse last transmitted payload (PTX device only) */
        constexpr uint8_t ACTIVATE = 0x50;            /**<*< This command, followed by 0x73, activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK */
        constexpr uint8_t R_RX_PL_WID = 0x60;         /**<*< Read RX payload width for the top payload in the RX FIFO */
        constexpr uint8_t W_ACK_PAYLOAD = 0xA8;       /**<*< Write Payload together with ACK packet */
        constexpr uint8_t W_TX_PAYLOAD_NO_ACK = 0xB0; /**<*< Disables AUTOACK on this specific packet */
        constexpr uint8_t NOP = 0xFF;                 /**<*< No operation */
    }

    /*----------------------------------------------
    Register Addresses
    ----------------------------------------------*/
    namespace Register
    {
        constexpr uint8_t CONFIG = 0x00;      /**< Configuration Register */
        constexpr uint8_t EN_AA = 0x01;       /**< Enable Auto Acknowledgment */
        constexpr uint8_t EN_RXADDR = 0x02;   /**< Enable RX Addresses */
        constexpr uint8_t SETUP_AW = 0x03;    /**< Setup of Address Width */
        constexpr uint8_t SETUP_RETR = 0x04;  /**< Setup of Automatic Retransmission */
        constexpr uint8_t RF_CH = 0x05;       /**< RF Channel Frequency Settings */
        constexpr uint8_t RF_SETUP = 0x06;    /**< RF Channel Settings Register */
        constexpr uint8_t STATUS = 0x07;      /**< Status Register */
        constexpr uint8_t OBSERVE_TX = 0x08;  /**< Transmit Observe */
        constexpr uint8_t CD = 0x09;          /**< Carrier Detect */
        constexpr uint8_t RX_ADDR_P0 = 0x0A;  /**< Receive Address Data Pipe 0 */
        constexpr uint8_t RX_ADDR_P1 = 0x0B;  /**< Receive Address Data Pipe 1 */
        constexpr uint8_t RX_ADDR_P2 = 0x0C;  /**< Receive Address Data Pipe 2 */
        constexpr uint8_t RX_ADDR_P3 = 0x0D;  /**< Receive Address Data Pipe 3 */
        constexpr uint8_t RX_ADDR_P4 = 0x0E;  /**< Receive Address Data Pipe 4 */
        constexpr uint8_t RX_ADDR_P5 = 0x0F;  /**< Receive Address Data Pipe 5 */
        constexpr uint8_t TX_ADDR = 0x10;     /**< Transmit Address */
        constexpr uint8_t RX_PW_P0 = 0x11;    /**< Number of bytes in RX Payload Data Pipe 0 */
        constexpr uint8_t RX_PW_P1 = 0x12;    /**< Number of bytes in RX Payload Data Pipe 1 */
        constexpr uint8_t RX_PW_P2 = 0x13;    /**< Number of bytes in RX Payload Data Pipe 2 */
        constexpr uint8_t RX_PW_P3 = 0x14;    /**< Number of bytes in RX Payload Data Pipe 3 */
        constexpr uint8_t RX_PW_P4 = 0x15;    /**< Number of bytes in RX Payload Data Pipe 4 */
        constexpr uint8_t RX_PW_P5 = 0x16;    /**< Number of bytes in RX Payload Data Pipe 5 */
        constexpr uint8_t FIFO_STATUS = 0x17; /**< FIFO Status Register */
        constexpr uint8_t DYNPD = 0x1C;       /**< Enable Dynamic Payload Length for Data Pipes */
        constexpr uint8_t FEATURE = 0x1D;     /**< Feature Register */
    }

    /*----------------------------------------------
    Register Bit Fields and Masks
    ----------------------------------------------*/
    namespace CONFIG
    {
        constexpr uint8_t Mask = 0x7F;
        constexpr uint8_t Reset = 0x08;

        constexpr uint8_t MASK_RX_DR_Pos = 6u;
        constexpr uint8_t MASK_RX_DR_Msk = 1u << MASK_RX_DR_Pos;
        constexpr uint8_t MASK_RX_DR = MASK_RX_DR_Msk;

        constexpr uint8_t MASK_TX_DS_Pos = 5u;
        constexpr uint8_t MASK_TX_DS_Msk = 1u << MASK_TX_DS_Pos;
        constexpr uint8_t MASK_TX_DS = MASK_TX_DS_Msk;

        constexpr uint8_t MASK_MAX_RT_Pos = 4u;
        constexpr uint8_t MASK_MAX_RT_Msk = 1u << MASK_MAX_RT_Pos;
        constexpr uint8_t MASK_MAX_RT = MASK_MAX_RT_Msk;

        constexpr uint8_t EN_CRC_Pos = 3u;
        constexpr uint8_t EN_CRC_Msk = 1u << EN_CRC_Pos;
        constexpr uint8_t EN_CRC = EN_CRC_Msk;

        constexpr uint8_t CRCO_Pos = 2u;
        constexpr uint8_t CRCO_Msk = 1u << CRCO_Pos;
        constexpr uint8_t CRCO = CRCO_Msk;

        constexpr uint8_t PWR_UP_Pos = 1u;
        constexpr uint8_t PWR_UP_Msk = 1u << PWR_UP_Pos;
        constexpr uint8_t PWR_UP = PWR_UP_Msk;

        constexpr uint8_t PRIM_RX_Pos = 0u;
        constexpr uint8_t PRIM_RX_Msk = 1u << PRIM_RX_Pos;
        constexpr uint8_t PRIM_RX = PRIM_RX_Msk;

        class BitField
        {
        public:
            bool bMASK_RX_DR = false;
            bool bMASK_TX_DS = false;
            bool bMASK_MAX_RT = false;
            bool bEN_CRC = false;
            bool bCRCO = false;
            bool bPWR_UP = false;
            bool bPRIM_RX = false;

            void operator=(const uint8_t reg)
            {
                bMASK_RX_DR = reg & MASK_RX_DR;
                bMASK_TX_DS = reg & MASK_TX_DS;
                bMASK_MAX_RT = reg & MASK_MAX_RT;
                bEN_CRC = reg & EN_CRC;
                bCRCO = reg & CRCO;
                bPWR_UP = reg & PWR_UP;
                bPRIM_RX = reg & PRIM_RX;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace EN_AA
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x3F;

        constexpr uint8_t P5_Pos = 5u;
        constexpr uint8_t P5_Msk = 1u << P5_Pos;
        constexpr uint8_t P5 = P5_Msk;

        constexpr uint8_t P4_Pos = 4u;
        constexpr uint8_t P4_Msk = 1u << P4_Pos;
        constexpr uint8_t P4 = P4_Msk;

        constexpr uint8_t P3_Pos = 3u;
        constexpr uint8_t P3_Msk = 1u << P3_Pos;
        constexpr uint8_t P3 = P3_Msk;

        constexpr uint8_t P2_Pos = 2u;
        constexpr uint8_t P2_Msk = 1u << P2_Pos;
        constexpr uint8_t P2 = P2_Msk;

        constexpr uint8_t P1_Pos = 1u;
        constexpr uint8_t P1_Msk = 1u << P1_Pos;
        constexpr uint8_t P1 = P1_Msk;

        constexpr uint8_t P0_Pos = 0u;
        constexpr uint8_t P0_Msk = 1u << P0_Pos;
        constexpr uint8_t P0 = P0_Msk;

        class BitField
        {
        public:
            bool bP5 = false;
            bool bP4 = false;
            bool bP3 = false;
            bool bP2 = false;
            bool bP1 = false;
            bool bP0 = false;

            void operator=(const uint8_t reg)
            {
                bP5 = reg & P5;
                bP4 = reg & P4;
                bP3 = reg & P3;
                bP2 = reg & P2;
                bP1 = reg & P1;
                bP0 = reg & P0;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace EN_RXADDR
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x03;

        constexpr uint8_t P5_Pos = 5u;
        constexpr uint8_t P5_Msk = 1u << P5_Pos;
        constexpr uint8_t P5 = P5_Msk;

        constexpr uint8_t P4_Pos = 4u;
        constexpr uint8_t P4_Msk = 1u << P4_Pos;
        constexpr uint8_t P4 = P4_Msk;

        constexpr uint8_t P3_Pos = 3u;
        constexpr uint8_t P3_Msk = 1u << P3_Pos;
        constexpr uint8_t P3 = P3_Msk;

        constexpr uint8_t P2_Pos = 2u;
        constexpr uint8_t P2_Msk = 1u << P2_Pos;
        constexpr uint8_t P2 = P2_Msk;

        constexpr uint8_t P1_Pos = 1u;
        constexpr uint8_t P1_Msk = 1u << P1_Pos;
        constexpr uint8_t P1 = P1_Msk;

        constexpr uint8_t P0_Pos = 0u;
        constexpr uint8_t P0_Msk = 1u << P0_Pos;
        constexpr uint8_t P0 = P0_Msk;

        class BitField
        {
        public:
            bool bP5 = false;
            bool bP4 = false;
            bool bP3 = false;
            bool bP2 = false;
            bool bP1 = false;
            bool bP0 = false;

            void operator=(const uint8_t reg)
            {
                bP5 = reg & P5;
                bP4 = reg & P4;
                bP3 = reg & P3;
                bP2 = reg & P2;
                bP1 = reg & P1;
                bP0 = reg & P0;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace SETUP_AW
    {
        constexpr uint8_t Msk = 0x03;
        constexpr uint8_t Reset = 0x03;

        constexpr uint8_t AW_Pos = 0u;
        constexpr uint8_t AW_Wid = 0x03;
        constexpr uint8_t AW_Msk = AW_Wid << AW_Pos;
        constexpr uint8_t AW = AW_Msk;

        class BitField
        {
        public:
            uint8_t fAW = 0u;

            void operator=(const uint8_t reg)
            {
                fAW = reg & AW;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace SETUP_RETR
    {
        constexpr uint8_t Mask = 0xFF;
        constexpr uint8_t Reset = 0x03;

        constexpr uint8_t ARD_Pos = 4u;
        constexpr uint8_t ARD_Msk = 0x0F << ARD_Pos;
        constexpr uint8_t ARD = ARD_Msk;

        constexpr uint8_t ARC_Pos = 0u;
        constexpr uint8_t ARC_Msk = 0x0F << ARC_Pos;
        constexpr uint8_t ARC = ARC_Msk;

        class BitField
        {
        public:
            uint8_t fARD = 0u;
            uint8_t fARC = 0u;

            void operator=(const uint8_t reg)
            {
                fARD = reg & ARD;
                fARC = reg & ARC;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RF_CH
    {
        constexpr uint8_t Mask = 0x7F;
        constexpr uint8_t Reset = 0x02;

        class BitField
        {
        public:
            uint8_t fRF_CH = 0u;

            void operator=(const uint8_t reg)
            {
                fRF_CH = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RF_SETUP
    {
        constexpr uint8_t Mask = 0x1F;
        constexpr uint8_t Reset = 0x0F;

        constexpr uint8_t RF_DR_LOW_Pos = 5u;
        constexpr uint8_t RF_DR_LOW_Msk = 1u << RF_DR_LOW_Pos;
        constexpr uint8_t RF_DR_LOW = RF_DR_LOW_Msk;

        constexpr uint8_t PLL_LOCK_Pos = 4u;
        constexpr uint8_t PLL_LOCK_Msk = 1u << PLL_LOCK_Pos;
        constexpr uint8_t PLL_LOCK = PLL_LOCK_Msk;

        constexpr uint8_t RF_DR_HIGH_Pos = 3u;
        constexpr uint8_t RF_DR_HIGH_Msk = 1u << RF_DR_HIGH_Pos;
        constexpr uint8_t RF_DR_HIGH = RF_DR_HIGH_Msk;

        constexpr uint8_t RF_DR_Pos = 3u;
        constexpr uint8_t RF_DR_Msk = 1u << RF_DR_Pos;
        constexpr uint8_t RF_DR = RF_DR_Msk;

        constexpr uint8_t RF_PWR_Pos = 1u;
        constexpr uint8_t RF_PWR_Wid = 0x03;
        constexpr uint8_t RF_PWR_Msk = RF_PWR_Wid << RF_PWR_Pos;
        constexpr uint8_t RF_PWR = RF_PWR_Msk;

        constexpr uint8_t LNA_HCURR_Pos = 0u;
        constexpr uint8_t LNA_HCURR_Msk = 1u << LNA_HCURR_Pos;
        constexpr uint8_t LNA_HCURR = LNA_HCURR_Msk;

        class BitField
        {
        public:
            bool bRF_DR_LOW = false;
            bool bPLL_LOCK = false;
            bool bRF_DR_HIGH = false;
            bool bRF_DR = false;
            bool bLNA_HCURR = false;
            uint8_t fRF_PWR = 0u;

            void operator=(const uint8_t reg)
            {
                bRF_DR_LOW = reg & RF_DR_LOW;
                bPLL_LOCK = reg & PLL_LOCK;
                bRF_DR_HIGH = reg & RF_DR_HIGH;
                bRF_DR = reg & RF_DR;
                bLNA_HCURR = reg & LNA_HCURR;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace STATUS
    {
        constexpr uint8_t Mask = 0x7F;
        constexpr uint8_t Reset = 0x0E;

        constexpr uint8_t RX_DR_Pos = 6u;
        constexpr uint8_t RX_DR_Msk = 1u << RX_DR_Pos;
        constexpr uint8_t RX_DR = RX_DR_Msk;

        constexpr uint8_t TX_DS_Pos = 5u;
        constexpr uint8_t TX_DS_Msk = 1u << TX_DS_Pos;
        constexpr uint8_t TX_DS = TX_DS_Msk;

        constexpr uint8_t MAX_RT_Pos = 4u;
        constexpr uint8_t MAX_RT_Msk = 1u << MAX_RT_Pos;
        constexpr uint8_t MAX_RT = MAX_RT_Msk;

        constexpr uint8_t RX_P_NO_Pos = 1u;
        constexpr uint8_t RX_P_NO_Wid = 0x07;
        constexpr uint8_t RX_P_NO_Msk = RX_P_NO_Wid << RX_P_NO_Pos;
        constexpr uint8_t RX_P_NO = RX_P_NO_Msk;

        constexpr uint8_t TX_FULL_Pos = 0u;
        constexpr uint8_t TX_FULL_Msk = 1u << TX_FULL_Pos;
        constexpr uint8_t TX_FULL = TX_FULL_Msk;

        class BitField
        {
        public:
            bool bRX_DR = false;
            bool bTX_DS = false;
            bool bMAX_RT = false;
            bool bTX_FULL = false;
            uint8_t fRX_P_NO = 0u;

            void operator=(const uint8_t reg)
            {
                bRX_DR = reg & RX_DR;
                bTX_DS = reg & TX_DS;
                bMAX_RT = reg & MAX_RT;
                bTX_FULL = reg & TX_FULL;
                fRX_P_NO = reg & RX_P_NO;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace OBSERVE_TX
    {
        constexpr uint8_t Mask = 0xFF;
        constexpr uint8_t Reset = 0x00;

        constexpr uint8_t PLOS_CNT_Pos = 4u;
        constexpr uint8_t PLOS_CNT_Wid = 0x0F;
        constexpr uint8_t PLOS_CNT_Msk = PLOS_CNT_Wid << PLOS_CNT_Pos;
        constexpr uint8_t PLOS_CNT = PLOS_CNT_Msk;

        constexpr uint8_t ARC_CNT_Pos = 0u;
        constexpr uint8_t ARC_CNT_Wid = 0x0F;
        constexpr uint8_t ARC_CNT_Msk = ARC_CNT_Wid << ARC_CNT_Pos;
        constexpr uint8_t ARC_CNT = ARC_CNT_Msk;

        class BitField
        {
        public:
            uint8_t fPLOS_CNT = 0u;
            uint8_t fARC_CNT = 0u;

            void operator=(const uint8_t reg)
            {
                fPLOS_CNT = reg & PLOS_CNT;
                fARC_CNT = reg & ARC_CNT;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace CD
    {
        constexpr uint8_t Mask = 0x01;
        constexpr uint8_t Reset = 0x00;

        constexpr uint8_t CD_Pos = 0u;
        constexpr uint8_t CD_Msk = 1u << CD_Pos;
        constexpr uint8_t CD = CD_Msk;

        class BitField
        {
        public:
            bool bCD = false;

            void operator=(const uint8_t reg)
            {
                bCD = reg & CD;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_ADDR_P0
    {
        constexpr uint8_t byteWidth = 5u;
        constexpr uint64_t Mask = 0xFFFFFFFFFF;
        constexpr uint64_t Reset = 0xE7E7E7E7E7;

        class BitField
        {
        public:
            uint64_t rxAddressP0 = 0u;

            void operator=(const uint64_t reg)
            {
                rxAddressP0 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_ADDR_P1
    {
        constexpr uint8_t byteWidth = 5u;
        constexpr uint64_t Mask = 0xFFFFFFFFFF;
        constexpr uint64_t Reset = 0xC2C2C2C2C2;

        class BitField
        {
        public:
            uint64_t rxAddressP1 = 0u;

            void operator=(const uint64_t reg)
            {
                rxAddressP1 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_ADDR_P2
    {
        constexpr uint8_t Mask = 0xFF;
        constexpr uint8_t Reset = 0xC3;

        class BitField
        {
        public:
            uint8_t rxAddressP2 = 0u;

            void operator=(const uint8_t reg)
            {
                rxAddressP2 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_ADDR_P3
    {
        constexpr uint8_t Mask = 0xFF;
        constexpr uint8_t Reset = 0xC4;

        class BitField
        {
        public:
            uint8_t rxAddressP3 = 0u;

            void operator=(const uint8_t reg)
            {
                rxAddressP3 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_ADDR_P4
    {
        constexpr uint8_t Mask = 0xFF;
        constexpr uint8_t Reset = 0xC5;

        class BitField
        {
        public:
            uint8_t rxAddressP4 = 0u;

            void operator=(const uint8_t reg)
            {
                rxAddressP4 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_ADDR_P5
    {
        constexpr uint8_t Mask = 0xFF;
        constexpr uint8_t Reset = 0xC6;

        class BitField
        {
        public:
            uint8_t rxAddressP5 = 0u;

            void operator=(const uint8_t reg)
            {
                rxAddressP5 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace TX_ADDR
    {
        constexpr uint8_t byteWidth = 5u;
        constexpr uint64_t Mask = 0xFFFFFFFFFF;
        constexpr uint64_t Reset = 0xE7E7E7E7E7;

        class BitField
        {
        public:
            uint64_t txAddress = 0u;

            void operator=(const uint64_t reg)
            {
                txAddress = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW_P0
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        class BitField
        {
        public:
            uint8_t rxPayloadWidthP0 = 0u;

            void operator=(const uint8_t reg)
            {
                rxPayloadWidthP0 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW_P1
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        class BitField
        {
        public:
            uint8_t rxPayloadWidthP1 = 0u;

            void operator=(const uint8_t reg)
            {
                rxPayloadWidthP1 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW_P2
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        class BitField
        {
        public:
            uint8_t rxPayloadWidthP2 = 0u;

            void operator=(const uint8_t reg)
            {
                rxPayloadWidthP2 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW_P3
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        class BitField
        {
        public:
            uint8_t rxPayloadWidthP3 = 0u;

            void operator=(const uint8_t reg)
            {
                rxPayloadWidthP3 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW_P4
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        class BitField
        {
        public:
            uint8_t rxPayloadWidthP4 = 0u;

            void operator=(const uint8_t reg)
            {
                rxPayloadWidthP4 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW_P5
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        class BitField
        {
        public:
            uint8_t rxPayloadWidthP5 = 0u;

            void operator=(const uint8_t reg)
            {
                rxPayloadWidthP5 = reg & Mask;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace RX_PW
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;
    }

    namespace FIFO_STATUS
    {
        constexpr uint8_t Mask = 0x7F;
        constexpr uint8_t Reset = 0x00;

        constexpr uint8_t TX_REUSE_Pos = 6u;
        constexpr uint8_t TX_REUSE_Msk = 1u << TX_REUSE_Pos;
        constexpr uint8_t TX_REUSE = TX_REUSE_Msk;

        constexpr uint8_t TX_FULL_Pos = 5u;
        constexpr uint8_t TX_FULL_Msk = 1u << TX_FULL_Pos;
        constexpr uint8_t TX_FULL = TX_FULL_Msk;

        constexpr uint8_t TX_EMPTY_Pos = 4u;
        constexpr uint8_t TX_EMPTY_Msk = 1u << TX_EMPTY_Pos;
        constexpr uint8_t TX_EMPTY = TX_EMPTY_Msk;

        constexpr uint8_t RX_FULL_Pos = 1u;
        constexpr uint8_t RX_FULL_Msk = 1u << RX_FULL_Pos;
        constexpr uint8_t RX_FULL = RX_FULL_Msk;

        constexpr uint8_t RX_EMPTY_Pos = 0u;
        constexpr uint8_t RX_EMPTY_Msk = 1u << RX_EMPTY_Pos;
        constexpr uint8_t RX_EMPTY = RX_EMPTY_Msk;

        class BitField
        {
        public:
            bool bTX_REUSE = false;
            bool bTX_FULL = false;
            bool bTX_EMPTY = false;
            bool bRX_FULL = false;
            bool bRX_EMPTY = false;

            void operator=(const uint8_t reg)
            {
                bTX_REUSE = reg & TX_REUSE;
                bTX_FULL = reg & TX_FULL;
                bTX_EMPTY = reg & TX_EMPTY;
                bRX_FULL = reg & RX_FULL;
                bRX_EMPTY = reg & RX_EMPTY;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace DYNPD
    {
        constexpr uint8_t Mask = 0x3F;
        constexpr uint8_t Reset = 0x00;

        constexpr uint8_t DPL_P5_Pos = 5u;
        constexpr uint8_t DPL_P5_Msk = 1u << DPL_P5_Pos;
        constexpr uint8_t DPL_P5 = DPL_P5_Msk;

        constexpr uint8_t DPL_P4_Pos = 4u;
        constexpr uint8_t DPL_P4_Msk = 1u << DPL_P4_Pos;
        constexpr uint8_t DPL_P4 = DPL_P4_Msk;

        constexpr uint8_t DPL_P3_Pos = 3u;
        constexpr uint8_t DPL_P3_Msk = 1u << DPL_P3_Pos;
        constexpr uint8_t DPL_P3 = DPL_P3_Msk;

        constexpr uint8_t DPL_P2_Pos = 2u;
        constexpr uint8_t DPL_P2_Msk = 1u << DPL_P2_Pos;
        constexpr uint8_t DPL_P2 = DPL_P2_Msk;

        constexpr uint8_t DPL_P1_Pos = 1u;
        constexpr uint8_t DPL_P1_Msk = 1u << DPL_P1_Pos;
        constexpr uint8_t DPL_P1 = DPL_P1_Msk;

        constexpr uint8_t DPL_P0_Pos = 0u;
        constexpr uint8_t DPL_P0_Msk = 1u << DPL_P0_Pos;
        constexpr uint8_t DPL_P0 = DPL_P0_Msk;

        class BitField
        {
        public:
            bool bDPL_P5 = false;
            bool bDPL_P4 = false;
            bool bDPL_P3 = false;
            bool bDPL_P2 = false;
            bool bDPL_P1 = false;
            bool bDPL_P0 = false;

            void operator=(const uint8_t reg)
            {
              bDPL_P5 = reg & static_cast<uint8_t>( bDPL_P5 );
              bDPL_P4 = reg & static_cast<uint8_t>( bDPL_P4 );
              bDPL_P3 = reg & static_cast<uint8_t>( bDPL_P3 );
              bDPL_P2 = reg & static_cast<uint8_t>( bDPL_P2 );
              bDPL_P1 = reg & static_cast<uint8_t>( bDPL_P1 );
              bDPL_P0 = reg & static_cast<uint8_t>( bDPL_P0 );
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    namespace FEATURE
    {
        constexpr uint8_t MSK = 0x07;
        constexpr uint8_t Reset = 0x00;

        constexpr uint8_t EN_DPL_Pos = 2u;
        constexpr uint8_t EN_DPL_Msk = 1u << EN_DPL_Pos;
        constexpr uint8_t EN_DPL = EN_DPL_Msk;

        constexpr uint8_t EN_ACK_PAY_Pos = 1u;
        constexpr uint8_t EN_ACK_PAY_Msk = 1u << EN_ACK_PAY_Pos;
        constexpr uint8_t EN_ACK_PAY = EN_ACK_PAY_Msk;

        constexpr uint8_t EN_DYN_ACK_Pos = 0u;
        constexpr uint8_t EN_DYN_ACK_Msk = 1u << EN_DYN_ACK_Pos;
        constexpr uint8_t EN_DYN_ACK = EN_DYN_ACK_Msk;

        class BitField
        {
        public:
            bool bEN_DPL = false;
            bool bEN_ACK_PAY = false;
            bool bEN_DYN_ACK = false;

            void operator=(const uint8_t reg)
            {
                bEN_DPL = reg & EN_DPL;
                bEN_ACK_PAY = reg & EN_ACK_PAY;
                bEN_DYN_ACK = reg & EN_DYN_ACK;
            }

            void update(Phy *const radio);
            void update(std::shared_ptr<Phy> &radio);
        };
    }

    /**
    *   Keeps track of the hardware registers so that config information and settings
    *   can be checked here rather than via the slow SPI bus. Also very useful as a
    *   debugging tool.
    */
    class NRF24L01Registers
    {
    public:
        CONFIG::BitField config;
        EN_AA::BitField en_aa;
        EN_RXADDR::BitField en_rxaddr;
        SETUP_AW::BitField setup_aw;
        SETUP_RETR::BitField setup_retr;
        RF_CH::BitField rf_ch;
        RF_SETUP::BitField rf_setup;
        STATUS::BitField status;
        OBSERVE_TX::BitField observe_tx;
        CD::BitField cd;
        RX_ADDR_P0::BitField rx_addr_p0;
        RX_ADDR_P1::BitField rx_addr_p1;
        RX_ADDR_P2::BitField rx_addr_p2;
        RX_ADDR_P3::BitField rx_addr_p3;
        RX_ADDR_P4::BitField rx_addr_p4;
        RX_ADDR_P5::BitField rx_addr_p5;
        TX_ADDR::BitField tx_addr;
        RX_PW_P0::BitField rx_pw_p0;
        RX_PW_P1::BitField rx_pw_p1;
        RX_PW_P2::BitField rx_pw_p2;
        RX_PW_P3::BitField rx_pw_p3;
        RX_PW_P4::BitField rx_pw_p4;
        RX_PW_P5::BitField rx_pw_p5;
        FIFO_STATUS::BitField fifo_status;
        DYNPD::BitField dynpd;
        FEATURE::BitField feature;
    };
}

#endif /* NRF24L01_DEFINITIONS_HPP */
