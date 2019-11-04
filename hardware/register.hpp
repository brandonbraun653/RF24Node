/********************************************************************************
*   File Name:
*     register.hpp
*
*   Description:
*     NRF24L01 register and command definitions
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef NRF24L01_HARDWARE_REGISTER_DEFINITIONS_HPP
#define NRF24L01_HARDWARE_REGISTER_DEFINITIONS_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

namespace RF24::Hardware
{
  struct RegisterMap
  {
    Reg8_t config;
    Reg8_t en_aa;
    Reg8_t en_rx_addr;
    Reg8_t setup_aw;
    Reg8_t setup_retr;
    Reg8_t rf_ch;
    Reg8_t rf_setup;
    Reg8_t status;
    Reg8_t observe_tx;
    Reg8_t rpd;
    Reg64_t rx_addr_p0;
    Reg64_t rx_addr_p1;
    Reg8_t rx_addr_p2;
    Reg8_t rx_addr_p3;
    Reg8_t rx_addr_p4;
    Reg8_t rx_addr_p5;
    Reg64_t tx_addr;
    Reg8_t rx_pw_p0;
    Reg8_t rx_pw_p1;
    Reg8_t rx_pw_p2;
    Reg8_t rx_pw_p3;
    Reg8_t rx_pw_p4;
    Reg8_t rx_pw_p5;
    Reg8_t fifo_status;
    Reg8_t dynpd;
    Reg8_t feature;
  };

  /*----------------------------------------------
  Command Instructions
  ----------------------------------------------*/
  constexpr uint8_t CMD_REGISTER_MASK       = 0x1F; /**< Masks off the largest available register address */
  constexpr uint8_t CMD_R_REGISTER          = 0x00; /**< Read command and status registers  */
  constexpr uint8_t CMD_W_REGISTER          = 0x20; /**< Write command and status registers  */
  constexpr uint8_t CMD_R_RX_PAYLOAD        = 0x61; /**< Read RX Payload (1-32 bytes) */
  constexpr uint8_t CMD_W_TX_PAYLOAD        = 0xA0; /**< Write TX Payload (1-32 bytes) */
  constexpr uint8_t CMD_FLUSH_TX            = 0xE1; /**< Flush TX FIFO, used in TX Mode */
  constexpr uint8_t CMD_FLUSH_RX            = 0xE2; /**< Flush RX FIFO, used in RX Mode */
  constexpr uint8_t CMD_REUSE_TX_PL         = 0xE3; /**< Reuse last transmitted payload (PTX device only) */
  constexpr uint8_t CMD_ACTIVATE            = 0x50; /**< Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK */
  constexpr uint8_t CMD_R_RX_PL_WID         = 0x60; /**< Read RX payload width for the top payload in the RX FIFO */
  constexpr uint8_t CMD_W_ACK_PAYLOAD       = 0xA8; /**< Write Payload together with ACK packet */
  constexpr uint8_t CMD_W_TX_PAYLOAD_NO_ACK = 0xB0; /**< Disables AUTOACK on this specific packet */
  constexpr uint8_t CMD_NOP                 = 0xFF; /**< No operation */

  /*----------------------------------------------
  Register Addresses
  ----------------------------------------------*/
  constexpr uint8_t REG_CONFIG      = 0x00; /**< Configuration Register */
  constexpr uint8_t REG_EN_AA       = 0x01; /**< Enable Auto Acknowledgment */
  constexpr uint8_t REG_EN_RXADDR   = 0x02; /**< Enable RX Addresses */
  constexpr uint8_t REG_SETUP_AW    = 0x03; /**< Setup of Address Width */
  constexpr uint8_t REG_SETUP_RETR  = 0x04; /**< Setup of Automatic Retransmission */
  constexpr uint8_t REG_RF_CH       = 0x05; /**< RF Channel Frequency Settings */
  constexpr uint8_t REG_RF_SETUP    = 0x06; /**< RF Channel Settings Register */
  constexpr uint8_t REG_STATUS      = 0x07; /**< Status Register */
  constexpr uint8_t REG_OBSERVE_TX  = 0x08; /**< Transmit Observe */
  constexpr uint8_t REG_CD          = 0x09; /**< Carrier Detect */
  constexpr uint8_t REG_RX_ADDR_P0  = 0x0A; /**< Receive Address Data Pipe 0 */
  constexpr uint8_t REG_RX_ADDR_P1  = 0x0B; /**< Receive Address Data Pipe 1 */
  constexpr uint8_t REG_RX_ADDR_P2  = 0x0C; /**< Receive Address Data Pipe 2 */
  constexpr uint8_t REG_RX_ADDR_P3  = 0x0D; /**< Receive Address Data Pipe 3 */
  constexpr uint8_t REG_RX_ADDR_P4  = 0x0E; /**< Receive Address Data Pipe 4 */
  constexpr uint8_t REG_RX_ADDR_P5  = 0x0F; /**< Receive Address Data Pipe 5 */
  constexpr uint8_t REG_TX_ADDR     = 0x10; /**< Transmit Address */
  constexpr uint8_t REG_RX_PW_P0    = 0x11; /**< Number of bytes in RX Payload Data Pipe 0 */
  constexpr uint8_t REG_RX_PW_P1    = 0x12; /**< Number of bytes in RX Payload Data Pipe 1 */
  constexpr uint8_t REG_RX_PW_P2    = 0x13; /**< Number of bytes in RX Payload Data Pipe 2 */
  constexpr uint8_t REG_RX_PW_P3    = 0x14; /**< Number of bytes in RX Payload Data Pipe 3 */
  constexpr uint8_t REG_RX_PW_P4    = 0x15; /**< Number of bytes in RX Payload Data Pipe 4 */
  constexpr uint8_t REG_RX_PW_P5    = 0x16; /**< Number of bytes in RX Payload Data Pipe 5 */
  constexpr uint8_t REG_FIFO_STATUS = 0x17; /**< FIFO Status Register */
  constexpr uint8_t REG_DYNPD       = 0x1C; /**< Enable Dynamic Payload Length for Data Pipes */
  constexpr uint8_t REG_FEATURE     = 0x1D; /**< Feature Register */

  /*----------------------------------------------
  Config Register
  ----------------------------------------------*/
  constexpr uint8_t CONFIG_Mask            = 0x7F;
  constexpr uint8_t CONFIG_Reset           = 0x08;
  constexpr uint8_t CONFIG_MASK_RX_DR_Pos  = 6u;
  constexpr uint8_t CONFIG_MASK_RX_DR_Msk  = 1u << CONFIG_MASK_RX_DR_Pos;
  constexpr uint8_t CONFIG_MASK_RX_DR      = CONFIG_MASK_RX_DR_Msk;
  constexpr uint8_t CONFIG_MASK_TX_DS_Pos  = 5u;
  constexpr uint8_t CONFIG_MASK_TX_DS_Msk  = 1u << CONFIG_MASK_TX_DS_Pos;
  constexpr uint8_t CONFIG_MASK_TX_DS      = CONFIG_MASK_TX_DS_Msk;
  constexpr uint8_t CONFIG_MASK_MAX_RT_Pos = 4u;
  constexpr uint8_t CONFIG_MASK_MAX_RT_Msk = 1u << CONFIG_MASK_MAX_RT_Pos;
  constexpr uint8_t CONFIG_MASK_MAX_RT     = CONFIG_MASK_MAX_RT_Msk;
  constexpr uint8_t CONFIG_EN_CRC_Pos      = 3u;
  constexpr uint8_t CONFIG_EN_CRC_Msk      = 1u << CONFIG_EN_CRC_Pos;
  constexpr uint8_t CONFIG_EN_CRC          = CONFIG_EN_CRC_Msk;
  constexpr uint8_t CONFIG_CRCO_Pos        = 2u;
  constexpr uint8_t CONFIG_CRCO_Msk        = 1u << CONFIG_CRCO_Pos;
  constexpr uint8_t CONFIG_CRCO            = CONFIG_CRCO_Msk;
  constexpr uint8_t CONFIG_PWR_UP_Pos      = 1u;
  constexpr uint8_t CONFIG_PWR_UP_Msk      = 1u << CONFIG_PWR_UP_Pos;
  constexpr uint8_t CONFIG_PWR_UP          = CONFIG_PWR_UP_Msk;
  constexpr uint8_t CONFIG_PRIM_RX_Pos     = 0u;
  constexpr uint8_t CONFIG_PRIM_RX_Msk     = 1u << CONFIG_PRIM_RX_Pos;
  constexpr uint8_t CONFIG_PRIM_RX         = CONFIG_PRIM_RX_Msk;

  union ConfigBitField
  {
    struct _internal
    {
      bool reserved : 1;
      bool mask_rx_dr : 1;
      bool mask_tx_ds : 1;
      bool mask_max_rt : 1;
      bool en_crc : 1;
      bool crco : 1;
      bool pwr_up : 1;
      bool prim_rx : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Enable Auto Acknowledgement Register
  -------------------------------------------------*/
  constexpr uint8_t EN_AA_Mask   = 0x3F;
  constexpr uint8_t EN_AA_Reset  = 0x3F;
  constexpr uint8_t EN_AA_P5_Pos = 5u;
  constexpr uint8_t EN_AA_P5_Msk = 1u << EN_AA_P5_Pos;
  constexpr uint8_t EN_AA_P5     = EN_AA_P5_Msk;
  constexpr uint8_t EN_AA_P4_Pos = 4u;
  constexpr uint8_t EN_AA_P4_Msk = 1u << EN_AA_P4_Pos;
  constexpr uint8_t EN_AA_P4     = EN_AA_P4_Msk;
  constexpr uint8_t EN_AA_P3_Pos = 3u;
  constexpr uint8_t EN_AA_P3_Msk = 1u << EN_AA_P3_Pos;
  constexpr uint8_t EN_AA_P3     = EN_AA_P3_Msk;
  constexpr uint8_t EN_AA_P2_Pos = 2u;
  constexpr uint8_t EN_AA_P2_Msk = 1u << EN_AA_P2_Pos;
  constexpr uint8_t EN_AA_P2     = EN_AA_P2_Msk;
  constexpr uint8_t EN_AA_P1_Pos = 1u;
  constexpr uint8_t EN_AA_P1_Msk = 1u << EN_AA_P1_Pos;
  constexpr uint8_t EN_AA_P1     = EN_AA_P1_Msk;
  constexpr uint8_t EN_AA_P0_Pos = 0u;
  constexpr uint8_t EN_AA_P0_Msk = 1u << EN_AA_P0_Pos;
  constexpr uint8_t EN_AA_P0     = EN_AA_P0_Msk;

  union ENAABitField
  {
    struct _internal
    {
      uint8_t reserved : 2;
      bool P5 : 1;
      bool P4 : 1;
      bool P3 : 1;
      bool P2 : 1;
      bool P1 : 1;
      bool P0 : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Enable Auto Acknowledgement Register
  -------------------------------------------------*/
  constexpr uint8_t EN_RXADDR_Mask   = 0x3F;
  constexpr uint8_t EN_RXADDR_Reset  = 0x03;
  constexpr uint8_t EN_RXADDR_P5_Pos = 5u;
  constexpr uint8_t EN_RXADDR_P5_Msk = 1u << EN_RXADDR_P5_Pos;
  constexpr uint8_t EN_RXADDR_P5     = EN_RXADDR_P5_Msk;
  constexpr uint8_t EN_RXADDR_P4_Pos = 4u;
  constexpr uint8_t EN_RXADDR_P4_Msk = 1u << EN_RXADDR_P4_Pos;
  constexpr uint8_t EN_RXADDR_P4     = EN_RXADDR_P4_Msk;
  constexpr uint8_t EN_RXADDR_P3_Pos = 3u;
  constexpr uint8_t EN_RXADDR_P3_Msk = 1u << EN_RXADDR_P3_Pos;
  constexpr uint8_t EN_RXADDR_P3     = EN_RXADDR_P3_Msk;
  constexpr uint8_t EN_RXADDR_P2_Pos = 2u;
  constexpr uint8_t EN_RXADDR_P2_Msk = 1u << EN_RXADDR_P2_Pos;
  constexpr uint8_t EN_RXADDR_P2     = EN_RXADDR_P2_Msk;
  constexpr uint8_t EN_RXADDR_P1_Pos = 1u;
  constexpr uint8_t EN_RXADDR_P1_Msk = 1u << EN_RXADDR_P1_Pos;
  constexpr uint8_t EN_RXADDR_P1     = EN_RXADDR_P1_Msk;
  constexpr uint8_t EN_RXADDR_P0_Pos = 0u;
  constexpr uint8_t EN_RXADDR_P0_Msk = 1u << EN_RXADDR_P0_Pos;
  constexpr uint8_t EN_RXADDR_P0     = EN_RXADDR_P0_Msk;

  union ENRXADDRBitField
  {
    struct _internal
    {
      uint8_t reserved : 2;
      bool P5 : 1;
      bool P4 : 1;
      bool P3 : 1;
      bool P2 : 1;
      bool P1 : 1;
      bool P0 : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Setup Address Widths
  -------------------------------------------------*/
  constexpr uint8_t SETUP_AW_Msk    = 0x03;
  constexpr uint8_t SETUP_AW_Reset  = 0x03;
  constexpr uint8_t SETUP_AW_AW_Pos = 0u;
  constexpr uint8_t SETUP_AW_AW_Wid = 0x03;
  constexpr uint8_t SETUP_AW_AW_Msk = SETUP_AW_AW_Wid << SETUP_AW_AW_Pos;
  constexpr uint8_t SETUP_AW_AW     = SETUP_AW_AW_Msk;

  union SETUPADDRBitField
  {
    struct _internal
    {
      uint8_t reserved : 6;
      uint8_t AW : 2;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Setup Automatic Retransmission
  -------------------------------------------------*/
  constexpr uint8_t SETUP_RETR_Mask    = 0xFF;
  constexpr uint8_t SETUP_RETR_Reset   = 0x03;
  constexpr uint8_t SETUP_RETR_ARD_Pos = 4u;
  constexpr uint8_t SETUP_RETR_ARD_Msk = 0x0F << SETUP_RETR_ARD_Pos;
  constexpr uint8_t SETUP_RETR_ARD     = SETUP_RETR_ARD_Msk;
  constexpr uint8_t SETUP_RETR_ARC_Pos = 0u;
  constexpr uint8_t SETUP_RETR_ARC_Msk = 0x0F << SETUP_RETR_ARC_Pos;
  constexpr uint8_t SETUP_RETR_ARC     = SETUP_RETR_ARC_Msk;

  union SETUPRETRBitField
  {
    struct _internal
    {
      uint8_t ARD : 4;
      uint8_t ARC : 4;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  RF Channel
  -------------------------------------------------*/
  constexpr uint8_t RF_CH_Mask  = 0x7F;
  constexpr uint8_t RF_CH_Reset = 0x02;

  union RFCHBitField
  {
    struct _internal
    {
      bool reserved : 1;
      uint8_t RF_CH : 7;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  RF Setup
  -------------------------------------------------*/
  constexpr uint8_t RF_SETUP_Mask           = 0x1F;
  constexpr uint8_t RF_SETUP_Reset          = 0x0F;
  constexpr uint8_t RF_SETUP_RF_DR_LOW_Pos  = 5u;
  constexpr uint8_t RF_SETUP_RF_DR_LOW_Msk  = 1u << RF_SETUP_RF_DR_LOW_Pos;
  constexpr uint8_t RF_SETUP_RF_DR_LOW      = RF_SETUP_RF_DR_LOW_Msk;
  constexpr uint8_t RF_SETUP_PLL_LOCK_Pos   = 4u;
  constexpr uint8_t RF_SETUP_PLL_LOCK_Msk   = 1u << RF_SETUP_PLL_LOCK_Pos;
  constexpr uint8_t RF_SETUP_PLL_LOCK       = RF_SETUP_PLL_LOCK_Msk;
  constexpr uint8_t RF_SETUP_RF_DR_HIGH_Pos = 3u;
  constexpr uint8_t RF_SETUP_RF_DR_HIGH_Msk = 1u << RF_SETUP_RF_DR_HIGH_Pos;
  constexpr uint8_t RF_SETUP_RF_DR_HIGH     = RF_SETUP_RF_DR_HIGH_Msk;
  constexpr uint8_t RF_SETUP_RF_DR_Pos      = 3u;
  constexpr uint8_t RF_SETUP_RF_DR_Msk      = 1u << RF_SETUP_RF_DR_Pos;
  constexpr uint8_t RF_SETUP_RF_DR          = RF_SETUP_RF_DR_Msk;
  constexpr uint8_t RF_SETUP_RF_PWR_Pos     = 1u;
  constexpr uint8_t RF_SETUP_RF_PWR_Wid     = 0x03;
  constexpr uint8_t RF_SETUP_RF_PWR_Msk     = RF_SETUP_RF_PWR_Wid << RF_SETUP_RF_PWR_Pos;
  constexpr uint8_t RF_SETUP_RF_PWR         = RF_SETUP_RF_PWR_Msk;
  constexpr uint8_t RF_SETUP_LNA_HCURR_Pos  = 0u;
  constexpr uint8_t RF_SETUP_LNA_HCURR_Msk  = 1u << RF_SETUP_LNA_HCURR_Pos;
  constexpr uint8_t RF_SETUP_LNA_HCURR      = RF_SETUP_LNA_HCURR_Msk;

  union RFSETUPBitfield
  {
    struct _internal
    {
      bool cont_wave  : 1;
      bool reserved   : 1;
      bool rf_dr_low  : 1;
      bool pll_lock   : 1;
      bool rf_dr_high : 1;
      uint8_t rf_pwr  : 2;
      bool reserved1  : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Status Register
  -------------------------------------------------*/
  constexpr uint8_t STATUS_Mask        = 0x7F;
  constexpr uint8_t STATUS_Reset       = 0x0E;
  constexpr uint8_t STATUS_RX_DR_Pos   = 6u;
  constexpr uint8_t STATUS_RX_DR_Msk   = 1u << STATUS_RX_DR_Pos;
  constexpr uint8_t STATUS_RX_DR       = STATUS_RX_DR_Msk;
  constexpr uint8_t STATUS_TX_DS_Pos   = 5u;
  constexpr uint8_t STATUS_TX_DS_Msk   = 1u << STATUS_TX_DS_Pos;
  constexpr uint8_t STATUS_TX_DS       = STATUS_TX_DS_Msk;
  constexpr uint8_t STATUS_MAX_RT_Pos  = 4u;
  constexpr uint8_t STATUS_MAX_RT_Msk  = 1u << STATUS_MAX_RT_Pos;
  constexpr uint8_t STATUS_MAX_RT      = STATUS_MAX_RT_Msk;
  constexpr uint8_t STATUS_RX_P_NO_Pos = 1u;
  constexpr uint8_t STATUS_RX_P_NO_Wid = 0x07;
  constexpr uint8_t STATUS_RX_P_NO_Msk = STATUS_RX_P_NO_Wid << STATUS_RX_P_NO_Pos;
  constexpr uint8_t STATUS_RX_P_NO     = STATUS_RX_P_NO_Msk;
  constexpr uint8_t STATUS_TX_FULL_Pos = 0u;
  constexpr uint8_t STATUS_TX_FULL_Msk = 1u << STATUS_TX_FULL_Pos;
  constexpr uint8_t STATUS_TX_FULL     = STATUS_TX_FULL_Msk;

  union STATUSBitfield
  {
    struct _internal
    {
      bool reserved   : 1;
      bool rx_dr      : 1;
      bool tx_ds      : 1;
      bool max_rt     : 1;
      uint8_t rx_p_no : 3;
      bool tx_full    : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Transmit Observe Register
  -------------------------------------------------*/
  constexpr uint8_t OBSERVE_TX_Mask         = 0xFF;
  constexpr uint8_t OBSERVE_TX_Reset        = 0x00;
  constexpr uint8_t OBSERVE_TX_PLOS_CNT_Pos = 4u;
  constexpr uint8_t OBSERVE_TX_PLOS_CNT_Wid = 0x0F;
  constexpr uint8_t OBSERVE_TX_PLOS_CNT_Msk = OBSERVE_TX_PLOS_CNT_Wid << OBSERVE_TX_PLOS_CNT_Pos;
  constexpr uint8_t OBSERVE_TX_PLOS_CNT     = OBSERVE_TX_PLOS_CNT_Msk;
  constexpr uint8_t OBSERVE_TX_ARC_CNT_Pos  = 0u;
  constexpr uint8_t OBSERVE_TX_ARC_CNT_Wid  = 0x0F;
  constexpr uint8_t OBSERVE_TX_ARC_CNT_Msk  = OBSERVE_TX_ARC_CNT_Wid << OBSERVE_TX_ARC_CNT_Pos;
  constexpr uint8_t OBSERVE_TX_ARC_CNT      = OBSERVE_TX_ARC_CNT_Msk;

  union OBSERVETXBitfield
  {
    struct _internal
    {
      uint8_t plos_cnt  : 4;
      uint8_t arc_cnt   : 4;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Receive Power Detect Register
  -------------------------------------------------*/
  constexpr uint8_t RPD_Mask    = 0x01;
  constexpr uint8_t RPD_Reset   = 0x00;
  constexpr uint8_t RPD_RPD_Pos = 0u;
  constexpr uint8_t RPD_RPD_Msk = 1u << RPD_RPD_Pos;
  constexpr uint8_t RPD_RPD     = RPD_RPD_Msk;

  union RPDBitfield
  {
    struct _internal
    {
      uint8_t reserved : 7;
      bool rpd : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  RX Address Register
  -------------------------------------------------*/
  constexpr size_t RX_ADDR_P0_byteWidth = 5u;
  constexpr uint64_t RX_ADDR_P0_Mask    = 0xFFFFFFFFFF;
  constexpr uint64_t RX_ADDR_P0_Reset   = 0xE7E7E7E7E7;
  constexpr size_t RX_ADDR_P1_byteWidth = 5u;
  constexpr uint64_t RX_ADDR_P1_Mask    = 0xFFFFFFFFFF;
  constexpr uint64_t RX_ADDR_P1_Reset   = 0xC2C2C2C2C2;
  constexpr uint8_t RX_ADDR_P2_Mask     = 0xFF;
  constexpr uint8_t RX_ADDR_P2_Reset    = 0xC3;
  constexpr uint8_t RX_ADDR_P3_Mask     = 0xFF;
  constexpr uint8_t RX_ADDR_P3_Reset    = 0xC4;
  constexpr uint8_t RX_ADDR_P4_Mask     = 0xFF;
  constexpr uint8_t RX_ADDR_P4_Reset    = 0xC5;
  constexpr uint8_t RX_ADDR_P5_Mask     = 0xFF;
  constexpr uint8_t RX_ADDR_P5_Reset    = 0xC6;

  /*-------------------------------------------------
  TX Address Register
  -------------------------------------------------*/
  constexpr uint8_t TX_ADDR_byteWidth   = 5u;
  constexpr uint64_t TX_ADDR_Mask       = 0xFFFFFFFFFF;
  constexpr uint64_t TX_ADDR_Reset      = 0xE7E7E7E7E7;

  /*-------------------------------------------------
  RX Payload Width Register
  -------------------------------------------------*/
  constexpr uint8_t RX_PW_P0_Mask  = 0x3F;
  constexpr uint8_t RX_PW_P0_Reset = 0x00;
  constexpr uint8_t RX_PW_P1_Mask  = 0x3F;
  constexpr uint8_t RX_PW_P1_Reset = 0x00;
  constexpr uint8_t RX_PW_P2_Mask  = 0x3F;
  constexpr uint8_t RX_PW_P2_Reset = 0x00;
  constexpr uint8_t RX_PW_P3_Mask  = 0x3F;
  constexpr uint8_t RX_PW_P3_Reset = 0x00;
  constexpr uint8_t RX_PW_P4_Mask  = 0x3F;
  constexpr uint8_t RX_PW_P4_Reset = 0x00;
  constexpr uint8_t RX_PW_P5_Mask  = 0x3F;
  constexpr uint8_t RX_PW_P5_Reset = 0x00;

  /*-------------------------------------------------
  FIFO Status Register
  -------------------------------------------------*/
  constexpr uint8_t FIFO_STATUS_Mask         = 0x7F;
  constexpr uint8_t FIFO_STATUS_Reset        = 0x00;
  constexpr uint8_t FIFO_STATUS_TX_REUSE_Pos = 6u;
  constexpr uint8_t FIFO_STATUS_TX_REUSE_Msk = 1u << FIFO_STATUS_TX_REUSE_Pos;
  constexpr uint8_t FIFO_STATUS_TX_REUSE     = FIFO_STATUS_TX_REUSE_Msk;
  constexpr uint8_t FIFO_STATUS_TX_FULL_Pos  = 5u;
  constexpr uint8_t FIFO_STATUS_TX_FULL_Msk  = 1u << FIFO_STATUS_TX_FULL_Pos;
  constexpr uint8_t FIFO_STATUS_TX_FULL      = FIFO_STATUS_TX_FULL_Msk;
  constexpr uint8_t FIFO_STATUS_TX_EMPTY_Pos = 4u;
  constexpr uint8_t FIFO_STATUS_TX_EMPTY_Msk = 1u << FIFO_STATUS_TX_EMPTY_Pos;
  constexpr uint8_t FIFO_STATUS_TX_EMPTY     = FIFO_STATUS_TX_EMPTY_Msk;
  constexpr uint8_t FIFO_STATUS_RX_FULL_Pos  = 1u;
  constexpr uint8_t FIFO_STATUS_RX_FULL_Msk  = 1u << FIFO_STATUS_RX_FULL_Pos;
  constexpr uint8_t FIFO_STATUS_RX_FULL      = FIFO_STATUS_RX_FULL_Msk;
  constexpr uint8_t FIFO_STATUS_RX_EMPTY_Pos = 0u;
  constexpr uint8_t FIFO_STATUS_RX_EMPTY_Msk = 1u << FIFO_STATUS_RX_EMPTY_Pos;
  constexpr uint8_t FIFO_STATUS_RX_EMPTY     = FIFO_STATUS_RX_EMPTY_Msk;

  union FIFOSTATUSBitfield
  {
    struct _internal
    {
      uint8_t reserved : 1;
      bool tx_reuse : 1;
      bool tx_full : 1;
      bool tx_empty : 1;
      uint8_t reserved1 : 2;
      bool rx_full : 1;
      bool rx_empty : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Dynamic Payload Enable Register
  -------------------------------------------------*/
  constexpr uint8_t DYNPD_Mask       = 0x3F;
  constexpr uint8_t DYNPD_Reset      = 0x00;
  constexpr uint8_t DYNPD_DPL_P5_Pos = 5u;
  constexpr uint8_t DYNPD_DPL_P5_Msk = 1u << DYNPD_DPL_P5_Pos;
  constexpr uint8_t DYNPD_DPL_P5     = DYNPD_DPL_P5_Msk;
  constexpr uint8_t DYNPD_DPL_P4_Pos = 4u;
  constexpr uint8_t DYNPD_DPL_P4_Msk = 1u << DYNPD_DPL_P4_Pos;
  constexpr uint8_t DYNPD_DPL_P4     = DYNPD_DPL_P4_Msk;
  constexpr uint8_t DYNPD_DPL_P3_Pos = 3u;
  constexpr uint8_t DYNPD_DPL_P3_Msk = 1u << DYNPD_DPL_P3_Pos;
  constexpr uint8_t DYNPD_DPL_P3     = DYNPD_DPL_P3_Msk;
  constexpr uint8_t DYNPD_DPL_P2_Pos = 2u;
  constexpr uint8_t DYNPD_DPL_P2_Msk = 1u << DYNPD_DPL_P2_Pos;
  constexpr uint8_t DYNPD_DPL_P2     = DYNPD_DPL_P2_Msk;
  constexpr uint8_t DYNPD_DPL_P1_Pos = 1u;
  constexpr uint8_t DYNPD_DPL_P1_Msk = 1u << DYNPD_DPL_P1_Pos;
  constexpr uint8_t DYNPD_DPL_P1     = DYNPD_DPL_P1_Msk;
  constexpr uint8_t DYNPD_DPL_P0_Pos = 0u;
  constexpr uint8_t DYNPD_DPL_P0_Msk = 1u << DYNPD_DPL_P0_Pos;
  constexpr uint8_t DYNPD_DPL_P0     = DYNPD_DPL_P0_Msk;

  union DYNPDBitfield
  {
    struct _internal
    {
      uint8_t reserved : 2;
      bool p5 : 1;
      bool p4 : 1;
      bool p3 : 1;
      bool p2 : 1;
      bool p1 : 1; 
      bool p0 : 1;
    };

    uint8_t allFields;
  };

  /*-------------------------------------------------
  Feature Register
  -------------------------------------------------*/
  constexpr uint8_t FEATURE_MSK            = 0x07;
  constexpr uint8_t FEATURE_Reset          = 0x00;
  constexpr uint8_t FEATURE_EN_DPL_Pos     = 2u;
  constexpr uint8_t FEATURE_EN_DPL_Msk     = 1u << FEATURE_EN_DPL_Pos;
  constexpr uint8_t FEATURE_EN_DPL         = FEATURE_EN_DPL_Msk;
  constexpr uint8_t FEATURE_EN_ACK_PAY_Pos = 1u;
  constexpr uint8_t FEATURE_EN_ACK_PAY_Msk = 1u << FEATURE_EN_ACK_PAY_Pos;
  constexpr uint8_t FEATURE_EN_ACK_PAY     = FEATURE_EN_ACK_PAY_Msk;
  constexpr uint8_t FEATURE_EN_DYN_ACK_Pos = 0u;
  constexpr uint8_t FEATURE_EN_DYN_ACK_Msk = 1u << FEATURE_EN_DYN_ACK_Pos;
  constexpr uint8_t FEATURE_EN_DYN_ACK     = FEATURE_EN_DYN_ACK_Msk;

  union FEATUREBitfield
  {
    struct _internal
    {
      uint8_t reserved : 5;
      bool en_dpl : 1;
      bool en_ack_payload : 1;
      bool en_dyn_ack : 1;
    };

    uint8_t allFields;
  };
}    // namespace RF24Phy

#endif /* NRF24L01_HARDWARE_REGISTER_DEFINITIONS_HPP */
