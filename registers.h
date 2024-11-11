#if !defined(_SX1278_REG_H)
#define _SX1278_REG_H

#include <stdint.h>
#include "./constants.h"

class Sx1278Reg
{
public:
  enum class sx_register_t : uint8_t
  {
    FIFO = 0x00,
    OP_MODE = 0x01,
    RX_NB_BYTES = 0x13,
    MODEM_STAT = 0x18,
    PKT_SNR_VALUE = 0x19,
    MODEM_CFG_1 = 0x1D,
    MODEM_CFG_2 = 0x1E,
    SYMB_TIMEOUT_MSB = 0x1F,
    MODEM_CFG_3 = 0x26,
  };

  enum class mode_t : uint8_t
  {
    SLEEP,
    STDBY,
    FSTX,
    TX,
    FSRX,
    RX_CONT,
    RX_SINGLE,
    CAD
  };

  enum class bandwidth_t : uint8_t
  {
    KHZ_7_8,
    KHZ_10_4,
    KHZ_15_6,
    KHZ_20_8,
    KHZ_31_25,
    KHZ_41_7,
    KHZ_62_5,
    KHZ_125,
    KHZ_250,
    KHZ_500,
  };

  enum class coding_rate_t : uint8_t
  {
    CR_4_5 = 1,
    CR_4_6,
    CR_4_7,
    CR_4_8,
  };

  enum class spreading_factor_t : uint8_t
  {
    SF_64 = 6,
    SF_128,
    SF_256,
    SF_512,
    SF_1024,
    SF_2048,
    SF_4096,
  };

  struct reg_modem_cfg_t
  {
    bandwidth_t bw = bandwidth_t::KHZ_125;
    coding_rate_t coding_rate = coding_rate_t::CR_4_5;
    bool implicit_header_mode = false;

    spreading_factor_t spreading_factor = spreading_factor_t::SF_128;
    bool tx_cont_mode = false;
    bool rx_payload_crc = false;
    uint8_t symb_timeout_msb = 0;

    bool low_data_rate_optimize = false;
    bool agc_auto_on = false;

    reg_modem_cfg_t(uint8_t reg_modem_cfg_1, uint8_t reg_modem_cfg_2, uint8_t reg_modem_cfg_3)
    {
      fromCfg1(reg_modem_cfg_1);
      fromCfg2(reg_modem_cfg_2);
      fromCfg3(reg_modem_cfg_3);
    }

    void toValues(uint8_t &reg_modem_cfg_1, uint8_t &reg_modem_cfg_2, uint8_t &reg_modem_cfg_3) const
    {
      reg_modem_cfg_1 = toCfg1();
      reg_modem_cfg_2 = toCfg2();
      reg_modem_cfg_3 = toCfg3();
    }

    void fromCfg1(uint8_t reg_modem_cfg_1)
    {
      bw = static_cast<bandwidth_t>(reg_modem_cfg_1 >> 4);
      coding_rate = static_cast<coding_rate_t>((reg_modem_cfg_1 >> 1) & 0b111);
      implicit_header_mode = reg_modem_cfg_1 & 1;
    }

    void fromCfg2(uint8_t reg_modem_cfg_2)
    {
      spreading_factor = static_cast<spreading_factor_t>(reg_modem_cfg_2 >> 4);
      tx_cont_mode = (reg_modem_cfg_2 >> 3) & 1;
      rx_payload_crc = (reg_modem_cfg_2 >> 2) & 1;
      symb_timeout_msb = reg_modem_cfg_2 & 0b11;
    }

    void fromCfg3(uint8_t reg_modem_cfg_3)
    {
      low_data_rate_optimize = (reg_modem_cfg_3 >> 3) & 1;
      agc_auto_on = (reg_modem_cfg_3 >> 2) & 1;
    }

    uint8_t toCfg1() const
    {
      uint8_t reg_modem_cfg_1 = static_cast<uint8_t>(bw) << 4;
      reg_modem_cfg_1 |= static_cast<uint8_t>(coding_rate) << 1;
      reg_modem_cfg_1 |= implicit_header_mode;

      return reg_modem_cfg_1;
    }

    uint8_t toCfg2() const
    {
      uint8_t reg_modem_cfg_2 = static_cast<uint8_t>(spreading_factor) << 4;
      reg_modem_cfg_2 |= tx_cont_mode << 3;
      reg_modem_cfg_2 |= rx_payload_crc << 2;
      reg_modem_cfg_2 |= symb_timeout_msb & 0b11;

      return reg_modem_cfg_2;
    }

    uint8_t toCfg3() const
    {
      uint8_t reg_modem_cfg_3 = low_data_rate_optimize << 3;
      reg_modem_cfg_3 |= agc_auto_on << 2;

      return reg_modem_cfg_3;
    }
  };

  struct reg_op_mode_t
  {
    bool long_range_mode = false;
    bool access_shared_reg = false;
    uint8_t reserved = 0; // not writable
    bool low_freq_mode = false;
    mode_t mode = mode_t::STDBY;

    reg_op_mode_t(uint8_t reg_op_mode)
    {
      long_range_mode = (reg_op_mode >> 7) & 1;
      access_shared_reg = (reg_op_mode >> 6) & 1;
      reserved = (reg_op_mode >> 4) & 0b11;
      low_freq_mode = (reg_op_mode >> 3) & 1;
      mode = static_cast<mode_t>(reg_op_mode & 0b111);
    };

    uint8_t toValue() const
    {
      uint8_t value = (long_range_mode << 7);
      value |= (access_shared_reg << 6);
      value |= (reserved << 4);
      value |= (low_freq_mode << 3);
      value |= static_cast<uint8_t>(mode);

      return value;
    }
  };
};

#endif // _SX1278_REG_H
