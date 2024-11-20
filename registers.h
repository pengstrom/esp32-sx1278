#if !defined(_SX1278_REG_H)
#define _SX1278_REG_H

#include <stdint.h>
#include "./constants.h"

class Sx1278Reg : public Sx1278Const
{
public:
  struct reg_irq_flags_t
  {
    bool rx_timeout : 1;
    bool rx_done : 1;
    bool payload_crc_error : 1;
    bool valid_header : 1;
    bool tx_done : 1;
    bool cad_done : 1;
    bool fhss_change_channel : 1;
    bool cad_detected : 1;

    reg_irq_flags_t() {}

    reg_irq_flags_t fromValue(uint8_t reg_irq_flags)
    {
      cad_detected = reg_irq_flags & 1;
      fhss_change_channel = (reg_irq_flags >> 1) & 1;
      cad_done = (reg_irq_flags >> 2) & 1;
      tx_done = (reg_irq_flags >> 3) & 1;
      valid_header = (reg_irq_flags >> 4) & 1;
      payload_crc_error = (reg_irq_flags >> 5) & 1;
      rx_done = (reg_irq_flags >> 6) & 1;
      rx_timeout = (reg_irq_flags >> 7) & 1;

      return *this;
    }

    uint8_t toValue() const
    {
      uint8_t reg_irq_flags = 0;
      reg_irq_flags |= cad_detected;
      reg_irq_flags |= (cad_done << 2);
      reg_irq_flags |= (fhss_change_channel << 1);
      reg_irq_flags |= (tx_done << 3);
      reg_irq_flags |= (valid_header << 4);
      reg_irq_flags |= (payload_crc_error << 5);
      reg_irq_flags |= (tx_done << 6);
      reg_irq_flags |= (rx_timeout << 7);
      return reg_irq_flags;
    }
  };

  struct reg_hop_channel_t
  {
    bool pll_timeout : 1;
    bool crc_on_payload : 1;
    uint8_t fhss_present_channel : 6;

    reg_hop_channel_t() {}

    void fromValue(uint8_t reg_hop_channel)
    {
      pll_timeout = reg_hop_channel >> 7;
      crc_on_payload = (reg_hop_channel >> 6) & 1;
      fhss_present_channel = reg_hop_channel & 0x3F;
    }

    uint8_t toValue() const
    {
      uint8_t reg_hop_channel = 0;
      reg_hop_channel |= pll_timeout << 7;
      reg_hop_channel |= crc_on_payload << 6;
      reg_hop_channel |= fhss_present_channel;
      return reg_hop_channel;
    }
  };

  struct reg_modem_cfg_1_t
  {
    bandwidth_t bw : 4 = bandwidth_t::KHZ_125;
    coding_rate_t coding_rate : 3 = coding_rate_t::CR_4_5;
    bool implicit_header_mode : 1 = false;

    reg_modem_cfg_1_t() {}

    void fromValue(uint8_t reg_modem_cfg_1)
    {
      bw = static_cast<bandwidth_t>(reg_modem_cfg_1 >> 4);
      coding_rate = static_cast<coding_rate_t>((reg_modem_cfg_1 >> 1) & 0b111);
      implicit_header_mode = reg_modem_cfg_1 & 1;
    }

    uint8_t toValue() const
    {
      uint8_t reg_modem_cfg_1 = static_cast<uint8_t>(bw) << 4;
      reg_modem_cfg_1 |= static_cast<uint8_t>(coding_rate) << 1;
      reg_modem_cfg_1 |= implicit_header_mode;

      return reg_modem_cfg_1;
    }
  };

  struct reg_modem_cfg_2_t
  {
    spreading_factor_t spreading_factor : 4 = spreading_factor_t::SF_128;
    bool tx_cont_mode : 1 = false;
    bool rx_payload_crc : 1 = false;
    uint8_t symb_timeout_msb : 2 = 0;

    reg_modem_cfg_2_t() {}

    void fromValue(uint8_t reg_modem_cfg_2)
    {
      spreading_factor = static_cast<spreading_factor_t>(reg_modem_cfg_2 >> 4);
      tx_cont_mode = (reg_modem_cfg_2 >> 3) & 1;
      rx_payload_crc = (reg_modem_cfg_2 >> 2) & 1;
      symb_timeout_msb = reg_modem_cfg_2 & 0b11;
    }

    uint8_t toValue() const
    {
      uint8_t reg_modem_cfg_2 = static_cast<uint8_t>(spreading_factor) << 4;
      reg_modem_cfg_2 |= tx_cont_mode << 3;
      reg_modem_cfg_2 |= rx_payload_crc << 2;
      reg_modem_cfg_2 |= symb_timeout_msb & 0b11;

      return reg_modem_cfg_2;
    }
  };

  struct reg_modem_cfg_3_t
  {
    bool low_data_rate_optimize : 1 = false;
    bool agc_auto_on : 1 = false;

    reg_modem_cfg_3_t() {}

    void fromValue(uint8_t reg_modem_cfg_3)
    {
      low_data_rate_optimize = (reg_modem_cfg_3 >> 3) & 1;
      agc_auto_on = (reg_modem_cfg_3 >> 2) & 1;
    }

    uint8_t toValue() const
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

    void fromValue(uint8_t reg_op_mode)
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
