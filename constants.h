
#if !defined(_SX1278_CONS_H)
#define _SX1278_CONS_H

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <sstream>

class Sx1278Const
{
public:
  enum class sx_register_t : uint8_t
  {
    FIFO = 0x00,
    OP_MODE = 0x01,
    FRF_MSB = 0x06,
    FRF_MID = 0x07,
    FRF_LSB = 0x08,
    FIFO_ADDR_PTR = 0x0D,
    FIFO_TX_BASE_ADDR = 0x0E,
    FIFO_RX_BASE_ADDR = 0x0F,
    FIFO_RX_CURRENT = 0x10,
    IRQ_FLAGS = 0x12,
    RX_NB_BYTES = 0x13,
    MODEM_STAT = 0x18,
    PKT_SNR_VALUE = 0x19,
    HOP_CHANNEL = 0x1C,
    MODEM_CFG_1 = 0x1D,
    MODEM_CFG_2 = 0x1E,
    SYMB_TIMEOUT_MSB = 0x1F,
    PAYLOAD_LENGTH = 0x22,
    FIFO_RX_BYTE_ADDR = 0x25,
    MODEM_CFG_3 = 0x26,
    PPM_CORRECTION = 0x27,
    FEI_MSB = 0x28,
    FEI_MID = 0x29,
    FEI_LSB = 0x2A,
    LAST = 0x3F,
    DIO_MAPPING_1 = 0x40,
    DIO_MAPPING_2 = 0x41,
    VERSION = 0x42,
    TCXO = 0x4B
  };

  static const size_t FIFO_SIZE = 256;
  static const int CLOCK_MAX_FREQ = 10 * 1000 * 1000; // 10 MHx
  static const int DATA_SETUP_TIME_NS = 250;
  static const int NSS_SETUP_TIME_NS = 30;
  static const int NSS_HOLD_TIME_NS = 100;
  static const uint8_t WRITE_FLAG = 1 << 7;
  static const uint32_t XTAL_FREQ = 32 * 1000 * 1000;  // 32 MHz
  static const uint32_t NS_PER_S = 1000 * 1000 * 1000; // 1 billion

  static const size_t REG_COUNT = 128;
  static const sx_register_t REG_FIRST = sx_register_t::FIFO;
  static const sx_register_t REG_LAST = sx_register_t::LAST;
  static const uint8_t REG_START = 0;
  static const uint8_t REG_END = static_cast<uint8_t>(REG_LAST);
  static constexpr float F_STEP = 61.03515625;

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

  static int spreadingFactor(spreading_factor_t sf)
  {
    int out = 1;
    uint8_t count = static_cast<uint8_t>(sf);
    for (size_t i = 0; i < count; i++)
    {
      out *= 2;
    }

    return out;
  }

  static std::string operationMode(mode_t mode)
  {
    switch (mode)
    {
    case mode_t::SLEEP:
      return "Sleep";
    case mode_t::STDBY:
      return "Standby";
    case mode_t::FSTX:
      return "TX Frequency synthesis";
    case mode_t::TX:
      return "TX";
    case mode_t::FSRX:
      return "RX Frequency synthesis";
    case mode_t::RX_SINGLE:
      return "RX Single mode";
    case mode_t::RX_CONT:
      return "RX Continuos mode";
    case mode_t::CAD:
      return "Channel activity detection";
    }

    return "Unrecognized mode";
  }

  static std::string codingRate(coding_rate_t cr)
  {
    uint8_t denom = 4;
    uint8_t numer = static_cast<uint8_t>(cr) + 4;
    std::stringstream s;
    s << denom << "/" << numer;
    return std::to_string(denom) + "/" + std::to_string(numer);
  }

  static float bandwidthFrequencykHz(bandwidth_t bw)
  {
    switch (bw)
    {
    case bandwidth_t::KHZ_7_8:
      return 7.8;
    case bandwidth_t::KHZ_10_4:
      return 10.4;
    case bandwidth_t::KHZ_15_6:
      return 15.6;
    case bandwidth_t::KHZ_20_8:
      return 20.8;
    case bandwidth_t::KHZ_31_25:
      return 21.25;
    case bandwidth_t::KHZ_41_7:
      return 41.7;
    case bandwidth_t::KHZ_62_5:
      return 62.5;
    case bandwidth_t::KHZ_125:
      return 125.0;
    case bandwidth_t::KHZ_250:
      return 250.0;
    case bandwidth_t::KHZ_500:
      return 500.0;
    }
    return 0;
  }
};

#endif // _SX1278_CONS_H
