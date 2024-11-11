
#if !defined(_SX1278_CONS_H)
#define _SX1278_CONS_H

#include <stdint.h>
#include <stddef.h>

class Sx1278Const
{
public:
  static const size_t FIFO_SIZE = 256;
  static const int CLOCK_MAX_FREQ = 10 * 1000 * 1000; // 10 MHx
  static const int DATA_SETUP_TIME_NS = 250;
  static const int NSS_SETUP_TIME_NS = 30;
  static const int NSS_HOLD_TIME_NS = 100;
  static const uint8_t READ_FLAG = 1 << 7;
  static const size_t REG_COUNT = 128;

  enum class mode_t : uint8_t
  {
    // Low-power mode. In this mode only SPI and configuration registers are accessible. Lora FIFO is not accessible.
    // Note that this is the only mode permissible to switch between FSK/OOK mode and LoRa mode.
    Sleep,
    // both Crystal oscillator and Lora baseband blocks are turned on.RF part and PLLs are disabled
    Standby,
    // This is a frequency synthesis mode for transmission.
    // The PLL selected for transmission is locked and active at the transmit frequency. The RF part is off.
    Fstx,
    //  This is a frequency synthesis mode for reception.
    // The PLL selected for reception is locked and active at the receive frequency. The RF part is off.
    Fsrx,
    //  When activated the SX1276/77/78/79 powers all remaining blocks required for transmit, ramps the PA, transmits the packet and returns to Standby mode.
    Tx,
    //  When activated the SX1276/77/78/79 powers all remaining blocks required for reception, processing all received data until a new user request is made to change operating mode.
    RxContinuous,
    //  When activated the SX1276/77/78/79 powers all remaining blocks required for reception, remains in this state until a valid packet has been received and then returns to Standby mode.
    RxSingle,
    //  When in CAD mode, the device will check a given channel to detect LoRa preamble signal
    Cad,
  };
};

#endif // _SX1278_CONS_H
