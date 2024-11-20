#include "sx1278.h"
#include <algorithm>
#include <stdexcept>
#include <string>
#include <strstream>
#include <rom/ets_sys.h>
#include <math.h>

#define NS_PER_S (1000 * 1000 * 1000)

Sx1278::Sx1278(config_t cfg) : Sx1278Spi(cfg.spi), _cfg(cfg)
{
  setupPins();

  // syncSettings();
  // setDio0Mapping(dio0_t::CAD_DONE);
}

Sx1278::~Sx1278()
{
}

void Sx1278::setDio0Mapping(dio0_t mp)
{
  writeSingle(sx_register_t::DIO_MAPPING_1, static_cast<uint8_t>(mp));
}

void Sx1278::syncSettings()
{
  uint8_t cfg1 = readSingle(sx_register_t::MODEM_CFG_1);
  _reg_settings.modem_cfg_1.fromValue(cfg1);

  uint8_t cfg2 = readSingle(sx_register_t::MODEM_CFG_2);
  _reg_settings.modem_cfg_2.fromValue(cfg2);

  uint8_t cfg3 = readSingle(sx_register_t::MODEM_CFG_3);
  _reg_settings.modem_cfg_3.fromValue(cfg3);

  reg_op_mode_t op_mode = readOpMode();
  _reg_settings.op_mode = op_mode;
}

void Sx1278::reset()
{
  gpio_set_level(_cfg.rst_pin, 0);
  ets_delay_us(150);
  gpio_set_level(_cfg.rst_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(5));
}

Sx1278::crc_status_t Sx1278::checkCrc()
{
  // Check if crc in use
  uint8_t hop_chan = readSingle(sx_register_t::HOP_CHANNEL);
  reg_hop_channel_t hc = {};
  hc.fromValue(hop_chan);
  if (!hc.crc_on_payload)
  {
    return crc_status_t::CRC_NA;
  }

  // Check for errors
  reg_irq_flags_t reg = readIrqFlags();

  // reset interrupt
  reg_irq_flags_t reset = {};
  reset.payload_crc_error = true;
  writeIrqFlags(reset);

  return reg.payload_crc_error ? crc_status_t::CRC_INVALID : crc_status_t::CRC_VALID;
}

void Sx1278::correctFrequencyError()
{
  // read RF frequency
  uint32_t frf = _reg_settings.frf;
  float old_f = frf * F_STEP;

  // read frequency error
  uint8_t fei[3];
  readBurst(sx_register_t::FEI_MSB, fei, sizeof(fei));
  int32_t freq_error = (fei[1] << 8) + fei[2];
  freq_error += (fei[0] & 0b0111) << 16;
  freq_error -= (fei[0] & 0b1000) << 16;

  float bw_khz = bandwidthFrequencykHz(_reg_settings.modem_cfg_1.bw);

  float f_err = ((freq_error * exp2f(24)) / XTAL_FREQ) * (bw_khz / 500.0);

  // update frequency
  float new_f = old_f - f_err;
  uint32_t new_frf = std::round(new_f / F_STEP);
  writeFrf(new_frf);

  // update data rate
  int16_t corr = std::round(0.95 * freq_error);
  writeSingle(sx_register_t::PPM_CORRECTION, corr);

  syncSettings();
}

uint8_t Sx1278::rxSize()
{
  return readSingle(sx_register_t::RX_NB_BYTES);
}

size_t Sx1278::readRx(uint8_t *data)
{
  uint8_t size = readSingle(sx_register_t::RX_NB_BYTES);
  uint8_t last_packet = readSingle(sx_register_t::FIFO_RX_CURRENT);
  writeSingle(sx_register_t::FIFO_ADDR_PTR, last_packet);
  readFifo(data, size);
  return size;
}

void Sx1278::writeTx(uint8_t *data, uint8_t len)
{
  writeSingle(sx_register_t::PAYLOAD_LENGTH, len);
  writeSingle(sx_register_t::FIFO_ADDR_PTR, TX_START);
  writeFifo(data, len);
}

void Sx1278::modeSleep()
{
  setMode(mode_t::SLEEP);
}

void Sx1278::modeStandby()
{
  setMode(mode_t::STDBY);
}

void Sx1278::modeTx()
{
  setMode(mode_t::TX);
}

void Sx1278::modeRxSingle()
{
  setMode(mode_t::RX_SINGLE);
}

void Sx1278::modeRxCont()
{
  setMode(mode_t::RX_CONT);
}

void Sx1278::modeCad()
{
  setMode(mode_t::CAD);
}

void Sx1278::setMode(mode_t mode)
{
  reg_op_mode_t op_mode = readOpMode();
  op_mode.mode = mode;
  writeOpMode(op_mode);

  // Let the mode switch complete
  int x = 0;
  reg_op_mode_t old_op = op_mode;
  do
  {
    op_mode = old_op;
    op_mode.mode = mode;
    uint8_t old_value = writeSingle(sx_register_t::OP_MODE, op_mode.toValue());
    old_op.fromValue(old_value);
    vTaskDelay(1);
    x++;
  } while (old_op.mode != mode);
  printf("Waited %d * 10 ms for mode\n", x);
}

void Sx1278::transmit()
{
  setMode(mode_t::TX);

  // Wait for completion
  reg_irq_flags_t flags;
  do
  {
    flags = readIrqFlags();
  } while (!flags.tx_done);

  // clear tx_done flag
  reg_irq_flags_t clear;
  clear.tx_done = true;
  writeIrqFlags(clear);

  // we are now in mode STANDBY
}

void Sx1278::receive()
{
  reg_irq_flags_t flags;
  do
  {
    flags = readIrqFlags();
  } while (!flags.rx_done || flags.rx_timeout);

  reg_irq_flags_t clear;
  if (flags.rx_timeout)
  {
    // handle
    clear.rx_timeout = true;
  }
  else
  {
    // ready to read RX data
    clear.rx_done = true;
  }
  writeIrqFlags(clear);
}

void Sx1278::waitForReady()
{
  while (gpio_get_level(_cfg.rst_pin) == 0)
  {
    vTaskDelay(1);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Sx1278::setupPins()
{
  gpio_config_t cfg_reset = {
      .pin_bit_mask = (1U << _cfg.rst_pin),
      .mode = GPIO_MODE_INPUT_OUTPUT_OD,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  gpio_config(&cfg_reset);
  gpio_set_level(_cfg.rst_pin, 1);

  gpio_config_t cfg_dio0 = {
      .pin_bit_mask = (1U << _cfg.dio0_pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  gpio_config(&cfg_dio0);
}

void Sx1278::registerDio0Callback(void (*cb)(void *arg), void *arg)
{
  gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_EDGE);
  gpio_isr_handler_add(_cfg.dio0_pin, cb, arg);
}

void Sx1278::resetRx()
{
  // clear rx flags
  reg_irq_flags_t flags;
  flags.valid_header = true;
  flags.rx_done = true;
  flags.payload_crc_error = true;
  writeIrqFlags(flags);

  writeSingle(sx_register_t::FIFO_RX_BASE_ADDR, RX_START);
}
