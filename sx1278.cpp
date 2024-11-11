#include "sx1278.h"
#include <algorithm>
#include <stdexcept>
#include <string>
#include <strstream>
#include <rom/ets_sys.h>

#define NS_PER_S (1000 * 1000 * 1000)

Sx1278::Sx1278(config_t cfg) : _cfg(cfg)
{
  if (cfg.clock_freq > CLOCK_MAX_FREQ)
  {
    std::ostrstream err;
    err << "SX1278 maximum SPI clock frequency is " << CLOCK_MAX_FREQ << ", " << cfg.clock_freq << " given!";
    throw std::runtime_error(err.str());
  }

  setupSpi();
}

Sx1278::~Sx1278()
{
  esp_err_t err = spi_bus_remove_device(_device_handle);
  if (err != ESP_OK)
  {
    return;
  }

  err = spi_bus_free(_cfg.host);
  if (err != ESP_OK)
  {
    return;
  }
}

void Sx1278::reset()
{
  gpio_set_level(_cfg.rst_pin, 0);
  ets_delay_us(150);
  gpio_set_level(_cfg.rst_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(5));
}

void Sx1278::dumpRegisters(reg_data_t data[REG_COUNT], uint8_t max_addr, uint8_t min_addr)
{
  uint8_t start = min_addr;
  size_t count = max_addr - min_addr;

  spi_transaction_t cfg = {
      .addr = start,
      .length = count * 8,
      .rx_buffer = data,
  };
  esp_err_t err = spi_device_transmit(_device_handle, &cfg);
  if (err != ESP_OK)
  {
    // handle3
  }
}

Sx1278::reg_modem_cfg_t Sx1278::readModemCfg()
{
  uint8_t regs[3];

  // cfg 1 and 2
  spi_transaction_t cfg = {
      .addr = regAddrRead(sx_register_t::MODEM_CFG_1),
      .length = 2 * 8, // cfg 1 and 2 are adjecent
      .rx_buffer = regs,
  };
  esp_err_t err = spi_device_transmit(_device_handle, &cfg);

  // cfg 3
  spi_transaction_t cfg = {
      .addr = regAddrRead(sx_register_t::MODEM_CFG_3),
      .length = 8,
      .rx_buffer = regs + 2,
  };
  esp_err_t err = spi_device_transmit(_device_handle, &cfg);
  if (err != ESP_OK)
  {
    // nhandle
  }

  return reg_modem_cfg_t(regs[0], regs[1], regs[2]);
}

void Sx1278::writeModemCfg(reg_modem_cfg_t modem_cfg)
{

  uint8_t regs[3];
  modem_cfg.toValues(regs[0], regs[1], regs[3]);

  // cfg 1 and 2
  spi_transaction_t cfg = {
      .addr = regAddrWrite(sx_register_t::MODEM_CFG_1),
      .length = 2 * 8, // cfg 1 and 2 are adjecent
      .tx_buffer = regs,
  };
  esp_err_t err = spi_device_transmit(_device_handle, &cfg);
  if (err != ESP_OK)
  {
    // nhandle
  }

  // cfg 3
  spi_transaction_t cfg = {
      .addr = regAddrWrite(sx_register_t::MODEM_CFG_3),
      .length = 8,
      .tx_buffer = regs + 2,
  };

  esp_err_t err = spi_device_transmit(_device_handle, &cfg);
  if (err != ESP_OK)
  {
    // nhandle
  }
}

void Sx1278::writeSingle(sx_register_t reg, uint8_t value, uint8_t *previous_value)
{
  sendReceive(reg, &value, previous_value, 1);
}

void Sx1278::writeBurst(sx_register_t reg, uint8_t *data, size_t data_bytes, uint8_t *previous_data)
{
  sendReceive(reg, data, previous_data, data_bytes);
}

void Sx1278::writeFifo(uint8_t *data, size_t data_bytes, uint8_t *previous_data)
{
  sendReceive(sx_register_t::FIFO, data, previous_data, data_bytes);
}

void Sx1278::readSingle(sx_register_t reg, uint8_t *value)
{
  sendReceive(reg, NULL, value, 1);
}

void Sx1278::readBurst(sx_register_t reg, uint8_t *data, size_t data_bytes)
{
  sendReceive(reg, NULL, data, data_bytes);
}

void Sx1278::readFifo(uint8_t *data, size_t data_bytes)
{
  sendReceive(sx_register_t::FIFO, NULL, data, data_bytes);
}

void Sx1278::waitForReady()
{
  while (gpio_get_level(_cfg.rst_pin) == 0)
  {
    vTaskDelay(1);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

void Sx1278::setupSpi()
{
  _bus_cfg = {};
  _bus_cfg.quadwp_io_num = -1;
  _bus_cfg.quadhd_io_num = -1;
  _bus_cfg.data4_io_num = -1;
  _bus_cfg.data5_io_num = -1;
  _bus_cfg.data6_io_num = -1;
  _bus_cfg.data7_io_num = -1;
  _bus_cfg.max_transfer_sz = FIFO_SIZE * 2; // 1 byte data + 1 byte addres for each FIFO entry at maximum
  _bus_cfg.flags = SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MASTER;

  if (_cfg.miso_pin != GPIO_NUM_NC && _cfg.mosi_pin != GPIO_NUM_NC && _cfg.sck_pin != GPIO_NUM_NC)
  {
    // use GPIO matrix pins
    _bus_cfg.mosi_io_num = _cfg.mosi_pin;
    _bus_cfg.miso_io_num = _cfg.miso_pin;
    _bus_cfg.sclk_io_num = _cfg.sck_pin;
  }
  else
  {
    // use direct pin for bus
    _bus_cfg.mosi_io_num = _cfg.mosi_pin,
    _bus_cfg.miso_io_num = _cfg.miso_pin,
    _bus_cfg.sclk_io_num = _cfg.sck_pin,
    _bus_cfg.flags = _bus_cfg.flags | SPICOMMON_BUSFLAG_IOMUX_PINS;
  }

  uint16_t cycles_before_data = (_cfg.clock_freq * NSS_SETUP_TIME_NS) / NS_PER_S;
  uint8_t cycles_after_data = (_cfg.clock_freq * NSS_HOLD_TIME_NS) / NS_PER_S;
  _device_cfg = {
      .command_bits = 8,
      .address_bits = 8,
      .mode = 0,
      .cs_ena_pretrans = cycles_before_data,
      .cs_ena_posttrans = cycles_after_data,
      .clock_speed_hz = _cfg.clock_freq,
      .input_delay_ns = DATA_SETUP_TIME_NS,
      .spics_io_num = _cfg.nss_pin,
      .flags = SPI_DEVICE_3WIRE,
      .pre_cb = 0,
      .post_cb = 0,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &_bus_cfg, SPI_DMA_CH_AUTO));
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &_device_cfg, &_device_handle));
}

void Sx1278::setupPins()
{
  gpio_config_t cfg_reset = {
      .pin_bit_mask = 1 << _cfg.rst_pin,
      .mode = GPIO_MODE_INPUT_OUTPUT_OD,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  gpio_config(&cfg_reset);
  gpio_set_level(_cfg.rst_pin, 1);

  // gpio_config_t cfg_dio0 = {
  //     .pin_bit_mask = 1 << _cfg.dio0_pin,
  //     .mode = GPIO_MODE_INPUT,
  //     .pull_up_en = GPIO_PULLUP_ENABLE,
  //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
  // };
  // gpio_config(&cfg_dio0);
}

uint8_t Sx1278::regAddrRead(sx_register_t reg)
{
  return regAddrWrite(reg) | READ_FLAG;
}

uint8_t Sx1278::regAddrWrite(sx_register_t reg)
{
  return static_cast<uint8_t>(reg);
}

void Sx1278::sendReceive(sx_register_t reg, uint8_t *send, uint8_t *receive, size_t data_bytes)
{
  if (!send && !receive)
  {
    return;
  }

  uint8_t addr;
  if (!send)
  {
    addr = regAddrRead(reg);
  }
  else
  {
    addr = regAddrWrite(reg);
  }
  spi_transaction_t cfg =
      {
          .addr = regAddrWrite(reg),
          .length = 8 * data_bytes,
          .tx_buffer = send,
          .rx_buffer = receive,
      };

  esp_err_t err = spi_device_transmit(_device_handle, &cfg);
  if (err != ESP_OK)
  {
    // handle
  }
}

// const Sx1278::register_t registers[] =
// {

// {0x00, "RegFifo 0x00 FIFO read/write access
// 0x01 RegOpMode 0x01 Operating mode & LoRaTM / FSK selection
// 0x02 RegBitrateMsb
// Unused
// 0x1A Bit Rate setting, Most Significant Bits
// 0x03 RegBitrateLsb 0x0B Bit Rate setting, Least Significant Bits
// 0x04 RegFdevMsb 0x00 Frequency Deviation setting, Most Significant Bits
// 0x05 RegFdevLsb 0x52 Frequency Deviation setting, Least Significant Bits
// 0x06 RegFrfMsb 0x6C RF Carrier Frequency, Most Significant Bits
// 0x07 RegFrfMid 0x80 RF Carrier Frequency, Intermediate Bits
// 0x08 RegFrfLsb 0x00 RF Carrier Frequency, Least Significant Bits
// 0x09 RegPaConfig 0x4F PA selection and Output Power control
// 0x0A RegPaRamp 0x09 Control of PA ramp time, low phase noise PLL
// 0x0B RegOcp 0x2B Over Current Protection control
// 0x0C RegLna 0x20 LNA settings
// 0x0D RegRxConfig RegFifoAddrPtr 0x08 0x0E AFC, AGC, ctrl FIFO SPI pointer
// 0x0E RegRssiConfig RegFifoTxBa-
// seAddr 0x02 RSSI Start Tx data
// 0x0F RegRssiCollision RegFifoRxBa-
// seAddr 0x0A RSSI Collision detector Start Rx data
// 0x10 RegRssiThresh FifoRxCurren-
// tAddr 0xFF RSSI Threshold control Start address of last
// packet received
// 0x11 RegRssiValue RegIrqFlagsMask n/a n/a RSSI value in dBm Optional IRQ flag mask
// 0x12 RegRxBw RegIrqFlags 0x15 Channel Filter BW Control IRQ flags
// 0x13 RegAfcBw RegRxNbBytes 0x0B AFC Channel Filter BW Number of received bytes
// 0x14 RegOokPeak RegRxHeaderCnt
// ValueMsb 0x28 OOK demodulator Number of valid headers
// received
// 0x15 RegOokFix RegRxHeaderCnt
// ValueLsb 0x0C Threshold of the OOK demod
// 0x16 RegOokAvg RegRxPacketCnt
// ValueMsb 0x12 Average of the OOK demod Number of valid packets
// received
// 0x17 Reserved17 RegRxPacketCnt
// ValueLsb 0x47 -
// 0x18 Reserved18 RegModemStat 0x32 - Live LoRaTM modem
// status
// 0x19 Reserved19 RegPktSnrValue 0x3E - Espimation of last packet
// SNR
// 0x1A RegAfcFei RegPktRssiValue 0x00 AFC and FEI control RSSI of last packet

// };