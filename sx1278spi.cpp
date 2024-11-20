#include "sx1278spi.h"
#include <strstream>
#include <rom/ets_sys.h>

#define CLK (_spi_cfg.sck_pin)
#define CS (_spi_cfg.nss_pin)
#define MOSI (_spi_cfg.mosi_pin)
#define MISO (_spi_cfg.miso_pin)

void Sx1278Spi::setupBitbang()
{
  gpio_config_t cfg = {
      .pin_bit_mask = (1U << MOSI) | (1U << CLK) | (1U << CS),
      .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&cfg);
  cfg = {
      .pin_bit_mask = (1U << MISO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  gpio_config(&cfg);

  gpio_set_level(CS, 1);
  gpio_set_level(CLK, 0);
  gpio_set_level(MOSI, 0);
}

Sx1278Spi::Sx1278Spi(spi_config_t cfg) : _spi_cfg(cfg)
{
  if (cfg.clock_freq > CLOCK_MAX_FREQ)
  {
    std::ostrstream err;
    err << "Sx1278 maximum SPI clock frequency is " << CLOCK_MAX_FREQ << ", " << cfg.clock_freq << " given!";
    throw std::runtime_error(err.str());
  }

  setupSpi();
}

Sx1278Spi::~Sx1278Spi()
{
  esp_err_t err = spi_bus_remove_device(_device_handle);
  if (err != ESP_OK)
  {
    return;
  }

  err = spi_bus_free(_spi_cfg.host);
  if (err != ESP_OK)
  {
    return;
  }
}

Sx1278Spi::reg_irq_flags_t Sx1278Spi::readIrqFlags()
{
  uint8_t value = readSingle(sx_register_t::IRQ_FLAGS);
  reg_irq_flags_t flags;
  flags.fromValue(value);
  return flags;
}

void Sx1278Spi::writeIrqFlags(reg_irq_flags_t flags)
{
  writeSingle(sx_register_t::IRQ_FLAGS, flags.toValue());
}

size_t Sx1278Spi::dumpRegisters(uint8_t data[REG_COUNT], sx_register_t max_addr, sx_register_t min_addr)
{
  return dumpRegisters(data, static_cast<uint8_t>(max_addr), static_cast<uint8_t>(min_addr));
}

size_t Sx1278Spi::dumpRegisters(uint8_t values[REG_COUNT], uint8_t max_addr, uint8_t min_addr)
{
  if (max_addr > REG_END)
  {
    max_addr = REG_END;
  }
  size_t count = max_addr - min_addr;

  sendReceive(min_addr, NULL, values, count);
  return count;
}

uint32_t Sx1278Spi::readFrf()
{
  uint8_t data[3];
  readBurst(sx_register_t::FRF_MSB, data, 3);
  uint32_t frf = 0;
  frf += (data[0] << 16);
  frf += (data[1] << 8);
  frf += data[2];
  return frf;
}

void Sx1278Spi::writeFrf(uint32_t frf)
{
  uint8_t data[] = {static_cast<uint8_t>(frf >> 16 & 0xFF), static_cast<uint8_t>((frf >> 8) & 0xFF), static_cast<uint8_t>(frf & 0xFF)};
  writeBurst(sx_register_t::FRF_MSB, data, sizeof(data));
}

Sx1278Spi::reg_op_mode_t Sx1278Spi::readOpMode()
{
  uint8_t value = readSingle(sx_register_t::OP_MODE);
  reg_op_mode_t reg;
  reg.fromValue(value);
  return reg;
}

void Sx1278Spi::writeOpMode(reg_op_mode_t cfg)
{
  writeSingle(sx_register_t::OP_MODE, cfg.toValue());
}

Sx1278Spi::reg_modem_cfg_1_t Sx1278Spi::readCfg1()
{
  uint8_t value = readSingle(sx_register_t::MODEM_CFG_1);
  reg_modem_cfg_1_t cfg1;
  cfg1.fromValue(value);
  return cfg1;
}

void Sx1278Spi::writeCfg1(reg_modem_cfg_1_t cfg)
{
  writeSingle(sx_register_t::MODEM_CFG_1, cfg.toValue());
}

Sx1278Spi::reg_modem_cfg_2_t Sx1278Spi::readCfg2()
{
  uint8_t value = readSingle(sx_register_t::MODEM_CFG_2);
  reg_modem_cfg_2_t cfg2;
  cfg2.fromValue(value);
  return cfg2;
}

void Sx1278Spi::writeCfg2(reg_modem_cfg_2_t cfg)
{
  writeSingle(sx_register_t::MODEM_CFG_2, cfg.toValue());
}

Sx1278Spi::reg_modem_cfg_3_t Sx1278Spi::readCfg3()
{
  uint8_t value = readSingle(sx_register_t::MODEM_CFG_3);
  reg_modem_cfg_3_t cfg3;
  cfg3.fromValue(value);
  return cfg3;
}

void Sx1278Spi::writeCfg3(reg_modem_cfg_3_t cfg)
{
  writeSingle(sx_register_t::MODEM_CFG_3, cfg.toValue());
}

Sx1278Spi::reg_hop_channel_t Sx1278Spi::readHopChannel()
{
  uint8_t value = readSingle(sx_register_t::HOP_CHANNEL);
  reg_hop_channel_t hc;
  hc.fromValue(value);
  return hc;
}

void Sx1278Spi::writeHopChannel(reg_hop_channel_t hc)
{
  writeSingle(sx_register_t::HOP_CHANNEL, hc.toValue());
}

uint8_t Sx1278Spi::writeSingle(sx_register_t reg, uint8_t value)
{
  uint8_t previous_value;
  writeBurst(reg, &value, 1, &previous_value);
  return previous_value;
  // uint8_t addr = static_cast<uint8_t>(reg) | READ_FLAG;
  // gpio_set_level(CS, 0);
  // bitbangByte(addr);
  // bitbangByte(value);
  // gpio_set_level(CS, 1);
  // return 0;
}

void Sx1278Spi::writeBurst(sx_register_t reg, uint8_t *data, size_t data_bytes, uint8_t *previous_data)
{
  sendReceive(reg, data, previous_data, data_bytes);
}

void Sx1278Spi::writeFifo(uint8_t *data, size_t data_bytes, uint8_t *previous_data)
{
  writeBurst(sx_register_t::FIFO, data, data_bytes, previous_data);
}

uint8_t Sx1278Spi::readSingle(sx_register_t reg)
{
  uint8_t values[4];
  readBurst(reg, values, 1);
  // printf("Read value 0x%02X from reg 0x%02X\n", value, static_cast<uint8_t>(reg));
  return values[0];

  // gpio_set_level(CS, 0);
  // uint8_t addr = static_cast<uint8_t>(reg);
  // bitbangByte(addr);
  // return bitbangByteRead();
  // gpio_set_level(CS, 1);
}

uint8_t Sx1278Spi::bitbangByteRead()
{
  uint8_t byte = 0;
  for (size_t i = 0; i < 8; i++)
  {
    ets_delay_us(500);
    int level = gpio_get_level(MISO);
    if (level == 1)
    {
      byte += (1 << (7 - i));
    }
    gpio_set_level(CLK, 1);
    gpio_set_level(CLK, 0);
  }
  return byte;
}

void Sx1278Spi::bitbangByte(uint8_t byte)
{
  for (size_t i = 0; i < 8; i++)
  {
    uint8_t msb = (byte >> (7 - i)) & 1;
    if (msb == 1)
    {
      gpio_set_level(MOSI, 1);
    }
    else
    {
      gpio_set_level(MOSI, 0);
    }
    gpio_set_level(CLK, 1);
    gpio_set_level(CLK, 0);
  }
}

void Sx1278Spi::readBurst(sx_register_t reg, uint8_t *data, size_t data_bytes)
{
  sendReceive(reg, NULL, data, data_bytes);
}

void Sx1278Spi::readFifo(uint8_t *data, size_t data_bytes)
{
  readBurst(sx_register_t::FIFO, data, data_bytes);
}

void Sx1278Spi::setupSpi()
{
  _bus_cfg = {};
  _bus_cfg.mosi_io_num = _spi_cfg.mosi_pin;
  _bus_cfg.miso_io_num = _spi_cfg.miso_pin;
  _bus_cfg.sclk_io_num = _spi_cfg.sck_pin;
  // _bus_cfg.quadwp_io_num = -1;
  // _bus_cfg.quadhd_io_num = -1;
  // _bus_cfg.data4_io_num = -1;
  // _bus_cfg.data5_io_num = -1;
  // _bus_cfg.data6_io_num = -1;
  // _bus_cfg.data7_io_num = -1;
  _bus_cfg.max_transfer_sz = FIFO_SIZE * 2; // 1 byte data + 1 byte addres for each FIFO entry at maximum
  _bus_cfg.flags = SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MASTER;

  uint16_t cycles_before_data = (_spi_cfg.clock_freq * NSS_SETUP_TIME_NS) / NS_PER_S;
  uint8_t cycles_after_data = (_spi_cfg.clock_freq * NSS_HOLD_TIME_NS) / NS_PER_S;
  _device_cfg = {
      .address_bits = 8,
      .mode = 0,
      .cs_ena_pretrans = cycles_before_data,
      // .cs_ena_pretrans = 0,
      .cs_ena_posttrans = cycles_after_data,
      // .cs_ena_posttrans = 0,
      .clock_speed_hz = _spi_cfg.clock_freq,
      .input_delay_ns = DATA_SETUP_TIME_NS,
      // .input_delay_ns = 0,
      .spics_io_num = _spi_cfg.nss_pin,
      .queue_size = 1,
      .pre_cb = 0,
      .post_cb = 0,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(_spi_cfg.host, &_bus_cfg, SPI_DMA_CH_AUTO));
  vTaskDelay(1);
  ESP_ERROR_CHECK(spi_bus_add_device(_spi_cfg.host, &_device_cfg, &_device_handle));
  // setupBitbang();
}

uint8_t Sx1278Spi::regAddrRead(sx_register_t reg)
{
  return static_cast<uint8_t>(reg);
}

uint8_t Sx1278Spi::regAddrWrite(sx_register_t reg)
{
  return static_cast<uint8_t>(reg) | WRITE_FLAG;
}

void Sx1278Spi::sendReceive(uint8_t reg, uint8_t *send, uint8_t *receive, size_t data_bytes)
{
  if (!send && !receive)
  {
    printf("Neither sending or receiving!\n");
    return;
  }

  uint8_t addr = reg;
  if (send)
  {
    addr |= WRITE_FLAG;
    // printf("Writing %u value(s), starting with 0x%02X, to register 0x%02X\n", data_bytes, send[0], reg);
  }
  else
  {
    // printf("Reading %u value(s), from register 0x%02X\n", data_bytes, reg);
  }
  spi_transaction_t cfg =
      {
          .addr = addr,
          .length = 8 * data_bytes,
          .tx_buffer = send ? send : NULL,
          .rx_buffer = receive ? receive : NULL,
      };

  // transmitBitbang(&cfg);
  esp_err_t err = spi_device_transmit(_device_handle, &cfg);
  if (err != ESP_OK)
  {
    // handle
    printf("SPI transmit error: %d\n", err);
  }
}

void Sx1278Spi::sendReceive(sx_register_t reg, uint8_t *send, uint8_t *receive, size_t data_bytes)
{
  sendReceive(static_cast<uint8_t>(reg), send, receive, data_bytes);
}

void Sx1278Spi::debugStatus()
{
  reg_hop_channel_t hc = readHopChannel();
  reg_irq_flags_t flags = readIrqFlags();
  reg_modem_cfg_1_t cfg_1 = readCfg1();
  reg_modem_cfg_2_t cfg_2 = readCfg2();
  reg_modem_cfg_3_t cfg_3 = readCfg3();
  reg_op_mode_t om = readOpMode();

  printf("\n--- LoRa Debug -------------------------\n");

  printf("Operation mode: 0x%02X\n", om.toValue());
  printf("  Access shared regs: %s\n", om.access_shared_reg ? "true" : "false");
  printf("  LoRa mode:          %s\n", om.long_range_mode ? "true" : "false");
  printf("  Low freq mode:      %s\n", om.low_freq_mode ? "true" : "false");
  printf("  Mode:               %s\n", operationMode(om.mode).c_str());

  if (om.long_range_mode)
  {
    uint32_t frf = readFrf();
    float f = frf * F_STEP;
    printf("Frf: %f Hz\n", f);

    printf("Modem config 1: 0x%02X\n", cfg_1.toValue());
    printf("  Bandwidth:            %f\n", bandwidthFrequencykHz(cfg_1.bw));
    printf("  Coding rate:          %s\n", codingRate(cfg_1.coding_rate).c_str());
    printf("  Implicit header mode: %s\n", cfg_1.implicit_header_mode ? "true" : "false");

    printf("Modem config 2: 0x%02X\n", cfg_2.toValue());
    printf(" Spreading factor:  %d\n", spreadingFactor(cfg_2.spreading_factor));
    printf(" RX payload CRC:    %s\n", cfg_2.rx_payload_crc ? "true" : "false");
    printf(" TX continous mode: %s\n", cfg_2.tx_cont_mode ? "true" : "false");

    printf("Modem config 3: 0x%02X\n", cfg_3.toValue());
    printf("  AGC auto on:            %s\n", cfg_3.agc_auto_on ? "true" : "false");
    printf("  Low data rate optimize: %s\n", cfg_3.low_data_rate_optimize ? "true" : "false");

    printf("Hop channel: 0x%02X\n", hc.toValue());
    printf("  CRC on payload:      %s\n", hc.crc_on_payload ? "true" : "false");
    printf("  PII timeout:         %s\n", hc.pll_timeout ? "true" : "false");
    printf("  FHSS preset channel: %u\n", hc.fhss_present_channel);

    printf("IRQ flags: 0x%02X\n", flags.toValue());
    printf("  CAD detected:        %s\n", flags.cad_detected ? "true" : "false");
    printf("  CAD done:            %s\n", flags.cad_done ? "true" : "false");
    printf("  FHSS change channel: %s\n", flags.fhss_change_channel ? "true" : "false");
    printf("  Payload CRC error:   %s\n", flags.payload_crc_error ? "true" : "false");
    printf("  RX done:             %s\n", flags.rx_done ? "true" : "false");
    printf("  RX timeout:          %s\n", flags.rx_timeout ? "true" : "false");
    printf("  TX done:             %s\n", flags.tx_done ? "true" : "false");
    printf("  Valid header:        %s\n", flags.valid_header ? "true" : "false");
  }
  printf("----------------------------------------\n");
}
