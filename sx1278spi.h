#if !defined(_SX1278_SPI_H)
#define _SX1278_SPI_H

#include "./constants.h"
#include "./registers.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <stdint.h>

class Sx1278Spi : public Sx1278Reg
{
public:
  struct spi_config_t
  {
    spi_host_device_t host = SPI2_HOST;
    int clock_freq = CLOCK_MAX_FREQ;
    gpio_num_t mosi_pin = GPIO_NUM_NC;
    gpio_num_t miso_pin = GPIO_NUM_NC;
    gpio_num_t nss_pin = GPIO_NUM_NC;
    gpio_num_t sck_pin = GPIO_NUM_NC;
  };

  Sx1278Spi(spi_config_t spi_conf);
  ~Sx1278Spi();

  reg_irq_flags_t readIrqFlags();
  void writeIrqFlags(reg_irq_flags_t flags);

  uint32_t readFrf();
  void writeFrf(uint32_t frf);

  reg_op_mode_t readOpMode();
  void writeOpMode(reg_op_mode_t op_mode);

  reg_modem_cfg_1_t readCfg1();
  void writeCfg1(reg_modem_cfg_1_t cfg);

  reg_modem_cfg_2_t readCfg2();
  void writeCfg2(reg_modem_cfg_2_t cfg);

  reg_modem_cfg_3_t readCfg3();
  void writeCfg3(reg_modem_cfg_3_t cfg);

  reg_hop_channel_t readHopChannel();
  void writeHopChannel(reg_hop_channel_t hc);

  uint8_t writeSingle(sx_register_t reg, uint8_t value);
  void writeBurst(sx_register_t reg, uint8_t *data, size_t data_bytes, uint8_t *previous_data = NULL);
  void writeFifo(uint8_t *data, size_t data_bytes, uint8_t *previous_data = NULL);

  uint8_t readSingle(sx_register_t reg);
  void readBurst(sx_register_t reg, uint8_t *data, size_t data_bytes);
  void readFifo(uint8_t *data, size_t data_bytes);

  uint8_t regAddrRead(sx_register_t reg);
  uint8_t regAddrWrite(sx_register_t reg);

  void sendReceive(sx_register_t reg, uint8_t *send, uint8_t *receive, size_t data_bytes);
  void sendReceive(uint8_t reg, uint8_t *send, uint8_t *receive, size_t data_bytes);

  void setupSpi();

  size_t dumpRegisters(uint8_t data[REG_COUNT], sx_register_t max_addr = REG_LAST, sx_register_t min_addr = REG_FIRST);
  size_t dumpRegisters(uint8_t data[REG_COUNT], uint8_t max_addr = REG_END, uint8_t min_addr = REG_START);

  void debugStatus();

  void setupBitbang();
  void bitbangBit();
  void transmitBitbang(spi_transaction_t *cfg);
  void bitbangByte(uint8_t byte);
  uint8_t bitbangByteRead();

protected:
  spi_device_handle_t _device_handle;
  spi_bus_config_t _bus_cfg;
  spi_device_interface_config_t _device_cfg;
  spi_config_t _spi_cfg;
};

#endif // _SX1278_SPI_H
