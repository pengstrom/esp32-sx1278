#if !defined(_SX1278)
#define _SX1278

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "./registers.h"
#include "./constants.h"

class Sx1278 : Sx1278Reg, Sx1278Const
{
public:
  // enum class modulation_t
  // {
  //   LoRa,
  //   Ook,
  //   Fsk,
  //   Gfsk,
  //   Msk,
  //   Gmsk
  // };

  struct config_t
  {
    spi_host_device_t host = SPI2_HOST;
    int clock_freq = CLOCK_MAX_FREQ;
    gpio_num_t mosi_pin = GPIO_NUM_NC;
    gpio_num_t miso_pin = GPIO_NUM_NC;
    gpio_num_t nss_pin = GPIO_NUM_NC;
    gpio_num_t sck_pin = GPIO_NUM_NC;
    gpio_num_t rst_pin = GPIO_NUM_NC;
    // gpio_num_t dio0_pin = GPIO_NUM_NC; // Not used in LoRa
  };

  Sx1278(config_t cfg);
  ~Sx1278();

  void reset();

  struct reg_data_t
  {
    uint8_t reg;
    uint8_t value;
    bool valid;
  };
  void dumpRegisters(reg_data_t data[REG_COUNT], uint8_t max_addr = 0x7F, uint8_t min_addr = 0x00);
  reg_modem_cfg_t readModemCfg();
  void writeModemCfg(reg_modem_cfg_t cfg);

  void writeSingle(sx_register_t reg, uint8_t value, uint8_t *previous_value = NULL);
  void writeBurst(sx_register_t reg, uint8_t *data, size_t data_bytes, uint8_t *previous_data = NULL);
  void writeFifo(uint8_t *data, size_t data_bytes, uint8_t *previous_data = NULL);

  void readSingle(sx_register_t reg, uint8_t *value);
  void readBurst(sx_register_t reg, uint8_t *data, size_t data_bytes);
  void readFifo(uint8_t *data, size_t data_bytes);

private:
  config_t _cfg;
  spi_device_handle_t _device_handle;
  spi_bus_config_t _bus_cfg;
  spi_device_interface_config_t _device_cfg;

  void waitForReady();

  void setupSpi();
  void setupPins();

  uint8_t regAddrRead(sx_register_t reg);
  uint8_t regAddrWrite(sx_register_t reg);

  void sendReceive(sx_register_t reg, uint8_t *send, uint8_t *receive, size_t data_bytes);
};

#endif // _SX1278
