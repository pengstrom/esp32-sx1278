#if !defined(_SX1278)
#define _SX1278

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "./registers.h"
#include "./constants.h"
#include <sx1278spi.h>

class Sx1278 : public Sx1278Spi
{
public:
  typedef void (*dio_cb_t)(void *arg, uint8_t *buffer, size_t size);

  static const uint8_t RX_START = 0;
  static const uint8_t TX_START = 0x80;

  struct config_t
  {
    gpio_num_t dio0_pin = GPIO_NUM_NC;
    gpio_num_t rst_pin = GPIO_NUM_NC;
    spi_config_t spi;
  };

  enum class crc_status_t
  {
    CRC_VALID,
    CRC_INVALID,
    CRC_NA
  };

  enum class dio0_t : uint8_t
  {
    RX_DONE = 0,
    TX_DONE,
    CAD_DONE
  };

  Sx1278(config_t cfg);
  ~Sx1278();

  void reset();

  crc_status_t checkCrc();
  void correctFrequencyError();

  uint8_t rxSize();
  size_t readRx(uint8_t *data);
  void writeTx(uint8_t *data, uint8_t len);

  void modeSleep();
  void modeStandby();
  void modeTx();
  void modeRxSingle();
  void modeRxCont();
  void modeCad();

  void transmit();
  void receive();
  void setMode(mode_t mode);
  void setDio0Mapping(dio0_t mp);

  void registerDio0Callback(dio_cb_t cb, void *arg);

  void resetRx();

private:
  config_t _cfg;

  struct settings_t
  {
    reg_modem_cfg_1_t modem_cfg_1;
    reg_modem_cfg_2_t modem_cfg_2;
    reg_modem_cfg_3_t modem_cfg_3;
    reg_op_mode_t op_mode;
    uint32_t frf;
  };
  settings_t _reg_settings;
  dio_cb_t _dio_cb = nullptr;
  void *_dio_cb_arg = NULL;

  static void rx_callback(void *arg);
  void handleRx();

  void syncSettings();

  void waitForReady();
  void waitForModeReady();

  void setupPins();
};

#endif // _SX1278
