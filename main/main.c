#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "BMP180";

#define BMP180_ADDRESS 0x77         // BMP180のI2Cアドレス
#define I2C_MASTER_SCL_IO 21        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 22        /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define ACK_CHECK_EN 0x1            // I2C master will check ack from slave
#define CHIP_ID_REG_ADDR 0xD0       // チップIDレジスタのアドレス
#define CHIP_ID_EXPECTED 0x55       // 期待されるチップID
#define BMP180_ULTRA_HIGH_RES 3
#define BMP180_CALIB_DATA_START 0xAA // 校正データの開始レジスタアドレス
#define BMP180_CALIB_DATA_SIZE 22    // 校正データのサイズ（バイト）
#define BMP180_READ_PRESSURE_CMD 0x34
#define READ_PRESSURE_ADDR BMP180_READ_PRESSURE_CMD + (oversampling << 6)
#define CONTROL_REGISTER_ADDR 0xF4 // Control register Control register value (register address 0xF4)
#define BMP180_DATA_TO_READ 0xF6   // Read results here
#define READ_TEMPERATURE_ADDR 0x2E // Request temperature measurement

static int16_t AC1;
static int16_t AC2;
static int16_t AC3;
static uint16_t AC4;
static uint16_t AC5;
static uint16_t AC6;
static int16_t B1;
static int16_t B2;
static int16_t MB;
static int16_t MC;
static int16_t MD;
static uint8_t oversampling = BMP180_ULTRA_HIGH_RES;

static esp_err_t bmp180_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t bmp180_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  if (size > 1)
  {
    i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * Read uncompensated temperature value
 */
static esp_err_t read_temperature_registers(i2c_port_t i2c_num, uint8_t reg, uint8_t *data_rd)
{
  esp_err_t ret;
  ret = bmp180_master_write_slave(i2c_num, &reg, 1);
  // read reg 0xF6(MSB), 0xF7(LSB)
  ret = bmp180_master_read_slave(i2c_num, data_rd, 2);
  return ret;
}

static esp_err_t bmp180_read_uncompensated_temperature_value(int16_t *ut)
{
  // write 0x2E into reg 0xF4
  uint8_t data_wr[2] = {CONTROL_REGISTER_ADDR, READ_TEMPERATURE_ADDR};
  esp_err_t ret = bmp180_master_write_slave(I2C_NUM_0, data_wr, 2);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Write [0x%02x] = 0x%02x failed", CONTROL_REGISTER_ADDR, READ_TEMPERATURE_ADDR);
  }

  if (ret == ESP_OK)
  {
    // wait 4.5ms at least
    vTaskDelay((TickType_t)(10 / portTICK_PERIOD_MS));
    // read reg 0xF6(MSB), 0xF7(LSB)
    uint8_t data_rd[2] = {0};
    ret = read_temperature_registers(I2C_NUM_0, BMP180_DATA_TO_READ, data_rd);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Read temperature registers failed");
    }
    uint8_t msb = data_rd[0];
    uint8_t lsb = data_rd[1];
    *ut = (int16_t)((msb << 8) | lsb);
  }

  return ret;
}

/**
 * Read uncompensated pressure value
 */
static esp_err_t read_pressure_registers(i2c_port_t i2c_num, uint8_t reg, uint8_t *data_rd)
{
  esp_err_t ret;
  ret = bmp180_master_write_slave(i2c_num, &reg, 1);
  // read reg 0xF6(MSB), 0xF7(LSB), 0xF8(XLSB)
  ret = bmp180_master_read_slave(i2c_num, data_rd, 3);
  return ret;
}

static esp_err_t bmp180_read_uncompensated_pressure_value(uint32_t *up)
{
  // write 0x34 + (oss << 6) into reg 0xF4
  esp_err_t ret;
  uint8_t data_wr[] = {CONTROL_REGISTER_ADDR, READ_PRESSURE_ADDR};
  ret = bmp180_master_write_slave(I2C_NUM_0, data_wr, 2);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Write [0x%02x] = 0x%02x failed", CONTROL_REGISTER_ADDR, READ_PRESSURE_ADDR);
  }

  if (ret == ESP_OK)
  {
    // wait conversion time pressure
    vTaskDelay((TickType_t)(30 / portTICK_PERIOD_MS));

    // read reg 0xF6(MSB), 0xF7(LSB), 0xF8(XLSB)
    uint8_t data_rd[3] = {0};
    ret = read_pressure_registers(I2C_NUM_0, BMP180_DATA_TO_READ, data_rd);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Read pressure registers failed");
    }
    uint8_t msb = data_rd[0];
    uint8_t lsb = data_rd[1];
    uint8_t xlsb = data_rd[2];
    // UP = (MSB<<16 + LSB<<8 + XLSB) >> (8-oss)
    *up = (uint32_t)((msb << 16) | (lsb << 8) | xlsb) >> (8 - oversampling);
  }

  return ret;
}

/**
 * Read calibration data
 */
static esp_err_t bmp180_read_coefficients(i2c_port_t i2c_num, uint8_t *coefficients)
{
  esp_err_t ret;
  uint8_t reg_addr = BMP180_CALIB_DATA_START;

  ret = bmp180_master_write_slave(i2c_num, &reg_addr, 1);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Write calibration start address failed");
    return ret;
  }

  ret = bmp180_master_read_slave(i2c_num, coefficients, BMP180_CALIB_DATA_SIZE);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Read calibration data failed");
    return ret;
  }

  return ESP_OK;
}

static void parse_bmp180_coefficients(uint8_t *coefficients)
{
  AC1 = (coefficients[0] << 8) | coefficients[1];
  AC2 = (coefficients[2] << 8) | coefficients[3];
  AC3 = (coefficients[4] << 8) | coefficients[5];
  AC4 = (coefficients[6] << 8) | coefficients[7];
  AC5 = (coefficients[8] << 8) | coefficients[9];
  AC6 = (coefficients[10] << 8) | coefficients[11];
  B1 = (coefficients[12] << 8) | coefficients[13];
  B2 = (coefficients[14] << 8) | coefficients[15];
  MB = (coefficients[16] << 8) | coefficients[17];
  MC = (coefficients[18] << 8) | coefficients[19];
  MD = (coefficients[20] << 8) | coefficients[21];

  ESP_LOGI(TAG, "Coefficients:\nAC1=%d\nAC2=%d\nAC3=%d\nAC4=%u\nAC5=%u\nAC6=%u\nB1=%d\nB2=%d\nMB=%d\nMC=%d\nMD=%d",
           AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD);
}

static esp_err_t read_chip_id(uint8_t *chip_id)
{
  esp_err_t ret;
  uint8_t reg_addr = CHIP_ID_REG_ADDR;

  ret = bmp180_master_write_slave(I2C_NUM_0, &reg_addr, 1);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to write chip ID register address");
    return ret;
  }

  ret = bmp180_master_read_slave(I2C_NUM_0, chip_id, 1);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to read chip ID");
    return ret;
  }

  if (*chip_id != CHIP_ID_EXPECTED)
  {
    ESP_LOGE(TAG, "Chip ID is: 0x%02X, NOT BMP180!", *chip_id);
    return ESP_ERR_INVALID_RESPONSE;
  }

  ESP_LOGI(TAG, "Read Chip ID: 0x%02X", *chip_id);
  return ESP_OK;
}

static esp_err_t initialize_i2c_master(void)
{
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t ret;
  ret = i2c_param_config(I2C_NUM_0, &conf);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set I2C parameters.");
    return ret;
  }
  ret = i2c_driver_install(I2C_NUM_0, conf.mode,
                           I2C_MASTER_TX_BUF_DISABLE,
                           I2C_MASTER_RX_BUF_DISABLE, 0);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to install I2C driver.");
    return ret;
  }

  ESP_LOGI(TAG, "I2C master driver has been installed.");
  return ESP_OK;
}

static void compute(int16_t ut, uint32_t up)
{
  int32_t X1, X2, X3, B3, B5, B6, p;
  uint32_t B4, B7;

  // Calculate true temperature
  X1 = (ut - AC6) * AC5 >> 15;
  X2 = ((int32_t)MC << 11) / (X1 + MD);
  B5 = X1 + X2;
  int T = (B5 + 8) >> 4; // Temperature in 0.1C units

  ESP_LOGI(TAG, "Measured temperature: %.1f C", T / 10.0);

  // Calculate true pressure
  B6 = B5 - 4000;
  X1 = (B2 * (B6 * B6 >> 12)) >> 11;
  X2 = AC2 * B6 >> 11;
  X3 = X1 + X2;
  B3 = (((((int32_t)AC1) * 4 + X3) << 3) + 2) / 4;
  X1 = AC3 * B6 >> 13;
  X2 = (B1 * (B6 * B6 >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) / 4;
  B4 = AC4 * (uint32_t)(X3 + 32768) >> 15;
  B7 = ((uint32_t)(up - B3) * (50000 >> 3));
  if (B7 < 0x80000000)
  {
    p = (B7 * 2) / B4;
  }
  else
  {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4); // Pressure in Pa

  ESP_LOGI(TAG, "Measured air pressure: %.2f hPa", p / 100.0);
}

void app_main(void)
{
  uint8_t chip_id = 0;
  uint8_t coefficients[BMP180_CALIB_DATA_SIZE];

  ESP_ERROR_CHECK(initialize_i2c_master());
  ESP_ERROR_CHECK(read_chip_id(&chip_id));
  ESP_ERROR_CHECK(bmp180_read_coefficients(I2C_NUM_0, coefficients));
  parse_bmp180_coefficients(coefficients);

  int count = 0;
  while (count < 15)
  {
    int16_t ut;
    uint32_t up;
    ESP_ERROR_CHECK(bmp180_read_uncompensated_temperature_value(&ut));
    ESP_ERROR_CHECK(bmp180_read_uncompensated_pressure_value(&up));
    compute(ut, up);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    count++;
  }

  ESP_ERROR_CHECK(i2c_driver_delete(I2C_NUM_0));
  ESP_LOGI(TAG, "I2C de-initialized successfully");
}