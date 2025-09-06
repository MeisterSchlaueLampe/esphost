#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/log.h"

#ifdef USE_HOST
#include <libusb-1.0/libusb.h>
#include <vector>
#include <memory>
#include <mutex>
#endif

namespace esphome {
namespace ch341_i2c {

#ifdef USE_HOST

// CH341 USB Vendor/Product IDs
static const uint16_t CH341_VID = 0x1a86;
static const uint16_t CH341_PID = 0x5512;

// CH341 USB endpoints
static const uint8_t CH341_EP_OUT = 0x02;
static const uint8_t CH341_EP_IN = 0x82;

// CH341 I2C commands
static const uint8_t CH341A_CMD_I2C_STREAM = 0xAA;
static const uint8_t CH341A_CMD_I2C_STM_STA = 0x74;
static const uint8_t CH341A_CMD_I2C_STM_STO = 0x75;
static const uint8_t CH341A_CMD_I2C_STM_OUT = 0x80;
static const uint8_t CH341A_CMD_I2C_STM_IN = 0xC0;
static const uint8_t CH341A_CMD_I2C_STM_SET = 0x60;
static const uint8_t CH341A_CMD_I2C_STM_END = 0x00;

// I2C speed settings
enum CH341I2CSpeed {
  CH341A_I2C_20KHZ = 0,
  CH341A_I2C_100KHZ = 1,
  CH341A_I2C_400KHZ = 2,
  CH341A_I2C_750KHZ = 3
};

class CH341I2CBus : public i2c::I2CBus, public Component {
 public:
  CH341I2CBus() = default;
  ~CH341I2CBus();

  // Component lifecycle
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }
  
  // Configuration setters
  void set_frequency(uint32_t frequency);
  void set_scan(bool scan) { this->scan_ = scan; }

  // I2C Bus interface implementation
  i2c::ErrorCode write_readv( uint8_t address, const uint8_t *write_buffer, size_t write_count, uint8_t *read_buffer, size_t read_count ) override;

 protected:
  // USB device management
  bool init_usb_device_();
  void cleanup_usb_device_();
  
  // Low-level USB communication
  int usb_transfer_(uint8_t *out_buf, int out_len, uint8_t *in_buf, int in_len);
  
  // I2C speed configuration
  bool set_i2c_speed_(CH341I2CSpeed speed);
  CH341I2CSpeed frequency_to_speed_(uint32_t frequency);
  
  // Core I2C operations
  i2c::ErrorCode write_bytes_(uint8_t address, const uint8_t *data, size_t len, bool stop);
  i2c::ErrorCode read_bytes_(uint8_t address, uint8_t *data, size_t len, bool stop );
  i2c::ErrorCode write_read_bytes_(uint8_t address, const uint8_t *write_data, size_t write_len,
                                   uint8_t *read_data, size_t read_len);
  
  // Address scanning
  bool test_address_(uint8_t address);
  void scan_i2c_bus_();

 private:
  // Configuration
  uint32_t frequency_{100000};  // Default 100kHz
  bool scan_{true};
  
  // USB device state
  libusb_context *usb_context_{nullptr};
  libusb_device_handle *usb_handle_{nullptr};
  std::mutex usb_mutex_;  // Thread safety for USB operations
  
  // Current I2C speed setting
  CH341I2CSpeed current_speed_{CH341A_I2C_100KHZ};
  
  // Constants
  static const size_t MAX_TRANSACTION_SIZE = 25;  // Maximum I2C transaction size
  static const int USB_TIMEOUT_MS = 1000;         // USB transfer timeout
};

#else

// Stub implementation for non-host platforms
class CH341I2CBus : public Component {
 public:
  void setup() override {
    ESP_LOGE("ch341_i2c", "CH341 I2C component is only available on Linux host platform");
    this->mark_failed();
  }
  void set_frequency(uint32_t frequency) {}
  void set_scan(bool scan) {}
  float get_setup_priority() const override { return setup_priority::BUS; }
};

#endif  // USE_HOST

}  // namespace ch341_i2c
}  // namespace esphome

