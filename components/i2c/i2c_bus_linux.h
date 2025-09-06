#pragma once

#ifdef USE_HOST

#include "esphome/core/component.h"
#include "i2c_bus.h"
#include <string>

namespace esphome {
namespace i2c {

class LinuxI2CBus : public I2CBus, public Component {
 public:
  void set_linux_device(const std::string &device) { this->device_ = device; }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  ErrorCode write_readv(uint8_t address, const uint8_t *write_buffer, size_t write_count, uint8_t *read_buffer, size_t read_count) override;

 protected:
  std::string device_{"/dev/i2c-1"};  // Default I2C device
  int fd_{-1};  // File descriptor for I2C device

  ErrorCode open_device_();
  void close_device_();
};

}  // namespace i2c
}  // namespace esphome

#endif  // USE_HOST

