#ifdef USE_HOST

#include "i2c_bus_linux.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <cstring>

namespace esphome {
namespace i2c {

static const char *const TAG = "i2c.linux";

void LinuxI2CBus::setup() {
  ESP_LOGV(TAG, "Setting up Linux I2C Bus...");

  ErrorCode err = this->open_device_();
  if (err != ERROR_OK) {
    ESP_LOGE(TAG, "Failed to open I2C device %s: %s", this->device_.c_str(), strerror(errno));
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "Linux I2C Bus setup complete on device %s", this->device_.c_str());

  if (this->scan_) {
    ESP_LOGV(TAG, "Scanning bus for active devices");
    this->i2c_scan_();
  }
}

void LinuxI2CBus::dump_config() {
  ESP_LOGCONFIG(TAG, "I2C Bus (Linux):");
  ESP_LOGCONFIG(TAG, "  Device: %s", this->device_.c_str());
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed!");
  }
}

ErrorCode LinuxI2CBus::open_device_() {
  if (this->fd_ >= 0) {
    return ERROR_OK;  // Already open
  }

  this->fd_ = ::open(this->device_.c_str(), O_RDWR);
  if (this->fd_ < 0) {
    ESP_LOGE(TAG, "Failed to open I2C device %s: %s", this->device_.c_str(), strerror(errno));
    return ERROR_UNKNOWN;
  }

  return ERROR_OK;
}

void LinuxI2CBus::close_device_() {
  if (this->fd_ >= 0) {
    ::close(this->fd_);
    this->fd_ = -1;
  }
}

ErrorCode LinuxI2CBus::write_readv( uint8_t address, const uint8_t *write_buffer, size_t write_count, uint8_t *read_buffer, size_t read_count )
{
  if (this->fd_ < 0) {
    ErrorCode err = this->open_device_();
    if (err != ERROR_OK) {
      return err;
    }
  }

  if ( write_count > 0 && read_count == 0 )
  {
    // Set slave address
    if (::ioctl(this->fd_, I2C_SLAVE, address) < 0) {
      ESP_LOGW(TAG, "Failed to set I2C slave address 0x%02X: %s", address, strerror(errno));
      return ERROR_UNKNOWN;
    }

    ssize_t result = ::write(this->fd_, write_buffer, write_count );

    if (result < 0) {
      ESP_LOGW(TAG, "I2C write failed for address 0x%02X: %s", address, strerror(errno));
      return ERROR_UNKNOWN;
    }

    if ((size_t) result != write_count ) {
      ESP_LOGW(TAG, "I2C write incomplete for address 0x%02X: expected %zu bytes, wrote %zd", address, write_count, result);
      return ERROR_UNKNOWN;
    }

    return ERROR_OK;
  }
  else if ( read_count > 0 && write_count == 0 )
  {
    // Set slave address
    if (::ioctl(this->fd_, I2C_SLAVE, address) < 0) {
      ESP_LOGW(TAG, "Failed to set I2C slave address 0x%02X: %s", address, strerror(errno));
      return ERROR_UNKNOWN;
    }
    ssize_t result = ::read( this->fd_, read_buffer, read_count );

    if (result < 0) {
      ESP_LOGW(TAG, "I2C read failed for address 0x%02X: %s", address, strerror(errno));
      return ERROR_UNKNOWN;
    }

    if ((size_t) result != read_count ) {
      ESP_LOGW(TAG, "I2C read incomplete for address 0x%02X: expected %zu bytes, got %zd", address, read_count, result);
      return ERROR_UNKNOWN;
    }

    return ERROR_OK;
  }
  else
  {
    const size_t max_count = 0xffff;
    if ( write_count > max_count || read_count > max_count )
    {
      ESP_LOGW(TAG, "I2C write read max count exceeded: wcnt %zu, rcnt %zu, max cnt %zu", write_count, read_count, max_count);
      return ERROR_UNKNOWN;
    }

    struct i2c_msg messages[2]
      { { .addr = address, .flags = 0, .len = (unsigned short int) write_count, .buf = (uint8_t *) write_buffer },
        { .addr = address, .flags = I2C_M_RD, .len = (unsigned short int) read_count, .buf = read_buffer } };

    struct i2c_rdwr_ioctl_data transfer
      { .msgs = messages, .nmsgs = ( read_count > 0 ) ? 2 : 1 };

    if( ::ioctl( this->fd_, I2C_RDWR, &transfer ) < 0 ) {
      ESP_LOGW(TAG, "I2C write read failed for address 0x%02X: %s", address, strerror(errno));
      return ERROR_UNKNOWN;
    }

    return ERROR_OK;
  }
}

}  // namespace i2c
}  // namespace esphome

#endif  // USE_HOST

