#include "ch341_i2c.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_HOST

namespace esphome {
namespace ch341_i2c {

static const char *const TAG = "ch341_i2c";

CH341I2CBus::~CH341I2CBus() {
  this->cleanup_usb_device_();
}

void CH341I2CBus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CH341 I2C Bus...");

  if (!this->init_usb_device_()) {
    ESP_LOGE(TAG, "Failed to initialize CH341 USB device");
    this->mark_failed();
    return;
  }

  // Set I2C speed
  ESP_LOGCONFIG(TAG, "Setting I2C Bus clock frequency %d", this->frequency_ );
  CH341I2CSpeed speed = this->frequency_to_speed_(this->frequency_);
  if (!this->set_i2c_speed_(speed)) {
    ESP_LOGE(TAG, "Failed to set I2C speed");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "CH341 I2C Bus initialized successfully");

  // Perform bus scan if requested
  if (this->scan_) {
    this->scan_i2c_bus_();
  }
}

void CH341I2CBus::dump_config() {
  ESP_LOGCONFIG(TAG, "CH341 I2C Bus:");
  ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Scan: %s", YESNO(this->scan_));
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Status: FAILED");
  }
}

void CH341I2CBus::set_frequency(uint32_t frequency) {
  this->frequency_ = frequency;
}

CH341I2CSpeed CH341I2CBus::frequency_to_speed_(uint32_t frequency) {
  if (frequency <= 20000) return CH341A_I2C_20KHZ;
  if (frequency <= 100000) return CH341A_I2C_100KHZ;
  if (frequency <= 400000) return CH341A_I2C_400KHZ;
  return CH341A_I2C_750KHZ;
}

bool CH341I2CBus::init_usb_device_() {
  std::lock_guard<std::mutex> lock(this->usb_mutex_);
  
  // Initialize libusb
  int ret = libusb_init(&this->usb_context_);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to initialize libusb: %s", libusb_error_name(ret));
    return false;
  }

  // Find CH341 device
  libusb_device **device_list;
  ssize_t device_count = libusb_get_device_list(this->usb_context_, &device_list);
  if (device_count < 0) {
    ESP_LOGE(TAG, "Failed to get USB device list");
    libusb_exit(this->usb_context_);
    this->usb_context_ = nullptr;
    return false;
  }

  bool found = false;
  for (ssize_t i = 0; i < device_count; i++) {
    libusb_device *device = device_list[i];
    struct libusb_device_descriptor desc;
    
    ret = libusb_get_device_descriptor(device, &desc);
    if (ret != 0) continue;
    
    if (desc.idVendor == CH341_VID && desc.idProduct == CH341_PID) {
      ret = libusb_open(device, &this->usb_handle_);
      if (ret == 0) {
        ESP_LOGD(TAG, "Found CH341 device");
        found = true;
        break;
      } else {
        ESP_LOGW(TAG, "Found CH341 device but failed to open: %s", libusb_error_name(ret));
      }
    }
  }

  libusb_free_device_list(device_list, 1);

  if (!found) {
    ESP_LOGE(TAG, "CH341 device not found");
    libusb_exit(this->usb_context_);
    this->usb_context_ = nullptr;
    return false;
  }

  // Detach kernel driver if active
  if (libusb_kernel_driver_active(this->usb_handle_, 0) == 1) {
    ESP_LOGD(TAG, "Detaching kernel driver");
    ret = libusb_detach_kernel_driver(this->usb_handle_, 0);
    if (ret != 0) {
      ESP_LOGW(TAG, "Could not detach kernel driver: %s", libusb_error_name(ret));
    }
  }

  // Claim interface
  ret = libusb_claim_interface(this->usb_handle_, 0);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to claim interface: %s", libusb_error_name(ret));
    libusb_close(this->usb_handle_);
    this->usb_handle_ = nullptr;
    libusb_exit(this->usb_context_);
    this->usb_context_ = nullptr;
    return false;
  }

  return true;
}

void CH341I2CBus::cleanup_usb_device_() {
  std::lock_guard<std::mutex> lock(this->usb_mutex_);
  
  if (this->usb_handle_) {
    libusb_release_interface(this->usb_handle_, 0);
    libusb_close(this->usb_handle_);
    this->usb_handle_ = nullptr;
  }
  
  if (this->usb_context_) {
    libusb_exit(this->usb_context_);
    this->usb_context_ = nullptr;
  }
}

int CH341I2CBus::usb_transfer_(uint8_t *out_buf, int out_len, uint8_t *in_buf, int in_len) {
  std::lock_guard<std::mutex> lock(this->usb_mutex_);
  
  if (!this->usb_handle_) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  int actual_length;
  int ret;

  // Send command
  if (out_len > 0) {
    ret = libusb_bulk_transfer(this->usb_handle_, CH341_EP_OUT, out_buf, out_len, 
                               &actual_length, USB_TIMEOUT_MS);
    if (ret != 0) {
      ESP_LOGE(TAG, "USB out transfer failed: %s", libusb_error_name(ret));
      return ret;
    }
  }

  // Receive response
  if (in_len > 0) {
    ret = libusb_bulk_transfer(this->usb_handle_, CH341_EP_IN, in_buf, in_len, 
                               &actual_length, USB_TIMEOUT_MS);
    if (ret != 0) {
      ESP_LOGE(TAG, "USB in transfer failed: %s", libusb_error_name(ret));
      return ret;
    }
    return actual_length;
  }

  return 0;
}

bool CH341I2CBus::set_i2c_speed_(CH341I2CSpeed speed) {
  uint8_t cmd_buf[3];
  
  cmd_buf[0] = CH341A_CMD_I2C_STREAM;
  cmd_buf[1] = CH341A_CMD_I2C_STM_SET | speed;
  cmd_buf[2] = CH341A_CMD_I2C_STM_END;
  
  int ret = this->usb_transfer_(cmd_buf, 3, nullptr, 0);
  if (ret == 0) {
    this->current_speed_ = speed;
    return true;
  }
  return false;
}

bool CH341I2CBus::test_address_(uint8_t address) {
  uint8_t cmd_buf[8];
  uint8_t in_buf[1];
  
  // Simple address probe: START + ADDR + STOP
  cmd_buf[0] = CH341A_CMD_I2C_STREAM;
  cmd_buf[1] = CH341A_CMD_I2C_STM_STA;      // START
  cmd_buf[2] = CH341A_CMD_I2C_STM_OUT;      // Output address
  cmd_buf[3] = (address << 1) | 0;          // Write address
  cmd_buf[4] = CH341A_CMD_I2C_STM_STO;      // STOP
  cmd_buf[5] = CH341A_CMD_I2C_STM_END;      // End of stream
  
  int ret = this->usb_transfer_(cmd_buf, 6, in_buf, 1);
  if (ret < 1) return false;
  
  // Check ACK (bit 7 = 0 means ACK, bit 7 = 1 means NACK)
  return (in_buf[0] & 0x80) == 0;
}

void CH341I2CBus::scan_i2c_bus_() {
  ESP_LOGI(TAG, "Scanning I2C bus...");
  ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  
  int found = 0;
  for (int row = 0; row < 8; row++) {
    std::string line = str_sprintf("%02x: ", row * 16);
    
    for (int col = 0; col < 16; col++) {
      uint8_t address = row * 16 + col;
      
      // Skip reserved addresses
      if (address < 0x08 || address > 0x77) {
        line += "   ";
        continue;
      }
      
      if (this->test_address_(address)) {
        line += str_sprintf("%02x ", address);
        found++;
      } else {
        line += "-- ";
      }
    }
    ESP_LOGI(TAG, "%s", line.c_str());
  }
  
  ESP_LOGI(TAG, "Scan complete. Found %d device(s).", found);
}

i2c::ErrorCode CH341I2CBus::write_bytes_(uint8_t address, const uint8_t *data, size_t len, bool stop) {
  if ( len > MAX_TRANSACTION_SIZE ) {
    ESP_LOGE(TAG, "Write transaction too large: %zu bytes (max %zu)", len, MAX_TRANSACTION_SIZE);
    return i2c::ERROR_INVALID_ARGUMENT;
  }

  uint8_t cmd_buf[64];
  uint8_t in_buf[32];
  int cmd_len = 0;
  int expected_acks = 1; // Address ACK only

  // Build command sequence
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STREAM;
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STA;
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_OUT | 0;
  cmd_buf[cmd_len++] = (address << 1) | 0;   // Device Write Address

  // Add data bytes
  if ( len > 0 ) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_OUT | len;
  for (size_t i = 0; i < len; i++) cmd_buf[cmd_len++] = data[i];

  // Add STOP if requested
  if ( stop ) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STO;

  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_END;

  // Execute transaction and read ACK bits
  int ret = this->usb_transfer_(cmd_buf, cmd_len, in_buf, expected_acks);
  if (ret < 0) {
    ESP_LOGE(TAG, "Write USB transfer failed");
    return i2c::ERROR_UNKNOWN;
  }

  // Check ACKs - first bit should be 0 (ACK) for each byte
  for (int i = 0; i < expected_acks && i < ret; i++) {
    if (in_buf[i] & 0x80) {  // NACK received
      if (i == 0) {
        ESP_LOGD(TAG, "Address NACK for 0x%02X", address);
        return i2c::ERROR_NOT_ACKNOWLEDGED;
      } else {
        ESP_LOGD(TAG, "Data NACK at byte %d for address 0x%02X", i - 1, address);
        return i2c::ERROR_NOT_ACKNOWLEDGED;
      }
    }
  }

  return i2c::ERROR_OK;
}

i2c::ErrorCode CH341I2CBus::read_bytes_(uint8_t address, uint8_t *data, size_t len, bool stop ) {
  if (len > MAX_TRANSACTION_SIZE) {
    ESP_LOGE(TAG, "Read transaction too large: %zu bytes (max %zu)", len, MAX_TRANSACTION_SIZE);
    return i2c::ERROR_INVALID_ARGUMENT;
  }

  uint8_t cmd_buf[64];
  uint8_t in_buf[64];
  int cmd_len = 0;

  // Build command sequence
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STREAM;
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STA;
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_OUT | 1;
  cmd_buf[cmd_len++] = (address << 1) | 1; // Device Read Address

  // Add read bytes (ACK all but last)
  for (size_t i = 1; i < len; i++) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_IN | 1;
  if ( len > 0 ) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_IN;

  // Add STOP if requested
  if ( stop ) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STO;

  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_END;

  // Execute transaction - expect 1 address ACK + len data bytes
  int ret = this->usb_transfer_(cmd_buf, cmd_len, in_buf, len+0);
  if (ret < 0) {
    ESP_LOGE(TAG, "Read USB transfer failed");
    return i2c::ERROR_UNKNOWN;
  }
/*
  // Check address ACK
  if (ret < 1 || (in_buf[0] & 0x80)) {
    ESP_LOGD(TAG, "Address NACK for read from 0x%02X", address);
    return i2c::ERROR_NOT_ACKNOWLEDGED;
  }
*/
  // Copy data bytes
  int bytes_read = ret - 0;  // Subtract address ACK
  if (bytes_read > (int)len) bytes_read = len;

  if (bytes_read < (int)len) {
    ESP_LOGE(TAG, "Read incomplete: got %d bytes, expected %zu", bytes_read, len);
    return i2c::ERROR_UNKNOWN;
  }

  for (int i = 0; i < len; i++) data[i] = in_buf[i+0];

  return i2c::ERROR_OK;
}

i2c::ErrorCode CH341I2CBus::write_read_bytes_(uint8_t address, const uint8_t *write_data, size_t write_len,
                                              uint8_t *read_data, size_t read_len) {
  if (write_len + read_len > MAX_TRANSACTION_SIZE) {
    ESP_LOGE(TAG, "Write-read transaction too large: write=%zu, read=%zu", write_len, read_len);
    return i2c::ERROR_INVALID_ARGUMENT;
  }

  uint8_t cmd_buf[64];
  uint8_t in_buf[64];
  int cmd_len = 0;
  int expected_response = 1 + 1 + read_len;  // addr_ack + addr_ack + read_data

  // Build command sequence
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STREAM;

  // Write phase
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STA;
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_OUT;
  cmd_buf[cmd_len++] = (address << 1) | 0;      // Device Write Address

  // Write data bytes
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_OUT | write_len;
  for ( size_t i = 0; i < write_len; i++ ) cmd_buf[cmd_len++] = write_data[i];

  // Read phase
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STA;  // RESTART
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_OUT | 0;
  cmd_buf[cmd_len++] = (address << 1) | 1;      // Device Read Address

  // Read data bytes
  for (size_t i = 1; i < read_len; i++) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_IN | 1;
  if ( read_len > 0 ) cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_IN;

  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_STO;  // STOP
  cmd_buf[cmd_len++] = CH341A_CMD_I2C_STM_END;

  // Execute transaction
  int ret = this->usb_transfer_(cmd_buf, cmd_len, in_buf, expected_response);
  if (ret < 0) {
    ESP_LOGE(TAG, "Write-read USB transfer failed");
    return i2c::ERROR_UNKNOWN;
  }

  // Check write address ACK
  if (ret < 1 || (in_buf[0] & 0x80)) {
    ESP_LOGD(TAG, "Write address NACK for 0x%02X", address);
    return i2c::ERROR_NOT_ACKNOWLEDGED;
  }

  // Check read address ACK
  int read_addr_ack_pos = 1;
  if (ret <= read_addr_ack_pos || (in_buf[read_addr_ack_pos] & 0x80)) {
    ESP_LOGD(TAG, "Read address NACK for 0x%02X", address);
    return i2c::ERROR_NOT_ACKNOWLEDGED;
  }

  // Copy read data
  int data_start = read_addr_ack_pos + 1;
  int bytes_read = ret - data_start;
  if (bytes_read > (int)read_len) bytes_read = read_len;

  if (bytes_read < (int)read_len) {
    ESP_LOGE(TAG, "Write-read incomplete: got %d read bytes, expected %zu", bytes_read, read_len);
    return i2c::ERROR_UNKNOWN;
  }

  for ( int i = 0; i < bytes_read; i++ ) read_data[i] = in_buf[data_start + i];

  return i2c::ERROR_OK;
}

i2c::ErrorCode CH341I2CBus::write_readv( uint8_t address, const uint8_t *write_buffer, size_t write_count, uint8_t *read_buffer, size_t read_count )
{
  if ( read_count == 0 )
    return this->write_bytes_( address, write_buffer, write_count, true );
  else if ( write_count == 0 )
    return this->read_bytes_( address, read_buffer, read_count, true );
  else
    return this->write_read_bytes_( address, write_buffer, write_count, read_buffer, read_count );
}

}  // namespace ch341_i2c
}  // namespace esphome

#endif  // USE_HOST

