#include "emh1_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome.h"

namespace esphome {
namespace emh1_modbus {

static const char *const TAG = "emh1_modbus";
static const char PROTOCOL_STARTBYTE = '!';
static const char MODULE_ADDRESS = '0';

void eMH1Modbus::setup() {
  if (this->flow_control_pin != nullptr) {
    this->flow_control_pin->setup();
  }
  this->state_ = CommState::IDLE;
  this->timeout_start_ = 0;
  this->tx_echo_ignore_ = 0;
}

void eMH1Modbus::loop() {
  const uint32_t now = millis();

  if (this->state_ == CommState::WAITING_FOR_REPLY) {
    if ((now - this->timeout_start_) > 100) {
      ESP_LOGW(TAG, "Timeout waiting for response");
      this->rx_buffer.clear();
      this->state_ = CommState::IDLE;
    }
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);

    if (this->tx_echo_ignore_ > 0) {
      this->tx_echo_ignore_--;
      continue;
    }

    if (this->parse_emh1_modbus_byte(byte)) {
      ESP_LOGD(TAG, "Got response in %d ms", now - this->timeout_start_);
      this->state_ = CommState::IDLE;
    }
  }

  if (this->state_ == CommState::IDLE && !this->command_queue_.empty()) {
    Command cmd = this->command_queue_.front();
    this->command_queue_.pop();
    this->send_immediate(cmd.address, cmd.function.c_str(), cmd.data.c_str(), cmd.data.length());
  }
}

bool eMH1Modbus::parse_emh1_modbus_byte(uint8_t byte) {
  // Returns true when a packet was parsed
  size_t at = this->rx_buffer.size();
  this->rx_buffer.push_back(byte);

  if (byte != 0x0A)
    return false;

  this->rx_buffer.push_back('\0');
  char *frame = &this->rx_buffer[0];

  switch (frame[0]) {
    case PROTOCOL_STARTBYTE:
      ESP_LOGW(TAG, "Ignore Master transmission: '%.*s'", at - 1, frame);
      return false;
    case '>':
      ESP_LOGD(TAG, "Received client transmission: '%.*s'", at - 1, frame);
      break;
    default:
      ESP_LOGW(TAG, "Unknown data of length %d", at + 1);
      return false;
  }

  uint16_t function = (frame[3] - '0') * 10 + frame[4] - '0';
  bool found = false;
  int deviceid = frame[1] - '0';

  for (auto *device : this->devices) {
    if (device->address == deviceid) {
      device->on_emh1_modbus_data(function, frame + 7);
      found = true;
    }
  }

  if (!found) {
    ESP_LOGW(TAG, "Got eMH1 frame from unknown device address: %i", deviceid);
  }

  this->rx_buffer.clear();
  return true;
}

void eMH1Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "eMH1Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin);
  this->check_uart_settings(38400);
}

float eMH1Modbus::get_setup_priority() const {
  return setup_priority::BUS - 1.0f;
}

void eMH1Modbus::request_mode() {
  ESP_LOGW(TAG, "Get mode");
  this->queue_command(MODULE_ADDRESS, "02");
}

void eMH1Modbus::request_current() {
  ESP_LOGW(TAG, "Get current");
  this->queue_command(MODULE_ADDRESS, "11");
}

void eMH1Modbus::set_current(float amps) {
  ESP_LOGW(TAG, "Set current: %f", amps);
  int pwm = std::round(amps * 100 / 6);
  if(pwm < 80) {
    pwm = 999; // Charging not allowed
  }
  char pwmstr[4];
  this->int_to_str(pwm, pwmstr);
  this->queue_command(MODULE_ADDRESS, "12", pwmstr);
}

void eMH1Modbus::request_address() {
  ESP_LOGW(TAG, "Get address");
  this->queue_command(MODULE_ADDRESS, "23");
}

void eMH1Modbus::stop_charging() {
  ESP_LOGW(TAG, "Request stop charging");
  this->queue_command(MODULE_ADDRESS, "25");
}

void eMH1Modbus::request_max_current() {
  ESP_LOGW(TAG, "Get max current");
  this->queue_command(MODULE_ADDRESS, "26");
}

void eMH1Modbus::set_charging_enabled(bool enable) {
  ESP_LOGW(TAG, "Set charging enabled: %b", enable);
  this->queue_command(MODULE_ADDRESS, enable ? "28" : "27"); // Clear : Set BreakCharge flag
}

void eMH1Modbus::request_charging_enabled() {
  ESP_LOGW(TAG, "Request charging enabled");
  this->queue_command(MODULE_ADDRESS, "29");
}

void eMH1Modbus::int_to_str(uint16_t val, char out[4]) {
  int cx = snprintf(out, 5, "%04d", val);
  if (cx != 4) {
    ESP_LOGE(TAG, "Failed formatting integer to string. snprintf returned: %d", cx);
  }
}

void eMH1Modbus::send(char address, const char function[2]) {
  this->queue_command(address, function);
}

void eMH1Modbus::send(char address, const char function[2], const char* data, size_t datasize) {
  this->queue_command(address, function, data, datasize);
}

void eMH1Modbus::send_immediate(char address, const char function[2], const char* data, size_t datasize) {
  ESP_LOGD(TAG, "TX -> '%c%c %.2s %.*s\\x0D\\x0A'", PROTOCOL_STARTBYTE, address, function, datasize, data);

  if (this->flow_control_pin != nullptr)
    this->flow_control_pin->digital_write(true);

  std::vector<uint8_t> tx_buf;
  tx_buf.push_back(PROTOCOL_STARTBYTE);
  tx_buf.push_back(address);
  tx_buf.push_back(' ');
  tx_buf.insert(tx_buf.end(), function, function + 2);
  if (datasize > 0) {
    tx_buf.push_back(' ');
    tx_buf.insert(tx_buf.end(), data, data + datasize);
  }
  tx_buf.push_back(0x0D);
  tx_buf.push_back(0x0A);

  this->write_array(tx_buf);
  this->flush();

  if (this->flow_control_pin != nullptr)
    this->flow_control_pin->digital_write(false);

  this->tx_echo_ignore_ = tx_buf.size();
  this->timeout_start_ = millis();
  this->state_ = CommState::WAITING_FOR_REPLY;
}

void eMH1Modbus::queue_command(char address, const char function[2], const char* data, size_t datasize) {
  Command cmd;
  cmd.address = address;
  cmd.function = std::string(function, 2);
  if (data != nullptr && datasize > 0) {
    cmd.data = std::string(data, datasize);
  }
  this->command_queue_.push(cmd);
}

void eMH1Modbus::queue_command(char address, const char function[2], const char* data) {
  this->queue_command(address, function, data, 4);
}

void eMH1Modbus::queue_command(char address, const char function[2]) {
  this->queue_command(address, function, nullptr, 0);
}

}  // namespace emh1_modbus
}  // namespace esphome
