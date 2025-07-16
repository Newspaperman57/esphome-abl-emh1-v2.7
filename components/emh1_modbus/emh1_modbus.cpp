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
}

// loop() receives incoming replies from ABL
void eMH1Modbus::loop() {
  const uint32_t now = millis();
  if (now - this->last_emh1_modbus_byte > 50) {
    this->rx_buffer.clear();
    this->last_emh1_modbus_byte = now;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_emh1_modbus_byte(byte)) {
      this->last_emh1_modbus_byte = now;
    } else {
      this->rx_buffer.clear();
    }
  }
}

// parse_emh1_modbus_byte will read a byte from RS485
// if it's the last byte of a transmission, it will
// proceed to parse the buffer, trying to make sense
// of the received package
//
bool eMH1Modbus::parse_emh1_modbus_byte(uint8_t byte) {
	size_t at = this->rx_buffer.size();
  this->rx_buffer.push_back(byte);
  if (byte != 0x0A) // 0x0A == LF == End of transmission
	  return true;
	this->rx_buffer.push_back('\0');
  char *frame = &this->rx_buffer[0];

	// check contents of first byte
	switch (frame[0]) {
	  case PROTOCOL_STARTBYTE:
   	  ESP_LOGD(TAG, "Ignore Master transmission: '%.*s'", at-1, frame);
		  return false;
		case '>':
    	ESP_LOGD(TAG, "Received client transmission: '%.*s'", at-1, frame);
			break;
		default:
      ESP_LOGW(TAG, "Unknown data of length %d", at+1);
		  return false;
	}

	// Get function-code
  uint16_t function = (frame[3] - '0')*10 + frame[4] - '0';

  // Check Device ID
	bool found = false;
	int deviceid = frame[1] - '0';
	for (auto *device : this->devices) {
	  if (device->address == deviceid) {
			device->on_emh1_modbus_data(function, frame+7);
			found = true;
		}
	}
	if (!found) {
	  ESP_LOGW(TAG, "Got eMH1 frame from unknown device address: %i", deviceid);
	}

	this->rx_buffer.clear();
  // ESP_LOGD(TAG, "Cleared buffer");
	return true;
}

void eMH1Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "eMH1Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin);
  this->check_uart_settings(38400);
}

float eMH1Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void eMH1Modbus::request_mode() {
  ESP_LOGW(TAG, "Get mode");
  this->send(MODULE_ADDRESS, "02"); // "Get mode" == 02
}

void eMH1Modbus::request_current() {
  ESP_LOGW(TAG, "Get current");
  this->send(MODULE_ADDRESS, "11"); // Get L_c
}

void eMH1Modbus::set_current(float amps) {
  ESP_LOGW(TAG, "Set current: %f", amps);
  // Amps to PWM-value = 6A == 10% == pwm 100
  int pwm = std::round(amps*100/6);
  // Int to string
  char pwmstr[4];
  this->int_to_str(pwm, pwmstr);

  this->send(MODULE_ADDRESS, "12", pwmstr, sizeof(pwmstr));
}

void eMH1Modbus::request_address() {
  ESP_LOGW(TAG, "Get address");
  this->send(MODULE_ADDRESS, "23");
}

void eMH1Modbus::stop_charging() {
  ESP_LOGW(TAG, "Request stop charging");
  this->send(MODULE_ADDRESS, "25");
}

void eMH1Modbus::request_max_current() {
  ESP_LOGW(TAG, "Get max current");
  this->send(MODULE_ADDRESS, "26"); // Get L_default
}

void eMH1Modbus::set_charging_enabled(bool enable) {
  ESP_LOGW(TAG, "Set charging enabled: %b", enable);
  if(!enable) {
  	this->send(MODULE_ADDRESS, "27"); // Set BreakCharge flag
  } else {
	  this->send(MODULE_ADDRESS, "28"); // Clear BreakCharge flag
  }
}

void eMH1Modbus::request_charging_enabled() {
  ESP_LOGW(TAG, "Request charging enabled");
  this->send(MODULE_ADDRESS, "29");
}

// convert single uint16_t value to 4-byte ascii-string
void eMH1Modbus::int_to_str(uint16_t val, char out[4]) {
	// Get each digit
	int cx = snprintf(out, 5, "%04d", val);
	if(cx != 4) {
	  ESP_LOGE(TAG, "Failed formatting integer to string. snprintf returned: %d", cx);
	}
}

void eMH1Modbus::send(char address, const char function[2]) {
	return this->send(address, function, nullptr, 0);
}
// Send a query or command to ABL eMH1 via RS485
void eMH1Modbus::send(char address, const char function[2], const char* data, size_t datasize) {
  // Send Modbus query as ASCII text
  ESP_LOGD(TAG, "TX -> '%c%c%.2s%.*s\\x0D\\x0A'", PROTOCOL_STARTBYTE, address, function, datasize, data);
  if (this->flow_control_pin != nullptr)
    this->flow_control_pin->digital_write(true);
	this->write(PROTOCOL_STARTBYTE);
	this->write(address);
	this->write(' ');
  this->write_array((const uint8_t *)function, 2);
	if(datasize > 0) {
		this->write(' ');
	  this->write_array((const uint8_t *)data, datasize);
	}
	this->write(0x0D);
	this->write(0x0A);
  this->flush();
  if (this->flow_control_pin != nullptr)
    this->flow_control_pin->digital_write(false);
}

}  // namespace emh1_modbus
}  // namespace esphome
