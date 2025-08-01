#include "abl_emh1.h"
#include "esphome/core/log.h"

namespace esphome {
namespace abl_emh1 {

static const char *const TAG = "abl_emh1";


// static const uint8_t STATE_SIZE = 8;
// static const char *const STATE[STATE_SIZE] = {
// 	"EV not detected",  											             // A
// 	"EV detected", 							                           // B1.1
// 	"EV detected, Waiting for bBreakCharge-flag cleared",  // B1.2
// 	"Waiting for EV to request charge",                    // B2
// 	"EV requesting charging",		                           // C
// 	"EV stopped charging",		                             // B'
// 	"Error",												                       // E
//   "EV not detected / Production test",                   // A'
// };

int ABLeMH1::str_to_int(const char* buffer, size_t buffersize) {
  int value = 0;
  for (int i = 0; i < buffersize; ++i) {
    int digit = buffer[i] - '0';
    if(digit < 0 || digit > 9) { // This covers null-terminator and non-digit characters
      break;
    }
    value = value * 10 + digit;
  }
  return value;
}

void ABLeMH1::on_emh1_modbus_data(uint16_t function, const char* data) {
  switch (function) {
    case 2: { // Mode
      int mode = this->str_to_int(data, 4);
      ESP_LOGW(TAG, "Mode is %d", mode);
      this->publish_state_(this->mode_sensor, mode);
      break;
    }
    case 11: { // Get l_C
      int L_c = this->str_to_int(data, 4); // PWM scaled by 0.1%, each 1% == 0.06A
      float amps = L_c*0.06;
      if(L_c == 999) {
        amps = 0;
      }
      ESP_LOGW(TAG, "Current is %f", amps);
      this->publish_state_(this->current_sensor, amps);
      break;
    }
    case 12: { // Current set confirmed
      ESP_LOGW(TAG, "Confirmed: Current set");
      break;
    }
    case 23: { // Get address
      int address = this->str_to_int(data, 4);
      ESP_LOGW(TAG, "Address is: %i", address);
      break;
    }
    case 25: { // Charging stopped
      ESP_LOGW(TAG, "Confirmed: Charging stopped");
      break;
    }
    case 26: { // Get l_default
      int L_c = this->str_to_int(data, 4); // PWM scaled by 0.1%, each 0.1% == 0.6A
      float amps = L_c*0.06;
      ESP_LOGW(TAG, "Max-current is %f", amps);
      this->publish_state_(this->max_current_sensor, amps);
      break;
    }
    case 27: { // Charging disabled confirmed
      ESP_LOGW(TAG, "Confirmed: Charging disabled");
      this->publish_state_(this->charging_enabled_sensor, 1);
      break;
    }
    case 28: { // Charging enabled confirmed
      ESP_LOGW(TAG, "Confirmed: Charging enabled");
      this->publish_state_(this->charging_enabled_sensor, 0);
      break;
    }
    case 29: { // Charging enabled
      bool enabled = this->str_to_int(data, 4); // When breakcharge is == 1, charger is enabled
      ESP_LOGW(TAG, "Charging enabled: %d", enabled);
      this->publish_state_(this->charging_enabled_sensor, enabled); 
      break;
    }
    case 30: { // Jump to A'
      ESP_LOGW(TAG, "Confirmed: Jump to A");
      break;
    }
    case 31: { // Restart
      ESP_LOGW(TAG, "Confirmed: Restart");
      break;
    }
    default:
      // ESP_LOGW(TAG, "Unhandled ABL frame: %s", format_hex_pretty(&data.front(), data.size()).c_str());
      ESP_LOGW(TAG, "Unhandled ABL function: %d", function);
      return;
  }
  this->no_response_count = 0;
}

void ABLeMH1::publish_device_offline_() {
  this->publish_state_(this->mode_sensor, -1);
  this->publish_state_(this->current_sensor, NAN);
  this->publish_state_(this->max_current_sensor, NAN);
  this->publish_state_(this->charging_enabled_sensor, NAN);
}

void ABLeMH1::update() {
  ESP_LOGD(TAG, "ABLeMH1::update");
  if (this->config_age >= CONFIG_AGE_THRESHOLD) {
    ESP_LOGD(TAG, "Get device address");
	  this->parent->request_address();
    this->config_age = 0;
	  return;
	}

	if (this->no_response_count >= REDISCOVERY_THRESHOLD) {
    this->publish_device_offline_();
    ESP_LOGD(TAG, "The device is or was offline. Broadcasting discovery for address configuration...");
    this->parent->request_address();
    // this->query_device_info(this->address);
    // Try to query live data on next update again. The device doesn't
    // respond to the discovery broadcast if it's already configured.
    this->no_response_count = 0;
  } else {
    ESP_LOGD(TAG, "Device online, requesting data");
    this->parent->request_mode();
    this->parent->request_current();
    this->parent->request_max_current();
    this->parent->request_charging_enabled();
    this->no_response_count++;
  }
}

void ABLeMH1::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void ABLeMH1::dump_config() {
  ESP_LOGCONFIG(TAG, "ABLeMH1:");
  ESP_LOGCONFIG(TAG, "  Address: %d", this->address);
  LOG_SENSOR("", "Mode", this->mode_sensor);
  LOG_SENSOR("", "Charging enabled", this->charging_enabled_sensor);
  LOG_SENSOR("", "Current", this->current_sensor);
  LOG_SENSOR("", "Max current", this->max_current_sensor);
}

}  // namespace abl_emh1
}  // namespace esphome
