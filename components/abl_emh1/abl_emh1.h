#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/emh1_modbus/emh1_modbus.h"

namespace esphome {
namespace abl_emh1 {

static const uint8_t REDISCOVERY_THRESHOLD = 5;
static const uint16_t CONFIG_AGE_THRESHOLD = 10;

// class ABLeMH1;
// ABLeMH1 *my_abl_emh1;

class ABLeMH1: public PollingComponent, public emh1_modbus::eMH1ModbusDevice {
 public:
  void set_mode_sensor(sensor::Sensor *mode_sensor) { this->mode_sensor = mode_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { this->current_sensor = current_sensor; }
  void set_max_current_sensor(sensor::Sensor *max_current_sensor) { this->max_current_sensor = max_current_sensor; }
  void set_charging_enabled_sensor(sensor::Sensor *charging_enabled_sensor) { this->charging_enabled_sensor = charging_enabled_sensor; }

  void update() override;
  void on_emh1_modbus_data(uint16_t function, const char* data) override;
  void dump_config() override;

 protected:
  sensor::Sensor *mode_sensor;
  sensor::Sensor *current_sensor;
  sensor::Sensor *max_current_sensor;
  sensor::Sensor *charging_enabled_sensor;

  uint8_t no_response_count = REDISCOVERY_THRESHOLD;
  uint16_t config_age = CONFIG_AGE_THRESHOLD;

  int str_to_int(const char* buffer, size_t buffersize);
  void publish_state_(sensor::Sensor *sensor, float value);
  void publish_device_offline_();
};


}  // namespace abl_emh1
}  // namespace esphome
