#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <vector>
#include <queue>

namespace esphome {
namespace emh1_modbus {

enum class CommState {
  IDLE,
  WAITING_FOR_REPLY
};

struct Command {
  char address;
  std::string function;
  std::string data;
};

struct eMH1MessageT {
  uint8_t DeviceId;
	uint8_t FunctionCode;
	uint16_t Destination;
	uint16_t DataLength;
	uint8_t LRC;
	uint8_t WriteBytes;
	uint8_t Data[100];
};

class eMH1ModbusDevice;

class eMH1Modbus : public uart::UARTDevice, public Component {
 public:
  eMH1Modbus() = default;

  void setup() override;
  void loop() override;

  void dump_config() override;

  void register_device(eMH1ModbusDevice *device) { this->devices.push_back(device); }
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin = flow_control_pin; }

  float get_setup_priority() const override;

  void send(char, const char*);
  void send(char, const char*, const char*, size_t);
  void request_mode();
  void request_current();
  void set_current(float amps);
  void request_address();
  void stop_charging();
  void request_max_current();
  void set_charging_enabled(bool enable);
  void request_charging_enabled();
  void queue_command(char address, const char function[2], const char* data, size_t datasize);
  void queue_command(char address, const char function[2], const char* data);
  void queue_command(char address, const char function[2]);
  void int_to_str(uint16_t val, char out[4]);

 protected:

  void send_immediate(char address, const char function[2], const char* data, size_t datasize);

  bool parse_emh1_modbus_byte(uint8_t byte);
  GPIOPin *flow_control_pin{nullptr};

  eMH1MessageT emh1_tx_message;
  std::vector<char> rx_buffer;
  uint32_t last_emh1_modbus_byte{0};
  std::vector<eMH1ModbusDevice *> devices;

  Command current_command_;
  size_t tx_echo_ignore_ = 0;
  std::queue<Command> command_queue_;
  CommState state_;
  uint32_t timeout_start_;
};

class eMH1ModbusDevice {
 public:
  void set_parent(eMH1Modbus *parent) { this->parent = parent; }
  void set_address(uint8_t address) { this->address = address; }
	virtual void on_emh1_modbus_data(uint16_t function, const char* data) = 0;
  
 protected:
  friend eMH1Modbus;

  eMH1Modbus *parent = nullptr;
  uint8_t address;
};

}  // namespace emh1_modbus
}  // namespace esphome
