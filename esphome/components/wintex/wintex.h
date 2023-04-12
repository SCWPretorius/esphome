#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/application.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

namespace esphome {
namespace wintex {

static std::vector<uint8_t> NO_DATA = {};

enum class WintexCommandType : uint8_t {
  NONE = 'Q',
  ARM = 'A',
  RESET = 'C',
  DISARM = 'D',
  HANGUP = 'H',
  WRITE_CONFIGURATION = 'I',
  KEYPAD = 'K',
  READ_CONFIGURATION = 'O',
  READ_VOLATILE = 'R',
  PART_ARM = 'S',
  COMMIT = 'U',
  WRITE_VOLATILE = 'W',
  SESSION = 'Z',
};

enum class WintexResponseType : uint8_t {
  SESSION = 'Z',
  READ_CONFIGURATION = 'I',
  READ_VOLATILE = 'W',
  ACK = 0x06,
  HANGUP = 0x0F,
};

enum class WintexInitState : uint8_t {
  UNAUTH = 0x00,
  LOGIN_FAILED,
  AUTH,
  ZONE_QUERY,
  INIT_DONE,
  IDLE,
  MESSAGE_SENT,
};

std::vector<uint8_t> read_payload(uint32_t address, uint8_t length);
std::vector<uint8_t> write_payload(uint32_t address, std::vector<uint8_t> payload);

struct WintexResponse {
  WintexResponseType answer;
  std::vector<uint8_t> data;
  WintexResponse(const std::vector<uint8_t> &rx_message)
      : answer{static_cast<WintexResponseType>(rx_message[1])}, data{rx_message.begin() + 2, rx_message.end() - 1} {}
  WintexResponse() {}
};

// struct WintexCommand {
//   WintexCommandType command;
//   std::vector<uint8_t> payload;
//   WintexCommand(WintexCommandType command, std::vector<uint8_t> &payload) : command{command},
//   payload{payload.begin(), payload.end()} {} WintexCommand(WintexCommandType command) : command{command}, payload{}
//   {} WintexCommand() : command{WintexCommandType::NONE} {} ~WintexCommand() { ESP_LOGV("WintexCommand",
//   "WintexCommand Destructor called for %c : %s", command, format_hex_pretty(payload).c_str()); }
// };

struct AsyncWintexCommand;

using ResponseCallback = std::function<optional<AsyncWintexCommand>(WintexResponse)>;

struct AsyncWintexCommand {
  WintexCommandType command;
  const std::vector<uint8_t> payload;
  ResponseCallback callback;
  AsyncWintexCommand(WintexCommandType command_, std::vector<uint8_t> &&payload_, ResponseCallback callback_)
      : command{command_}, payload{std::move(payload_)}, callback{callback_} {};
  AsyncWintexCommand(WintexCommandType command_, ResponseCallback callback_)
      : command{command_}, payload{}, callback{callback_} {};
  AsyncWintexCommand() : command{WintexCommandType::NONE}, payload{} {
    ESP_LOGE("AsyncWintexCommand", "AsyncWintexCommand Null Constructor called");
  }
  ~AsyncWintexCommand() {
    ESP_LOGE("AsyncWintexCommand", "AsyncWintexCommand Destructor called for %c : %s", static_cast<uint8_t>(command),
             format_hex_pretty(payload).c_str());
  }
};

class Wintex;

class WintexSensorBase {
  friend class Wintex;

 public:
  WintexSensorBase(uint32_t address, uint8_t length, uint8_t offset) {
    address_ = address;
    length_ = length;
    offset_ = offset;
  }
  uint32_t get_address() { return this->address_; }
  uint8_t get_length() { return this->length_; }

 protected:
  virtual void update_state(const uint8_t *memory) = 0;
  uint32_t address_{0};
  uint8_t length_{0};
  uint8_t offset_{0};
};

class WintexBinarySensor : public WintexSensorBase, public binary_sensor::BinarySensor {
 public:
  WintexBinarySensor(uint32_t address, uint8_t length, uint8_t offset, uint8_t mask)
      : WintexSensorBase(address, length, offset) {
    mask_ = mask;
  }

 protected:
  void update_state(const uint8_t *memory) override { this->publish_state(memory[offset_] & mask_); };
  uint8_t mask_;
};

class WintexVoltageSensor : public WintexSensorBase, public sensor::Sensor {
 public:
  WintexVoltageSensor(uint32_t address, uint8_t length, uint8_t offset) : WintexSensorBase(address, length, offset) {}

 protected:
  void update_state(const uint8_t *memory) { this->publish_state(memory[offset_] / 255 * 17.93); };
};

class WintexButton : public button::Button {
  friend class Wintex;

 public:
  WintexButton(Wintex *wintex, WintexCommandType command, std::vector<uint8_t> payload, bool commit_required)
      : wintex_{wintex},
        command_{[&]() {
          std::vector<uint8_t> payload_vector{payload.begin(), payload.end()};
          return AsyncWintexCommand(command, payload_vector,
                                    [this](WintexResponse response) { return command_callback(response); });
        }()},
        commit_required_{commit_required} {}
  WintexButton(const WintexButton &button)
      : wintex_{button.wintex_},
        command_{button.command_.command, button.command_.payload,
                 [this](WintexResponse response) { return command_callback(response); }},
        commit_required_{button.commit_required_} {}
  WintexButton(WintexButton &&button)
      : wintex_{button.wintex_},
        command_{button.command_.command, button.command_.payload,
                 [this](WintexResponse response) { return command_callback(response); }},
        commit_required_{button.commit_required_} {}
  virtual ~WintexButton() = default;

 protected:
  void press_action();

 private:
  const char *TAG = "wintex_button";
  Wintex *wintex_;
  AsyncWintexCommand command_;
  bool commit_required_;
  ResponseCallback commit_callback_ = [this](WintexResponse response) -> optional<AsyncWintexCommand> {
    if (response.answer != WintexResponseType::ACK) {
      ESP_LOGE(TAG, "Unexpected response to command: %d", (uint8_t) response.answer);
    }
    return {};
  };
  optional<AsyncWintexCommand> command_callback(WintexResponse response) {
    AsyncWintexCommand commit = AsyncWintexCommand(WintexCommandType::COMMIT, this->commit_callback_);
    if (response.answer != WintexResponseType::ACK) {
      ESP_LOGE(TAG, "Unexpected response to command: %d", (uint8_t) response.answer);
    } else {
      if (commit_required_) {
        ESP_LOGE("wintex_button", "COMMIT required, heap free = %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        return commit;
      }
    }
    ESP_LOGE("wintex_button", "No COMMIT required");
    return {};
  };
};

class WintexSwitch : public WintexSensorBase, public switch_::Switch {
 public:
  WintexSwitch(uint32_t address, uint8_t length, uint8_t offset, uint8_t mask, WintexButton on, WintexButton off)
      : WintexSensorBase(address, length, offset), mask_{mask}, on_{on}, off_{off} {}
  void write_state(bool state);
  void set_name(const std::string &name) {
    Switch::set_name(name);
    on_.set_name(name + " ON");
    off_.set_name(name + " OFF");
  };

 protected:
  void update_state(const uint8_t *memory) override { this->publish_state(memory[offset_] & mask_); };
  uint8_t mask_;
  WintexButton on_, off_;
};

class WintexZoneBypassSwitch : public WintexSwitch {
 public:
  WintexZoneBypassSwitch(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset)
      : WintexSwitch(
            address, length, offset, 0x20,
            WintexButton(wintex, WintexCommandType::WRITE_VOLATILE, write_payload(address + offset, {0xa0}), true),
            WintexButton(wintex, WintexCommandType::WRITE_VOLATILE, write_payload(address + offset, {0x80}), true)) {}
};

class WintexPartitionArmSwitch : public WintexSwitch {
 public:
  WintexPartitionArmSwitch(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset, uint8_t partition_mask)
      : WintexSwitch(address, length, offset, partition_mask,
                     WintexButton(wintex, WintexCommandType::ARM, {partition_mask}, false),
                     WintexButton(wintex, WintexCommandType::DISARM, {partition_mask}, false)) {}
};

class WintexPartitionResetButton : public WintexButton {
 public:
  WintexPartitionResetButton(Wintex *wintex, uint8_t partition_mask)
      : WintexButton(wintex, WintexCommandType::RESET, {partition_mask}, false) {}
};

/**
 * @brief WintexZone represents an entire zone, with a number of binary_sensors
 * We expose the zone's status directly as a binary_sensor using a callback
 * that mirrors the zone_status sensor to the zone itself.
 *
 */
class WintexZone : public binary_sensor::BinarySensor {
  friend class Wintex;

 public:
  WintexZone(uint16_t zone) { zone_ = zone; }
  // called after the hub has queried the various base addresses,
  // as well as the zone names, from the panel
  // Can be overridden by YAML configuration.
  WintexBinarySensor *status, *tamper, *test, *alarmed, *auto_bypassed;
  WintexSwitch *bypass;

 protected:
  void setup(Wintex *wintex, uint32_t zone_base_address, uint16_t zone_group_size, std::string zone_name);

 private:
  uint16_t zone_;
  bool setup_{false};
};

class WintexPartition : public binary_sensor::BinarySensor {
  friend class Wintex;

 public:
  WintexPartition(uint8_t partition) { partition_ = partition; }
  WintexBinarySensor *panic, *burglar, *bell, *strobe, *entry, *exit, *armed, *stay_armed, *ready, *bypass,
      *reset_required, *ack_required, *confirmed_alarm, *away_armed, *armed_alarm, *arm_failed, *all_armed;
  WintexSwitch *arm;
  WintexButton *reset;

 protected:
  void setup(Wintex *wintex, uint32_t partition_base_address, uint8_t partition_group_size, std::string partition_name);

 private:
  uint8_t partition_;
  bool setup_{false};
};

class Wintex : public Component, public uart::UARTDevice {
  friend class WintexButton;
  friend class WintexPartition;
  friend class WintexZone;

 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_udl(std::string udl) {
    this->login_ = AsyncWintexCommand(WintexCommandType::SESSION, std::vector<uint8_t>{udl.begin(), udl.end()},
                                      [this](WintexResponse response) { return this->handle_login_(response); });
  }
  void register_zone(WintexZone *zone);
  void register_partition(WintexPartition *partition);

 protected:
  void queue_command_(AsyncWintexCommand command);
  void register_sensor(WintexSensorBase *sensor);

 private:
  void process_response_();
  // optional<WintexResponse> send_command_blocking_(WintexCommand command);
  void send_command_now_(AsyncWintexCommand command);
  optional<WintexResponse> parse_response_();
  void setup_zones_();
  void setup_partitions_();
  void update_sensors_();

  optional<AsyncWintexCommand> handle_login_(WintexResponse response);
  optional<AsyncWintexCommand> handle_heartbeat_(WintexResponse response);
  optional<AsyncWintexCommand> handle_sensors_(WintexResponse response);

  void process_command_queue_();

  WintexInitState init_state_ = WintexInitState::UNAUTH;
  uint32_t last_command_timestamp_{0};
  std::string product_ = "";
  AsyncWintexCommand login_ = AsyncWintexCommand(
      WintexCommandType::SESSION, NO_DATA, [this](WintexResponse response) { return this->handle_login_(response); });
  AsyncWintexCommand heartbeat_ =
      AsyncWintexCommand(WintexCommandType::SESSION, NO_DATA,
                         [this](WintexResponse response) { return this->handle_heartbeat_(response); });
  std::vector<WintexSensorBase *> sensors_;
  std::vector<WintexZone *> zones_;
  std::vector<WintexPartition *> partitions_;
  std::vector<uint8_t> rx_message_;
  std::vector<AsyncWintexCommand> command_queue_;
  uint16_t current_sensor_{0};
  optional<AsyncWintexCommand> current_command_{};
  optional<WintexResponse> sensor_response_;
  // WintexButton *test_button_;
  // std::vector<uint8_t> fake_response_ = {
  //   0x0b, 0x5a, 0x05, 0x01, 0x00, 0x07, 0x09, 0x04, 0x07, 0x01, 0x78,
  //   0x13, 0x5a, 0x50, 0x72, 0x65, 0x6d, 0x69, 0x65, 0x72, 0x20, 0x38, 0x33, 0x32, 0x20, 0x56, 0x34, 0x2e, 0x30, 0xf9,
  //   0x03, 0x06, 0xf6, };
};

}  // namespace wintex
}  // namespace esphome
