#include "wintex.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wintex {

static const char *TAG = "wintex";
static const int COMMAND_DELAY = 1000;

std::vector<uint8_t> read_payload(uint32_t address, uint8_t length) {
  auto payload = std::vector<uint8_t>();
  payload.push_back((address >> 16) & 0xff);
  payload.push_back((address >> 8) & 0xff);
  payload.push_back((address) &0xff);
  payload.push_back(length);
  return payload;
};
std::vector<uint8_t> write_payload(uint32_t address, std::vector<uint8_t> data) {
  assert(data.size() <= 0xff);
  auto payload = read_payload(address, data.size() & 0xff);
  for (auto it = data.cbegin(); it != data.cend(); ++it) {
    payload.push_back(*it);
  }
  return payload;
};

void WintexButton::press_action() { this->wintex_->queue_command_(this->command_); }

void WintexSwitch::write_state(bool state) {
  if (state) {
    this->on_.press();
  } else {
    this->off_.press();
  }
  publish_state(state);  // FIXME: Check if this results in toggling?
}

void WintexPartition::setup(Wintex *wintex, uint32_t partition_base_address, uint8_t partition_group_size,
                            std::string partition_name) {
  if (setup_)
    return;
  setup_ = true;
  if (get_name() == "")
    set_name(partition_name.c_str());
  std::string name = get_name();
  App.register_binary_sensor(this);

  uint8_t partition = 1 << (this->partition_ - 1);
  panic = new WintexBinarySensor(partition_base_address, partition_group_size, 0x00, partition);
  panic->set_name((name + " panic").c_str());
  panic->set_disabled_by_default(true);
  wintex->register_sensor(panic);
  App.register_binary_sensor(panic);
  burglar = new WintexBinarySensor(partition_base_address, partition_group_size, 0x01, partition);
  burglar->set_name((name + " burglar").c_str());
  burglar->set_disabled_by_default(true);
  wintex->register_sensor(burglar);
  App.register_binary_sensor(burglar);
  bell = new WintexBinarySensor(partition_base_address, partition_group_size, 0x06, partition);
  bell->set_name((name + " bell").c_str());
  bell->set_disabled_by_default(true);
  wintex->register_sensor(bell);
  App.register_binary_sensor(bell);
  strobe = new WintexBinarySensor(partition_base_address, partition_group_size, 0x06, partition << 4);
  strobe->set_name((name + " strobe").c_str());
  strobe->set_disabled_by_default(true);
  wintex->register_sensor(strobe);
  App.register_binary_sensor(strobe);
  entry = new WintexBinarySensor(partition_base_address, partition_group_size, 0x07, partition);
  entry->set_name((name + " entry").c_str());
  entry->set_disabled_by_default(true);
  wintex->register_sensor(entry);
  App.register_binary_sensor(entry);
  exit = new WintexBinarySensor(partition_base_address, partition_group_size, 0x07, partition << 4);
  exit->set_name((name + " exit").c_str());
  exit->set_disabled_by_default(true);
  wintex->register_sensor(exit);
  App.register_binary_sensor(exit);
  armed = new WintexBinarySensor(partition_base_address, partition_group_size, 0x08, partition);
  armed->set_name((name + " armed").c_str());
  wintex->register_sensor(armed);
  App.register_binary_sensor(armed);
  ready = new WintexBinarySensor(partition_base_address, partition_group_size, 0x09, partition);
  ready->set_name((name + " ready").c_str());
  ready->set_disabled_by_default(true);
  wintex->register_sensor(ready);
  App.register_binary_sensor(ready);
  bypass = new WintexBinarySensor(partition_base_address, partition_group_size, 0x09, partition << 4);
  bypass->set_name((name + " bypass").c_str());
  bypass->set_disabled_by_default(true);
  wintex->register_sensor(bypass);
  App.register_binary_sensor(bypass);
  reset_required = new WintexBinarySensor(partition_base_address, partition_group_size, 0x0e, partition);
  reset_required->set_name((name + " reset required").c_str());
  wintex->register_sensor(reset_required);
  App.register_binary_sensor(reset_required);
  ack_required = new WintexBinarySensor(partition_base_address, partition_group_size, 0x0e, partition << 4);
  ack_required->set_name((name + " ack required").c_str());
  ack_required->set_disabled_by_default(true);
  wintex->register_sensor(ack_required);
  App.register_binary_sensor(ack_required);
  confirmed_alarm = new WintexBinarySensor(partition_base_address, partition_group_size, 0x0f, partition);
  confirmed_alarm->set_name((name + " confirmed alarm").c_str());
  wintex->register_sensor(confirmed_alarm);
  App.register_binary_sensor(confirmed_alarm);
  armed_alarm = new WintexBinarySensor(partition_base_address, partition_group_size, 0x11, partition << 4);
  armed_alarm->set_name((name + " armed/alarm").c_str());
  armed_alarm->set_disabled_by_default(true);
  wintex->register_sensor(armed_alarm);
  App.register_binary_sensor(armed_alarm);
  arm_failed = new WintexBinarySensor(partition_base_address, partition_group_size, 0x12, partition);
  arm_failed->set_name((name + " arm failed").c_str());
  wintex->register_sensor(arm_failed);
  App.register_binary_sensor(arm_failed);
  all_armed = new WintexBinarySensor(partition_base_address, partition_group_size, 0x12, partition << 4);
  all_armed->set_name((name + " all armed").c_str());
  all_armed->set_disabled_by_default(true);
  wintex->register_sensor(all_armed);
  App.register_binary_sensor(all_armed);

  away_arm = new WintexPartitionArmSwitch(wintex, partition_base_address, partition_group_size, 0x10, partition);
  away_arm->set_name((name + " away arm").c_str());
  wintex->register_sensor(away_arm);
  App.register_switch(away_arm);
  stay_arm = new WintexPartitionStayArmSwitch(wintex, partition_base_address, partition_group_size, 0x08, partition);
  stay_arm->set_name((name + " stay arm").c_str());
  wintex->register_sensor(stay_arm);
  App.register_switch(stay_arm);

  reset = new WintexPartitionResetButton(wintex, partition);
  reset->set_name((name + " reset").c_str());
  App.register_button(reset);

  away_arm->add_on_state_callback([this](bool state) { this->publish_state(away_arm->state || stay_arm->state); });
  stay_arm->add_on_state_callback([this](bool state) { this->publish_state(away_arm->state || stay_arm->state); });
}

void WintexZone::setup(Wintex *wintex, uint32_t zone_base_address, uint16_t zone_group_size, std::string zone_name) {
  if (setup_)
    return;
  setup_ = true;
  if (get_name() == "")
    set_name((zone_name).c_str());
  std::string name = get_name();
  App.register_binary_sensor(this);
  uint16_t zone = this->zone_ - 1;
  status = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x01);
  status->set_name((name + " status").c_str());
  status->set_internal(true);
  status->add_on_state_callback([this](bool state) { this->publish_state(state); });
  wintex->register_sensor(status);
  App.register_binary_sensor(status);
  tamper = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x02);
  tamper->set_name((name + " tamper").c_str());
  tamper->set_disabled_by_default(true);
  wintex->register_sensor(tamper);
  App.register_binary_sensor(tamper);
  test = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x08);
  test->set_name((name + " test").c_str());
  test->set_disabled_by_default(true);
  wintex->register_sensor(test);
  App.register_binary_sensor(test);
  alarmed = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x10);
  alarmed->set_name((name + " alarmed").c_str());
  alarmed->set_disabled_by_default(true);
  wintex->register_sensor(alarmed);
  App.register_binary_sensor(alarmed);
  bypass = new WintexZoneBypassSwitch(wintex, zone_base_address, zone_group_size, zone);
  bypass->set_name((name + " bypassed").c_str());
  wintex->register_sensor(bypass);
  App.register_switch(bypass);
  auto_bypassed = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x40);
  auto_bypassed->set_name((name + " auto bypassed").c_str());
  auto_bypassed->set_disabled_by_default(true);
  wintex->register_sensor(auto_bypassed);
  App.register_binary_sensor(auto_bypassed);
  // Should only enable this once we are sorting the sensors by base address
  // faulty = new WintexBinarySensor(zone_base_address + 0x20, zone_group_size, zone_, 0x02);
  // faulty->set_name(name + " faulty");
  // wintex->register_sensor(faulty);
  // App.register_binary_sensor(faulty);
}

void Wintex::setup() {
  last_command_timestamp_ = millis();
  // system_voltage = new WintexVoltageSensor();

  setup_partitions_();
  setup_zones_();
  /*
  optional<WintexResponse> response = send_command_blocking_(this->login_);
  if (!response.has_value()) {
    return;
  }
  std::vector<uint16_t> addresses = get_panel_addresses();
  std::vector<std::string> zones = get_zone_names(0, 0x20);
  setup_zones(0, 0x20, addresses[n], zones)
  */
  test_button_ = new WintexButton(this, this->query_addresses_);
  test_button_->set_name("Test");
  App.register_button(test_button_);
  system_voltage = new WintexVoltageSensor(0x0DB7 + 0x10, 4, 0);
  system_voltage->set_name("System Voltage");
  register_sensor(system_voltage);
  App.register_sensor(system_voltage);
  battery_voltage = new WintexVoltageSensor(0x0DB7 + 0x10, 4, 2);
  battery_voltage->set_name("Battery Voltage");
  register_sensor(battery_voltage);
  App.register_sensor(battery_voltage);
  // make sure sensors are properly sorted by address, to reduce traffic
  std::sort(sensors_.begin(), sensors_.end());
}

/*
optional<WintexResponse> Wintex::send_command_blocking_(WintexCommand command) {
  send_command_now_(command.cmd, command.data);
  uint32_t start = millis();
  uint32_t elapsed=0;
  optional<WintexResponse> response;
  do {
    uint8_t c;
    read_byte(&c);
    rx_message_.push_back(c);
    response = this->parse_response_();
    elapsed = millis() - start;
  } while (!response.has_value() || elapsed < 1000);
  if (!response.has_value()) {
    ESP_LOGE(TAG, "Timeout waiting for response from panel");
    this->mark_failed();
  }
  return response;
}
*/
optional<AsyncWintexCommand> Wintex::handle_addresses_(WintexResponse response) {
  ESP_LOGD(TAG, "Addresses are %s", format_hex_pretty(response.data).c_str());
  return {};
}

optional<AsyncWintexCommand> Wintex::handle_login_(WintexResponse response) {
  if (response.answer == WintexResponseType::SESSION) {
    if (response.data.size() == 16) {
      // check it is a valid string made up of printable characters
      bool valid = true;
      for (int i = 0; i < response.data.size(); i++) {
        if (!std::isprint(response.data[i])) {
          valid = false;
          break;
        }
      }
      if (valid) {
        product_ = std::string(response.data.begin(), response.data.end());
        ESP_LOGV(TAG, "Successful authentication, product [%s]", product_.c_str());
        init_state_ = WintexInitState::AUTH;
        this->update_sensors_();
        return {};
      }
    }
  }
  ESP_LOGE(TAG, "Authentication failed");
  product_ = R"({"p":"INVALID"})";
  this->mark_failed();
  return {};
}

optional<AsyncWintexCommand> Wintex::handle_heartbeat_(WintexResponse response) {
  // if (response.answer != WintexResponseType::SESSION
  // && response.data.size() != 0x09) {
  // ESP_LOGE(TAG, "Unexpected heartbeat response: %d", (uint8_t) response.answer);
  return login_;
  // }
  // return {};
}

void Wintex::loop() {
  if (available()) {
    while (available()) {
      uint8_t c;
      read_byte(&c);
      rx_message_.push_back(c);
    }
    // ESP_LOGD(TAG, "Calling process_response_ with %d bytes", rx_message_.size());
    process_response_();
  }

  if (millis() - last_command_timestamp_ > 10000) {
    if (command_queue_.size() == 0) {
      queue_command_(heartbeat_);
    } else {
      current_command_ = {};
      ESP_LOGE(TAG, "No response from panel, trying again");
      // ESP_LOGE(TAG, "No response from panel, marking integration as failed");
      // this->mark_failed();
    }
  }
  if (init_state_ == WintexInitState::AUTH &&
      (command_queue_.size() == 0 && millis() - last_command_timestamp_ > 2000)) {
    update_sensors_();
  }
  process_command_queue_();
}

void Wintex::dump_config() {
  ESP_LOGCONFIG(TAG, "Wintex:");
  if (init_state_ != WintexInitState::INIT_DONE) {
    ESP_LOGCONFIG(TAG, "  Configuration will be reported when setup is complete. Current init_state: %u",
                  static_cast<uint8_t>(init_state_));
    ESP_LOGCONFIG(TAG, "  If no further output is received, confirm that this is a supported Wintex device.");
    return;
  }
  ESP_LOGCONFIG(TAG, "  Product: '%s'", product_.c_str());
}

optional<WintexResponse> Wintex::parse_response_() {
  size_t length = rx_message_[0];

  if (rx_message_.size() < length)
    return {};

  ESP_LOGVV(TAG, "Received message DATA=[%s]", format_hex_pretty(&rx_message_[0], length).c_str());

  // validate checksum
  // Byte LEN: CHECKSUM - sum of all bytes (including header) ^ 0xFF
  uint8_t rx_checksum = rx_message_[length - 1];
  uint8_t calc_checksum = 0;
  for (uint8_t i = 0; i < length - 1; i++)
    calc_checksum += rx_message_[i];
  calc_checksum ^= 0xFF;

  if (rx_checksum != calc_checksum) {
    ESP_LOGW(TAG, "Wintex Received invalid message checksum DATA=[%s] Checksum: %02X!=%02X",
             format_hex_pretty(&rx_message_[0], length).c_str(), rx_checksum, calc_checksum);
    rx_message_.clear();
    return {};
  }

  // valid message
  WintexResponse response = WintexResponse(rx_message_);
  rx_message_.clear();
  ESP_LOGD(TAG, "Received Response: type=0x%02X DATA=[%s]", static_cast<uint8_t>(response.answer),
           format_hex_pretty(response.data).c_str());
  return response;
}

void Wintex::process_response_() {
  optional<WintexResponse> response = this->parse_response_();
  if (response.has_value() && this->current_command_.has_value()) {
    ResponseCallback callback = this->current_command_.value().callback;
    optional<AsyncWintexCommand> next_command = callback(response.value());
    if (next_command.has_value()) {
      this->current_command_ = {};
      this->send_command_now_(next_command.value());
    } else {
      this->current_command_ = {};
    }
  }
}

void Wintex::update_sensors_() {
  // ESP_LOGD(TAG, "Updating sensors");
  // this->current_sensor_ = 0;
  if (sensors_.size() == 0)
    return;
  auto sensor = sensors_[current_sensor_];
  auto payload = read_payload(sensor->get_address(), sensor->get_length());
  auto read_sensor = AsyncWintexCommand(WintexCommandType::READ_VOLATILE, payload,
                                        [this](WintexResponse response) { return this->handle_sensors_(response); });
  queue_command_(read_sensor);
}

optional<AsyncWintexCommand> Wintex::handle_sensors_(WintexResponse response) {
  uint32_t address = (response.data[0] << 16) | (response.data[1] << 8) | response.data[2];
  uint8_t length = response.data[3];
  const uint8_t *data = &response.data[4];
  if (sensors_.size() == 0) {
    ESP_LOGV(TAG, "No sensors to update");
    return {};
  }
  while (current_sensor_ < sensors_.size()) {
    auto sensor = sensors_[current_sensor_];
    if (address == sensor->get_address() && length >= sensor->get_length()) {
      sensor->update_state(data, length);
      current_sensor_++;
    } else {
      ESP_LOGV(TAG, "Changing requested address at sensor %d", current_sensor_);
      break;
    }
  }
  current_sensor_ = current_sensor_ % sensors_.size();
  return {};
}

void Wintex::send_command_now_(AsyncWintexCommand command) {
  this->current_command_ = command;

  ESP_LOGD(TAG, "Sending Wintex: COMMAND=%c PAYLOAD=[%s]", static_cast<uint8_t>(command.command),
           format_hex_pretty(command.payload).c_str());

  uint8_t len = (uint8_t)(command.payload.size()) + 3;

  uint8_t checksum = len + (uint8_t)(command.command);
  for (auto &payload : command.payload)
    checksum += payload;
  checksum ^= 0xFF;

  write_array({len, (uint8_t) command.command});
  if (!command.payload.empty())
    write_array(command.payload.data(), command.payload.size());
  write_byte(checksum);
  flush();
}

void Wintex::process_command_queue_() {
  if (!this->current_command_.has_value() && !command_queue_.empty()) {
    uint32_t delay = millis() - last_command_timestamp_;
    if (delay < COMMAND_DELAY)
      return;

    last_command_timestamp_ = millis();
    send_command_now_(command_queue_.front());
    command_queue_.erase(command_queue_.begin());
  }
}

void Wintex::register_sensor(WintexSensorBase *sensor) { sensors_.push_back(sensor); }

void Wintex::register_partition(WintexPartition *partition) { partitions_.push_back(partition); }

void Wintex::register_zone(WintexZone *zone) { zones_.push_back(zone); }

void Wintex::setup_partitions_() {
  for (WintexPartition *partition : this->partitions_) {
    partition->setup(this, (uint32_t) 0xd85, (uint16_t) 0x13, "");
  }
}

void Wintex::setup_zones_() {
  for (WintexZone *zone : this->zones_) {
    zone->setup(this, (uint32_t) 0x4f8, (uint16_t) 0x20, "");
  }
}

void Wintex::queue_command_(AsyncWintexCommand command) {
  if (command_queue_.size() > 1)
    ESP_LOGD(TAG, "Queue size: %d", command_queue_.size());
  command_queue_.push_back(command);
  process_command_queue_();
}

}  // namespace wintex
}  // namespace esphome
