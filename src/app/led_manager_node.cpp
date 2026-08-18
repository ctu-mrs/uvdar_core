#include "uvdar_core/app/led_manager_node.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace uvdar_core::app {

namespace {

/**
 * @brief Sleep for the fixed inter-frame delay used by the Baca serial protocol.
 */
void sleepMs(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

} // namespace

LedManagerNode::LedManagerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("led_manager", options)
{
    uav_name_ = declare_parameter<std::string>("uav_name", std::string { });
    if (uav_name_.empty()) {
        // Fall back to the node namespace (set via the launch file) so this
        // still works without an explicit uav_name override.
        std::string ns = get_namespace();
        if (!ns.empty() && ns.front() == '/') {
            ns.erase(ns.begin());
        }
        uav_name_ = ns;
    }

    sequence_file_ = declare_parameter<std::string>("sequence_file", std::string { });

    baca_protocol_publisher_ = create_publisher<mrs_modules_msgs::msg::BacaProtocol>("~/baca_protocol_out", 1);

    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: Loading sequences from file " << sequence_file_);
    if ((!loadSequenceFile(sequence_file_)) || (sequences_.size() < 1)) {
        RCLCPP_ERROR_STREAM(get_logger(), "[UVDARLedManager]: Failed to load file " << sequence_file_);
        return;
    }

    serv_set_active_ = create_service<std_srvs::srv::SetBool>(
        "~/set_active", std::bind(&LedManagerNode::callbackSetActive, this, _1, _2));
    serv_frequency_ = create_service<mrs_msgs::srv::Float64Srv>(
        "~/set_frequency", std::bind(&LedManagerNode::callbackSetFrequency, this, _1, _2));
    serv_load_sequences_ = create_service<std_srvs::srv::Trigger>(
        "~/load_sequences", std::bind(&LedManagerNode::callbackLoadSequences, this, _1, _2));
    serv_load_single_sequence_ = create_service<mrs_msgs::srv::SetInt>(
        "~/load_single_sequence", std::bind(&LedManagerNode::callbackLoadSingleSequence, this, _1, _2));
    serv_select_single_sequence_ = create_service<mrs_msgs::srv::SetInt>(
        "~/select_single_sequence", std::bind(&LedManagerNode::callbackSelectSingleSequence, this, _1, _2));
    serv_select_sequences_ = create_service<uvdar_core::srv::SetInts>(
        "~/select_sequences", std::bind(&LedManagerNode::callbackSelectSequences, this, _1, _2));
    serv_quick_start_ = create_service<mrs_msgs::srv::SetInt>(
        "~/quick_start", std::bind(&LedManagerNode::callbackQuickStart, this, _1, _2));
    serv_set_mode_ = create_service<mrs_msgs::srv::SetInt>(
        "~/set_mode", std::bind(&LedManagerNode::callbackSetMode, this, _1, _2));
    serv_set_message_ = create_service<uvdar_core::srv::SetLedMessage>(
        "~/set_message", std::bind(&LedManagerNode::callbackSetMessage, this, _1, _2));

    // For simulation: mirror commands to the per-LED Gazebo plugin services.
    for (int i = 0; i < kGazeboLedCount; i++) {
        const std::string suffix = uav_name_ + "_" + std::to_string(i + 1);
        clients_set_sq_gz_.push_back(create_client<mrs_msgs::srv::SetInt>("/gazebo/ledSignalSetter/" + suffix));
        clients_set_fr_gz_.push_back(create_client<mrs_msgs::srv::Float64Srv>("/gazebo/ledFrequencySetter/" + suffix));
        clients_set_md_gz_.push_back(create_client<mrs_msgs::srv::SetInt>("/gazebo/ledModeSetter/" + suffix));
        clients_set_ac_gz_.push_back(create_client<std_srvs::srv::SetBool>("/gazebo/ledActiveSetter/" + suffix));
    }

    initialized_ = true;

    RCLCPP_INFO(get_logger(), "[UVDARLedManager]: blinking sequence setter node initiated");
}

bool LedManagerNode::loadSequenceFile(const std::string& sequence_file)
{
    RCLCPP_WARN(get_logger(), "[UVDARLedManager]: Add sanitation - sequences must be of equal, non-zero length");
    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: Loading sequence from file: [ " + sequence_file + " ]");

    std::ifstream ifs(sequence_file);
    if (!ifs.good()) {
        RCLCPP_ERROR_STREAM(get_logger(), "[UVDARLedManager]: Failed to load sequence file " << sequence_file << "! Returning.");
        return false;
    }

    std::vector<std::vector<bool>> sequences;
    RCLCPP_INFO(get_logger(), "[UVDARLedManager]: Loaded Sequences: [: ");
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::string show_string;
        std::vector<bool> sequence;
        std::stringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')) {
            sequence.push_back(token == "1");
            show_string += sequence.back() ? "1," : "0,";
        }
        sequences.push_back(sequence);
        RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]:   [" << show_string << "]");
    }
    RCLCPP_INFO(get_logger(), "[UVDARLedManager]: ]");

    sequences_ = sequences;
    return true;
}

void LedManagerNode::callbackSetActive(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    const unsigned char state = req->data ? 0x01 : 0x00;

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    serial_msg.payload.push_back(0x90); // set frequency
    serial_msg.payload.push_back(state); // # Hz
    baca_protocol_publisher_->publish(serial_msg);

    res->message = req->data ? "Activating the LEDs" : "Deactivating the LEDs";
    res->success = true;

    auto led_state = std::make_shared<std_srvs::srv::SetBool::Request>();
    led_state->data = req->data;
    for (auto& client : clients_set_ac_gz_) {
        if (client->service_is_ready()) {
            client->async_send_request(led_state);
        }
    }
}

void LedManagerNode::callbackSetFrequency(
    const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> req,
    std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    const unsigned short int_frequency = static_cast<unsigned short>(req->value); // Hz

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    serial_msg.payload.push_back(0x96); // set frequency
    serial_msg.payload.push_back(static_cast<unsigned char>(int_frequency & 0x00FF)); // LSB-first
    serial_msg.payload.push_back(static_cast<unsigned char>((int_frequency & 0xFF00) >> 8));
    baca_protocol_publisher_->publish(serial_msg);

    res->message = "Setting the frequency to " + std::to_string(static_cast<int>(int_frequency)) + " Hz";
    res->success = true;
    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: " << res->message);

    auto led_state = std::make_shared<mrs_msgs::srv::Float64Srv::Request>();
    led_state->value = static_cast<double>(int_frequency);
    for (auto& client : clients_set_fr_gz_) {
        if (client->service_is_ready()) {
            client->async_send_request(led_state);
        }
    }
}

void LedManagerNode::callbackLoadSequences(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: Loading sequences into the LED driver");

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();

    const unsigned char sequence_length = static_cast<unsigned char>(sequences_[0].size());
    serial_msg.payload.push_back(0x97); // set sequence length
    serial_msg.payload.push_back(sequence_length); // # bits
    baca_protocol_publisher_->publish(serial_msg);
    sleepMs(250);

    unsigned char i = 0;
    const int local_sleep_ms = 250 + static_cast<int>(100 * sequences_[0].size());
    for (const auto& sq : sequences_) {
        serial_msg.payload.clear();
        serial_msg.payload.push_back(0x99); // write sequences
        serial_msg.payload.push_back(i); // sequence index is i
        for (const bool b : sq) {
            serial_msg.payload.push_back(b ? 0x01 : 0x00); // bit of the sequence
        }

        baca_protocol_publisher_->publish(serial_msg);
        sleepMs(local_sleep_ms);
        i++;
    }

    res->message = "Loaded the sequences";
    res->success = true;
}

void LedManagerNode::callbackLoadSingleSequence(
    const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
    std::shared_ptr<mrs_msgs::srv::SetInt::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    const unsigned char index = static_cast<unsigned char>(req->value);
    if (index >= sequences_.size()) {
        RCLCPP_ERROR_STREAM(get_logger(), "[UVDARLedManager]: Failed to load sequence " << static_cast<int>(index) << " into the LED driver - no such sequence!");
        res->message = "Failed to load sequence " + std::to_string(static_cast<int>(index)) + " into the LED driver - no such sequence!";
        res->success = false;
        return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: Loading sequence " << static_cast<int>(index) << " into the LED driver");

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    serial_msg.payload.push_back(0x99); // write sequences
    serial_msg.payload.push_back(index); // sequence index is i
    for (const bool b : sequences_[index]) {
        serial_msg.payload.push_back(b ? 0x01 : 0x00); // bit of the sequence
    }
    baca_protocol_publisher_->publish(serial_msg);
    sleepMs(250);

    res->message = "Loaded sequence " + std::to_string(static_cast<int>(index));
    res->success = true;
}

void LedManagerNode::callbackSelectSingleSequence(
    const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
    std::shared_ptr<mrs_msgs::srv::SetInt::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    if (mode_ != 0) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: Requesting sequence selection, but the appropriate mode is not set!");
        res->success = false;
        res->message = "Requesting sequence selection, but the appropriate mode is not set!";
        return;
    }

    const unsigned char index = static_cast<unsigned char>(req->value);
    if (index >= sequences_.size()) {
        RCLCPP_ERROR_STREAM(get_logger(), "[UVDARLedManager]: Failed to set sequence " << static_cast<int>(index) << " - no such sequence!");
        res->message = "Failed to select sequence " + std::to_string(static_cast<int>(index)) + " - no such sequence!";
        res->success = false;
        return;
    }

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    serial_msg.payload.push_back(0x98); // select sequence index
    serial_msg.payload.push_back(index); // sequence #
    if (!kUvdarClassic) {
        serial_msg.payload.push_back(index); // sequence #
        serial_msg.payload.push_back(index); // sequence #
        serial_msg.payload.push_back(index); // sequence #
    }
    baca_protocol_publisher_->publish(serial_msg);

    res->success = true;
    res->message = "Selecting sequence " + std::to_string(static_cast<int>(index));
    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: " << res->message);

    auto led_state = std::make_shared<mrs_msgs::srv::SetInt::Request>();
    led_state->value = index;
    for (auto& client : clients_set_sq_gz_) {
        if (client->service_is_ready()) {
            client->async_send_request(led_state);
        }
    }
}

void LedManagerNode::callbackSelectSequences(
    const std::shared_ptr<uvdar_core::srv::SetInts::Request> req,
    std::shared_ptr<uvdar_core::srv::SetInts::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    if (mode_ != 0) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: Requesting sequence selection, but the appropriate mode is not set!");
        res->success = false;
        res->message = "Requesting sequence selection, but the appropriate mode is not set!";
        return;
    }

    if (kUvdarClassic) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: Failed to set sequencess because the attached UVDAR board does not support multiple sequence setting!");
        res->message = "Failed to set sequencess because the attached UVDAR board does not support multiple sequence setting!";
        res->success = false;
        return;
    }

    std::vector<unsigned char> selected_sequences;
    for (const auto sq : req->value) {
        const unsigned char index = static_cast<unsigned char>(sq);
        if (index >= sequences_.size()) {
            RCLCPP_ERROR_STREAM(get_logger(), "[UVDARLedManager]: Failed to set sequencess due to " << static_cast<int>(index) << " - no such sequence!");
            res->message = "Failed to select sequences due to " + std::to_string(static_cast<int>(index)) + " - no such sequence!";
            res->success = false;
            return;
        }
        selected_sequences.push_back(index);
    }

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    res->message = "Selecting sequences to [ ";
    serial_msg.payload.push_back(0x98); // select sequence index
    for (const auto sq : selected_sequences) {
        serial_msg.payload.push_back(sq); // sequence #
        res->message += std::to_string(static_cast<int>(sq)) + " ";
    }
    res->message += "]";
    RCLCPP_ERROR_STREAM(get_logger(), "[UVDARLedManager]: " << res->message);

    baca_protocol_publisher_->publish(serial_msg);
    res->success = true;

    std::size_t i = 0;
    for (auto& client : clients_set_sq_gz_) {
        if (!selected_sequences.empty() && client->service_is_ready()) {
            auto led_state = std::make_shared<mrs_msgs::srv::SetInt::Request>();
            led_state->value = selected_sequences[(i / 2) % selected_sequences.size()];
            client->async_send_request(led_state);
        }
        i++;
    }
}

void LedManagerNode::callbackQuickStart(
    const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
    std::shared_ptr<mrs_msgs::srv::SetInt::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    auto trig_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto trig_res = std::make_shared<std_srvs::srv::Trigger::Response>();
    callbackLoadSequences(trig_req, trig_res);
    if (!trig_res->success) {
        res->message = trig_res->message;
        res->success = false;
        return;
    }

    auto freq_req = std::make_shared<mrs_msgs::srv::Float64Srv::Request>();
    auto freq_res = std::make_shared<mrs_msgs::srv::Float64Srv::Response>();
    freq_req->value = 60.0;
    sleepMs(250);
    callbackSetFrequency(freq_req, freq_res);
    sleepMs(250);

    auto seq_req = std::make_shared<uvdar_core::srv::SetInts::Request>();
    auto seq_res = std::make_shared<uvdar_core::srv::SetInts::Response>();
    const unsigned char val = static_cast<unsigned char>(req->value);
    seq_req->value.push_back(4 * val + 0);
    seq_req->value.push_back(4 * val + 1);
    seq_req->value.push_back(4 * val + 2);
    seq_req->value.push_back(4 * val + 3);
    callbackSelectSequences(seq_req, seq_res);

    res->success = true;
    res->message = "Quickstart done. Sequences set to [" + std::to_string(4 * val + 0) + "," + std::to_string(4 * val + 1) + "," +
        std::to_string(4 * val + 2) + "," + std::to_string(4 * val + 3) + "].";
}

void LedManagerNode::callbackSetMode(
    const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
    std::shared_ptr<mrs_msgs::srv::SetInt::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    const unsigned char index = static_cast<unsigned char>(req->value); // 0 - tracking mode; 1 - communication mode

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    serial_msg.payload.push_back(0xF0); // select sequence index
    serial_msg.payload.push_back(index); // sequence #
    baca_protocol_publisher_->publish(serial_msg);

    switch (index) {
        case 0:
            res->message = "Selecting tracking mode";
            break;
        case 1:
            res->message = "Selecting communication mode";
            break;
        default:
            res->message = "Selecting unknown mode " + std::to_string(static_cast<int>(index));
            break;
    }
    RCLCPP_INFO_STREAM(get_logger(), "[UVDARLedManager]: " << res->message);

    auto led_state = std::make_shared<mrs_msgs::srv::SetInt::Request>();
    led_state->value = index;
    for (auto& client : clients_set_md_gz_) {
        if (client->service_is_ready()) {
            client->async_send_request(led_state);
        }
    }

    mode_ = index;
    res->success = true;
}

void LedManagerNode::callbackSetMessage(
    const std::shared_ptr<uvdar_core::srv::SetLedMessage::Request> req,
    std::shared_ptr<uvdar_core::srv::SetLedMessage::Response> res)
{
    if (!initialized_) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: LED manager is NOT initialized!");
        res->success = false;
        res->message = "LED manager is NOT initialized!";
        return;
    }

    if (mode_ != 1) {
        RCLCPP_ERROR(get_logger(), "[UVDARLedManager]: Requesting message transmission, but the appropriate mode is not set!");
        res->success = false;
        res->message = "Requesting message transmission, but the appropriate mode is not set!";
        return;
    }

    RCLCPP_INFO(get_logger(), "[UVDARLedManager]: Sending message");

    mrs_modules_msgs::msg::BacaProtocol serial_msg;
    serial_msg.stamp = now();
    serial_msg.payload.push_back(0x93);
    for (const auto& bit : req->data_frame) {
        serial_msg.payload.push_back(bit);
    }
    baca_protocol_publisher_->publish(serial_msg);

    res->message = "Sending message";
    res->success = true;
}

} // namespace uvdar_core::app
