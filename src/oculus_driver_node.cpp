/*
 * ROS 2 driver for Blueprint Subsea Oculus forward looking sonars.
 *
 * This driver reuses the GPL-3.0-or-later Oculus wire protocol layout from
 * Blueprint Subsea's Oculus Viewer SDK.
 */

#include "bpsubsea_oculus_ros2_driver/oculus_protocol.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bpsubsea_oculus_ros2_driver/msg/ping.hpp"
#include "bpsubsea_oculus_ros2_driver/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

using namespace std::chrono_literals;
namespace protocol = bpsubsea_oculus_ros2_driver::protocol;

constexpr size_t kMaxPacketSize = 32U * 1024U * 1024U;

std::string ipv4_to_string(uint32_t raw_address)
{
  if (raw_address == 0) {
    return "";
  }

  char text[INET_ADDRSTRLEN] = {};
  in_addr address {};
  address.s_addr = raw_address;
  if (!inet_ntop(AF_INET, &address, text, sizeof(text))) {
    return "";
  }
  return text;
}

std::string sockaddr_to_string(const sockaddr_in & address)
{
  char text[INET_ADDRSTRLEN] = {};
  if (!inet_ntop(AF_INET, &address.sin_addr, text, sizeof(text))) {
    return "";
  }
  return text;
}

template<typename T>
bool copy_struct(const std::vector<uint8_t> & packet, T & out)
{
  if (packet.size() < sizeof(T)) {
    return false;
  }
  std::memcpy(&out, packet.data(), sizeof(T));
  return true;
}

}  // namespace

class OculusDriverNode : public rclcpp::Node
{
public:
  OculusDriverNode()
  : Node("oculus_driver")
  {
    declare_parameter<std::string>("sonar_ip", "auto");
    declare_parameter<int>("data_port", protocol::kDefaultDataPort);
    declare_parameter<int>("status_port", protocol::kDefaultStatusPort);
    declare_parameter<std::string>("frame_id", "oculus");
    declare_parameter<int>("mode", 1);
    declare_parameter<int>("ping_rate", protocol::pingRateHigh);
    declare_parameter<double>("range", 10.0);
    declare_parameter<double>("gain", 50.0);
    declare_parameter<double>("speed_of_sound", 0.0);
    declare_parameter<double>("salinity", 0.0);
    declare_parameter<bool>("gain_assist", true);
    declare_parameter<int>("gamma_correction", 127);
    declare_parameter<int>("network_speed_limit", 255);
    declare_parameter<bool>("enable_512_beams", true);
    declare_parameter<int>("reconnect_delay_ms", 1000);
    declare_parameter<int>("fire_message_version", 1);

    status_pub_ = create_publisher<bpsubsea_oculus_ros2_driver::msg::Status>(
      "/oculus/status", rclcpp::QoS(10));
    ping_pub_ = create_publisher<bpsubsea_oculus_ros2_driver::msg::Ping>(
      "/oculus/ping", rclcpp::SensorDataQoS());
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/oculus/image", rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/oculus/imu", rclcpp::SensorDataQoS());

    running_.store(true);
    status_thread_ = std::thread(&OculusDriverNode::status_loop, this);
    tcp_thread_ = std::thread(&OculusDriverNode::tcp_loop, this);
  }

  ~OculusDriverNode() override
  {
    running_.store(false);
    if (status_thread_.joinable()) {
      status_thread_.join();
    }
    if (tcp_thread_.joinable()) {
      tcp_thread_.join();
    }
  }

private:
  std::string frame_id() const
  {
    return get_parameter("frame_id").as_string();
  }

  int int_parameter(const std::string & name, int min_value, int max_value) const
  {
    const int value = static_cast<int>(get_parameter(name).as_int());
    return std::clamp(value, min_value, max_value);
  }

  protocol::OculusSimpleFireMessage build_fire_message()
  {
    protocol::OculusSimpleFireMessage fire {};
    fire.head.oculusId = protocol::kOculusCheckId;
    fire.head.msgId = protocol::messageSimpleFire;
    fire.masterMode = static_cast<uint8_t>(int_parameter("mode", 0, 255));
    fire.pingRate = static_cast<protocol::PingRateType>(int_parameter("ping_rate", 0, 5));
    fire.networkSpeed = static_cast<uint8_t>(int_parameter("network_speed_limit", 0, 255));
    fire.gammaCorrection = static_cast<uint8_t>(int_parameter("gamma_correction", 0, 255));
    fire.range = get_parameter("range").as_double();
    fire.gainPercent = get_parameter("gain").as_double();
    fire.speedOfSound = get_parameter("speed_of_sound").as_double();
    fire.salinity = get_parameter("salinity").as_double();

    uint8_t flags = 0x01;  // Range is specified in metres.
    flags |= 0x08;         // Preserve SDK simple-fire behaviour.
    if (get_parameter("gain_assist").as_bool()) {
      flags |= 0x10;
    }
    if (get_parameter("enable_512_beams").as_bool()) {
      flags |= 0x40;
    }
    fire.flags = flags;

    return fire;
  }

  protocol::OculusSimpleFireMessage2 build_fire_message2()
  {
    const auto fire = build_fire_message();

    protocol::OculusSimpleFireMessage2 fire2 {};
    fire2.head = fire.head;
    fire2.head.msgVersion = 2;
    fire2.head.payloadSize = sizeof(fire2) - sizeof(fire2.head);
    fire2.masterMode = fire.masterMode;
    fire2.pingRate = fire.pingRate;
    fire2.networkSpeed = fire.networkSpeed;
    fire2.gammaCorrection = fire.gammaCorrection;
    fire2.flags = fire.flags;
    fire2.range = fire.range;
    fire2.gainPercent = fire.gainPercent;
    fire2.speedOfSound = fire.speedOfSound;
    fire2.salinity = fire.salinity;

    return fire2;
  }

  bool send_fire(int socket_fd)
  {
    if (get_parameter("fire_message_version").as_int() == 2) {
      const auto fire = build_fire_message2();
      return send_bytes(socket_fd, reinterpret_cast<const uint8_t *>(&fire), sizeof(fire));
    }

    const auto fire = build_fire_message();
    return send_bytes(socket_fd, reinterpret_cast<const uint8_t *>(&fire), sizeof(fire));
  }

  bool send_bytes(int socket_fd, const uint8_t * data, size_t size)
  {
    size_t sent_total = 0;

    while (sent_total < size && running_.load()) {
      const ssize_t sent = ::send(socket_fd, data + sent_total, size - sent_total, MSG_NOSIGNAL);
      if (sent <= 0) {
        return false;
      }
      sent_total += static_cast<size_t>(sent);
    }
    return true;
  }

  void status_loop()
  {
    const int port = int_parameter("status_port", 1, 65535);
    const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create Oculus status UDP socket");
      return;
    }

    int reuse = 1;
    ::setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    ::setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in bind_address {};
    bind_address.sin_family = AF_INET;
    bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_address.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(socket_fd, reinterpret_cast<sockaddr *>(&bind_address), sizeof(bind_address)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind Oculus status UDP socket on port %d", port);
      ::close(socket_fd);
      return;
    }

    RCLCPP_INFO(get_logger(), "Listening for Oculus status broadcasts on UDP port %d", port);

    while (running_.load()) {
      protocol::OculusStatusMsg status {};
      sockaddr_in source {};
      socklen_t source_len = sizeof(source);
      const ssize_t received = ::recvfrom(
        socket_fd, &status, sizeof(status), 0, reinterpret_cast<sockaddr *>(&source), &source_len);

      if (received < 0) {
        continue;
      }
      if (static_cast<size_t>(received) != sizeof(status) ||
        status.hdr.oculusId != protocol::kOculusCheckId)
      {
        continue;
      }

      const std::string source_ip = sockaddr_to_string(source);
      {
        std::lock_guard<std::mutex> lock(auto_ip_mutex_);
        latest_status_source_ip_ = source_ip;
      }

      publish_status(status);
    }

    ::close(socket_fd);
  }

  void publish_status(const protocol::OculusStatusMsg & status)
  {
    bpsubsea_oculus_ros2_driver::msg::Status msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id();
    msg.device_id = status.deviceId;
    msg.device_type = static_cast<uint16_t>(status.deviceType);
    msg.part_number = static_cast<uint16_t>(status.partNumber);
    msg.model = protocol::part_number_to_model(msg.part_number);
    msg.status = status.status;
    msg.ip_address = ipv4_to_string(status.ipAddr);
    msg.ip_mask = ipv4_to_string(status.ipMask);
    msg.connected_ip_address = ipv4_to_string(status.connectedIpAddr);
    msg.mac_address = {
      status.macAddr0, status.macAddr1, status.macAddr2,
      status.macAddr3, status.macAddr4, status.macAddr5};
    msg.temperatures = {
      status.temperature0, status.temperature1, status.temperature2, status.temperature3,
      status.temperature4, status.temperature5, status.temperature6, status.temperature7};
    msg.pressure = status.pressure;
    msg.firmware_version0 = status.versionInfo.firmwareVersion0;
    msg.firmware_date0 = status.versionInfo.firmwareDate0;
    msg.firmware_version1 = status.versionInfo.firmwareVersion1;
    msg.firmware_date1 = status.versionInfo.firmwareDate1;
    msg.firmware_version2 = status.versionInfo.firmwareVersion2;
    msg.firmware_date2 = status.versionInfo.firmwareDate2;

    status_pub_->publish(msg);
  }

  std::string target_ip()
  {
    const auto configured = get_parameter("sonar_ip").as_string();
    if (configured != "auto" && !configured.empty()) {
      return configured;
    }

    std::lock_guard<std::mutex> lock(auto_ip_mutex_);
    return latest_status_source_ip_;
  }

  void tcp_loop()
  {
    while (running_.load()) {
      const std::string ip = target_ip();
      if (ip.empty()) {
        std::this_thread::sleep_for(250ms);
        continue;
      }

      const int port = int_parameter("data_port", 1, 65535);
      const int socket_fd = connect_tcp(ip, port);
      if (socket_fd < 0) {
        sleep_reconnect_delay();
        continue;
      }

      RCLCPP_INFO(get_logger(), "Connected to Oculus sonar at %s:%d", ip.c_str(), port);
      receive_loop(socket_fd);
      ::close(socket_fd);

      if (running_.load()) {
        RCLCPP_WARN(get_logger(), "Disconnected from Oculus sonar");
        sleep_reconnect_delay();
      }
    }
  }

  int connect_tcp(const std::string & ip, int port)
  {
    const int socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create Oculus TCP socket");
      return -1;
    }

    int flag = 1;
    ::setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    ::setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    ::setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid Oculus sonar IP address: %s", ip.c_str());
      ::close(socket_fd);
      return -1;
    }

    if (::connect(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      RCLCPP_DEBUG(get_logger(), "Failed to connect to Oculus sonar at %s:%d", ip.c_str(), port);
      ::close(socket_fd);
      return -1;
    }

    return socket_fd;
  }

  void sleep_reconnect_delay()
  {
    const auto delay = std::chrono::milliseconds(
      int_parameter("reconnect_delay_ms", 100, 60000));
    const auto end = std::chrono::steady_clock::now() + delay;
    while (running_.load() && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(50ms);
    }
  }

  void receive_loop(int socket_fd)
  {
    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(256 * 1024);

    auto last_fire = std::chrono::steady_clock::now();
    if (!send_fire(socket_fd)) {
      return;
    }

    std::array<uint8_t, 65536> chunk {};
    while (running_.load()) {
      const ssize_t received = ::recv(socket_fd, chunk.data(), chunk.size(), 0);
      if (received == 0) {
        return;
      }
      if (received < 0) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_fire > 500ms) {
          if (!send_fire(socket_fd)) {
            return;
          }
          last_fire = now;
        }
        continue;
      }

      rx_buffer.insert(rx_buffer.end(), chunk.begin(), chunk.begin() + received);
      process_rx_buffer(rx_buffer, socket_fd);
      last_fire = std::chrono::steady_clock::now();
    }
  }

  void process_rx_buffer(std::vector<uint8_t> & rx_buffer, int socket_fd)
  {
    while (rx_buffer.size() >= sizeof(protocol::OculusMessageHeader)) {
      protocol::OculusMessageHeader header {};
      std::memcpy(&header, rx_buffer.data(), sizeof(header));

      if (header.oculusId != protocol::kOculusCheckId) {
        auto next = rx_buffer.end();
        for (auto it = rx_buffer.begin() + 1; it + 1 < rx_buffer.end(); ++it) {
          if (*it == 0x53 && *(it + 1) == 0x4f) {
            next = it;
            break;
          }
        }
        rx_buffer.erase(rx_buffer.begin(), next);
        continue;
      }

      const size_t packet_size = sizeof(protocol::OculusMessageHeader) + header.payloadSize;
      if (packet_size > kMaxPacketSize) {
        RCLCPP_WARN(get_logger(), "Dropping oversized Oculus packet: %zu bytes", packet_size);
        rx_buffer.erase(rx_buffer.begin());
        continue;
      }
      if (rx_buffer.size() < packet_size) {
        return;
      }

      std::vector<uint8_t> packet(rx_buffer.begin(), rx_buffer.begin() + packet_size);
      rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + packet_size);
      process_packet(packet);

      if (header.msgId == protocol::messageSimplePingResult) {
        send_fire(socket_fd);
      }
    }
  }

  void process_packet(const std::vector<uint8_t> & packet)
  {
    protocol::OculusMessageHeader header {};
    if (!copy_struct(packet, header)) {
      return;
    }

    if (header.msgId != protocol::messageSimplePingResult) {
      return;
    }

    if (header.msgVersion != last_ping_version_) {
      RCLCPP_INFO(get_logger(), "Receiving ping messages of version %u", header.msgVersion);
      last_ping_version_ = header.msgVersion;
    }

    if (header.msgVersion == 2) {
      protocol::OculusSimplePingResult2 ping {};
      if (!copy_struct(packet, ping)) {
        return;
      }
      publish_ping_v2(packet, ping);
    } else {
      protocol::OculusSimplePingResult ping {};
      if (!copy_struct(packet, ping)) {
        return;
      }
      publish_ping_v1(packet, ping);
    }
  }

  void publish_ping_v1(
    const std::vector<uint8_t> & packet,
    const protocol::OculusSimplePingResult & ping)
  {
    publish_ping_common(
      packet,
      ping.fireMessage.head.msgVersion,
      ping.fireMessage.head.partNumber,
      ping.pingId,
      ping.status,
      ping.frequency,
      ping.temperature,
      ping.pressure,
      0.0,
      0.0,
      0.0,
      ping.speedOfSoundUsed,
      static_cast<double>(ping.pingStartTime),
      static_cast<uint8_t>(ping.dataSize),
      ping.rangeResolution,
      ping.nRanges,
      ping.nBeams,
      ping.imageOffset,
      ping.imageSize,
      sizeof(protocol::OculusSimplePingResult),
      false);
  }

  void publish_ping_v2(
    const std::vector<uint8_t> & packet,
    const protocol::OculusSimplePingResult2 & ping)
  {
    publish_ping_common(
      packet,
      ping.fireMessage.head.msgVersion,
      ping.fireMessage.head.partNumber,
      ping.pingId,
      ping.status,
      ping.frequency,
      ping.temperature,
      ping.pressure,
      ping.heading,
      ping.pitch,
      ping.roll,
      ping.speedOfSoundUsed,
      ping.pingStartTime,
      static_cast<uint8_t>(ping.dataSize),
      ping.rangeResolution,
      ping.nRanges,
      ping.nBeams,
      ping.imageOffset,
      ping.imageSize,
      sizeof(protocol::OculusSimplePingResult2),
      true);
  }

  void publish_ping_common(
    const std::vector<uint8_t> & packet,
    uint16_t message_version,
    uint16_t part_number,
    uint32_t ping_id,
    uint32_t status,
    double frequency,
    double temperature,
    double pressure,
    double heading,
    double pitch,
    double roll,
    double speed_of_sound,
    double ping_start_time,
    uint8_t data_size,
    double range_resolution,
    uint16_t n_ranges,
    uint16_t n_beams,
    uint32_t image_offset,
    uint32_t image_size,
    size_t bearing_offset,
    bool has_attitude)
  {
    const size_t bearing_size = static_cast<size_t>(n_beams) * sizeof(int16_t);
    const size_t image_end = static_cast<size_t>(image_offset) + image_size;
    if (bearing_offset + bearing_size > packet.size() || image_end > packet.size()) {
      RCLCPP_WARN(get_logger(), "Dropping malformed Oculus ping packet");
      return;
    }

    const auto stamp = now();
    const auto frame = frame_id();
    const auto * image_begin = packet.data() + image_offset;
    const auto * image_finish = image_begin + image_size;
    const auto encoding = std::string(protocol::image_encoding(data_size));
    const uint8_t bytes_per_sample = protocol::data_size_bytes(data_size);

    bpsubsea_oculus_ros2_driver::msg::Ping ping_msg;
    ping_msg.header.stamp = stamp;
    ping_msg.header.frame_id = frame;
    ping_msg.ping_id = ping_id;
    ping_msg.message_version = message_version;
    ping_msg.part_number = part_number;
    ping_msg.status = status;
    ping_msg.frequency = frequency;
    ping_msg.temperature = temperature;
    ping_msg.pressure = pressure;
    ping_msg.heading = heading;
    ping_msg.pitch = pitch;
    ping_msg.roll = roll;
    ping_msg.speed_of_sound = speed_of_sound;
    ping_msg.ping_start_time = ping_start_time;
    ping_msg.data_size = data_size;
    ping_msg.image_encoding = encoding;
    ping_msg.n_ranges = n_ranges;
    ping_msg.n_beams = n_beams;
    ping_msg.range = static_cast<double>(n_ranges) * range_resolution;
    ping_msg.range_resolution = range_resolution;
    ping_msg.bearings.resize(n_beams);
    std::memcpy(ping_msg.bearings.data(), packet.data() + bearing_offset, bearing_size);
    ping_msg.image_data.assign(image_begin, image_finish);
    ping_pub_->publish(ping_msg);

    sensor_msgs::msg::Image image_msg;
    image_msg.header = ping_msg.header;
    image_msg.height = n_ranges;
    image_msg.width = n_beams;
    image_msg.encoding = encoding;
    image_msg.is_bigendian = false;
    image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(n_beams * bytes_per_sample);
    image_msg.data.assign(image_begin, image_finish);
    image_pub_->publish(image_msg);

    if (has_attitude) {
      publish_imu(ping_msg.header, heading, pitch, roll);
    }
  }

  void publish_imu(const std_msgs::msg::Header & header, double heading, double pitch, double roll)
  {
    const double deg = M_PI / 180.0;
    const double half_roll = 0.5 * roll * deg;
    const double half_pitch = 0.5 * pitch * deg;
    const double half_yaw = 0.5 * (90.0 - heading) * deg;

    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header = header;
    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu_msg);
  }

  std::atomic_bool running_ {false};
  std::thread status_thread_;
  std::thread tcp_thread_;
  uint16_t last_ping_version_ {0xFFFF};

  std::mutex auto_ip_mutex_;
  std::string latest_status_source_ip_;

  rclcpp::Publisher<bpsubsea_oculus_ros2_driver::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<bpsubsea_oculus_ros2_driver::msg::Ping>::SharedPtr ping_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OculusDriverNode>());
  rclcpp::shutdown();
  return 0;
}
