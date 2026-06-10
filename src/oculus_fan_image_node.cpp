/*
 * Human-visualization fan image renderer for Blueprint Subsea Oculus pings.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "bpsubsea_oculus_ros2_driver/msg/ping.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

uint8_t clamp_u8(int value)
{
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

struct Rgb
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

Rgb palette_colour(uint8_t value, const std::string & palette)
{
  const int v = value;

  if (palette == "amber") {
    return {clamp_u8(v), clamp_u8((v * v) / 255), clamp_u8(v / 10)};
  }

  if (palette == "blue_green") {
    return {
      clamp_u8(std::max(0, (v - 170) * 3)),
      clamp_u8(std::min(255, v * 2)),
      clamp_u8(64 + v / 2)};
  }

  if (palette == "fire") {
    return {
      clamp_u8(v < 128 ? v * 2 : 255),
      clamp_u8(v < 128 ? 0 : (v - 128) * 2),
      clamp_u8(v < 220 ? 0 : (v - 220) * 7)};
  }

  return {value, value, value};
}

uint8_t sample_to_u8(
  const bpsubsea_oculus_ros2_driver::msg::Ping & ping,
  size_t sample_index)
{
  const size_t pixel_count = static_cast<size_t>(ping.n_ranges) * ping.n_beams;

  if (ping.data_size == bpsubsea_oculus_ros2_driver::msg::Ping::DATA_SIZE_16_BIT) {
    const size_t byte_index = sample_index * 2;
    if (sample_index >= pixel_count || byte_index + 1 >= ping.image_data.size()) {
      return 0;
    }
    uint16_t raw = 0;
    std::memcpy(&raw, ping.image_data.data() + byte_index, sizeof(raw));
    return static_cast<uint8_t>((raw & 0xff00) >> 8);
  }

  if (ping.data_size == bpsubsea_oculus_ros2_driver::msg::Ping::DATA_SIZE_24_BIT) {
    const size_t byte_index = sample_index * 3;
    if (sample_index >= pixel_count || byte_index + 2 >= ping.image_data.size()) {
      return 0;
    }
    return std::max({ping.image_data[byte_index], ping.image_data[byte_index + 1],
      ping.image_data[byte_index + 2]});
  }

  if (ping.data_size == bpsubsea_oculus_ros2_driver::msg::Ping::DATA_SIZE_32_BIT) {
    const size_t byte_index = sample_index * 4;
    if (sample_index >= pixel_count || byte_index + 3 >= ping.image_data.size()) {
      return 0;
    }
    uint32_t raw = 0;
    std::memcpy(&raw, ping.image_data.data() + byte_index, sizeof(raw));
    return static_cast<uint8_t>((raw >> 24) & 0xff);
  }

  if (sample_index >= ping.image_data.size()) {
    return 0;
  }
  return ping.image_data[sample_index];
}

bool find_beam_index(
  const std::vector<int16_t> & bearings,
  double bearing_radians,
  double & beam_index)
{
  if (bearings.size() < 2) {
    return false;
  }

  const double bearing_centidegrees = bearing_radians / kDegToRad * 100.0;
  const bool ascending = bearings.front() <= bearings.back();
  const double first = static_cast<double>(bearings.front());
  const double last = static_cast<double>(bearings.back());
  const double low = std::min(first, last);
  const double high = std::max(first, last);

  if (bearing_centidegrees < low || bearing_centidegrees > high) {
    return false;
  }

  size_t upper_index = 0;
  if (ascending) {
    const auto it = std::lower_bound(bearings.begin(), bearings.end(), bearing_centidegrees);
    if (it == bearings.begin()) {
      beam_index = 0.0;
      return true;
    }
    if (it == bearings.end()) {
      beam_index = static_cast<double>(bearings.size() - 1);
      return true;
    }
    upper_index = static_cast<size_t>(std::distance(bearings.begin(), it));
  } else {
    const auto it = std::lower_bound(
      bearings.begin(), bearings.end(), bearing_centidegrees, std::greater<int16_t>());
    if (it == bearings.begin()) {
      beam_index = 0.0;
      return true;
    }
    if (it == bearings.end()) {
      beam_index = static_cast<double>(bearings.size() - 1);
      return true;
    }
    upper_index = static_cast<size_t>(std::distance(bearings.begin(), it));
  }

  const size_t lower_index = upper_index - 1;
  const double a = static_cast<double>(bearings[lower_index]);
  const double b = static_cast<double>(bearings[upper_index]);
  if (std::abs(b - a) < std::numeric_limits<double>::epsilon()) {
    beam_index = static_cast<double>(lower_index);
  } else {
    const double t = (bearing_centidegrees - a) / (b - a);
    beam_index = static_cast<double>(lower_index) + t;
  }
  beam_index = std::clamp(beam_index, 0.0, static_cast<double>(bearings.size() - 1));
  return true;
}

uint8_t bilinear_sample(
  const bpsubsea_oculus_ros2_driver::msg::Ping & ping,
  double range_index,
  double beam_index)
{
  if (range_index < 0.0 || beam_index < 0.0 ||
    range_index > static_cast<double>(ping.n_ranges - 1) ||
    beam_index > static_cast<double>(ping.n_beams - 1))
  {
    return 0;
  }

  const auto r0 = static_cast<size_t>(std::floor(range_index));
  const auto b0 = static_cast<size_t>(std::floor(beam_index));
  const size_t r1 = std::min<size_t>(r0 + 1, ping.n_ranges - 1);
  const size_t b1 = std::min<size_t>(b0 + 1, ping.n_beams - 1);
  const double rt = range_index - static_cast<double>(r0);
  const double bt = beam_index - static_cast<double>(b0);

  const auto at = [&](size_t range, size_t beam) {
      return sample_to_u8(ping, range * ping.n_beams + beam);
    };

  const double v00 = at(r0, b0);
  const double v10 = at(r1, b0);
  const double v01 = at(r0, b1);
  const double v11 = at(r1, b1);

  const double near_beam = v00 * (1.0 - rt) + v10 * rt;
  const double far_beam = v01 * (1.0 - rt) + v11 * rt;
  return clamp_u8(static_cast<int>(std::round(near_beam * (1.0 - bt) + far_beam * bt)));
}

}  // namespace

class OculusFanImageNode : public rclcpp::Node
{
public:
  OculusFanImageNode()
  : Node("oculus_fan_image")
  {
    declare_parameter<std::string>("input_topic", "/oculus/ping");
    declare_parameter<std::string>("output_topic", "/oculus/fan_image");
    declare_parameter<int>("output_width", 800);
    declare_parameter<int>("output_height", 800);
    declare_parameter<double>("meters_per_pixel", 0.0);
    declare_parameter<double>("max_range", 0.0);
    declare_parameter<double>("center_x", -1.0);
    declare_parameter<double>("center_y", -1.0);
    declare_parameter<std::string>("palette", "mono8");
    declare_parameter<bool>("flip_x", false);
    declare_parameter<bool>("flip_y", false);
    declare_parameter<bool>("head_down", true);
    declare_parameter<int>("background_value", 0);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    fan_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, rclcpp::SensorDataQoS());
    ping_sub_ = create_subscription<bpsubsea_oculus_ros2_driver::msg::Ping>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const bpsubsea_oculus_ros2_driver::msg::Ping::SharedPtr msg) {
        render(*msg);
      });

    RCLCPP_INFO(
      get_logger(), "Rendering Oculus fan images from %s to %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  int int_parameter(const std::string & name, int min_value, int max_value) const
  {
    const int value = static_cast<int>(get_parameter(name).as_int());
    return std::clamp(value, min_value, max_value);
  }

  void render(const bpsubsea_oculus_ros2_driver::msg::Ping & ping)
  {
    if (ping.n_ranges < 2 || ping.n_beams < 2 || ping.bearings.size() != ping.n_beams ||
      ping.range_resolution <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipping malformed Oculus ping for fan rendering");
      return;
    }

    const int width = int_parameter("output_width", 1, 8192);
    const int height = int_parameter("output_height", 1, 8192);
    const double configured_max_range = get_parameter("max_range").as_double();
    const double max_range = configured_max_range > 0.0 ?
      std::min(configured_max_range, ping.range) : ping.range;
    const bool flip_x = get_parameter("flip_x").as_bool();
    const bool flip_y = get_parameter("flip_y").as_bool();
    const bool head_down = get_parameter("head_down").as_bool();
    const auto palette = get_parameter("palette").as_string();
    const uint8_t background = static_cast<uint8_t>(int_parameter("background_value", 0, 255));
    const bool colour = palette != "mono8" && palette != "mono" && palette != "gray" &&
      palette != "grey";

    double center_x = get_parameter("center_x").as_double();
    double center_y = get_parameter("center_y").as_double();
    if (center_x < 0.0) {
      center_x = (static_cast<double>(width) - 1.0) * 0.5;
    }
    if (center_y < 0.0) {
      center_y = head_down ? static_cast<double>(height) * 0.05 :
        static_cast<double>(height) * 0.95;
    }

    double meters_per_pixel = get_parameter("meters_per_pixel").as_double();
    if (meters_per_pixel <= 0.0) {
      const double margin = 0.95;
      const double left_bearing = static_cast<double>(ping.bearings.front()) / 100.0 * kDegToRad;
      const double right_bearing = static_cast<double>(ping.bearings.back()) / 100.0 * kDegToRad;
      const double lateral_extent = max_range *
        std::max(std::abs(std::sin(left_bearing)), std::abs(std::sin(right_bearing)));
      const double horizontal_pixels = std::max(1.0, std::min(
        center_x, static_cast<double>(width - 1) - center_x) * margin);
      const double vertical_pixels = std::max(1.0, (
        head_down ? static_cast<double>(height - 1) - center_y : center_y) * margin);
      meters_per_pixel = std::max(lateral_extent / horizontal_pixels, max_range / vertical_pixels);
    }
    meters_per_pixel = std::max(1e-6, meters_per_pixel);

    sensor_msgs::msg::Image image;
    image.header = ping.header;
    image.height = static_cast<sensor_msgs::msg::Image::_height_type>(height);
    image.width = static_cast<sensor_msgs::msg::Image::_width_type>(width);
    image.encoding = colour ? "rgb8" : "mono8";
    image.is_bigendian = false;
    image.step = static_cast<sensor_msgs::msg::Image::_step_type>(width * (colour ? 3 : 1));
    image.data.assign(static_cast<size_t>(image.step) * height, background);

    if (colour) {
      const Rgb bg = palette_colour(background, palette);
      for (size_t i = 0; i + 2 < image.data.size(); i += 3) {
        image.data[i] = bg.r;
        image.data[i + 1] = bg.g;
        image.data[i + 2] = bg.b;
      }
    }

    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        double pixel_x = static_cast<double>(u);
        double pixel_y = static_cast<double>(v);
        if (flip_x) {
          pixel_x = static_cast<double>(width - 1) - pixel_x;
        }
        if (flip_y) {
          pixel_y = static_cast<double>(height - 1) - pixel_y;
        }

        const double x = (pixel_x - center_x) * meters_per_pixel;
        const double y = head_down ?
          (pixel_y - center_y) * meters_per_pixel :
          (center_y - pixel_y) * meters_per_pixel;
        if (y < 0.0) {
          continue;
        }

        const double range = std::sqrt(x * x + y * y);
        if (range > max_range) {
          continue;
        }

        double beam_index = 0.0;
        if (!find_beam_index(ping.bearings, std::atan2(x, y), beam_index)) {
          continue;
        }

        const double range_index = range / ping.range_resolution;
        const uint8_t intensity = bilinear_sample(ping, range_index, beam_index);
        const size_t output_index = static_cast<size_t>(v) * image.step + u * (colour ? 3 : 1);

        if (colour) {
          const Rgb rgb = palette_colour(intensity, palette);
          image.data[output_index] = rgb.r;
          image.data[output_index + 1] = rgb.g;
          image.data[output_index + 2] = rgb.b;
        } else {
          image.data[output_index] = intensity;
        }
      }
    }

    fan_pub_->publish(image);
  }

  rclcpp::Subscription<bpsubsea_oculus_ros2_driver::msg::Ping>::SharedPtr ping_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OculusFanImageNode>());
  rclcpp::shutdown();
  return 0;
}
