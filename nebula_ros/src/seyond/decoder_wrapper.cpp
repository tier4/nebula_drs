#include "nebula_ros/seyond/decoder_wrapper.hpp"

#include <cstdint>

namespace nebula
{
namespace ros
{

using namespace std::chrono_literals;

SeyondDecoderWrapper::SeyondDecoderWrapper(
  rclcpp::Node * const parent_node, const std::shared_ptr<SeyondHwInterface> & hw_interface,
  std::shared_ptr<const SeyondSensorConfiguration> & config, bool decode_flag)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("SeyondDecoder")),
  hw_interface_(hw_interface),
  sensor_cfg_(config)
{
  if (!config) {
    throw std::runtime_error("SeyondDecoderWrapper cannot be instantiated without a valid config!");
  }

  if (config->sensor_model == drivers::SensorModel::SEYOND_ROBIN_W) {
    calibration_file_path_ =
      parent_node->declare_parameter<std::string>("calibration_file_path", param_read_write());
    auto calibration_result = GetCalibrationData(calibration_file_path_);
    if (!calibration_result.has_value()) {
      throw std::runtime_error("No valid calibration found");
    }
    calibration_cfg_ptr_ = calibration_result.value();
    driver_ptr_ = std::make_shared<SeyondDriver>(config, calibration_cfg_ptr_);
  } else {
    driver_ptr_ = std::make_shared<SeyondDriver>(config, calibration_cfg_ptr_);
  }

  RCLCPP_INFO(logger_, "Starting Decoder");

  status_ = driver_ptr_->GetStatus();

  if (Status::OK != status_) {
    throw std::runtime_error("Error instantiating decoder");
  }

  // Publish packets only if HW interface is connected
  if (hw_interface_) {
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    packets_pub_ = parent_node->create_publisher<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  decode_ = decode_flag;

  nebula_points_pub_ = decode_flag ? parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
                                       "nebula_points", pointcloud_qos)
                                   : nullptr;

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 110'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });
}

void SeyondDecoderWrapper::OnConfigChange(
  const std::shared_ptr<const SeyondSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<SeyondDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

SeyondDecoderWrapper::get_calibration_result_t SeyondDecoderWrapper::GetCalibrationData(
  const std::string & calibration_file_path)
{
  std::shared_ptr<drivers::SeyondCalibrationConfiguration> calib;
  calib = std::make_shared<drivers::SeyondCalibrationConfiguration>();

  bool hw_connected = hw_interface_ != nullptr;

  // If a sensor is connected, try to download and save its calibration data
  if (hw_connected && sensor_cfg_->sensor_model == drivers::SensorModel::SEYOND_ROBIN_W) {
    try {
      auto calibration_data = hw_interface_->GetLidarCalibrationString();

      RCLCPP_INFO(logger_, "Downloading calibration data from sensor.");
      auto status = calib->LoadFromString(calibration_data);
      if (status != Status::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Could not download calibration data: " << status_);
      } else if (calibration_file_path != "") {
        status = calib->SaveToFile(calibration_file_path);
        if (status != Status::OK) {
          RCLCPP_ERROR_STREAM(
            logger_, "Could not save calibration data to " << calibration_file_path);
        } else {
          RCLCPP_INFO_STREAM(
            logger_, "Read calibration data from sensor, saved to " << calibration_file_path);
        }
      }
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Could not download calibration data: " << e.what());
    }
    // Otherwise read from the provided file
  } else if (
    sensor_cfg_->sensor_model == drivers::SensorModel::SEYOND_ROBIN_W &&
    calibration_file_path != "") {
    try {
      RCLCPP_INFO(logger_, "Reading calibration from file.");
      auto status = calib->LoadFromFile(calibration_file_path);
      if (status != Status::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Could not read calibration data: " << status_);
      } else {
        RCLCPP_INFO_STREAM(logger_, "Read calibration data from " << calibration_file_path);
      }
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Could not read calibration data from file: " << e.what());
    }
  }
  return calib;
}

void SeyondDecoderWrapper::ProcessCloudPacket(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  bool publish_packets =
    hw_interface_ && (packets_pub_->get_subscription_count() > 0 ||
                      packets_pub_->get_intra_process_subscription_count() > 0);
  bool has_scanned = false;

  // ////////////////////////////////////////
  // Decode packet
  // ////////////////////////////////////////

  if (decode_) {
    std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
    nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;
    {
      std::lock_guard lock(mtx_driver_ptr_);
      pointcloud_ts = driver_ptr_->ParseCloudPacket(packet_msg->data);
      pointcloud = std::get<0>(pointcloud_ts);
    }

    if (pointcloud != nullptr) {
      // ////////////////////////////////////////
      // If scan completed, publish pointcloud
      // ////////////////////////////////////////

      // A pointcloud has been produced, reset the watchdog timer
      has_scanned = true;
      if (
        nebula_points_pub_->get_subscription_count() > 0 ||
        nebula_points_pub_->get_intra_process_subscription_count() > 0) {
        auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::duration<double>(std::get<1>(pointcloud_ts)))
                             .count();
        ros_pc_msg_ptr->header.stamp = rclcpp::Time(nanoseconds);
        PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
      }
    }
  } else {
    {
      std::lock_guard lock(mtx_driver_ptr_);
      has_scanned = driver_ptr_->PeekCloudPacket(packet_msg->data);
    }
  }

  if (has_scanned && publish_packets) {
    // Publish scan message only if it has been written to
    if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
      packets_pub_->publish(std::move(current_scan_msg_));
      current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    }
  }

  // ////////////////////////////////////////
  // Accumulate packets for recording
  // ////////////////////////////////////////

  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (publish_packets) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
      // Add the calibration packet
      if (calibration_cfg_ptr_) {
        if (
          (!calibration_cfg_ptr_->GetCalibrationString().empty()) &&
          (sensor_cfg_->sensor_model == drivers::SensorModel::SEYOND_ROBIN_W)) {
          const auto now = std::chrono::high_resolution_clock::now();
          const auto timestamp_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

          auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
          msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
          msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
          std::string calibration_string = calibration_cfg_ptr_->GetCalibrationString();
          std::vector<uint8_t> calibration_packet(
            calibration_string.begin(), calibration_string.end());
          msg_ptr->data.swap(calibration_packet);
          current_scan_msg_->packets.emplace_back(*msg_ptr);
        }
      }
    }
    current_scan_msg_->packets.emplace_back(*packet_msg);
  }
  if (has_scanned) cloud_watchdog_->update();
}

void SeyondDecoderWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

nebula::Status SeyondDecoderWrapper::Status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->GetStatus();
}
}  // namespace ros
}  // namespace nebula
