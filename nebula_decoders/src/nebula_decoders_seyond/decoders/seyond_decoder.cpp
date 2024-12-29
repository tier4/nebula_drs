#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_decoder.hpp"

#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>

namespace nebula
{
namespace drivers
{
namespace seyond_packet
{

float SeyondDecoder::sin_table_[kAngleTableSize + 1];
float SeyondDecoder::cos_table_[kAngleTableSize + 1];
int32_t SeyondDecoder::asin_table_[2 * kASinTableSize];
int32_t SeyondDecoder::atan_table_[2 * kATanTableSize];

// for robinW
const uint8_t SeyondDecoder::robinw_channel_mapping[48] = {
  0, 4, 8,  12, 16, 20, 24, 28, 32, 36, 40, 44, 1, 5, 9,  13, 17, 21, 25, 29, 33, 37, 41, 45,
  2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 3, 7, 11, 15, 19, 23, 27, 31, 35, 39, 43, 47,
};

const uint8_t SeyondDecoder::robine_channel_mapping[48] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

int SeyondDecoder::v_angle_offset_[SEYOND_ITEM_TYPE_MAX][kSeyondChannelNumber];
int8_t SeyondDecoder::nps_adjustment_[kVTableSize_][kHTableSize_][kSeyondChannelNumber]
                                     [kXZSize_];  // NOLINT
const double SeyondDecoder::kAdjustmentUnitInMeter_ = 0.0025;
const double SeyondDecoder::kAdjustmentUnitInMeterRobin_ = 0.001;
int8_t SeyondDecoder::robin_nps_adjustment_[kRobinScanlines_][kHTableSize_][kXYZSize_];  // NOLINT

SeyondDecoder::SeyondDecoder(
  const std::shared_ptr<const SeyondSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const SeyondCalibrationConfiguration> & calibration_configuration)
: sensor_configuration_(sensor_configuration),
  calibration_configuration_(calibration_configuration),
  logger_(rclcpp::get_logger("SeyondDecoder"))
{
  logger_.set_level(rclcpp::Logger::Level::Debug);
  RCLCPP_INFO_STREAM(logger_, "Sensor configured");

  decode_pc_.reset(new NebulaPointCloud);
  output_pc_.reset(new NebulaPointCloud);

  // TODO(drwnz): Find the right values for each sensor
  decode_pc_->reserve(2000000);
  output_pc_->reserve(2000000);

  xyz_from_sphere_.resize(kConvertSize);
  setup_table_(kRadPerSeyondAngleUnit);
  init_f();

  // TODO: add to functions
  if (sensor_configuration->sensor_model == nebula::drivers::SensorModel::SEYOND_ROBIN_W) {
    size_t table_packet_size = sizeof(SeyondDataPacket) + sizeof(SeyondAngleHVTable);
    auto * new_anglehv_table = reinterpret_cast<SeyondDataPacket *>(new char[table_packet_size]);
    memset(new_anglehv_table, 0, table_packet_size);

    if (calibration_configuration_->GetCalibrationString().length() == table_packet_size) {
      memcpy(
        new_anglehv_table, calibration_configuration_->GetCalibrationString().c_str(),
        table_packet_size);

      // Copy out the table
      AngleHV tmp_table[kPolygonMaxFacets][kPolygonTableSize][kMaxSet][kMaxReceiverInSet];
      memcpy(
        tmp_table, reinterpret_cast<const SeyondAngleHVTable *>(new_anglehv_table->payload)->table,
        sizeof(tmp_table));

      new_anglehv_table->common.version.magic_number = kSeyondMagicNumberDataPacket;
      new_anglehv_table->type = SEYOND_ROBINW_ITEM_TYPE_ANGLEHV_TABLE;
      new_anglehv_table->common.size = sizeof(SeyondAngleHVTable) + sizeof(SeyondDataPacket);
      reinterpret_cast<SeyondAngleHVTable *>(new_anglehv_table->payload)
        ->version_number.major_version = kSeyondAngleHVTableVersionMajor;
      reinterpret_cast<SeyondAngleHVTable *>(new_anglehv_table->payload)
        ->version_number.minor_version = kSeyondAngleHVTableVersionMinor;
      reinterpret_cast<SeyondAngleHVTable *>(new_anglehv_table->payload)->id = 0;
      new_anglehv_table->item_number = 1;
      new_anglehv_table->is_first_sub_frame = 1;
      new_anglehv_table->is_last_sub_frame = 1;
      new_anglehv_table->sub_idx = 0;
      new_anglehv_table->item_size = sizeof(SeyondAngleHVTable);
      memcpy(
        reinterpret_cast<SeyondAngleHVTable *>(new_anglehv_table->payload)->table, tmp_table,
        sizeof(tmp_table));
      anglehv_table_ = std::as_const(new_anglehv_table);
    } else {
      RCLCPP_WARN_STREAM(
        logger_,
        "No calibration data found from the sensor or calibration file. Checking in packets");
    }
  }
}

template <typename PointType>
void SeyondDecoder::point_xyz_data_parse_(
  bool is_en_data, bool is_use_refl, uint32_t point_num, PointType point_ptr)
{
  for (uint32_t i = 0; i < point_num; ++i, ++point_ptr) {
    drivers::NebulaPoint point{};
    if (point_ptr->channel >= kSeyondChannelNumber) {
      RCLCPP_ERROR_STREAM(logger_, "bad channel " << point_ptr->channel);
      continue;
    }

    if (
      (point_ptr->radius < sensor_configuration_->min_range) ||
      (point_ptr->radius > sensor_configuration_->max_range)) {
      continue;
    }

    if constexpr (std::is_same<PointType, const SeyondEnXyzPoint *>::value) {
      if (is_use_refl) {
        point.intensity = point_ptr->reflectance;
      } else {
        point.intensity = point_ptr->intensity;
      }
    } else if constexpr (std::is_same<PointType, const SeyondXyzPoint *>::value) {
      point.intensity = point_ptr->refl;
    }

    point.time_stamp = static_cast<uint32_t>(point_ptr->ts_10us) * 10000;
    point.distance = static_cast<float>(point_ptr->radius)/400.0;
    point.x = point_ptr->z;
    point.y = -(point_ptr->y);
    point.z = point_ptr->x;
    decode_pc_->points.emplace_back(point);
  }
}

bool SeyondDecoder::IsPacketValid(const std::vector<uint8_t> & buffer)
{
  uint32_t send_packet_size = 0;
  std::memcpy(&send_packet_size, &buffer[kSeyondPktSizeSectionIndex], kSeyondPktSizeSectionLength);

  if (buffer.size() < send_packet_size) {
    std::cout << "receive buffer size " << buffer.size() << " < packet size " << send_packet_size
              << std::endl;
    return false;
  }

  uint16_t magic_number = ((buffer[1] << 8) | buffer[0]);
  uint16_t packet_type = buffer[kSeyondPktTypeIndex];

  // check packet is data packet
  // TODO(drwnz): [Falcon] sometimes report data packet as 51 not 1.
  // TODO(drwnz): [Falcon] here packet size check is used to prevent un-decodable short data packet
  // causing errors.
  if (
    (magic_number != kSeyondMagicNumberDataPacket) || (packet_type == 2) || (packet_type == 3) ||
    (send_packet_size < 60)) {
    return false;
  }

  return true;
}

void SeyondDecoder::ProtocolCompatibility(std::vector<uint8_t> & buffer)
{
  uint8_t major_version = buffer[kSeyondPktMajorVersionSection];
  uint32_t * packet_size = reinterpret_cast<uint32_t *>(&buffer[kSeyondPktSizeSectionIndex]);

  if (major_version == kSeyondProtocolMajorV1) {
    // add 16 bytes to the headr
    *packet_size += 16;
    buffer.insert(buffer.begin() + kSeyondProtocolOldHeaderLen, 16, 0);
  }
}

void SeyondDecoder::data_packet_parse_(const SeyondDataPacket * pkt)
{
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;
  // adapt different data structures form different lidar
  if (is_en_xyz_data(pkt->type)) {
    const SeyondEnXyzPoint * pt = reinterpret_cast<const SeyondEnXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(SeyondDataPacket));
    point_xyz_data_parse_<const SeyondEnXyzPoint *>(
      true, pkt->use_reflectance, pkt->item_number, pt);
  } else {
    const SeyondXyzPoint * pt = reinterpret_cast<const SeyondXyzPoint *>(
      reinterpret_cast<const char *>(pkt) + sizeof(SeyondDataPacket));
    point_xyz_data_parse_<const SeyondXyzPoint *>(
      false, pkt->use_reflectance, pkt->item_number, pt);
  }
  output_scan_timestamp_ns_ = pkt->common.ts_start_us * 1000;
}

void SeyondDecoder::compact_data_packet_parse_(const SeyondDataPacket * pkt)
{
  uint32_t return_number;
  uint32_t unit_size;
  if (
    pkt->multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
    pkt->multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
    unit_size = sizeof(SeyondCoBlock2);
    return_number = 2;
  } else if (pkt->multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
    unit_size = sizeof(SeyondCoBlock1);
    return_number = 1;
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Invalid return mode");
  }

  const SeyondCoBlock * block = reinterpret_cast<const SeyondCoBlock *>(pkt->payload);
  for (uint32_t i = 0; i < pkt->item_number;
       i++, block = reinterpret_cast<const SeyondCoBlock *>(
              reinterpret_cast<const char *>(block) + unit_size)) {
    SeyondCoBlockFullAngles full_angles{};
    get_block_full_angles(
      &full_angles, block->header,
      reinterpret_cast<const char *>(
        reinterpret_cast<const SeyondAngleHVTable *>(anglehv_table_->payload)->table));

    const uint8_t * channel_mapping = &SeyondDecoder::robinw_channel_mapping[0];
    const uint32_t tdc_channel_number = SeyondDecoder::RobinWTDCChannelNumber;

    for (uint32_t channel = 0; channel < kSeyondCompactChannelNumber; channel++) {
      for (uint32_t m = 0; m < return_number; m++) {
        const SeyondCoChannelPoint & pt = block->points[seyondcoblock_get_idx(channel, m)];
        SeyondXyzrD xyzr;
        uint32_t scan_id = 0;
        if (
          pt.radius < sensor_configuration_->min_range ||
          pt.radius > sensor_configuration_->max_range) {
          /*&& is_robinw_inside_fov_point(full_angles.angles[channel])*/
          int index = block->header.scan_id * kMaxReceiverInSet + channel;
          scan_id = channel_mapping[index] + block->header.facet * tdc_channel_number;
          get_xyzr_meter(full_angles.angles[channel], pt.radius, scan_id, &xyzr);

          drivers::NebulaPoint point{};
          point.x = xyzr.z;
          point.y = -(xyzr.y);
          point.z = xyzr.x;
          point.azimuth = static_cast<float>(full_angles.angles[channel].h_angle * kRadPerSeyondAngleUnit);
          point.elevation = static_cast<float>(full_angles.angles[channel].v_angle * kRadPerSeyondAngleUnit);
          point.distance = static_cast<float>(pt.radius)/400.0;
          point.intensity = std::ceil(static_cast<double>(pt.refl) * 255 / 4095);
          point.time_stamp = static_cast<uint32_t>(block->header.ts_10us) * 10000;
          decode_pc_->points.emplace_back(point);
        }
      }
    }
  }
  output_scan_timestamp_ns_ = pkt->common.ts_start_us * 1000;
}

int SeyondDecoder::unpack(const std::vector<uint8_t> & packet, bool decode)
{
  std::vector<uint8_t> packet_copy = packet;
  if (!IsPacketValid(packet_copy)) {
    return -1;
  }

  ProtocolCompatibility(packet_copy);
  const SeyondDataPacket * seyond_pkt =
    reinterpret_cast<const SeyondDataPacket *>(reinterpret_cast<const char *>(&packet_copy[0]));

  bool decodable = is_sphere_data(seyond_pkt->type) || is_xyz_data(seyond_pkt->type) ||
                   is_robinw_compact_data(seyond_pkt->type);

  if (decodable) {
    if (current_packet_id_ == 0) {
      current_packet_id_ = seyond_pkt->idx;
      RCLCPP_INFO_STREAM(logger_, "First packet received");
    }

    if (has_scanned_) {
      has_scanned_ = false;
    }

    // Publish the whole frame data if scan is complete
    // TODO(drwnz): have to handle out-of-order packets. Currently just tossing anything that
    // arrives out of order.
    if (current_packet_id_ != seyond_pkt->idx) {
      // std::cout << "Old packet ID: " << current_packet_id_ << ", New packet ID: " << seyond_pkt->idx
      //           << " No. points: " << decode_pc_->size() << std::endl;
      if ((current_packet_id_ < seyond_pkt->idx) || (seyond_pkt->idx == 0)) {
        if (decode) {
          std::swap(decode_pc_, output_pc_);
          decode_pc_->clear();
        }
        has_scanned_ = true;
        current_packet_id_ = seyond_pkt->idx;
      } else {
        RCLCPP_INFO_STREAM(logger_, "Packet arrived out of order, discarded");
        return -1;
      }
    }

    if (decode) {
      // std::cout << "Packet type: " << seyond_pkt->type << std::endl;
      if (is_sphere_data(seyond_pkt->type)) {
        // convert sphere to xyz
        bool ret_val = convert_to_xyz_pointcloud(
          *seyond_pkt, reinterpret_cast<SeyondDataPacket *>(&xyz_from_sphere_[0]), kConvertSize,
          false);
        if (!ret_val) {
          RCLCPP_ERROR_STREAM(logger_, "convert_to_xyz_pointcloud failed");
          return -1;
        }
        data_packet_parse_(reinterpret_cast<SeyondDataPacket *>(&xyz_from_sphere_[0]));
      } else if (is_xyz_data(seyond_pkt->type)) {
        data_packet_parse_(seyond_pkt);
      } else if (is_robinw_compact_data(seyond_pkt->type)) {
        if (anglehv_table_ == nullptr) {
          RCLCPP_ERROR_STREAM(logger_, "No calibration data found, packet not decoded");
        } else {
          compact_data_packet_parse_(seyond_pkt);
        }
      }
    }
  } else if (is_anglehv_table(seyond_pkt->type)) {
    // If there is no calibration data currently available, apply
    if (anglehv_table_ == nullptr) {
      anglehv_table_ = seyond_pkt;
      RCLCPP_INFO_STREAM(logger_, "Calibration data found in packets, decoding will be attempted");

      size_t table_packet_size = sizeof(SeyondDataPacket) + sizeof(SeyondAngleHVTable);
      auto * new_anglehv_table = reinterpret_cast<SeyondDataPacket *>(new char[table_packet_size]);

      memcpy(new_anglehv_table, seyond_pkt, table_packet_size);
      anglehv_table_ = std::as_const(new_anglehv_table);
    }
  } else {
    RCLCPP_ERROR_STREAM(logger_, "cframe type" << seyond_pkt->type << "is not supported");
  }
  decode_scan_timestamp_ns_ = seyond_pkt->common.ts_start_us * 1000;

  if (has_scanned_) {
    output_scan_timestamp_ns_ = decode_scan_timestamp_ns_;
  }
  return 0;
}

std::tuple<drivers::NebulaPointCloudPtr, double> SeyondDecoder::getPointcloud()
{
  double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
  return std::make_pair(output_pc_, scan_timestamp_s);
}

double SeyondDecoder::lookup_cos_table_in_unit(int i)
{
  return cos_table_[i];
}

double SeyondDecoder::lookup_sin_table_in_unit(int i)
{
  return sin_table_[i];
}

void SeyondDecoder::setup_table_(double seyond_angle_unit)
{
  for (int32_t i = 0; i <= kAngleTableSize; ++i) {
    double angle = i * kRadPerSeyondAngleUnit;
    cos_table_[i] = cos(angle);
    sin_table_[i] = sin(angle);
  }

  for (int32_t i = -kASinTableSize; i < kASinTableSize; i++) {
    asin_table_[i + kASinTableSize] =
      static_cast<int32_t>(asin(i / static_cast<double>(kAsinTableScale)) / seyond_angle_unit);
  }

  for (int32_t i = -kATanTableSize; i < kATanTableSize; i++) {
    atan_table_[i + kATanTableSize] =
      static_cast<int32_t>(atan(i / static_cast<double>(kAtanTableScale)) / seyond_angle_unit);
  }
}

void SeyondDecoder::init_f(void)
{
  init_f_falcon();
  init_f_robin();
}

int SeyondDecoder::init_f_falcon(void)
{
  for (uint32_t ich = 0; ich < kSeyondChannelNumber; ich++) {
    v_angle_offset_[SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD][ich] = ich * kSeyondFaconVAngleDiffBase;
    // falconII NT3
    v_angle_offset_[SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD][ich] =
      ich * kSeyondFaconVAngleDiffBase;
  }
  // init the nps_adjustment_
  size_t input_size =
    (kVTableEffeHalfSize_ * 2 + 1) * (kHTableEffeHalfSize_ * 2 + 1) * 2 * kSeyondChannelNumber;
  memset(nps_adjustment_, 0, sizeof(nps_adjustment_));
  static double k_max[2] = {-100, -100};
  static double k_min[2] = {100, 100};
  for (uint32_t v = 0; v < kVTableEffeHalfSize_ * 2 + 1; v++) {
    for (uint32_t h = 0; h < kHTableEffeHalfSize_ * 2 + 1; h++) {
      for (uint32_t ich = 0; ich < kSeyondChannelNumber; ich++) {
        for (uint32_t xz = 0; xz < kXZSize_; xz++) {
          double k = kSeyondPs2Nps[xz][ich][v][h];
          double u = k / kAdjustmentUnitInMeter_;
          double q = std::floor(u + 0.5);
          nps_adjustment_[v][h][ich][xz] = q;
          k_max[xz] = std::max(k_max[xz], k);
          k_min[xz] = std::min(k_min[xz], k);
        }
      }
    }
  }
  return 0;
}

int SeyondDecoder::init_f_robin(void)
{
  for (uint32_t ich = 0; ich < kSeyondChannelNumber; ich++) {
    v_angle_offset_[SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD][ich] =
      ich * kSeyondRobinEVAngleDiffBase;
    v_angle_offset_[SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD][ich] =
      ich * kSeyondRobinWVAngleDiffBase;
  }

  // init the nps_adjustment_
  size_t input_size = kRobinScanlines_ * (kHRobinTableEffeHalfSize_ * 2 + 1) * kXYZSize_;
  memset(robin_nps_adjustment_, 0, sizeof(robin_nps_adjustment_));
  static double k_max[3] = {-200, -200, -200};
  static double k_min[3] = {200, 200, 200};
  for (uint32_t scan_id = 0; scan_id < kRobinScanlines_; scan_id++) {
    for (uint32_t h = 0; h < kHRobinTableEffeHalfSize_ * 2 + 1; h++) {
      for (uint32_t xyz = 0; xyz < kXYZSize_; xyz++) {
        double k = robinW_kSeyondPs2Nps[xyz][scan_id][h];
        double u = k * 0.001 / kAdjustmentUnitInMeterRobin_;
        double q = std::floor(u + 0.5);
        robin_nps_adjustment_[scan_id][h][xyz] = q;
        k_max[xyz] = std::max(k_max[xyz], k);
        k_min[xyz] = std::min(k_min[xyz], k);
      }
    }
  }
  return 0;
}

void SeyondDecoder::get_xyzr_meter(
  const SeyondBlockAngles angles, const uint32_t radius_unit, const uint32_t channel,
  SeyondXyzrD * result, SeyondItemType type)
{
  if (type == SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD) {
    result->radius = radius_unit * kMeterPerSeyondDistanceUnit200;

  } else {
    result->radius = radius_unit * kMeterPerSeyondDistanceUnit400;
  }
  double t;
  if (angles.v_angle >= 0) {
    t = result->radius * lookup_cos_table_in_unit(angles.v_angle);
    result->x = result->radius * lookup_sin_table_in_unit(angles.v_angle);
  } else {
    t = result->radius * lookup_cos_table_in_unit(-angles.v_angle);
    result->x = -result->radius * lookup_sin_table_in_unit(-angles.v_angle);
  }
  if (angles.h_angle >= 0) {
    result->y = t * lookup_sin_table_in_unit(angles.h_angle);
    result->z = t * lookup_cos_table_in_unit(angles.h_angle);
  } else {
    result->y = -t * lookup_sin_table_in_unit(-angles.h_angle);
    result->z = t * lookup_cos_table_in_unit(-angles.h_angle);
  }

  // don't do nps for robinE, no data yet
  if (type == SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD) {
    return;
  }

  // falconI & falconII
  if (
    type == SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD ||
    type == SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
    double x_adj, z_adj;
    lookup_xz_adjustment_(angles, channel, &x_adj, &z_adj);
    result->x += x_adj;
    result->z += z_adj;
  } else if (
    type == SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD ||
    type == SEYOND_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD) {
    double adj[3];
    lookup_xyz_adjustment_(angles, channel, adj);
    result->x += adj[0];
    result->y += adj[1];
    result->z += adj[2];
  }

  return;
}

bool SeyondDecoder::check_data_packet(const SeyondDataPacket & pkt, size_t size)
{
  if (pkt.common.version.magic_number != kSeyondMagicNumberDataPacket) {
    std::cout << "bad magic " << pkt.common.version.magic_number << std::endl;
    return false;
  }
  if (size && (pkt.common.size > size)) {
    std::cout << "bad size " << size << " " << pkt.common.size << std::endl;
    return false;
  }
  bool is_data = true;
  switch (pkt.type) {
    case SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD:
      if (pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
        if (pkt.item_size != sizeof(SeyondBlock1)) {
          std::cout << "bad block1 item size " << pkt.item_size << " " << sizeof(SeyondBlock1)
                    << std::endl;
          return false;
        }
      } else if (
        pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
        pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        if (pkt.item_size != sizeof(SeyondBlock2)) {
          std::cout << "bad block2 item size " << pkt.item_size << " " << sizeof(SeyondBlock2)
                    << std::endl;
          return false;
        }
      } else {
        std::cout << "bad return_mode " << pkt.multi_return_mode << std::endl;
        return false;
      }
      break;
    case SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD:
    case SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD:
    case SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD:
      if (pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
        if (pkt.item_size != sizeof(SeyondEnBlock1)) {
          std::cout << "bad block1 item size " << pkt.item_size << " " << sizeof(SeyondEnBlock1)
                    << std::endl;
          return false;
        }
      } else if (
        pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
        pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        if (pkt.item_size != sizeof(SeyondEnBlock2)) {
          std::cout << "bad block2 item size " << pkt.item_size << " " << sizeof(SeyondEnBlock2)
                    << std::endl;
          return false;
        }
      } else {
        std::cout << "bad return_mode " << pkt.multi_return_mode << std::endl;
        return false;
      }
      break;
    case SEYOND_ITEM_TYPE_XYZ_POINTCLOUD:
      if (pkt.item_size != sizeof(SeyondXyzPoint)) {
        std::cout << "bad SeyondXyzPoint item size " << pkt.item_size << std::endl;
        return false;
      }
      break;
    case SEYOND_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD:
    case SEYOND_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD:
    case SEYOND_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD:
      if (pkt.item_size != sizeof(SeyondEnXyzPoint)) {
        std::cout << "bad SeyondEnXyzPoint item size " << pkt.item_size << std::endl;
        return false;
      }
      break;
    default:
      is_data = false;
      break;
  }

  if (is_data) {
    size_t s = get_data_packet_size(
      SeyondItemType(pkt.type), pkt.item_number, SeyondMultipleReturnMode(pkt.multi_return_mode));
    if (pkt.common.size != s) {
      std::cout << "bad size " << s << " " << pkt.common.size << std::endl;
      return false;
    }
    if (pkt.common.version.major_version > kSeyondMajorVersionDataPacket) {
      std::cout << "please upgrade client sdk, lidar protocol major version:"
                << pkt.common.version.major_version
                << " sdk major version:" << kSeyondMajorVersionDataPacket << std::endl;
      return false;
    }
    return true;
  } else if (pkt.type == SEYOND_ITEM_TYPE_MESSAGE || pkt.type == SEYOND_ITEM_TYPE_MESSAGE_LOG) {
    if (pkt.item_number != 1) {
      std::cout << "bad item_number " << pkt.item_number << std::endl;
      return false;
    }
    const SeyondMessage * messages = reinterpret_cast<const SeyondMessage *>(pkt.payload);
    if (
      static_cast<uint32_t>(pkt.item_size) != messages[0].size ||
      pkt.item_size <= sizeof(SeyondMessage)) {
      std::cout << "bad message size " << pkt.item_size << " " << messages[0].size << std::endl;
      return false;
    }
    if (pkt.common.size != pkt.item_size + sizeof(SeyondDataPacket)) {
      std::cout << "bad message size " << pkt.common.size << " "
                << pkt.item_size + sizeof(SeyondDataPacket) << std::endl;
      return false;
    }
    return true;
  } else {
    std::cout << "bad type " << pkt.type << std::endl;
    return false;
  }
}

bool SeyondDecoder::convert_to_xyz_pointcloud(
  const SeyondDataPacket & src, SeyondDataPacket * dest, size_t dest_size, bool append)
{
  if (!is_sphere_data(src.type)) {
    std::cout << "invalid type: " << src.type << std::endl;
    return false;
  }
  if (!check_data_packet(src, 0)) {
    std::cout << "invalid src datapacket" << std::endl;
    return false;
  }

  uint32_t item_count = 0;
  uint32_t dummy_count = 0;
  item_count = get_points_count(src);

  if (append) {
    if (!check_data_packet(*dest, dest_size)) {
      std::cout << "invalid dest datapacket" << std::endl;
      return false;
    }
    item_count += dest->item_number;
  }

  size_t required_size = 0;
  uint16_t time_adjust_10us = 0;
  if (!append) {
    required_size = sizeof(SeyondDataPacket);
    if (required_size > dest_size) {
      std::cout << "not enough size "
                << "required_size " << required_size << "dest_size " << dest_size;
      return false;
    }
    memcpy(dest, &src, sizeof(SeyondDataPacket));
    if (
      src.type == SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD ||
      SEYOND_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD) {
      dest->type = SEYOND_ITEM_TYPE_XYZ_POINTCLOUD;
      dest->item_size = sizeof(SeyondXyzPoint);
    } else {
      dest->type += 1;  // robin & falconIII SeyondItemType xyz = sphere+1
      dest->item_size = sizeof(SeyondEnXyzPoint);
    }
    dest->item_number = 0;
  } else {
    required_size = dest->common.size;
    if (src.common.ts_start_us < dest->common.ts_start_us) {
      std::cout << "cannot merge earlier packet " << src.common.ts_start_us << " "
                << dest->common.ts_start_us << std::endl;
      return false;
    }
    time_adjust_10us = (src.common.ts_start_us - dest->common.ts_start_us) / 10;
  }

  {
    if (src.type == SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD) {
      (void)iterate_seyond_data_packet_cpoints<
        SeyondBlock, SeyondBlockHeader, SeyondBlock1, SeyondBlock2, SeyondChannelPoint>(
        src,
        [&](
          const SeyondDataPacketPointsCallbackParams<SeyondBlock, SeyondChannelPoint> & in_params) {
          if (in_params.pt.radius > 0) {
            SeyondXyzPoint & ipt = dest->xyz_points[dest->item_number];
            required_size += static_cast<uint32_t>(sizeof(SeyondXyzPoint));
            if (required_size > dest_size) {
              std::cerr << "no enough size required_size:" << required_size
                        << ",dest_size:" << dest_size << std::endl;
            } else {
              (void)get_xyz_point(
                in_params.block.header, in_params.pt, in_params.angle.angles[in_params.channel],
                in_params.channel, &ipt);
              ipt.multi_return = in_params.multi_return;
              ipt.is_2nd_return = in_params.pt.is_2nd_return;
              ipt.ts_10us += time_adjust_10us;
              dest->item_number++;
            }
          }
        });
    } else if (src.type == SEYOND_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD) {
      // (void)iterate_seyond_data_packet_co_cpoints<
      //   SeyondCoBlock, SeyondCoBlockHeader, SeyondCoBlock1, SeyondCoBlock2,
      //   SeyondCoChannelPoint>( src,
      //   [&](
      //     const SeyondDataPacketCoPointsCallbackParams<SeyondCoBlock, SeyondCoChannelPoint> &
      //     in_params) { if (in_params.pt.radius > 0) {
      //       SeyondXyzPoint & ipt = dest->xyz_points[dest->item_number];
      //       required_size += static_cast<uint32_t>(sizeof(SeyondXyzPoint));
      //       if (required_size > dest_size) {
      //         std::cerr << "no enough size required_size:" << required_size
      //                   << ",dest_size:" << dest_size << std::endl;
      //       } else {
      //         (void)get_xyz_point(
      //           in_params.block.header, in_params.pt, in_params.angle.angles[in_params.channel],
      //           in_params.channel, &ipt);
      //         ipt.multi_return = in_params.multi_return;
      //         ipt.is_2nd_return = in_params.pt.is_2nd_return;
      //         ipt.ts_10us += time_adjust_10us;
      //         dest->item_number++;
      //       }
      //     }
      //   },
      //   "fixme");
    } else {
      (void)iterate_seyond_data_packet_cpoints<
        SeyondEnBlock, SeyondEnBlockHeader, SeyondEnBlock1, SeyondEnBlock2, SeyondEnChannelPoint>(
        src, [&](const SeyondDataPacketPointsCallbackParams<SeyondEnBlock, SeyondEnChannelPoint> &
                   in_params) {
          if (in_params.pt.radius > 0) {
            SeyondEnXyzPoint & ipt = dest->en_xyz_points[dest->item_number];
            required_size += static_cast<uint32_t>(sizeof(SeyondEnXyzPoint));
            if (required_size > dest_size) {
              std::cerr << "no enough size required_size en:" << required_size
                        << ",dest_size:" << dest_size << std::endl;
            } else {
              (void)get_xyz_point(
                in_params.block.header, in_params.pt, in_params.angle.angles[in_params.channel],
                in_params.channel, &ipt, static_cast<SeyondItemType>(in_params.pkt.type));
              ipt.multi_return = in_params.multi_return;
              ipt.is_2nd_return = in_params.pt.is_2nd_return;
              ipt.ts_10us += time_adjust_10us;
              dest->item_number++;
            }
          }
        });
    }
  }

  if (item_count != dest->item_number) {
    std::cout << "item number: " << dest->item_number << " item count " << item_count << std::endl;
  }

  dest->common.size = required_size;

  if (!check_data_packet(*dest, dest_size)) {
    std::cout << "invalid dest datapacket" << std::endl;
    return false;
  }

  return true;
}

}  // namespace seyond_packet
}  // namespace drivers
}  // namespace nebula
