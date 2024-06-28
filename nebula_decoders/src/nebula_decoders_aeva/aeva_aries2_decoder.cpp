// Copyright 2024 TIER IV, Inc.

#include "nebula_decoders/nebula_decoders_aeva/aeva_aries2_decoder.hpp"

namespace nebula::drivers
{

void AevaAries2Decoder::processPointcloudMessage(const aeva::PointCloudMessage & message)
{
  DecoderState state{
    message.header.frame_sync_index, message.header.ns_per_index, message.header.line_index, 0,
    message.header.first_point_timestamp_ns};

  for (size_t i = 0; i < message.points.size(); ++i) {
    const auto & raw_point = message.points[i];

    // Point time increases every time a marker point (beam=0, peak=0) is encountered
    if (raw_point.beam_id == 0 && raw_point.peak_id == 0) {
      state.absolute_time_ns += state.time_per_marker_point_ns;
    }

    if (static_cast<ssize_t>(i) == state.new_frame_index) {
      std::scoped_lock lock(mtx_callback_);

      if (callback_) {
        callback_(std::move(cloud_state_.cloud), cloud_state_.timestamp);
      }
      // Cloud time gets reset below, on the first VALID point that will be
      // put in the cloud. This guarantees that the earliest point(s) in the cloud
      // have relative time 0
      cloud_state_ = {std::make_unique<AevaPointCloud>(), 0};
    }

    if (raw_point.line_transition) {
      state.line_index++;
    }

    if (!raw_point.valid || raw_point.signal_quality < 60) {
      continue;
    }

    if (cloud_state_.cloud->empty()) {
      cloud_state_.timestamp = state.absolute_time_ns;
    }

    AevaPoint point;

    point.distance = raw_point.range.value();
    point.azimuth = -raw_point.azimuth.value() * M_PI_2f;
    point.elevation = raw_point.elevation.value() * M_PI_4f;

    float xy_distance = point.distance * std::cos(point.elevation);
    point.x = xy_distance * std::sin(point.azimuth);
    point.y = xy_distance * std::cos(point.azimuth);
    point.z = point.distance * std::sin(point.elevation);

    point.v = raw_point.velocity.value();
    point.intensity = raw_point.intensity;

    point.time_stamp = state.absolute_time_ns - cloud_state_.timestamp;
    point.channel = state.line_index;

    cloud_state_.cloud->emplace_back(point);
  }
}

void AevaAries2Decoder::registerPointCloudCallback(
  std::function<void(std::unique_ptr<AevaPointCloud>, uint64_t)> callback)
{
  std::lock_guard lock(mtx_callback_);
  callback_ = std::move(callback);
}
}  // namespace nebula::drivers
