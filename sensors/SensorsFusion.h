/*
 * Copyright (C) 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <V2_1/SubHal.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {
namespace qsh_wrapper {

using ::android::hardware::sensors::V2_1::Event;

class FusionLight {
  public:
    static constexpr int32_t kInvalidSensorHandle = -1;
    static constexpr int32_t kTypeFusionRgbSensor = 33171001;
    static constexpr int32_t kTypeRearLightSensor = 33171055;
    static constexpr int32_t kTypeHighPwmRgbSensor = 33171070;

    void reset();
    void setSourceHandleForType(int32_t sensor_type, int32_t sensor_handle);
    bool isSourceEvent(const Event& event) const;
    bool isReportEvent(const Event& event) const;
    void updateSample(const Event& event);
    Event createLightEvent(const Event& trigger, int32_t fusion_light_handle) const;
    bool hasSource() const;
    int32_t primarySourceHandle() const;
    std::size_t sourceHandleCount() const;
    int32_t sourceHandleAt(std::size_t index) const;
    std::string sourceSummary() const;

  private:
    float calculateLux(const Event& trigger) const;
    float applyMedianEnqueue(float lux, float brightness, bool has_brightness) const;
    void resetMedianEnqueue() const;

    int32_t high_pwm_rgb_handle_ = kInvalidSensorHandle;
    int32_t rear_light_handle_ = kInvalidSensorHandle;
    int32_t fusion_rgb_handle_ = kInvalidSensorHandle;
    bool has_high_pwm_rgb_event_ = false;
    bool has_rear_light_event_ = false;
    bool has_rgb_event_ = false;
    Event last_high_pwm_rgb_event_{};
    Event last_rear_light_event_{};
    Event last_rgb_event_{};
    mutable std::vector<float> median_enqueue_values_;
    mutable std::size_t median_enqueue_head_ = 0;
    mutable int median_enqueue_size_ = 0;
};

}  // namespace qsh_wrapper
}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
