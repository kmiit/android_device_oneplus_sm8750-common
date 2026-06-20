/*
 * Copyright (C) 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SensorsSubHal.h"

#include <algorithm>
#include <android-base/logging.h>
#include <dlfcn.h>
#include <hardware/sensors.h>

#include <vector>

using ::android::hardware::sensors::V2_0::implementation::ScopedWakelock;
using ::android::hardware::sensors::V2_1::implementation::ISensorsSubHal;

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {
namespace qsh_wrapper {

namespace {
constexpr auto kLibName = "sensors.qsh.so";

// This is larger than any sensor handle returned by the HAL
constexpr auto kWrappedSensorHandleBase = 0x10000;
inline int32_t IsWrappedHandle(int32_t sensor_handle) {
    return sensor_handle >= kWrappedSensorHandleBase;
}
inline int32_t FromWrappedHandle(int32_t sensor_handle) {
    return IsWrappedHandle(sensor_handle) ? sensor_handle - kWrappedSensorHandleBase
                                          : sensor_handle;
}
inline int32_t ToWrappedHandle(int32_t sensor_handle) {
    return IsWrappedHandle(sensor_handle) ? sensor_handle
                                          : sensor_handle + kWrappedSensorHandleBase;
}

template <typename Operation>
Return<Result> ApplyToFusionSources(const std::vector<int32_t>& source_handles,
                                    Operation operation) {
    if (source_handles.empty()) {
        return Result::BAD_VALUE;
    }

    Result result = Result::OK;
    for (const auto source_handle : source_handles) {
        auto ret = operation(source_handle);
        if (!ret.isOk()) {
            return ret;
        }
        Result source_result = ret.withDefault(Result::BAD_VALUE);
        if (source_result != Result::OK && result == Result::OK) {
            result = source_result;
        }
    }
    return result;
}
};  // anonymous namespace

SensorsSubHal::SensorsSubHal()
    : lib_handle_(dlopen(kLibName, RTLD_NOW), [](void* p) {
          if (p) dlclose(p);
      }) {
    if (!lib_handle_) {
        LOG(FATAL) << __func__ << ": dlopen " << kLibName << " failed, exiting";
    }

    auto get_sub_hal = reinterpret_cast<ISensorsSubHal* (*)(uint32_t*)>(
            dlsym(lib_handle_.get(), "sensorsHalGetSubHal_2_1"));
    uint32_t version;
    impl_ = get_sub_hal(&version);
}

Return<Result> SensorsSubHal::setOperationMode(OperationMode mode) {
    return impl_->setOperationMode(mode);
}

Return<Result> SensorsSubHal::activate(int32_t sensor_handle, bool enabled) {
    if (sensor_handle == fusion_light_handle_) {
        return ApplyToFusionSources(fusion_light_.sourceHandles(), [&](int32_t source_handle) {
            return impl_->activate(source_handle, enabled);
        });
    }
    return impl_->activate(FromWrappedHandle(sensor_handle), enabled);
}

Return<Result> SensorsSubHal::batch(int32_t sensor_handle, int64_t sampling_period_ns,
                                    int64_t max_report_latency_ns) {
    if (sensor_handle == fusion_light_handle_) {
        return ApplyToFusionSources(fusion_light_.sourceHandles(), [&](int32_t source_handle) {
            return impl_->batch(source_handle, sampling_period_ns, max_report_latency_ns);
        });
    }
    return impl_->batch(FromWrappedHandle(sensor_handle), sampling_period_ns,
                        max_report_latency_ns);
}

Return<Result> SensorsSubHal::flush(int32_t sensor_handle) {
    if (sensor_handle == fusion_light_handle_) {
        return ApplyToFusionSources(fusion_light_.sourceHandles(), [&](int32_t source_handle) {
            return impl_->flush(source_handle);
        });
    }
    return impl_->flush(FromWrappedHandle(sensor_handle));
}

Return<void> SensorsSubHal::registerDirectChannel(const SharedMemInfo& mem,
                                                  ISensors::registerDirectChannel_cb _hidl_cb) {
    return impl_->registerDirectChannel(mem, _hidl_cb);
}

Return<Result> SensorsSubHal::unregisterDirectChannel(int32_t channel_handle) {
    return impl_->unregisterDirectChannel(channel_handle);
}

Return<void> SensorsSubHal::configDirectReport(int32_t sensor_handle, int32_t channel_handle,
                                               RateLevel rate,
                                               ISensors::configDirectReport_cb _hidl_cb) {
    return impl_->configDirectReport(FromWrappedHandle(sensor_handle), channel_handle, rate,
                                     _hidl_cb);
}

Return<void> SensorsSubHal::getSensorsList_2_1(ISensors::getSensorsList_2_1_cb _hidl_cb) {
    return impl_->getSensorsList_2_1([&](const auto& _hidl_out_list) {
        auto findSensorByType = [&](int32_t type) {
            return std::find_if(_hidl_out_list.begin(), _hidl_out_list.end(), [=](auto&& v) {
                return static_cast<int32_t>(v.type) == type;
            });
        };

        handle_type_.clear();
        fusion_light_.reset();
        fusion_light_handle_ = FusionLight::kInvalidSensorHandle;

        auto high_pwm_it = findSensorByType(FusionLight::kTypeHighPwmRgbSensor);
        auto rear_light_it = findSensorByType(FusionLight::kTypeRearLightSensor);
        auto fusion_rgb_it = findSensorByType(FusionLight::kTypeFusionRgbSensor);

        auto it = high_pwm_it;
        if (it == _hidl_out_list.end()) {
            it = rear_light_it;
        }
        if (it == _hidl_out_list.end()) {
            it = fusion_rgb_it;
        }

        if (high_pwm_it != _hidl_out_list.end()) {
            fusion_light_.setSourceHandleForType(static_cast<int32_t>(high_pwm_it->type),
                                                high_pwm_it->sensorHandle);
        }
        if (rear_light_it != _hidl_out_list.end()) {
            fusion_light_.setSourceHandleForType(static_cast<int32_t>(rear_light_it->type),
                                                rear_light_it->sensorHandle);
        }
        if (high_pwm_it == _hidl_out_list.end() && rear_light_it == _hidl_out_list.end() &&
            fusion_rgb_it != _hidl_out_list.end()) {
            fusion_light_.setSourceHandleForType(static_cast<int32_t>(fusion_rgb_it->type),
                                                fusion_rgb_it->sensorHandle);
        }

        if (it != _hidl_out_list.end() && fusion_light_.hasSource()) {
            auto last = _hidl_out_list.size();
            auto sensors = hidl_vec<SensorInfo>(last + 1);
            std::copy(_hidl_out_list.begin(), _hidl_out_list.end(), sensors.begin());

            sensors[last] = *it;
            sensors[last].sensorHandle = ToWrappedHandle(sensors[last].sensorHandle);
            sensors[last].name = "OPLUS Fusion Light Sensor";
            sensors[last].type = SensorType::LIGHT;
            sensors[last].typeAsString = "";  // Empty string is valid for known types

            fusion_light_handle_ = sensors[last].sensorHandle;
            handle_type_[fusion_light_handle_] = it->type;

            LOG(INFO) << "Fusion light source found: " << fusion_light_.sourceSummary();
            _hidl_cb(sensors);
        } else {
            _hidl_cb(_hidl_out_list);
        }
    });
}

Return<Result> SensorsSubHal::injectSensorData_2_1(const Event& event) {
    if (IsWrappedHandle(event.sensorHandle)) {
        auto it = handle_type_.find(event.sensorHandle);
        if (it == handle_type_.end()) {
            return Result::BAD_VALUE;
        }

        auto event_copy = event;
        event_copy.sensorHandle = FromWrappedHandle(event.sensorHandle);
        event_copy.sensorType = it->second;
        return impl_->injectSensorData_2_1(event_copy);
    }
    return impl_->injectSensorData_2_1(event);
}

Return<void> SensorsSubHal::debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args) {
    return impl_->debug(fd, args);
}

const std::string SensorsSubHal::getName() {
    return impl_->getName();
}

Return<Result> SensorsSubHal::initialize(const sp<IHalProxyCallback>& hal_proxy_callback) {
    hal_proxy_callback_ = hal_proxy_callback;
    return impl_->initialize(this);
}

Return<void> SensorsSubHal::onDynamicSensorsConnected(
        const hidl_vec<V1_0::SensorInfo>& sensor_infos) {
    return hal_proxy_callback_->onDynamicSensorsConnected(sensor_infos);
}

Return<void> SensorsSubHal::onDynamicSensorsDisconnected(const hidl_vec<int32_t>& sensor_handles) {
    return hal_proxy_callback_->onDynamicSensorsDisconnected(sensor_handles);
}

Return<void> SensorsSubHal::onDynamicSensorsConnected_2_1(
        const hidl_vec<V2_1::SensorInfo>& sensor_infos) {
    return hal_proxy_callback_->onDynamicSensorsConnected_2_1(sensor_infos);
}

void SensorsSubHal::postEvents(const std::vector<Event>& events, ScopedWakelock wakelock) {
    std::vector<Event> wrapped_events;
    for (auto&& e : events) {
        if (fusion_light_handle_ != FusionLight::kInvalidSensorHandle &&
            fusion_light_.isSourceEvent(e)) {
            fusion_light_.updateSample(e);
            wrapped_events.emplace_back(fusion_light_.createLightEvent(e, fusion_light_handle_));
        }
    }
    if (wrapped_events.empty()) {
        hal_proxy_callback_->postEvents(events, std::move(wakelock));
    } else {
        wrapped_events.insert(wrapped_events.end(), events.begin(), events.end());
        hal_proxy_callback_->postEvents(wrapped_events, std::move(wakelock));
    }
}

ScopedWakelock SensorsSubHal::createScopedWakelock(bool lock) {
    return hal_proxy_callback_->createScopedWakelock(lock);
}

}  // namespace qsh_wrapper
}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android

ISensorsSubHal* sensorsHalGetSubHal_2_1(uint32_t* version) {
    static ::android::hardware::sensors::V2_1::subhal::implementation::qsh_wrapper::SensorsSubHal
            sub_hal;
    *version = SUB_HAL_2_1_VERSION;
    return &sub_hal;
}
