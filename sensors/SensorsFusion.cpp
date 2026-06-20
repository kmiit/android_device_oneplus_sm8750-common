/*
 * Copyright (C) 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SensorsFusion.h"

#include <aidl/vendor/oplus/hardware/displaypanelfeature/IDisplayPanelFeature.h>
#include <android/binder_auto_utils.h>
#include <android/binder_manager.h>
#include <android-base/file.h>
#include <android-base/logging.h>
#include <json/json.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {
namespace qsh_wrapper {

namespace {
using ::aidl::vendor::oplus::hardware::displaypanelfeature::IDisplayPanelFeature;
struct ChannelData {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

struct LuxCoeff {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

struct IrThreshold {
    int level = 0;
    float min = 0.0f;
    float max = 1.0f;
};

struct BrightnessRange {
    int level = 0;
    int min = 0;
    int max = 4095;
};

struct FusionProfile {
    bool loaded = false;
    bool apollo_brightness_supported = false;
    bool screen_off_cal_lux_supported = false;
    bool median_enqueue_supported = false;
    int apollo_brightness_max = 0;
    int brightness_max = 4095;
    int median_enqueue_event_period = 200;
    int median_enqueue_event_size = 6;
    int median_enqueue_event_index = 2;
    int ir_ratio_formula_type = 0;
    float low_light_accuracy = 0.0f;
    std::string path;
    std::vector<LuxCoeff> lux_coeff_lir;
    std::vector<LuxCoeff> lux_coeff_hir;
    std::vector<LuxCoeff> lux_coeff_super_hir;
    std::vector<LuxCoeff> lux_coeff_lir_screen_off;
    std::vector<LuxCoeff> lux_coeff_hir_screen_off;
    std::vector<LuxCoeff> lux_coeff_super_hir_screen_off;
    std::vector<IrThreshold> ir_thresholds;
    std::vector<int> c_zero_thresholds;
    std::vector<BrightnessRange> ir_brightness_ranges;
    std::vector<BrightnessRange> linearity_ranges;
};

struct FusionInput {
    float raw_lux = 0.0f;
    float brightness = 0.0f;
    float ir_ratio = 0.0f;
    bool has_raw_lux = false;
    bool has_brightness = false;
    bool has_ir_ratio = false;
    bool has_rgb = false;
    ChannelData channels;
    int32_t rgb_sensor_type = 0;
};

inline bool IsFinitePositive(float value) {
    return std::isfinite(value) && value > 0.0f;
}

inline bool IsFiniteNonNegative(float value) {
    return std::isfinite(value) && value >= 0.0f;
}

inline float Clamp(float value, float low, float high) {
    if (!std::isfinite(value)) {
        return low;
    }
    return std::min(std::max(value, low), high);
}

inline float ClampNonNegative(float value) {
    return Clamp(value, 0.0f, std::numeric_limits<float>::max());
}

float GetFloat(const Json::Value& value, float default_value = 0.0f) {
    if (value.isNumeric()) {
        return value.asFloat();
    }
    if (value.isString()) {
        char* end = nullptr;
        const auto result = std::strtof(value.asCString(), &end);
        return end != value.asCString() ? result : default_value;
    }
    return default_value;
}

int GetInt(const Json::Value& value, int default_value = 0) {
    return value.isInt() || value.isUInt() ? value.asInt() : default_value;
}

bool GetBool(const Json::Value& value, bool default_value = false) {
    return value.isBool() ? value.asBool() : default_value;
}

LuxCoeff ParseCoeff(const Json::Value& value) {
    return {
            GetFloat(value["ChannelR"]),
            GetFloat(value["ChannelG"]),
            GetFloat(value["ChannelB"]),
            GetFloat(value["ChannelC"]),
    };
}

std::vector<LuxCoeff> ParseCoeffArray(const Json::Value& values) {
    std::vector<LuxCoeff> coeffs;
    if (!values.isArray()) {
        return coeffs;
    }

    for (const auto& value : values) {
        const auto level = GetInt(value["Level"], static_cast<int>(coeffs.size()));
        if (level < 0) {
            continue;
        }
        if (static_cast<size_t>(level) >= coeffs.size()) {
            coeffs.resize(level + 1);
        }
        coeffs[level] = ParseCoeff(value);
    }
    return coeffs;
}

std::vector<IrThreshold> ParseIrThresholds(const Json::Value& values) {
    std::vector<IrThreshold> thresholds;
    if (!values.isArray()) {
        return thresholds;
    }

    for (const auto& value : values) {
        thresholds.push_back({
                GetInt(value["Level"], static_cast<int>(thresholds.size())),
                GetFloat(value["IR_Ratio_Min"]),
                GetFloat(value["IR_Ratio_Max"], 1.0f),
        });
    }
    std::sort(thresholds.begin(), thresholds.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.level < rhs.level;
    });
    return thresholds;
}

std::vector<int> ParseLevelIntArray(const Json::Value& values, const char* field) {
    std::vector<int> result;
    if (!values.isArray()) {
        return result;
    }

    for (const auto& value : values) {
        const auto level = GetInt(value["Level"], static_cast<int>(result.size()));
        if (level < 0) {
            continue;
        }
        if (static_cast<size_t>(level) >= result.size()) {
            result.resize(level + 1);
        }
        result[level] = GetInt(value[field]);
    }
    return result;
}

std::vector<BrightnessRange> ParseBrightnessRanges(const Json::Value& values) {
    std::vector<BrightnessRange> ranges;
    if (!values.isArray()) {
        return ranges;
    }

    for (const auto& value : values) {
        ranges.push_back({
                GetInt(value["Level"], static_cast<int>(ranges.size())),
                GetInt(value["BrightnessMin"]),
                GetInt(value["BrightnessMax"], 4095),
        });
    }
    std::sort(ranges.begin(), ranges.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.level < rhs.level;
    });
    return ranges;
}

bool ParseProfile(const std::string& path, FusionProfile* profile) {
    std::string content;
    if (!android::base::ReadFileToString(path, &content)) {
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(content.data(), content.data() + content.size(), &root, &errors)) {
        LOG(WARNING) << "Failed to parse fusion light profile " << path << ": " << errors;
        return false;
    }

    profile->apollo_brightness_supported =
            GetBool(root["CommonConfig"]["ApolloBrightnessSupported"]);
    profile->apollo_brightness_max = GetInt(root["CommonConfig"]["ApolloBrightnessMax"]);
    profile->brightness_max = GetInt(root["CommonConfig"]["BrightnessMax"], 4095);
    profile->screen_off_cal_lux_supported =
            GetBool(root["CommonConfig"]["ScreenOffCalLuxSupported"]);
    profile->median_enqueue_supported =
            GetBool(root["CommonConfig"]["MedianEnqueueSupported"]);
    profile->median_enqueue_event_period =
            GetInt(root["CommonConfig"]["MedianEnqueueEventPeriod"], 200);
    profile->median_enqueue_event_size =
            GetInt(root["CommonConfig"]["MedianEnqueueEventSize"], 6);
    profile->median_enqueue_event_index =
            GetInt(root["CommonConfig"]["MedianEnqueueEventIndex"], 2);
    profile->ir_ratio_formula_type = GetInt(root["CommonConfig"]["IRRatioFormulaType"], 0);
    profile->low_light_accuracy = GetFloat(root["CommonConfig"]["LowLightAccuracy"]);
    profile->lux_coeff_lir = ParseCoeffArray(root["LuxCoeffLIR"]);
    profile->lux_coeff_hir = ParseCoeffArray(root["LuxCoeffHIR"]);
    profile->lux_coeff_super_hir = ParseCoeffArray(root["LuxCoeffSuperHIR"]);
    profile->lux_coeff_lir_screen_off = ParseCoeffArray(root["LuxCoeffLirScreenOff"]);
    profile->lux_coeff_hir_screen_off = ParseCoeffArray(root["LuxCoeffHirScreenOff"]);
    profile->lux_coeff_super_hir_screen_off = ParseCoeffArray(root["LuxCoeffSuperHirScreenOff"]);
    profile->ir_thresholds = ParseIrThresholds(root["IRThreshold"]);
    profile->c_zero_thresholds = ParseLevelIntArray(root["CZeroThreshold"], "CZeroMin");
    profile->ir_brightness_ranges = ParseBrightnessRanges(root["IRBrightness"]);
    profile->linearity_ranges = ParseBrightnessRanges(root["LinearityBrightnessRange"]);
    profile->loaded = !profile->lux_coeff_lir.empty() && !profile->ir_thresholds.empty();
    profile->path = path;
    return profile->loaded;
}

std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool IsPanelTokenChar(char ch) {
    return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_';
}

bool ContainsToken(const std::string& value, const std::string& token) {
    size_t pos = 0;
    while ((pos = value.find(token, pos)) != std::string::npos) {
        const auto end = pos + token.size();
        const bool has_left_boundary = pos == 0 || !IsPanelTokenChar(value[pos - 1]);
        const bool has_right_boundary = end == value.size() || !IsPanelTokenChar(value[end]);
        if (has_left_boundary && has_right_boundary) {
            return true;
        }
        pos = end;
    }
    return false;
}

int GetPanelManufactureFromName(const std::string& panel_info) {
    const auto lower = ToLower(panel_info);
    // Matches stock OplusSensorServiceUtils::lcdStrToLcdId IDs.
    if (lower.find("samsung") != std::string::npos || ContainsToken(lower, "p_1") ||
        ContainsToken(lower, "s_1")) {
        return 0;
    }
    if (lower.find("boe") != std::string::npos || ContainsToken(lower, "p_3") ||
        ContainsToken(lower, "s_3")) {
        return 1;
    }
    if (lower.find("tianma") != std::string::npos || lower.find("tm") != std::string::npos ||
        ContainsToken(lower, "p_7") || ContainsToken(lower, "s_7")) {
        return 2;
    }
    if (lower.find("visionox") != std::string::npos || ContainsToken(lower, "p_b") ||
        ContainsToken(lower, "s_b")) {
        return 3;
    }
    if (lower.find("hx") != std::string::npos || ContainsToken(lower, "p_d") ||
        ContainsToken(lower, "s_d")) {
        return 4;
    }
    return -1;
}

std::shared_ptr<IDisplayPanelFeature> GetDisplayPanelFeatureService() {
    static std::mutex mutex;
    static std::shared_ptr<IDisplayPanelFeature> service;

    std::lock_guard<std::mutex> lock(mutex);
    if (service != nullptr) {
        return service;
    }

    const std::string instance = std::string(IDisplayPanelFeature::descriptor) + "/default";
    ndk::SpAIBinder binder(AServiceManager_checkService(instance.c_str()));
    if (binder.get() == nullptr) {
        LOG(WARNING) << "DisplayPanelFeature service is unavailable";
        return nullptr;
    }

    service = IDisplayPanelFeature::fromBinder(binder);
    if (service == nullptr) {
        LOG(WARNING) << "Failed to create DisplayPanelFeature client";
    }
    return service;
}

int GetDisplayPanelManufactureFromAidl() {
    const auto service = GetDisplayPanelFeatureService();
    if (service == nullptr) {
        return -1;
    }

    std::vector<std::string> panel_info;
    int32_t result = -1;
    const auto status = service->getDisplayPanelInfo(9, &panel_info, &result);
    if (!status.isOk() || result != 0 || panel_info.empty()) {
        LOG(WARNING) << "getDisplayPanelInfo failed, status=" << status.getDescription()
                     << ", result=" << result << ", size=" << panel_info.size();
        return -1;
    }

    std::ostringstream info;
    for (const auto& item : panel_info) {
        info << item << ' ';
    }
    const auto manufacture = GetPanelManufactureFromName(info.str());
    if (manufacture < 0) {
        LOG(WARNING) << "Unknown display panel info: " << info.str();
    } else {
        LOG(INFO) << "Display panel manufacture=" << manufacture << ", info=" << info.str();
    }
    return manufacture;
}

bool GetDisplayPanelFeatureValueFromAidl(int32_t feature_id, int32_t* value) {
    const auto service = GetDisplayPanelFeatureService();
    if (service == nullptr) {
        return false;
    }

    std::vector<int32_t> feature_values;
    int32_t result = -1;
    const auto status = service->getDisplayPanelFeatureValue(feature_id, &feature_values, &result);
    if (!status.isOk() || result != 0 || feature_values.empty()) {
        LOG(WARNING) << "getDisplayPanelFeatureValue(" << feature_id
                     << ") failed, status=" << status.getDescription() << ", result=" << result
                     << ", size=" << feature_values.size();
        return false;
    }

    *value = feature_values[0];
    return true;
}

bool GetDisplayPanelBrightnessFromAidl(float* brightness) {
    int32_t value = -1;
    if (!GetDisplayPanelFeatureValueFromAidl(29, &value) || value < 0) {
        return false;
    }
    *brightness = static_cast<float>(value);
    return true;
}

bool LoadProfileForManufacture(int manufacture, int sensor_module, FusionProfile* profile) {
    const std::string root = "/odm/etc/fusionlight_profile/";
    const auto filename = "fusionlight_Main_" + std::to_string(manufacture) + "_" +
            std::to_string(sensor_module) + ".json";
    if (ParseProfile(root + filename, profile)) {
        LOG(INFO) << "Loaded fusion light profile " << profile->path;
        return true;
    }
    return false;
}

const FusionProfile& GetProfile() {
    static FusionProfile profile;
    static std::once_flag load_once;
    std::call_once(load_once, [] {
        constexpr int kSensorModule = 3;

        const int display_manufacture = GetDisplayPanelManufactureFromAidl();
        if (display_manufacture >= 0 &&
            LoadProfileForManufacture(display_manufacture, kSensorModule, &profile)) {
            return;
        }

        if (!LoadProfileForManufacture(0, kSensorModule, &profile) &&
            !LoadProfileForManufacture(1, kSensorModule, &profile)) {
            LOG(WARNING) << "No fusion light profile found; falling back to raw lux";
        }
    });
    return profile;
}

float Dot(const LuxCoeff& coeff, const ChannelData& data) {
    return data.r * coeff.r + data.g * coeff.g + data.b * coeff.b + data.c * coeff.c;
}

int SelectIrLevel(const FusionProfile& profile, float ir_ratio, bool screen_off) {
    if (profile.ir_thresholds.empty()) {
        return 0;
    }

    if (screen_off) {
        if (ir_ratio <= profile.ir_thresholds.front().max) {
            return profile.ir_thresholds.front().level;
        }
        for (size_t i = 1; i < profile.ir_thresholds.size(); ++i) {
            const auto& threshold = profile.ir_thresholds[i];
            if (ir_ratio > threshold.min && ir_ratio <= threshold.max) {
                return threshold.level;
            }
        }
        return profile.ir_thresholds.back().level;
    }

    for (const auto& threshold : profile.ir_thresholds) {
        if (ir_ratio < threshold.max) {
            return threshold.level;
        }
    }
    return profile.ir_thresholds.back().level;
}

int SelectBrightnessLevel(const std::vector<BrightnessRange>& ranges, float brightness) {
    if (ranges.empty()) {
        return 0;
    }
    for (const auto& range : ranges) {
        if (brightness >= range.min && brightness <= range.max) {
            return range.level;
        }
    }
    if (brightness < ranges.front().min) {
        return ranges.front().level;
    }
    return ranges.back().level;
}

int SelectCZeroLevel(const FusionProfile& profile, float channel_c) {
    int level = 0;
    for (size_t i = 0; i < profile.c_zero_thresholds.size(); ++i) {
        if (channel_c >= profile.c_zero_thresholds[i]) {
            level = static_cast<int>(i);
        }
    }
    return level;
}

const LuxCoeff* SelectLuxCoeff(const FusionProfile& profile, int ir_level, int brightness_level,
                               bool screen_off, int c_zero_level) {
    const std::vector<LuxCoeff>* coeffs = &profile.lux_coeff_lir;
    int index_level = brightness_level;
    if (screen_off) {
        index_level = c_zero_level;
        coeffs = &profile.lux_coeff_lir_screen_off;
        if (ir_level == 1) {
            coeffs = &profile.lux_coeff_hir_screen_off;
        } else if (ir_level >= 2) {
            coeffs = &profile.lux_coeff_super_hir_screen_off;
        }
    }
    if (coeffs->empty()) {
        coeffs = &profile.lux_coeff_lir;
        index_level = brightness_level;
        if (ir_level == 1) {
            coeffs = &profile.lux_coeff_hir;
        } else if (ir_level >= 2) {
            coeffs = &profile.lux_coeff_super_hir;
        }
    }
    if (coeffs->empty()) {
        return nullptr;
    }
    const auto index = std::min<size_t>(std::max(index_level, 0), coeffs->size() - 1);
    return &(*coeffs)[index];
}

float NormalizeBrightness(const FusionProfile& profile, float brightness) {
    if (!std::isfinite(brightness)) {
        return static_cast<float>(profile.brightness_max);
    }
    if (brightness <= 0.0f || brightness <= profile.brightness_max) {
        return ClampNonNegative(brightness);
    }

    if (profile.apollo_brightness_supported &&
        profile.apollo_brightness_max > profile.brightness_max) {
        return Clamp(brightness * profile.brightness_max / profile.apollo_brightness_max, 0.0f,
                     static_cast<float>(profile.brightness_max));
    }
    return Clamp(brightness, 0.0f, static_cast<float>(profile.brightness_max));
}

bool UsesAbsoluteType0IrRatio(int32_t sensor_type) {
    switch (sensor_type) {
        case FusionLight::kTypeHighPwmRgbSensor:
        case FusionLight::kTypeFusionRgbSensor:
            return true;
        default:
            return false;
    }
}

float EstimateIrRatio(const FusionProfile& profile, const FusionInput& input) {
    const auto& data = input.channels;
    if (!IsFinitePositive(data.c)) {
        return 0.0f;
    }

    float ratio = 0.0f;
    switch (profile.ir_ratio_formula_type) {
        case 1:
            ratio = (data.r + data.g + data.b) / (data.c * 3.0f);
            break;
        case 2:
            ratio = 1.0f - (0.299f * data.r + 0.587f * data.g + 0.114f * data.b) / data.c;
            break;
        case 3:
            ratio = IsFinitePositive(data.g) ? data.c / data.g : 0.0f;
            break;
        case 0:
        default:
            ratio = (data.r + data.g + data.b - data.c) / data.c * -1.0f;
            if (UsesAbsoluteType0IrRatio(input.rgb_sensor_type)) {
                ratio = std::fabs(ratio);
            }
            break;
    }
    return ratio < 0.0f || !std::isfinite(ratio) ? 0.0f : ratio;
}

float ExtractRawLux(const Event& event) {
    switch (static_cast<int32_t>(event.sensorType)) {
        case FusionLight::kTypeFusionRgbSensor:
            // Stock OplusFusionRGBSensor maps its source lux into data[9].
            return ClampNonNegative(event.u.data[9]);
        case FusionLight::kTypeRearLightSensor:
        case FusionLight::kTypeHighPwmRgbSensor:
        default:
            return ClampNonNegative(event.u.data[0]);
    }
}

bool FillRgbFromEvent(const Event& event, FusionInput* input) {
    input->rgb_sensor_type = static_cast<int32_t>(event.sensorType);
    switch (static_cast<int32_t>(event.sensorType)) {
        case FusionLight::kTypeHighPwmRgbSensor:
            input->channels.r = event.u.data[4];
            input->channels.g = event.u.data[5];
            input->channels.b = event.u.data[6];
            input->channels.c = event.u.data[7];
            input->ir_ratio = event.u.data[8];
            break;
        case FusionLight::kTypeFusionRgbSensor:
            input->channels.r = event.u.data[1];
            input->channels.g = event.u.data[2];
            input->channels.b = event.u.data[3];
            input->channels.c = event.u.data[4];
            input->ir_ratio = event.u.data[6];
            break;
        default:
            input->has_rgb = false;
            input->has_ir_ratio = false;
            return false;
    }
    input->has_rgb = IsFinitePositive(input->channels.r + input->channels.g + input->channels.b +
                                      input->channels.c);
    input->has_ir_ratio = std::isfinite(input->ir_ratio) && input->ir_ratio >= 0.0f;
    return input->has_rgb;
}

bool ExtractBrightness(const Event& event, float* brightness) {
    if (static_cast<int32_t>(event.sensorType) != FusionLight::kTypeHighPwmRgbSensor) {
        return false;
    }
    const auto value = event.u.data[3];
    if (!IsFiniteNonNegative(value)) {
        return false;
    }
    *brightness = value;
    return true;
}

float CalculateFusionLux(const FusionInput& input) {
    if (!input.has_rgb) {
        return input.has_raw_lux ? input.raw_lux : 0.0f;
    }

    const auto& profile = GetProfile();
    if (!profile.loaded) {
        return input.has_raw_lux ? input.raw_lux : ClampNonNegative(input.channels.c);
    }

    const auto brightness = NormalizeBrightness(
            profile, input.has_brightness ? input.brightness : profile.brightness_max);
    const auto ir_ratio = input.has_ir_ratio ? input.ir_ratio : EstimateIrRatio(profile, input);
    const bool screen_off = profile.screen_off_cal_lux_supported && input.has_brightness &&
            input.brightness <= 0.0f;
    const auto ir_level = SelectIrLevel(profile, ir_ratio, screen_off);
    const auto& brightness_ranges = !profile.ir_brightness_ranges.empty()
            ? profile.ir_brightness_ranges
            : profile.linearity_ranges;
    const auto brightness_level = SelectBrightnessLevel(brightness_ranges, brightness);
    const auto c_zero_level = screen_off ? SelectCZeroLevel(profile, input.channels.c) : 0;
    const auto* coeff =
            SelectLuxCoeff(profile, ir_level, brightness_level, screen_off, c_zero_level);
    if (coeff == nullptr) {
        return input.has_raw_lux ? input.raw_lux : ClampNonNegative(input.channels.c);
    }

    // Stock subtracts panel leakage before calculate_lux_V2_1. Without that source in
    // the HAL wrapper, use the sensor channels directly.
    const auto lux = ClampNonNegative(Dot(*coeff, input.channels));
    if (lux < profile.low_light_accuracy) {
        return 0.0f;
    }
    if (IsFinitePositive(lux)) {
        return lux;
    }
    return input.has_raw_lux ? input.raw_lux : 0.0f;
}
}  // anonymous namespace

void FusionLight::reset() {
    high_pwm_rgb_handle_ = kInvalidSensorHandle;
    rear_light_handle_ = kInvalidSensorHandle;
    fusion_rgb_handle_ = kInvalidSensorHandle;
    has_high_pwm_rgb_event_ = false;
    has_rear_light_event_ = false;
    has_rgb_event_ = false;
    median_enqueue_values_.clear();
    median_enqueue_head_ = 0;
    median_enqueue_size_ = 0;
}

void FusionLight::resetMedianEnqueue() const {
    median_enqueue_size_ = 0;
}

float FusionLight::applyMedianEnqueue(float lux, float brightness, bool has_brightness) const {
    const auto& profile = GetProfile();
    if (!profile.loaded || !profile.median_enqueue_supported || !has_brightness ||
        brightness <= 1.0f || lux >= profile.median_enqueue_event_period) {
        resetMedianEnqueue();
        return lux;
    }

    const int queue_size = profile.median_enqueue_event_size;
    const int queue_index = profile.median_enqueue_event_index;
    if (queue_size <= queue_index || queue_size > 20 || queue_index < 0) {
        return lux;
    }

    if (median_enqueue_values_.size() != static_cast<size_t>(queue_size)) {
        median_enqueue_values_.assign(queue_size, 0.0f);
        median_enqueue_head_ = 0;
        median_enqueue_size_ = 0;
    }

    if (median_enqueue_size_ == queue_size) {
        median_enqueue_values_[median_enqueue_head_] = lux;
        median_enqueue_head_ = (median_enqueue_head_ + 1) % queue_size;
        auto sorted = median_enqueue_values_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[queue_index];
    }

    median_enqueue_values_[median_enqueue_size_] = lux;
    ++median_enqueue_size_;
    return lux;
}

void FusionLight::setSourceHandleForType(int32_t sensor_type, int32_t sensor_handle) {
    switch (sensor_type) {
        case kTypeHighPwmRgbSensor:
            high_pwm_rgb_handle_ = sensor_handle;
            break;
        case kTypeRearLightSensor:
            rear_light_handle_ = sensor_handle;
            break;
        case kTypeFusionRgbSensor:
            fusion_rgb_handle_ = sensor_handle;
            break;
    }
}

bool FusionLight::isSourceEvent(const Event& event) const {
    switch (static_cast<int32_t>(event.sensorType)) {
        case kTypeHighPwmRgbSensor:
            return event.sensorHandle == high_pwm_rgb_handle_;
        case kTypeRearLightSensor:
            return event.sensorHandle == rear_light_handle_;
        case kTypeFusionRgbSensor:
            return event.sensorHandle == fusion_rgb_handle_;
    }
    return false;
}

bool FusionLight::isReportEvent(const Event& event) const {
    return event.sensorHandle == primarySourceHandle();
}

void FusionLight::updateSample(const Event& event) {
    switch (static_cast<int32_t>(event.sensorType)) {
        case kTypeHighPwmRgbSensor:
            last_high_pwm_rgb_event_ = event;
            has_high_pwm_rgb_event_ = true;
            break;
        case kTypeRearLightSensor:
            last_rear_light_event_ = event;
            has_rear_light_event_ = true;
            break;
        case kTypeFusionRgbSensor:
            last_rgb_event_ = event;
            has_rgb_event_ = true;
            break;
    }
}

Event FusionLight::createLightEvent(const Event& trigger, int32_t fusion_light_handle) const {
    Event event = trigger;
    event.sensorHandle = fusion_light_handle;
    event.sensorType = SensorType::LIGHT;
    event.u.scalar = calculateLux(trigger);
    return event;
}

bool FusionLight::hasSource() const {
    return primarySourceHandle() != kInvalidSensorHandle;
}

int32_t FusionLight::primarySourceHandle() const {
    if (high_pwm_rgb_handle_ != kInvalidSensorHandle) {
        return high_pwm_rgb_handle_;
    }
    if (rear_light_handle_ != kInvalidSensorHandle) {
        return rear_light_handle_;
    }
    return fusion_rgb_handle_;
}

std::size_t FusionLight::sourceHandleCount() const {
    std::size_t count = 0;
    if (high_pwm_rgb_handle_ != kInvalidSensorHandle) {
        ++count;
    }
    if (rear_light_handle_ != kInvalidSensorHandle) {
        ++count;
    }
    if (high_pwm_rgb_handle_ == kInvalidSensorHandle &&
        fusion_rgb_handle_ != kInvalidSensorHandle) {
        ++count;
    }
    return count;
}

int32_t FusionLight::sourceHandleAt(std::size_t index) const {
    if (high_pwm_rgb_handle_ != kInvalidSensorHandle) {
        if (index == 0) {
            return high_pwm_rgb_handle_;
        }
        --index;
    }
    if (rear_light_handle_ != kInvalidSensorHandle) {
        if (index == 0) {
            return rear_light_handle_;
        }
        --index;
    }
    if (high_pwm_rgb_handle_ == kInvalidSensorHandle &&
        fusion_rgb_handle_ != kInvalidSensorHandle && index == 0) {
        return fusion_rgb_handle_;
    }
    return kInvalidSensorHandle;
}

std::string FusionLight::sourceSummary() const {
    std::ostringstream os;
    os << "high_pwm=" << high_pwm_rgb_handle_ << ", rear=" << rear_light_handle_
       << ", rgb=" << fusion_rgb_handle_;
    return os.str();
}

float FusionLight::calculateLux(const Event& trigger) const {
    const Event* rawEvent = &trigger;
    if (has_rear_light_event_) {
        rawEvent = &last_rear_light_event_;
    } else if (has_rgb_event_) {
        rawEvent = &last_rgb_event_;
    } else if (has_high_pwm_rgb_event_) {
        rawEvent = &last_high_pwm_rgb_event_;
    }

    const Event* rgbEvent = &trigger;
    if (has_high_pwm_rgb_event_) {
        rgbEvent = &last_high_pwm_rgb_event_;
    } else if (has_rgb_event_) {
        rgbEvent = &last_rgb_event_;
    } else if (has_rear_light_event_) {
        rgbEvent = &last_rear_light_event_;
    }

    FusionInput input;
    input.raw_lux = ExtractRawLux(*rawEvent);
    input.has_raw_lux = IsFinitePositive(input.raw_lux);

    // Stock layouts seen in libsensorserviceextimpl.so:
    // - high_pwm_rgb: data[4..7] = R/G/B/C.
    // - virtual qti.sensor.rgb: data[1..4] = R/G/B/C, data[6] = IR ratio, data[9] = lux.
    FillRgbFromEvent(*rgbEvent, &input);

    input.has_brightness = ExtractBrightness(*rgbEvent, &input.brightness);
    if (!input.has_brightness) {
        input.has_brightness = ExtractBrightness(*rawEvent, &input.brightness);
    }
    if (!input.has_brightness) {
        input.has_brightness = GetDisplayPanelBrightnessFromAidl(&input.brightness);
    }

    const auto lux = CalculateFusionLux(input);
    return applyMedianEnqueue(lux, input.brightness, input.has_brightness);
}

}  // namespace qsh_wrapper
}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
