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
#include <android-base/properties.h>
#include <json/json.h>

#include <algorithm>
#include <cctype>
#include <array>
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

struct LinearityFunction {
    int function = 0;
    std::array<std::array<float, 4>, 4> channel_params{};
};

struct FusionProfile {
    bool loaded = false;
    int brightness_max = 4095;
    std::string path;
    std::vector<LuxCoeff> lux_coeff_lir;
    std::vector<LuxCoeff> lux_coeff_hir;
    std::vector<LuxCoeff> lux_coeff_super_hir;
    std::vector<IrThreshold> ir_thresholds;
    std::vector<BrightnessRange> linearity_ranges;
    std::vector<LinearityFunction> linearity;
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
};

inline bool IsFinitePositive(float value) {
    return std::isfinite(value) && value > 0.0f;
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

std::vector<LinearityFunction> ParseLinearity(const Json::Value& values) {
    std::vector<LinearityFunction> functions;
    if (!values.isArray()) {
        return functions;
    }

    for (const auto& value : values) {
        LinearityFunction function;
        function.function = GetInt(value["Function"], static_cast<int>(functions.size()));
        const auto& params = value["LinearityParameter"];
        if (params.isArray()) {
            for (const auto& channel : params) {
                const auto index = GetInt(channel["Channel"], -1);
                if (index < 0 || index >= static_cast<int>(function.channel_params.size())) {
                    continue;
                }
                function.channel_params[index] = {
                        GetFloat(channel["Parameter0"]),
                        GetFloat(channel["Parameter1"]),
                        GetFloat(channel["Parameter2"]),
                        GetFloat(channel["Parameter3"]),
                };
            }
        }
        if (function.function >= 0) {
            if (static_cast<size_t>(function.function) >= functions.size()) {
                functions.resize(function.function + 1);
            }
            functions[function.function] = function;
        }
    }
    return functions;
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

    profile->brightness_max = GetInt(root["CommonConfig"]["BrightnessMax"], 4095);
    profile->lux_coeff_lir = ParseCoeffArray(root["LuxCoeffLIR"]);
    profile->lux_coeff_hir = ParseCoeffArray(root["LuxCoeffHIR"]);
    profile->lux_coeff_super_hir = ParseCoeffArray(root["LuxCoeffSuperHIR"]);
    profile->ir_thresholds = ParseIrThresholds(root["IRThreshold"]);
    profile->linearity_ranges = ParseBrightnessRanges(root["LinearityBrightnessRange"]);
    profile->linearity = ParseLinearity(root["Linearity"]);
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

int GetDisplayPanelManufactureFromAidl() {
    const std::string instance = std::string(IDisplayPanelFeature::descriptor) + "/default";
    ndk::SpAIBinder binder(AServiceManager_checkService(instance.c_str()));
    if (binder.get() == nullptr) {
        LOG(WARNING) << "DisplayPanelFeature service is unavailable";
        return -1;
    }

    const auto service = IDisplayPanelFeature::fromBinder(binder);
    if (service == nullptr) {
        LOG(WARNING) << "Failed to create DisplayPanelFeature client";
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

bool LoadProfileForManufacture(int manufacture, int sensor_module, FusionProfile* profile) {
    const std::array<std::string, 4> roots = {
            "/odm/etc/fusionlight_profile/",
            "/vendor/etc/fusionlight_profile/",
            "/system/etc/fusionlight_profile/",
            "/etc/fusionlight_profile/",
    };
    const auto filename = "fusionlight_Main_" + std::to_string(manufacture) + "_" +
            std::to_string(sensor_module) + ".json";
    for (const auto& root : roots) {
        if (ParseProfile(root + filename, profile)) {
            LOG(INFO) << "Loaded fusion light profile " << profile->path;
            return true;
        }
    }
    return false;
}

const FusionProfile& GetProfile() {
    static FusionProfile profile;
    static std::once_flag load_once;
    std::call_once(load_once, [] {
        const int sensor_module = android::base::GetIntProperty(
                "persist.vendor.sensors.fusionlight.sensor_module", 3);
        const int override_manufacture = android::base::GetIntProperty(
                "persist.vendor.sensors.fusionlight.lcd_manufacture", -1);
        if (override_manufacture >= 0 &&
            LoadProfileForManufacture(override_manufacture, sensor_module, &profile)) {
            return;
        }

        const int display_manufacture = GetDisplayPanelManufactureFromAidl();
        if (display_manufacture >= 0 &&
            LoadProfileForManufacture(display_manufacture, sensor_module, &profile)) {
            return;
        }

        if (!LoadProfileForManufacture(0, sensor_module, &profile) &&
            !LoadProfileForManufacture(1, sensor_module, &profile)) {
            LOG(WARNING) << "No fusion light profile found; falling back to raw lux";
        }
    });
    return profile;
}

float Dot(const LuxCoeff& coeff, const ChannelData& data) {
    return data.r * coeff.r + data.g * coeff.g + data.b * coeff.b + data.c * coeff.c;
}

float Cubic(float x, const std::array<float, 4>& params) {
    return ((params[0] * x + params[1]) * x + params[2]) * x + params[3];
}

int SelectIrLevel(const FusionProfile& profile, float ir_ratio) {
    for (const auto& threshold : profile.ir_thresholds) {
        if (ir_ratio >= threshold.min && ir_ratio < threshold.max) {
            return threshold.level;
        }
    }
    return profile.ir_thresholds.empty() ? 0 : profile.ir_thresholds.back().level;
}

int SelectLinearityFunction(const FusionProfile& profile, float brightness) {
    for (const auto& range : profile.linearity_ranges) {
        if (brightness >= range.min && brightness < range.max) {
            return range.level;
        }
    }
    return profile.linearity_ranges.empty() ? -1 : profile.linearity_ranges.back().level;
}

const LuxCoeff* SelectLuxCoeff(const FusionProfile& profile, int ir_level) {
    const std::vector<LuxCoeff>* coeffs = &profile.lux_coeff_lir;
    if (ir_level == 1) {
        coeffs = &profile.lux_coeff_hir;
    } else if (ir_level >= 2) {
        coeffs = &profile.lux_coeff_super_hir;
    }
    if (coeffs->empty()) {
        return nullptr;
    }
    const auto index = std::min<size_t>(std::max(ir_level, 0), coeffs->size() - 1);
    return &(*coeffs)[index];
}

ChannelData ApplyLinearity(const FusionProfile& profile, const ChannelData& data, float brightness) {
    const auto function_index = SelectLinearityFunction(profile, brightness);
    if (function_index < 0 || static_cast<size_t>(function_index) >= profile.linearity.size()) {
        return data;
    }

    const auto& function = profile.linearity[function_index];
    ChannelData result;
    result.r = ClampNonNegative(data.r - Cubic(brightness, function.channel_params[0]));
    result.g = ClampNonNegative(data.g - Cubic(brightness, function.channel_params[1]));
    result.b = ClampNonNegative(data.b - Cubic(brightness, function.channel_params[2]));
    result.c = ClampNonNegative(data.c - Cubic(brightness, function.channel_params[3]));
    return result;
}

float EstimateIrRatio(const ChannelData& data) {
    if (!IsFinitePositive(data.c)) {
        return 0.0f;
    }
    return Clamp((data.c - data.r - data.g - data.b) / data.c, 0.0f, 1.0f);
}

float FirstPositive(float first, float second) {
    return IsFinitePositive(first) ? first : second;
}

float ExtractRawLux(const Event& event) {
    switch (static_cast<int32_t>(event.sensorType)) {
        case FusionLight::kTypeFusionRgbSensor:
            // Stock OplusFusionRGBSensor maps its source lux into data[9].
            return ClampNonNegative(FirstPositive(event.u.data[9], event.u.data[0]));
        case FusionLight::kTypeRearLightSensor:
        case FusionLight::kTypeHighPwmRgbSensor:
        default:
            return ClampNonNegative(event.u.data[0]);
    }
}

bool FillRgbFromEvent(const Event& event, FusionInput* input) {
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
            input->ir_ratio = event.u.data[8];
            break;
        default:
            input->channels.r = event.u.data[0];
            input->channels.g = event.u.data[1];
            input->channels.b = event.u.data[2];
            input->channels.c = event.u.data[3];
            input->ir_ratio = event.u.data[4];
            break;
    }
    input->has_rgb = IsFinitePositive(input->channels.r + input->channels.g + input->channels.b +
                                      input->channels.c);
    input->has_ir_ratio = std::isfinite(input->ir_ratio) && input->ir_ratio >= 0.0f &&
            input->ir_ratio <= 1.0f;
    return input->has_rgb;
}

float ExtractBrightness(const Event& event) {
    return event.u.data[3];
}

float CalculateFusionLux(const FusionInput& input) {
    if (!input.has_rgb) {
        return input.has_raw_lux ? input.raw_lux : 0.0f;
    }

    const auto& profile = GetProfile();
    if (!profile.loaded) {
        return input.has_raw_lux ? input.raw_lux : ClampNonNegative(input.channels.c);
    }

    const auto brightness = input.has_brightness ? input.brightness : profile.brightness_max;
    const auto ir_ratio = input.has_ir_ratio ? input.ir_ratio : EstimateIrRatio(input.channels);
    const auto ir_level = SelectIrLevel(profile, ir_ratio);
    const auto* coeff = SelectLuxCoeff(profile, ir_level);
    if (coeff == nullptr) {
        return input.has_raw_lux ? input.raw_lux : ClampNonNegative(input.channels.c);
    }

    const auto compensated = ApplyLinearity(profile, input.channels, brightness);
    const auto lux = ClampNonNegative(Dot(*coeff, compensated));
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

std::vector<int32_t> FusionLight::sourceHandles() const {
    std::vector<int32_t> handles;
    if (high_pwm_rgb_handle_ != kInvalidSensorHandle) {
        handles.push_back(high_pwm_rgb_handle_);
    }
    if (rear_light_handle_ != kInvalidSensorHandle) {
        handles.push_back(rear_light_handle_);
    }
    if (handles.empty() && fusion_rgb_handle_ != kInvalidSensorHandle) {
        handles.push_back(fusion_rgb_handle_);
    }
    return handles;
}

std::string FusionLight::sourceSummary() const {
    std::ostringstream os;
    os << "high_pwm=" << high_pwm_rgb_handle_ << ", rear=" << rear_light_handle_
       << ", rgb=" << fusion_rgb_handle_;
    return os.str();
}

float FusionLight::calculateLux(const Event& trigger) const {
    const Event* rawEvent = has_rear_light_event_ ? &last_rear_light_event_
                                                  : (has_rgb_event_ ? &last_rgb_event_
                                                                    : (has_high_pwm_rgb_event_
                                                                               ? &last_high_pwm_rgb_event_
                                                                               : &trigger));
    const Event* rgbEvent = has_high_pwm_rgb_event_ ? &last_high_pwm_rgb_event_
                                                    : (has_rgb_event_ ? &last_rgb_event_
                                                                      : (has_rear_light_event_
                                                                                 ? &last_rear_light_event_
                                                                                 : &trigger));

    FusionInput input;
    input.raw_lux = ExtractRawLux(*rawEvent);
    input.has_raw_lux = IsFinitePositive(input.raw_lux);

    // Stock layouts seen in libsensorserviceextimpl.so:
    // - high_pwm_rgb: data[4..7] = R/G/B/C.
    // - virtual qti.sensor.rgb: data[1..4] = R/G/B/C and data[9] = lux.
    FillRgbFromEvent(*rgbEvent, &input);

    input.brightness = ExtractBrightness(*rgbEvent);
    if (!IsFinitePositive(input.brightness)) {
        input.brightness = ExtractBrightness(*rawEvent);
    }
    input.has_brightness = IsFinitePositive(input.brightness);

    return CalculateFusionLux(input);
}

}  // namespace qsh_wrapper
}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
