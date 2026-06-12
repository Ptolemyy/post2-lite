#include "post2/core/control_models.hpp"

#include "post2/core/coordinates.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace post2::core {

namespace {

constexpr double kStandardGravityMps2 = 9.80665;

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

double evaluate_poly(const Poly2Config& poly, double time_s)
{
    return poly.c0 + poly.c1 * time_s + poly.c2 * time_s * time_s;
}

double available_propellant_kg(const post2::vehicle::VehicleRuntimeState& runtime)
{
    return post2::vehicle::active_stage_propellant_kg(runtime);
}

Vec3 cross_product(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

Vec3 normalized_or(const Vec3& value, const Vec3& fallback)
{
    const double length = post2::vehicle::norm(value);
    if (length <= 1.0e-12) {
        return fallback;
    }
    return value / length;
}

struct LocalEnuBasis {
    Vec3 east;
    Vec3 north;
    Vec3 up;
};

LocalEnuBasis local_enu_basis(const State& state)
{
    const Vec3 up = normalized_or(state.position_m, {1.0, 0.0, 0.0});
    const Vec3 east = normalized_or(cross_product({0.0, 0.0, 1.0}, up), {0.0, 1.0, 0.0});
    const Vec3 north = normalized_or(cross_product(up, east), {0.0, 0.0, 1.0});
    return {east, north, up};
}

Vec3 direction_from_azimuth_elevation_deg(const State& state, double azimuth_deg, double elevation_deg)
{
    const LocalEnuBasis basis = local_enu_basis(state);
    const double azimuth_rad = degrees_to_radians(azimuth_deg);
    const double elevation_rad = degrees_to_radians(elevation_deg);
    const double horizontal = std::cos(elevation_rad);
    return normalized_or(
        basis.north * (horizontal * std::cos(azimuth_rad)) +
            basis.east * (horizontal * std::sin(azimuth_rad)) +
            basis.up * std::sin(elevation_rad),
        basis.north);
}

Quaternion normalized_or_identity(Quaternion quat)
{
    const double length = std::sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    if (length <= 1.0e-12) {
        return {};
    }
    quat.w /= length;
    quat.x /= length;
    quat.y /= length;
    quat.z /= length;
    return quat;
}

Quaternion slerp(Quaternion lhs, Quaternion rhs, double alpha)
{
    lhs = normalized_or_identity(lhs);
    rhs = normalized_or_identity(rhs);
    double cos_theta = lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    if (cos_theta < 0.0) {
        rhs.w = -rhs.w;
        rhs.x = -rhs.x;
        rhs.y = -rhs.y;
        rhs.z = -rhs.z;
        cos_theta = -cos_theta;
    }

    if (cos_theta > 0.9995) {
        return normalized_or_identity({
            lhs.w + (rhs.w - lhs.w) * alpha,
            lhs.x + (rhs.x - lhs.x) * alpha,
            lhs.y + (rhs.y - lhs.y) * alpha,
            lhs.z + (rhs.z - lhs.z) * alpha,
        });
    }

    const double theta = std::acos(clamp(cos_theta, -1.0, 1.0));
    const double sin_theta = std::sin(theta);
    const double lhs_scale = std::sin((1.0 - alpha) * theta) / sin_theta;
    const double rhs_scale = std::sin(alpha * theta) / sin_theta;
    return normalized_or_identity({
        lhs.w * lhs_scale + rhs.w * rhs_scale,
        lhs.x * lhs_scale + rhs.x * rhs_scale,
        lhs.y * lhs_scale + rhs.y * rhs_scale,
        lhs.z * lhs_scale + rhs.z * rhs_scale,
    });
}

Vec3 rotate_body_x_by_quaternion(Quaternion quat)
{
    quat = normalized_or_identity(quat);
    const Vec3 u{quat.x, quat.y, quat.z};
    const Vec3 v{1.0, 0.0, 0.0};
    return normalized_or(
        u * (2.0 * post2::vehicle::dot(u, v)) +
            v * (quat.w * quat.w - post2::vehicle::dot(u, u)) +
            cross_product(u, v) * (2.0 * quat.w),
        {1.0, 0.0, 0.0});
}

class ThrottlePolyModel final : public IThrottleModel {
public:
    explicit ThrottlePolyModel(ThrottleModelConfig config)
        : config_(std::move(config))
    {
    }

    double throttle(double time_s, const post2::vehicle::VehicleRuntimeState&, const PhaseContext&) const override
    {
        return clamp_throttle(config_.c0 + config_.c1 * time_s + config_.c2 * time_s * time_s);
    }

private:
    ThrottleModelConfig config_;
};

class T2WThrottleModel final : public IThrottleModel {
public:
    explicit T2WThrottleModel(ThrottleModelConfig config)
        : config_(std::move(config))
    {
    }

    double throttle(
        double,
        const post2::vehicle::VehicleRuntimeState& runtime,
        const PhaseContext& context) const override
    {
        const auto* case_config = context.case_config;
        const double max_thrust_n = case_config
            ? post2::vehicle::active_max_thrust_n(case_config->vehicle, runtime)
            : 0.0;
        if (!runtime.engine.enabled ||
            !case_config ||
            max_thrust_n <= 0.0 ||
            available_propellant_kg(runtime) <= 0.0) {
            return 0.0;
        }
        return clamp_throttle(
            config_.target_t2w * runtime.vehicle.total_mass_kg *
            kStandardGravityMps2 / max_thrust_n);
    }

private:
    ThrottleModelConfig config_;
};

class ThrottleInterpolatedModel final : public IThrottleModel {
public:
    explicit ThrottleInterpolatedModel(ThrottleModelConfig config)
        : points_(std::move(config.points))
    {
        std::sort(points_.begin(), points_.end(), [](const ThrottlePoint& lhs, const ThrottlePoint& rhs) {
            return lhs.time_s < rhs.time_s;
        });
    }

    double throttle(double time_s, const post2::vehicle::VehicleRuntimeState&, const PhaseContext&) const override
    {
        if (points_.empty()) {
            return 0.0;
        }
        if (time_s <= points_.front().time_s) {
            return clamp_throttle(points_.front().throttle);
        }
        if (time_s >= points_.back().time_s) {
            return clamp_throttle(points_.back().throttle);
        }

        for (std::size_t i = 1; i < points_.size(); ++i) {
            if (time_s <= points_[i].time_s) {
                const auto& lhs = points_[i - 1];
                const auto& rhs = points_[i];
                const double span_s = rhs.time_s - lhs.time_s;
                if (span_s <= 0.0) {
                    return clamp_throttle(rhs.throttle);
                }
                const double alpha = (time_s - lhs.time_s) / span_s;
                return clamp_throttle(lhs.throttle + (rhs.throttle - lhs.throttle) * alpha);
            }
        }
        return clamp_throttle(points_.back().throttle);
    }

private:
    std::vector<ThrottlePoint> points_;
};

class FixedEciSteeringModel final : public ISteeringModel {
public:
    explicit FixedEciSteeringModel(Vec3 direction)
        : direction_(normalized_or(direction, {1.0, 0.0, 0.0}))
    {
    }

    Vec3 thrust_direction_eci(double, const State&, const post2::vehicle::VehicleRuntimeState&, const PhaseContext&) const override
    {
        return direction_;
    }

private:
    Vec3 direction_;
};

class RollPitchYawPolySteeringModel final : public ISteeringModel {
public:
    explicit RollPitchYawPolySteeringModel(SteeringModelConfig config)
        : config_(std::move(config))
    {
    }

    Vec3 thrust_direction_eci(
        double time_s,
        const State& state,
        const post2::vehicle::VehicleRuntimeState&,
        const PhaseContext&) const override
    {
        const double pitch_deg = evaluate_poly(config_.pitch_deg, time_s);
        const double yaw_deg = evaluate_poly(config_.yaw_deg, time_s);
        return direction_from_azimuth_elevation_deg(state, yaw_deg, pitch_deg);
    }

private:
    SteeringModelConfig config_;
};

class GenericPolySteeringModel final : public ISteeringModel {
public:
    explicit GenericPolySteeringModel(SteeringModelConfig config)
        : config_(std::move(config))
    {
    }

    Vec3 thrust_direction_eci(
        double time_s,
        const State& state,
        const post2::vehicle::VehicleRuntimeState&,
        const PhaseContext&) const override
    {
        return direction_from_azimuth_elevation_deg(
            state,
            evaluate_poly(config_.azimuth_deg, time_s),
            evaluate_poly(config_.elevation_deg, time_s));
    }

private:
    SteeringModelConfig config_;
};

class QuatInterpSteeringModel final : public ISteeringModel {
public:
    explicit QuatInterpSteeringModel(SteeringModelConfig config)
        : points_(std::move(config.points))
    {
        std::sort(points_.begin(), points_.end(), [](const QuaternionPoint& lhs, const QuaternionPoint& rhs) {
            return lhs.time_s < rhs.time_s;
        });
    }

    Vec3 thrust_direction_eci(double time_s, const State&, const post2::vehicle::VehicleRuntimeState&, const PhaseContext&) const override
    {
        if (points_.empty()) {
            return {1.0, 0.0, 0.0};
        }
        if (time_s <= points_.front().time_s) {
            return rotate_body_x_by_quaternion(points_.front().quat);
        }
        if (time_s >= points_.back().time_s) {
            return rotate_body_x_by_quaternion(points_.back().quat);
        }
        for (std::size_t i = 1; i < points_.size(); ++i) {
            if (time_s <= points_[i].time_s) {
                const auto& lhs = points_[i - 1];
                const auto& rhs = points_[i];
                const double span_s = rhs.time_s - lhs.time_s;
                const double alpha = span_s <= 0.0 ? 1.0 : (time_s - lhs.time_s) / span_s;
                return rotate_body_x_by_quaternion(slerp(lhs.quat, rhs.quat, alpha));
            }
        }
        return rotate_body_x_by_quaternion(points_.back().quat);
    }

private:
    std::vector<QuaternionPoint> points_;
};

class SelectableSteeringModel final : public ISteeringModel {
public:
    explicit SelectableSteeringModel(SteeringModelConfig config)
    {
        for (const auto& segment : config.segments) {
            const SteeringModelConfig child_config = segment.model ? *segment.model : SteeringModelConfig{};
            segments_.push_back({segment.start_time_s, make_steering_model(child_config)});
        }
        std::sort(segments_.begin(), segments_.end(), [](const Segment& lhs, const Segment& rhs) {
            return lhs.start_time_s < rhs.start_time_s;
        });
    }

    Vec3 thrust_direction_eci(
        double time_s,
        const State& state,
        const post2::vehicle::VehicleRuntimeState& runtime,
        const PhaseContext& context) const override
    {
        if (segments_.empty()) {
            return {1.0, 0.0, 0.0};
        }

        const Segment* selected = &segments_.front();
        for (const auto& segment : segments_) {
            if (time_s >= segment.start_time_s) {
                selected = &segment;
            }
        }
        return selected->model->thrust_direction_eci(time_s - selected->start_time_s, state, runtime, context);
    }

private:
    struct Segment {
        double start_time_s = 0.0;
        std::unique_ptr<ISteeringModel> model;
    };

    std::vector<Segment> segments_;
};

} // namespace

double clamp_throttle(double value)
{
    return clamp(value, 0.0, 1.0);
}

std::unique_ptr<IThrottleModel> make_throttle_model(const ThrottleModelConfig& config)
{
    const std::string type = lowercase(config.type);
    if (type == "t2w" || type == "target_t2w") {
        return std::make_unique<T2WThrottleModel>(config);
    }
    if (type == "interpolated" || type == "tabular") {
        return std::make_unique<ThrottleInterpolatedModel>(config);
    }
    return std::make_unique<ThrottlePolyModel>(config);
}

std::unique_ptr<ISteeringModel> make_steering_model(const SteeringModelConfig& config)
{
    const std::string type = lowercase(config.type);
    if (type == "fixed_eci") {
        return std::make_unique<FixedEciSteeringModel>(config.fixed_direction_eci);
    }
    if (type == "rpy_poly" || type == "roll_pitch_yaw_poly") {
        return std::make_unique<RollPitchYawPolySteeringModel>(config);
    }
    if (type == "generic_quat_interp" || type == "generic_tabular_quat_interp") {
        return std::make_unique<QuatInterpSteeringModel>(config);
    }
    if (type == "generic_selectable" || type == "selectable") {
        return std::make_unique<SelectableSteeringModel>(config);
    }
    return std::make_unique<GenericPolySteeringModel>(config);
}

} // namespace post2::core
