#include "post2/core/control_models.hpp"

#include "post2/core/coordinates.hpp"
#include "post2/core/upfg.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace post2::core {

namespace {

constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kSegmentTimeEpsilonS = 1.0e-9;

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

int sanitized_order(int order)
{
    return std::max(0, std::min(order, 8));
}

double evaluate_coefficients(const std::vector<double>& coefficients, double time_s, int order)
{
    const int usable_order = sanitized_order(order);
    double value = 0.0;
    double power = 1.0;
    for (int i = 0; i <= usable_order; ++i) {
        if (static_cast<std::size_t>(i) < coefficients.size()) {
            value += coefficients[static_cast<std::size_t>(i)] * power;
        }
        power *= time_s;
    }
    return value;
}

std::vector<double> coefficients_from_poly(const Poly2Config& poly, int order)
{
    std::vector<double> coefficients(static_cast<std::size_t>(sanitized_order(order) + 1), 0.0);
    if (!coefficients.empty()) {
        coefficients[0] = poly.c0;
    }
    if (coefficients.size() > 1) {
        coefficients[1] = poly.c1;
    }
    if (coefficients.size() > 2) {
        coefficients[2] = poly.c2;
    }
    return coefficients;
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

struct AzimuthElevationDeg {
    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
};

// Inverse of direction_from_azimuth_elevation_deg: recovers the local
// azimuth/elevation (degrees) of an ECI direction at the given state. Used to
// back out the "current" steering angles from a final thrust direction.
AzimuthElevationDeg azimuth_elevation_from_direction(const State& state, const Vec3& direction_eci)
{
    constexpr double kRadToDeg = 57.295779513082320876798154814105;
    const LocalEnuBasis basis = local_enu_basis(state);
    const Vec3 dir = normalized_or(direction_eci, basis.north);
    const double up_comp = clamp(post2::vehicle::dot(dir, basis.up), -1.0, 1.0);
    const double north_comp = post2::vehicle::dot(dir, basis.north);
    const double east_comp = post2::vehicle::dot(dir, basis.east);
    return {
        std::atan2(east_comp, north_comp) * kRadToDeg,
        std::asin(up_comp) * kRadToDeg,
    };
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

class ThrottleSegmentedPolyModel final : public IThrottleModel {
public:
    explicit ThrottleSegmentedPolyModel(ThrottleModelConfig config)
        : order_(sanitized_order(config.segmented_poly.order))
        , segments_(std::move(config.segmented_poly.segments))
    {
        if (segments_.empty()) {
            segments_.push_back({0.0, coefficients_from_poly(
                Poly2Config{config.c0, config.c1, config.c2, config.continuity},
                order_)});
        }
        std::sort(segments_.begin(), segments_.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.start_time_s < rhs.start_time_s;
        });
    }

    double throttle(double time_s, const post2::vehicle::VehicleRuntimeState&, const PhaseContext&) const override
    {
        const auto* segment = selected_segment(time_s);
        if (!segment) {
            return 0.0;
        }
        const double local_time_s = std::max(0.0, time_s - segment->start_time_s);
        return clamp_throttle(evaluate_coefficients(
            segment->coefficients,
            local_time_s,
            order_));
    }

private:
    const SegmentedPolySegmentConfig* selected_segment(double time_s) const
    {
        if (segments_.empty()) {
            return nullptr;
        }
        const auto* selected = &segments_.front();
        for (const auto& segment : segments_) {
            if (time_s + kSegmentTimeEpsilonS >= segment.start_time_s) {
                selected = &segment;
            }
        }
        return selected;
    }

    int order_ = 1;
    std::vector<SegmentedPolySegmentConfig> segments_;
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

class GenericSegmentedPolySteeringModel final : public ISteeringModel {
public:
    explicit GenericSegmentedPolySteeringModel(SteeringModelConfig config)
        : order_(sanitized_order(config.segmented_poly.order))
        , segments_(std::move(config.segmented_poly.segments))
    {
        if (segments_.empty()) {
            SegmentedSteeringPolySegmentConfig segment;
            segment.start_time_s = 0.0;
            segment.azimuth_coefficients = coefficients_from_poly(config.azimuth_deg, order_);
            segment.elevation_coefficients = coefficients_from_poly(config.elevation_deg, order_);
            segments_.push_back(std::move(segment));
        }
        std::sort(segments_.begin(), segments_.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.start_time_s < rhs.start_time_s;
        });
    }

    Vec3 thrust_direction_eci(
        double time_s,
        const State& state,
        const post2::vehicle::VehicleRuntimeState&,
        const PhaseContext&) const override
    {
        const auto* segment = selected_segment(time_s);
        if (!segment) {
            return direction_from_azimuth_elevation_deg(state, 0.0, 0.0);
        }
        const double local_time_s = std::max(0.0, time_s - segment->start_time_s);
        return direction_from_azimuth_elevation_deg(
            state,
            evaluate_coefficients(segment->azimuth_coefficients, local_time_s, order_),
            evaluate_coefficients(segment->elevation_coefficients, local_time_s, order_));
    }

private:
    const SegmentedSteeringPolySegmentConfig* selected_segment(double time_s) const
    {
        if (segments_.empty()) {
            return nullptr;
        }
        const auto* selected = &segments_.front();
        for (const auto& segment : segments_) {
            if (time_s + kSegmentTimeEpsilonS >= segment.start_time_s) {
                selected = &segment;
            }
        }
        return selected;
    }

    int order_ = 1;
    std::vector<SegmentedSteeringPolySegmentConfig> segments_;
};

// Linear / bilinear tangent steering: tan(elevation) is linear (or quadratic,
// "bilinear") in phase time. Azimuth still comes from the azimuth poly.
class LinearTangentSteeringModel final : public ISteeringModel {
public:
    LinearTangentSteeringModel(SteeringModelConfig config, bool bilinear)
        : config_(std::move(config))
        , bilinear_(bilinear)
    {
    }

    Vec3 thrust_direction_eci(
        double time_s,
        const State& state,
        const post2::vehicle::VehicleRuntimeState&,
        const PhaseContext&) const override
    {
        constexpr double kRadToDeg = 57.295779513082320876798154814105;
        const LinearTangentConfig& t = config_.tangent;
        const double dt = time_s + t.t_offset_s;
        double tan_value;
        if (bilinear_) {
            const double a_value = t.a + t.a_dot * dt;
            const double b_value = t.b + t.b_dot * dt;
            tan_value = a_value * dt + b_value;
        } else {
            tan_value = t.a * dt + t.b;
        }
        const double elevation_deg = std::atan(tan_value) * kRadToDeg;
        const double azimuth_deg = evaluate_poly(config_.azimuth_deg, time_s);
        return direction_from_azimuth_elevation_deg(state, azimuth_deg, elevation_deg);
    }

private:
    SteeringModelConfig config_;
    bool bilinear_ = false;
};

// Build the UPFG remaining-stage stack from the live vehicle, in burn order
// (active stage first, then subsequent attached propulsive stages). Each stage's
// mass_total is the vehicle mass at the moment it ignites: the current total
// mass minus the dry + propellant mass of every lower stage burned and
// jettisoned before it. Non-propulsive attached mass (payload, spent attached
// stages) is carried, not burned. UPFG assumes full vacuum thrust per stage.
std::vector<post2::core::UpfgStage> build_upfg_stages(
    const CaseConfig& case_config,
    const post2::vehicle::VehicleRuntimeState& runtime)
{
    std::vector<post2::core::UpfgStage> stages;
    const auto& cfg_stages = case_config.vehicle.stages;
    const auto& rt_stages = runtime.stages;
    const std::size_t count = std::min(cfg_stages.size(), rt_stages.size());
    const double total_mass = runtime.vehicle.total_mass_kg;

    double lower_mass = 0.0;  // mass jettisoned before the next stage ignites
    bool active_seen = false;
    for (std::size_t i = 0; i < count; ++i) {
        const auto& sc = cfg_stages[i];
        const auto& rs = rt_stages[i];
        if (!rs.attached) {
            continue;  // already separated -- not part of the vehicle mass
        }
        const double thrust = static_cast<double>(sc.engine.engine_count) * sc.engine.thrust_vac_n;
        const double isp = sc.engine.isp_vac_s;
        const bool propulsive = thrust > 0.0 && isp > 0.0;
        const double propellant = post2::vehicle::stage_propellant_kg(rs);

        if (!active_seen) {
            if (rs.active && propulsive) {
                active_seen = true;  // fall through and add this stage
            } else {
                continue;  // pre-active or non-propulsive: carried, skip
            }
        }
        if (!propulsive) {
            continue;  // payload / inert attached stage: carried mass
        }
        if (propellant <= 1.0e-6 && !rs.active) {
            continue;  // spent attached stage: carried dead mass
        }

        const double ve = isp * kStandardGravityMps2;
        const double mdot = ve > 0.0 ? thrust / ve : 0.0;
        if (!(mdot > 0.0)) {
            continue;
        }

        post2::core::UpfgStage stage;
        stage.mode = 1;  // constant thrust
        stage.thrust_n = thrust;
        stage.mdot_kgps = mdot;
        stage.exhaust_velocity_mps = ve;
        stage.mass_total_kg = std::max(0.0, total_mass - lower_mass);
        stage.max_burn_time_s = propellant / mdot;
        stages.push_back(stage);

        lower_mass += rs.dry_mass_kg + propellant;
    }
    return stages;
}

// Closed-loop NASA UPFG steering. Unlike the open-loop poly models this reads
// the live state and stage stack and computes the thrust direction that flies to
// the configured target orbit. It solves UPFG to convergence at the current
// state on every call (upfg_converge is a deterministic, reentrant function of
// state, made contractive by the steering-turn clamp), so the integrator's dense
// per-RK-stage sampling stays consistent. See post2/core/upfg.hpp.
class UpfgSteeringModel final : public ISteeringModel {
public:
    explicit UpfgSteeringModel(SteeringModelConfig config)
        : config_(std::move(config))
    {
    }

    Vec3 thrust_direction_eci(
        double,
        const State& state,
        const post2::vehicle::VehicleRuntimeState& runtime,
        const PhaseContext& context) const override
    {
        const Vec3 fallback =
            normalized_or(state.velocity_mps, normalized_or(state.position_m, {1.0, 0.0, 0.0}));
        const CaseConfig* case_config = context.case_config;
        if (!case_config) {
            return fallback;
        }
        const double mu = case_config->earth_mu_m3s2;
        const double body_radius_m = case_config->earth_radius_m;

        const std::vector<post2::core::UpfgStage> stages = build_upfg_stages(*case_config, runtime);
        if (stages.empty()) {
            return fallback;
        }

        const post2::core::UpfgTarget target = post2::core::make_upfg_orbit_target(
            config_.upfg.periapsis_km * 1000.0,
            config_.upfg.apoapsis_km * 1000.0,
            config_.upfg.inclination_deg,
            state.position_m,
            state.velocity_mps,
            mu,
            body_radius_m);

        post2::core::UpfgVehicleState upfg_state;
        upfg_state.time_s = runtime.time_s;
        upfg_state.mass_kg = runtime.vehicle.total_mass_kg;
        upfg_state.position_m = state.position_m;
        upfg_state.velocity_mps = state.velocity_mps;

        const post2::core::UpfgResult result =
            post2::core::upfg_converge(stages, target, upfg_state, mu);
        if (!result.ok) {
            return fallback;
        }
        return normalized_or(result.thrust_unit_eci, fallback);
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

void apply_steering_continuity(SteeringModelConfig* steering, const AzimuthElevationDeg& angles)
{
    const std::string type = lowercase(steering->type);
    if (type == "rpy_poly" || type == "roll_pitch_yaw_poly") {
        // yaw maps to azimuth, pitch to elevation (roll has no thrust effect).
        if (steering->yaw_deg.continuity) {
            steering->yaw_deg.c0 = angles.azimuth_deg;
        }
        if (steering->pitch_deg.continuity) {
            steering->pitch_deg.c0 = angles.elevation_deg;
        }
        return;
    }
    if (type == "generic_selectable" || type == "selectable") {
        for (auto& segment : steering->segments) {
            if (segment.model) {
                apply_steering_continuity(segment.model.get(), angles);
            }
        }
        return;
    }
    if (type == "linear_tangent" || type == "bilinear_tangent") {
        // Re-anchor the tangent intercept b = tan(boundary elevation) so the
        // pitch law continues from the previous phase's final attitude
        // (KSPTOT setBForContinuity). Azimuth re-anchors via its poly c0.
        if (steering->tangent.continuity) {
            steering->tangent.b = std::tan(degrees_to_radians(angles.elevation_deg));
        }
        if (steering->azimuth_deg.continuity) {
            steering->azimuth_deg.c0 = angles.azimuth_deg;
        }
        return;
    }
    if (type == "upfg") {
        // Closed-loop guidance: nothing to re-anchor, the command is recomputed
        // from the live state every call.
        return;
    }
    if (type == "segmented_poly" || type == "generic_segmented_poly" || type == "piecewise_poly") {
        auto set_c0 = [](std::vector<double>* coefficients, double value) {
            if (coefficients->empty()) {
                coefficients->push_back(value);
            } else {
                (*coefficients)[0] = value;
            }
        };
        if (steering->segmented_poly.segments.empty()) {
            SegmentedSteeringPolySegmentConfig segment;
            segment.start_time_s = 0.0;
            steering->segmented_poly.segments.push_back(std::move(segment));
        }
        auto& first = steering->segmented_poly.segments.front();
        if (steering->azimuth_deg.continuity) {
            set_c0(&first.azimuth_coefficients, angles.azimuth_deg);
        }
        if (steering->elevation_deg.continuity) {
            set_c0(&first.elevation_coefficients, angles.elevation_deg);
        }
        return;
    }
    // generic_poly (default) and any poly-style fallback.
    if (steering->azimuth_deg.continuity) {
        steering->azimuth_deg.c0 = angles.azimuth_deg;
    }
    if (steering->elevation_deg.continuity) {
        steering->elevation_deg.c0 = angles.elevation_deg;
    }
}

bool steering_has_continuity(const SteeringModelConfig& steering)
{
    if (steering.roll_deg.continuity || steering.pitch_deg.continuity ||
        steering.yaw_deg.continuity || steering.azimuth_deg.continuity ||
        steering.elevation_deg.continuity || steering.tangent.continuity) {
        return true;
    }
    for (const auto& segment : steering.segments) {
        if (segment.model && steering_has_continuity(*segment.model)) {
            return true;
        }
    }
    return false;
}

void set_coefficient_zero(std::vector<double>* coefficients, double value)
{
    if (!coefficients) {
        return;
    }
    if (coefficients->empty()) {
        coefficients->push_back(value);
    } else {
        (*coefficients)[0] = value;
    }
}

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
    if (type == "segmented_poly" || type == "piecewise_poly") {
        return std::make_unique<ThrottleSegmentedPolyModel>(config);
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
    if (type == "segmented_poly" ||
        type == "generic_segmented_poly" ||
        type == "piecewise_poly") {
        return std::make_unique<GenericSegmentedPolySteeringModel>(config);
    }
    if (type == "linear_tangent") {
        return std::make_unique<LinearTangentSteeringModel>(config, /*bilinear=*/false);
    }
    if (type == "bilinear_tangent") {
        return std::make_unique<LinearTangentSteeringModel>(config, /*bilinear=*/true);
    }
    if (type == "upfg") {
        return std::make_unique<UpfgSteeringModel>(config);
    }
    return std::make_unique<GenericPolySteeringModel>(config);
}

void apply_phase_start_continuity(
    PhaseConfig* phase,
    const CaseConfig& case_config,
    const post2::vehicle::VehicleRuntimeState& start_runtime,
    double boundary_throttle,
    const Vec3& boundary_direction_eci)
{
    if (!phase) {
        return;
    }

    // Throttle: re-anchor so the new phase opens at the previous phase's final
    // throttle. Each model type expresses that hold differently.
    ThrottleModelConfig& throttle = phase->throttle_model;
    if (throttle.continuity) {
        const double held = clamp_throttle(boundary_throttle);
        const std::string type = lowercase(throttle.type);
        if (type == "t2w" || type == "target_t2w") {
            const double max_thrust_n =
                post2::vehicle::active_max_thrust_n(case_config.vehicle, start_runtime);
            const double mass_kg = start_runtime.vehicle.total_mass_kg;
            if (max_thrust_n > 0.0 && mass_kg > 0.0) {
                throttle.target_t2w = held * max_thrust_n / (mass_kg * kStandardGravityMps2);
            }
        } else if (type == "interpolated" || type == "tabular") {
            if (!throttle.points.empty()) {
                throttle.points.front().throttle = held;
            }
        } else if (type == "segmented_poly" || type == "piecewise_poly") {
            if (throttle.segmented_poly.segments.empty()) {
                throttle.segmented_poly.segments.push_back({0.0, {held}});
            } else {
                set_coefficient_zero(
                    &throttle.segmented_poly.segments.front().coefficients,
                    held);
            }
        } else {
            // poly (default): the constant term is the value at phase t=0.
            throttle.c0 = held;
        }
    }

    // Steering: re-anchor enabled angle constant terms to the local
    // azimuth/elevation of the previous phase's final thrust direction.
    if (steering_has_continuity(phase->steering_model)) {
        const AzimuthElevationDeg angles =
            azimuth_elevation_from_direction(start_runtime.vehicle.motion, boundary_direction_eci);
        apply_steering_continuity(&phase->steering_model, angles);
    }
}

} // namespace post2::core
