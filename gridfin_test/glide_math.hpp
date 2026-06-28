// glide_math.hpp -- small 3-vector helpers shared by the guidance/control code.
//
// Pure math (no kRPC types) so everything that uses it runs unchanged in the
// offline --dry-run sanity simulation as well as in the live kRPC loop.

#pragma once

#include <array>
#include <cmath>

namespace gridfin {

using Vec3 = std::array<double, 3>;

inline Vec3   v_add  (const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
inline Vec3   v_sub  (const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline Vec3   v_scale(const Vec3& a, double s)      { return {a[0]*s, a[1]*s, a[2]*s}; }
inline double v_dot  (const Vec3& a, const Vec3& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
inline Vec3   v_cross(const Vec3& a, const Vec3& b) {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
inline double v_norm (const Vec3& a) { return std::sqrt(std::max(0.0, v_dot(a, a))); }
inline Vec3   v_hat  (const Vec3& a) {
    const double n = v_norm(a);
    return n > 1e-12 ? v_scale(a, 1.0 / n) : Vec3{0.0, 0.0, 0.0};
}
// Component of `a` perpendicular to unit vector `u_hat`.
inline Vec3 v_perp(const Vec3& a, const Vec3& u_hat) {
    return v_sub(a, v_scale(u_hat, v_dot(a, u_hat)));
}

inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

constexpr double kRadToDeg = 57.29577951308232;
constexpr double kDegToRad = 0.017453292519943295;

} // namespace gridfin
