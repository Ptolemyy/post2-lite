#include "post2/core/frames.hpp"

#include <algorithm>
#include <cmath>

namespace post2::core::frames {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

double wrap_two_pi(double angle_rad)
{
    double y = std::fmod(angle_rad, kTwoPi);
    if (y < 0.0) {
        y += kTwoPi;
    }
    return y;
}

Vec3 rotate_z_local(const Vec3& value, double angle_rad)
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {
        value.x * c - value.y * s,
        value.x * s + value.y * c,
        value.z,
    };
}

Vec3 cross_z(double omega_rad_s, const Vec3& r)
{
    // (0, 0, omega) x (rx, ry, rz) = (-omega * ry, omega * rx, 0)
    return {-omega_rad_s * r.y, omega_rad_s * r.x, 0.0};
}

} // namespace

double prime_vertical_radius_m(double latitude_rad)
{
    const double sin_lat = std::sin(latitude_rad);
    const double denom = std::sqrt(1.0 - Wgs84::e2 * sin_lat * sin_lat);
    return Wgs84::a_m / denom;
}

Vec3 geodetic_to_ecef(const Geodetic& geo)
{
    const double sin_lat = std::sin(geo.latitude_rad);
    const double cos_lat = std::cos(geo.latitude_rad);
    const double sin_lon = std::sin(geo.longitude_rad);
    const double cos_lon = std::cos(geo.longitude_rad);
    const double n = prime_vertical_radius_m(geo.latitude_rad);
    return {
        (n + geo.altitude_m) * cos_lat * cos_lon,
        (n + geo.altitude_m) * cos_lat * sin_lon,
        (n * (1.0 - Wgs84::e2) + geo.altitude_m) * sin_lat,
    };
}

Geodetic ecef_to_geodetic(const Vec3& r)
{
    // Heikkinen, M. (1982): "Geschlossene Formeln zur Berechnung raumlicher
    // geodatischer Koordinaten aus rechtwinkligen Koordinaten."
    const double x = r.x;
    const double y = r.y;
    const double z = r.z;
    const double p2 = x * x + y * y;
    const double p = std::sqrt(p2);

    // Polar singularity: collapse longitude to 0 (undefined) and use simple
    // 1-D altitude.
    if (p < 1.0e-9) {
        Geodetic out;
        out.latitude_rad = (z >= 0.0 ? 1.0 : -1.0) * kPi / 2.0;
        out.longitude_rad = 0.0;
        out.altitude_m = std::abs(z) - Wgs84::b_m;
        return out;
    }

    const double a = Wgs84::a_m;
    const double b = Wgs84::b_m;
    const double a2 = a * a;
    const double b2 = b * b;
    const double e2 = Wgs84::e2;
    const double ep2 = Wgs84::ep2;
    const double e4 = e2 * e2;

    const double z2 = z * z;
    const double F = 54.0 * b2 * z2;
    const double G = p2 + (1.0 - e2) * z2 - e2 * (a2 - b2);
    const double c = (e4 * F * p2) / (G * G * G);
    const double s_inner = 1.0 + c + std::sqrt(std::max(0.0, c * c + 2.0 * c));
    const double s = std::cbrt(s_inner);
    const double k = s + 1.0 / s + 1.0;
    const double P = F / (3.0 * k * k * G * G);
    const double Q = std::sqrt(std::max(0.0, 1.0 + 2.0 * e4 * P));
    const double inv_1pQ = 1.0 / (1.0 + Q);

    const double r0_term1 = -(P * e2 * p) * inv_1pQ;
    const double r0_sqrt_arg =
        0.5 * a2 * (1.0 + 1.0 / Q) -
        (P * (1.0 - e2) * z2) / (Q * (1.0 + Q)) -
        0.5 * P * p2;
    const double r0 = r0_term1 + std::sqrt(std::max(0.0, r0_sqrt_arg));

    const double u_term = p - e2 * r0;
    const double U = std::sqrt(u_term * u_term + z2);
    const double V = std::sqrt(u_term * u_term + (1.0 - e2) * z2);
    const double z0 = (b2 * z) / (a * V);

    Geodetic out;
    out.altitude_m = U * (1.0 - b2 / (a * V));
    out.latitude_rad = std::atan2(z + ep2 * z0, p);
    out.longitude_rad = std::atan2(y, x);
    return out;
}

EnuBasis enu_basis_at(const Geodetic& geo)
{
    const double sin_lat = std::sin(geo.latitude_rad);
    const double cos_lat = std::cos(geo.latitude_rad);
    const double sin_lon = std::sin(geo.longitude_rad);
    const double cos_lon = std::cos(geo.longitude_rad);
    return {
        {-sin_lon, cos_lon, 0.0},
        {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat},
        {cos_lat * cos_lon, cos_lat * sin_lon, sin_lat},
    };
}

Vec3 eci_to_ecef_position(const Vec3& r_eci, double theta_rad)
{
    return rotate_z_local(r_eci, -theta_rad);
}

Vec3 ecef_to_eci_position(const Vec3& r_ecef, double theta_rad)
{
    return rotate_z_local(r_ecef, theta_rad);
}

EcefState eci_to_ecef_state(
    const Vec3& r_eci,
    const Vec3& v_eci,
    double theta_rad,
    double omega_rad_s)
{
    // v_ecef expressed in ECEF axes equals the time derivative of r in the
    // rotating frame, then rotated. v_rel_eci = v_eci - omega x r_eci is the
    // inertial-axes vector of that rotating-frame derivative; rotating it
    // into ECEF axes finishes the transform.
    const Vec3 v_rel_eci = v_eci - cross_z(omega_rad_s, r_eci);
    return {
        rotate_z_local(r_eci, -theta_rad),
        rotate_z_local(v_rel_eci, -theta_rad),
    };
}

EciState ecef_to_eci_state(
    const Vec3& r_ecef,
    const Vec3& v_ecef,
    double theta_rad,
    double omega_rad_s)
{
    const Vec3 r_eci = rotate_z_local(r_ecef, theta_rad);
    const Vec3 v_rel_eci = rotate_z_local(v_ecef, theta_rad);
    const Vec3 v_eci = v_rel_eci + cross_z(omega_rad_s, r_eci);
    return {r_eci, v_eci};
}

double julian_date_utc(const EpochUtc& epoch)
{
    // Meeus, Astronomical Algorithms ch.7, Gregorian calendar.
    int Y = epoch.year;
    int M = epoch.month;
    if (M <= 2) {
        Y -= 1;
        M += 12;
    }
    const int A = Y / 100;
    const int B = 2 - A + (A / 4);
    const double day_frac =
        epoch.day + (epoch.hour + epoch.minute / 60.0 + epoch.second / 3600.0) / 24.0;
    return std::floor(365.25 * (Y + 4716)) +
           std::floor(30.6001 * (M + 1)) +
           day_frac + B - 1524.5;
}

double gmst_rad(double jd_ut1)
{
    // IAU 1982 (Aoki et al.): GMST at 0h UT is a polynomial in T, then add
    // 1.00273790935 * elapsed seconds since 0h. Splitting the polynomial
    // evaluation at 0h from the within-day rotation preserves precision —
    // otherwise the T * 8.6e6 term loses ULPs in the modulo.
    const double jd_0h = std::floor(jd_ut1 - 0.5) + 0.5;
    const double T = (jd_0h - 2451545.0) / 36525.0;
    double gmst_at_0h_s =
        24110.54841 +
        8640184.812866 * T +
        0.093104 * T * T -
        6.2e-6 * T * T * T;
    const double seconds_since_0h = (jd_ut1 - jd_0h) * 86400.0;
    const double gmst_s = gmst_at_0h_s + 1.00273790935 * seconds_since_0h;
    const double gmst_rad_unwrapped = gmst_s * (kTwoPi / 86400.0);
    return wrap_two_pi(gmst_rad_unwrapped);
}

double earth_rotation_angle_rad(
    double gmst_at_epoch_rad,
    double omega_rad_s,
    double seconds_since_epoch)
{
    return wrap_two_pi(gmst_at_epoch_rad + omega_rad_s * seconds_since_epoch);
}

} // namespace post2::core::frames
