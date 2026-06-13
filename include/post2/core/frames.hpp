#pragma once

#include "post2/core/types.hpp"

namespace post2::core::frames {

// WGS84 reference ellipsoid constants.
struct Wgs84 {
    static constexpr double a_m = 6378137.0;                  // semi-major axis (equatorial radius)
    static constexpr double f = 1.0 / 298.257223563;          // flattening
    static constexpr double b_m = a_m * (1.0 - f);            // semi-minor axis (polar radius)
    static constexpr double e2 = f * (2.0 - f);               // first eccentricity squared
    static constexpr double ep2 = e2 / (1.0 - e2);            // second eccentricity squared
};

struct Geodetic {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double altitude_m = 0.0;
};

struct EcefState {
    Vec3 position_m;
    Vec3 velocity_mps;
};

struct EciState {
    Vec3 position_m;
    Vec3 velocity_mps;
};

struct EnuBasis {
    Vec3 east;
    Vec3 north;
    Vec3 up;
};

// EpochUtc lives in post2::core (types.hpp) so it can be embedded in
// configuration structs without inducing a header cycle. Re-exported here for
// callers that only #include frames.hpp.
using post2::core::EpochUtc;

// ---- WGS84 geodetic <-> ECEF ------------------------------------------------

// Prime vertical radius of curvature N(lat) = a / sqrt(1 - e^2 * sin^2(lat)).
double prime_vertical_radius_m(double latitude_rad);

// WGS84 geodetic (lat, lon, ellipsoid altitude) to ECEF.
Vec3 geodetic_to_ecef(const Geodetic& geo);

// ECEF to WGS84 geodetic using Heikkinen's closed-form solution.
// Handles polar singularity (p = sqrt(x^2 + y^2) ~= 0).
Geodetic ecef_to_geodetic(const Vec3& position_ecef_m);

// Local east-north-up basis vectors expressed in ECEF coordinates at a given
// geodetic position. east, north, up form a right-handed orthonormal triad.
EnuBasis enu_basis_at(const Geodetic& geo);

// ---- ECI <-> ECEF -----------------------------------------------------------
// theta_rad is the inertial-to-rotating angle (e.g. GMST + omega * t).
// position-only variants ignore the ω x r velocity coupling and are correct
// only when the input vector is a position (or a direction whose ECEF/ECI
// distinction is purely orientation, e.g. wind).

Vec3 eci_to_ecef_position(const Vec3& r_eci, double theta_rad);
Vec3 ecef_to_eci_position(const Vec3& r_ecef, double theta_rad);

// Full state transforms; these correctly include ω x r so v_ecef expresses
// the time derivative of r_ecef in the rotating frame.
EcefState eci_to_ecef_state(
    const Vec3& r_eci,
    const Vec3& v_eci,
    double theta_rad,
    double omega_rad_s);
EciState ecef_to_eci_state(
    const Vec3& r_ecef,
    const Vec3& v_ecef,
    double theta_rad,
    double omega_rad_s);

// ---- Time / Earth rotation --------------------------------------------------

// UTC calendar date to Julian Date (Meeus, Astronomical Algorithms ch.7).
double julian_date_utc(const EpochUtc& epoch);

// Greenwich Mean Sidereal Time, IAU 1982 simplified expression.
// Returned in radians, wrapped to [0, 2pi). Input is JD_UT1; UT1 ~= UTC at
// our fidelity (|UT1 - UTC| < 0.9 s == omega*dt < 6.6e-5 rad).
double gmst_rad(double jd_ut1);

// theta(t) = gmst_at_epoch + omega * seconds_since_epoch, wrapped to [0, 2pi).
double earth_rotation_angle_rad(
    double gmst_at_epoch_rad,
    double omega_rad_s,
    double seconds_since_epoch);

} // namespace post2::core::frames
