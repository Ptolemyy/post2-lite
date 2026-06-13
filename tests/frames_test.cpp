#include "post2/core/frames.hpp"

#include <cmath>
#include <iostream>

namespace {

using post2::core::Vec3;
using post2::core::frames::EcefState;
using post2::core::frames::EciState;
using post2::core::frames::EnuBasis;
using post2::core::frames::EpochUtc;
using post2::core::frames::Geodetic;
using post2::core::frames::Wgs84;

constexpr double kPi = 3.141592653589793238462643383279502884;

int g_failures = 0;

void fail(const char* what, double got, double expected, double tol)
{
    std::cerr << what << ": got=" << got << " expected=" << expected
              << " tol=" << tol << '\n';
    ++g_failures;
}

void check_close(const char* what, double got, double expected, double tol)
{
    if (std::abs(got - expected) > tol) {
        fail(what, got, expected, tol);
    }
}

void check_vec_close(const char* what, const Vec3& got, const Vec3& expected, double tol)
{
    if (std::abs(got.x - expected.x) > tol ||
        std::abs(got.y - expected.y) > tol ||
        std::abs(got.z - expected.z) > tol) {
        std::cerr << what << ": got=(" << got.x << ',' << got.y << ',' << got.z
                  << ") expected=(" << expected.x << ',' << expected.y << ',' << expected.z
                  << ") tol=" << tol << '\n';
        ++g_failures;
    }
}

double deg(double d) { return d * kPi / 180.0; }

void test_geodetic_to_ecef_known_points()
{
    using post2::core::frames::geodetic_to_ecef;

    // Equator on prime meridian -> (a, 0, 0)
    check_vec_close(
        "geodetic_to_ecef equator/prime meridian",
        geodetic_to_ecef({0.0, 0.0, 0.0}),
        {Wgs84::a_m, 0.0, 0.0},
        1.0e-3);

    // North pole -> (0, 0, b)
    check_vec_close(
        "geodetic_to_ecef north pole",
        geodetic_to_ecef({deg(90.0), 0.0, 0.0}),
        {0.0, 0.0, Wgs84::b_m},
        1.0e-3);

    // South pole -> (0, 0, -b)
    check_vec_close(
        "geodetic_to_ecef south pole",
        geodetic_to_ecef({deg(-90.0), 0.0, 0.0}),
        {0.0, 0.0, -Wgs84::b_m},
        1.0e-3);

    // Equator at 90E -> (0, a, 0)
    check_vec_close(
        "geodetic_to_ecef equator 90E",
        geodetic_to_ecef({0.0, deg(90.0), 0.0}),
        {0.0, Wgs84::a_m, 0.0},
        1.0e-3);

    // Equator at 0,0 alt=1000 -> (a + 1000, 0, 0)
    check_vec_close(
        "geodetic_to_ecef equator alt=1000",
        geodetic_to_ecef({0.0, 0.0, 1000.0}),
        {Wgs84::a_m + 1000.0, 0.0, 0.0},
        1.0e-6);
}

void test_geodetic_round_trip()
{
    using post2::core::frames::ecef_to_geodetic;
    using post2::core::frames::geodetic_to_ecef;

    const double lats_deg[] = {-89.0, -45.0, -28.5, 0.0, 28.5, 45.0, 89.0};
    const double lons_deg[] = {-179.0, -90.0, -30.0, 0.0, 30.0, 90.0, 179.0};
    const double alts_m[] = {0.0, 100.0, 100000.0, 1000000.0, 36000000.0};

    for (const double lat : lats_deg) {
        for (const double lon : lons_deg) {
            for (const double alt : alts_m) {
                const Geodetic in{deg(lat), deg(lon), alt};
                const Vec3 ecef = geodetic_to_ecef(in);
                const Geodetic back = ecef_to_geodetic(ecef);
                // ~1 micrometer worth of angle (~1.6e-13 rad) is unrealistic;
                // allow 1e-9 rad (~6 mm on Earth surface) and 1e-3 m for alt.
                check_close("round-trip lat", back.latitude_rad, in.latitude_rad, 1.0e-9);
                check_close("round-trip lon", back.longitude_rad, in.longitude_rad, 1.0e-9);
                check_close("round-trip alt", back.altitude_m, in.altitude_m, 1.0e-3);
            }
        }
    }
}

void test_enu_basis_orthonormal()
{
    using post2::core::frames::enu_basis_at;
    using post2::core::frames::geodetic_to_ecef;
    using post2::core::frames::prime_vertical_radius_m;

    const double lats_deg[] = {-60.0, -28.5, 0.0, 28.5, 45.0, 60.0};
    const double lons_deg[] = {-90.0, 0.0, 30.0, 120.0};

    for (const double lat : lats_deg) {
        for (const double lon : lons_deg) {
            const Geodetic geo{deg(lat), deg(lon), 0.0};
            const EnuBasis b = enu_basis_at(geo);
            check_close("ENU east . north", post2::vehicle::dot(b.east, b.north), 0.0, 1.0e-12);
            check_close("ENU east . up", post2::vehicle::dot(b.east, b.up), 0.0, 1.0e-12);
            check_close("ENU north . up", post2::vehicle::dot(b.north, b.up), 0.0, 1.0e-12);
            check_close("ENU east . east", post2::vehicle::dot(b.east, b.east), 1.0, 1.0e-12);
            check_close("ENU north . north", post2::vehicle::dot(b.north, b.north), 1.0, 1.0e-12);
            check_close("ENU up . up", post2::vehicle::dot(b.up, b.up), 1.0, 1.0e-12);

            // up should be the outward ellipsoid normal; numerically verify
            // by sampling two close geodetic altitudes along up.
            const Vec3 ecef_low = geodetic_to_ecef(geo);
            const Geodetic geo_high{geo.latitude_rad, geo.longitude_rad, 1.0};
            const Vec3 ecef_high = geodetic_to_ecef(geo_high);
            const Vec3 numeric_up{
                ecef_high.x - ecef_low.x,
                ecef_high.y - ecef_low.y,
                ecef_high.z - ecef_low.z,
            };
            // numeric_up has norm 1 m (since altitude delta is 1 m). compare
            // to analytic up
            check_close("ENU up.x", numeric_up.x, b.up.x, 1.0e-9);
            check_close("ENU up.y", numeric_up.y, b.up.y, 1.0e-9);
            check_close("ENU up.z", numeric_up.z, b.up.z, 1.0e-9);
        }
    }
}

void test_eci_ecef_round_trip()
{
    using post2::core::frames::ecef_to_eci_state;
    using post2::core::frames::eci_to_ecef_state;

    const double omegas[] = {0.0, 7.2921159e-5, 1.0e-3};
    const double thetas[] = {0.0, 0.7, 3.0, 6.0};

    const Vec3 r_eci{6378137.0 + 400000.0, 100000.0, 50000.0};
    const Vec3 v_eci{-50.0, 7600.0, 1000.0};

    for (const double omega : omegas) {
        for (const double theta : thetas) {
            const EcefState ecef = eci_to_ecef_state(r_eci, v_eci, theta, omega);
            const EciState back = ecef_to_eci_state(
                ecef.position_m, ecef.velocity_mps, theta, omega);
            check_vec_close("ECI roundtrip r", back.position_m, r_eci, 1.0e-6);
            check_vec_close("ECI roundtrip v", back.velocity_mps, v_eci, 1.0e-6);
        }
    }
}

void test_eci_ecef_position_only()
{
    using post2::core::frames::ecef_to_eci_position;
    using post2::core::frames::eci_to_ecef_position;

    // Position-only ECI->ECEF at theta=0 is identity.
    const Vec3 r{6378137.0, 0.0, 0.0};
    check_vec_close("position-only theta=0 forward", eci_to_ecef_position(r, 0.0), r, 1.0e-9);
    check_vec_close("position-only theta=0 inverse", ecef_to_eci_position(r, 0.0), r, 1.0e-9);

    // theta=pi/2 rotates (a, 0, 0) ECI to (0, -a, 0) ECEF.
    const Vec3 expected_at_pi_over_2{0.0, -6378137.0, 0.0};
    check_vec_close(
        "position-only theta=pi/2",
        eci_to_ecef_position(r, kPi / 2.0),
        expected_at_pi_over_2,
        1.0e-6);
}

void test_julian_date_known()
{
    using post2::core::frames::julian_date_utc;

    // J2000.0 == 2000-01-01T12:00:00 UTC == JD 2451545.0
    check_close(
        "JD(J2000)",
        julian_date_utc({2000, 1, 1, 12, 0, 0.0}),
        2451545.0,
        1.0e-9);

    // 1957-10-04 19:28:34 UT (Sputnik launch) ~ JD 2436116.31154
    check_close(
        "JD(Sputnik launch)",
        julian_date_utc({1957, 10, 4, 19, 28, 34.0}),
        2436116.31151,
        1.0e-4);

    // 1858-11-17 0:0:0 UT (MJD epoch) == JD 2400000.5
    check_close(
        "JD(MJD epoch)",
        julian_date_utc({1858, 11, 17, 0, 0, 0.0}),
        2400000.5,
        1.0e-9);
}

void test_gmst_known()
{
    using post2::core::frames::gmst_rad;

    // GMST at J2000.0 = 18h 41m 50.54841s = 67310.54841 s of arc / 240
    // = 67310.54841 * 2pi / 86400 rad
    const double gmst_j2000_expected = 67310.54841 * (2.0 * kPi / 86400.0);
    check_close("GMST(J2000)", gmst_rad(2451545.0), gmst_j2000_expected, 1.0e-4);

    // GMST result should always wrap to [0, 2pi).
    const double g = gmst_rad(2460000.5);
    if (g < 0.0 || g >= 2.0 * kPi) {
        std::cerr << "GMST not wrapped: " << g << '\n';
        ++g_failures;
    }
}

void test_earth_rotation_angle()
{
    using post2::core::frames::earth_rotation_angle_rad;

    // theta0 + omega*dt, wrapped.
    const double omega = 7.2921159e-5;
    check_close(
        "ERA dt=0",
        earth_rotation_angle_rad(1.0, omega, 0.0),
        1.0,
        1.0e-12);
    check_close(
        "ERA dt=1s",
        earth_rotation_angle_rad(1.0, omega, 1.0),
        1.0 + omega,
        1.0e-12);
    // wrap
    const double era_big = earth_rotation_angle_rad(0.0, 1.0, 7.0);
    if (era_big < 0.0 || era_big >= 2.0 * kPi) {
        std::cerr << "ERA not wrapped: " << era_big << '\n';
        ++g_failures;
    }
}

} // namespace

int main()
{
    test_geodetic_to_ecef_known_points();
    test_geodetic_round_trip();
    test_enu_basis_orthonormal();
    test_eci_ecef_round_trip();
    test_eci_ecef_position_only();
    test_julian_date_known();
    test_gmst_known();
    test_earth_rotation_angle();

    if (g_failures > 0) {
        std::cerr << g_failures << " frames_test failure(s)\n";
        return 1;
    }
    std::cout << "frames_test ok\n";
    return 0;
}
