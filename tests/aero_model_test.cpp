#include "post2/aero/aero_model.hpp"
#include "post2/aero/aero_table.hpp"
#include "post2/environment/atmosphere.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void check(const char* what, bool ok)
{
    if (!ok) {
        std::cerr << "FAIL: " << what << '\n';
        ++g_failures;
    }
}

post2::aero::AeroGeometry falcon9_geometry()
{
    post2::aero::AeroGeometry g;
    g.ref_diameter_m = 5.2;
    g.total_length_m = 70.0;
    g.nose_length_m = 12.6;
    g.base_diameter_m = 3.66;
    g.power_on = true;
    post2::aero::finalize_geometry(&g);
    return g;
}

double cd0(const post2::aero::AeroGeometry& g, double mach)
{
    return post2::aero::aero_coefficients(g, {}, mach, 0.0).cd;
}

void test_geometry()
{
    const post2::aero::AeroGeometry g = falcon9_geometry();
    check("ref area ~ pi/4 D^2", std::fabs(g.ref_area_m2 - 21.24) < 0.2);
    check("wetted area positive and large", g.wetted_area_m2 > g.ref_area_m2 * 5.0);
    check("nose fineness", std::fabs(g.nose_fineness() - 12.6 / 5.2) < 1e-6);
}

void test_transonic_drag_rise()
{
    const post2::aero::AeroGeometry g = falcon9_geometry();
    const double cd_sub = cd0(g, 0.5);
    const double cd_trans = cd0(g, 1.1);
    const double cd_suphi = cd0(g, 4.0);
    // Drag must rise into the transonic peak and fall off supersonically.
    check("transonic peak > subsonic", cd_trans > cd_sub * 1.3);
    check("transonic peak > high supersonic", cd_trans > cd_suphi);
    check("subsonic drag in sane range", cd_sub > 0.05 && cd_sub < 0.6);
    check("transonic drag in sane range", cd_trans > 0.1 && cd_trans < 1.2);
}

void test_lift_vs_alpha()
{
    const post2::aero::AeroGeometry g = falcon9_geometry();
    const double deg2rad = 3.14159265358979323846 / 180.0;
    const post2::aero::AeroCoefficients a0 = post2::aero::aero_coefficients(g, {}, 0.6, 0.0);
    const post2::aero::AeroCoefficients a5 = post2::aero::aero_coefficients(g, {}, 0.6, 5.0 * deg2rad);
    const post2::aero::AeroCoefficients a10 = post2::aero::aero_coefficients(g, {}, 0.6, 10.0 * deg2rad);
    check("CL ~ 0 at alpha 0", std::fabs(a0.cl) < 1e-9);
    check("CL increases with alpha", a5.cl > 0.01 && a10.cl > a5.cl);
    check("CD increases with alpha (induced)", a10.cd > a0.cd);
}

void test_table_roundtrip()
{
    const post2::aero::AeroGeometry g = falcon9_geometry();
    const post2::aero::AeroTable table = post2::aero::generate_aero_table(g);
    check("table not empty", !table.empty());
    check("table ref area set", std::fabs(table.reference_area_m2 - g.ref_area_m2) < 1e-6);

    const std::string path = "aero_table_roundtrip.csv";
    std::string err;
    check("write csv", post2::aero::write_aero_table_csv(path, table, &err));

    post2::aero::AeroTable loaded;
    check("read csv", post2::aero::read_aero_table_csv(path, &loaded, &err));

    // Compare a few interpolated lookups.
    double cd_a = 0.0, cl_a = 0.0, cd_b = 0.0, cl_b = 0.0;
    table.lookup(1.05, 3.0, &cd_a, &cl_a);
    loaded.lookup(1.05, 3.0, &cd_b, &cl_b);
    check("roundtrip cd", std::fabs(cd_a - cd_b) < 1e-4);
    check("roundtrip cl", std::fabs(cl_a - cl_b) < 1e-4);

    // CL must be odd in alpha (symmetric body).
    double cl_pos = 0.0, cl_neg = 0.0, dummy = 0.0;
    table.lookup(0.6, 6.0, &dummy, &cl_pos);
    table.lookup(0.6, -6.0, &dummy, &cl_neg);
    check("CL odd in alpha", std::fabs(cl_pos + cl_neg) < 1e-9 && cl_pos > 0.0);
    std::remove(path.c_str());
}

void test_us1976_atmosphere()
{
    const post2::environment::AtmosphereSample sea = post2::environment::us_standard_1976(0.0);
    const post2::environment::AtmosphereSample h11 = post2::environment::us_standard_1976(11000.0);
    const post2::environment::AtmosphereSample h20 = post2::environment::us_standard_1976(20000.0);
    check("sea-level density ~1.225", std::fabs(sea.density_kgpm3 - 1.225) < 0.01);
    check("sea-level pressure ~101325", std::fabs(sea.pressure_pa - 101325.0) < 50.0);
    check("sea-level sound ~340", std::fabs(sea.speed_of_sound_mps - 340.3) < 1.0);
    check("11km density ~0.364", std::fabs(h11.density_kgpm3 - 0.3639) < 0.01);
    check("density decreases with altitude", h20.density_kgpm3 < h11.density_kgpm3 &&
                                              h11.density_kgpm3 < sea.density_kgpm3);
    check("tropopause temp ~216.65", std::fabs(h11.temperature_k - 216.65) < 0.5);
}

} // namespace

int main()
{
    test_geometry();
    test_transonic_drag_rise();
    test_lift_vs_alpha();
    test_table_roundtrip();
    test_us1976_atmosphere();
    if (g_failures != 0) {
        std::cerr << g_failures << " aero checks failed\n";
        return 1;
    }
    std::cout << "aero model test passed\n";
    return 0;
}
