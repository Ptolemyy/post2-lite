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

void test_heat_flux()
{
    // Zero / non-positive inputs yield no heating (no stagnation point).
    check("heat flux zero at v=0",
          post2::aero::stagnation_heat_flux_wpm2(1.0, 0.0, 0.5) == 0.0);
    check("heat flux zero at rho=0",
          post2::aero::stagnation_heat_flux_wpm2(0.0, 1000.0, 0.5) == 0.0);
    check("heat flux zero at Rn=0",
          post2::aero::stagnation_heat_flux_wpm2(1.0, 1000.0, 0.0) == 0.0);

    // Sutton-Graves closed form: q = 1.7415e-4 * sqrt(rho/Rn) * V^3 [W/m^2].
    const double q = post2::aero::stagnation_heat_flux_wpm2(0.3, 400.0, 2.6);
    const double expected = 1.7415e-4 * std::sqrt(0.3 / 2.6) * 400.0 * 400.0 * 400.0;
    check("heat flux matches Sutton-Graves", std::fabs(q - expected) < 1e-6 * expected);

    // Scaling laws: cubic in speed, monotone in density, weaker with a blunter
    // (larger-radius) nose.
    const double q_fast = post2::aero::stagnation_heat_flux_wpm2(0.3, 800.0, 2.6);
    check("heat flux cubic in speed", std::fabs(q_fast - 8.0 * q) < 1e-6 * q_fast);
    check("heat flux rises with density",
          post2::aero::stagnation_heat_flux_wpm2(0.6, 400.0, 2.6) > q);
    check("blunter nose lowers heat flux",
          post2::aero::stagnation_heat_flux_wpm2(0.3, 400.0, 5.2) < q);

    // Effective nose radius: configured value wins; auto derives from diameter.
    check("nose radius uses configured value",
          std::fabs(post2::aero::effective_nose_radius_m(0.8, 5.2) - 0.8) < 1e-12);
    check("nose radius auto from diameter",
          std::fabs(post2::aero::effective_nose_radius_m(0.0, 5.2) - 0.52) < 1e-12);
    check("nose radius auto fallback when diameter unset",
          std::fabs(post2::aero::effective_nose_radius_m(0.0, 0.0) - 0.5) < 1e-12);
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

    // Above 86 km the density must keep decaying (Vallado bands), not freeze at
    // the ~7e-6 kg/m3 boundary value the old clamp produced.
    const post2::environment::AtmosphereSample h100 = post2::environment::us_standard_1976(100000.0);
    const post2::environment::AtmosphereSample h200 = post2::environment::us_standard_1976(200000.0);
    const post2::environment::AtmosphereSample h400 = post2::environment::us_standard_1976(400000.0);
    check("100km density ~5.30e-7", std::fabs(h100.density_kgpm3 - 5.297e-7) < 5.0e-8);
    check("200km density ~2.79e-10", std::fabs(h200.density_kgpm3 - 2.789e-10) < 5.0e-11);
    check("200km density far below 86km clamp", h200.density_kgpm3 < 1.0e-8);
    check("upper atmosphere keeps decaying",
          h400.density_kgpm3 < h200.density_kgpm3 &&
              h200.density_kgpm3 < h100.density_kgpm3);
    check("200km values finite",
          std::isfinite(h200.pressure_pa) && std::isfinite(h200.speed_of_sound_mps) &&
              h200.speed_of_sound_mps > 0.0);
}

void test_grid_fins()
{
    using namespace post2::aero;
    // Lattice fins choke transonically: drag peaks near M~1.1, above the subsonic
    // and high-supersonic values.
    check("grid fin transonic peak > subsonic",
          grid_fin_drag_coefficient(1.1) > grid_fin_drag_coefficient(0.5));
    check("grid fin transonic peak > supersonic",
          grid_fin_drag_coefficient(1.1) > grid_fin_drag_coefficient(4.0));
    check("grid fin disabled => zero",
          GridFinSpec{}.total_area_m2() == 0.0);

    // Generating a booster-only table with fins raises Cd vs without.
    AeroGeometry g;
    g.ref_diameter_m = 3.66;
    g.total_length_m = 42.0;
    g.nose_length_m = 5.0;
    g.base_diameter_m = 3.66;
    g.power_on = true;
    GridFinSpec fins;
    fins.count = 4;
    fins.area_per_fin_m2 = 1.25;
    const AeroTable bare = generate_aero_table(g, {}, {});
    const AeroTable finned = generate_aero_table(g, {}, {}, fins);
    const std::size_t na = bare.alpha_deg.size();
    bool any_higher = false;
    for (std::size_t i = 0; i < bare.mach.size(); ++i) {
        if (finned.cd[i * na] > bare.cd[i * na] + 1.0e-9) {
            any_higher = true;
            break;
        }
    }
    check("grid fins raise booster table Cd", any_higher);
}

} // namespace

int main()
{
    test_geometry();
    test_transonic_drag_rise();
    test_lift_vs_alpha();
    test_table_roundtrip();
    test_heat_flux();
    test_us1976_atmosphere();
    test_grid_fins();
    if (g_failures != 0) {
        std::cerr << g_failures << " aero checks failed\n";
        return 1;
    }
    std::cout << "aero model test passed\n";
    return 0;
}
