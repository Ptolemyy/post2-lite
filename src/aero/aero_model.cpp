#include "post2/aero/aero_model.hpp"

#include <algorithm>
#include <cmath>

#include "post2/environment/atmosphere.hpp"

namespace post2::aero {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clampd(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

// Representative geometric altitude a launch vehicle has when passing through a
// given Mach number on a nominal ascent. Used to fix a reference Reynolds number
// for the skin-friction term (which otherwise needs the live trajectory). The
// friction variation with altitude is second-order versus transonic wave drag.
double reference_altitude_for_mach(double mach) {
    static const double m[] = {0.0, 0.3, 0.5, 0.8, 1.0, 1.2, 1.5,
                               2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0};
    static const double h[] = {0.0,    1500.0,  3000.0,  5500.0,  7500.0,
                               9000.0, 11000.0, 14000.0, 17000.0, 20000.0,
                               27000.0, 33000.0, 40000.0, 50000.0, 60000.0};
    const int n = static_cast<int>(sizeof(m) / sizeof(m[0]));
    if (mach <= m[0]) {
        return h[0];
    }
    if (mach >= m[n - 1]) {
        return h[n - 1];
    }
    for (int i = 1; i < n; ++i) {
        if (mach <= m[i]) {
            const double t = (mach - m[i - 1]) / (m[i] - m[i - 1]);
            return h[i - 1] + (h[i] - h[i - 1]) * t;
        }
    }
    return h[n - 1];
}

// Turbulent flat-plate skin friction (Schlichting) with a compressibility
// correction, evaluated at the reference flight condition for this Mach.
double skin_friction_cd(const AeroGeometry& geom, const AeroModelTuning& tuning, double mach) {
    const double alt = reference_altitude_for_mach(std::max(mach, 0.05));
    const post2::environment::AtmosphereSample atmo = post2::environment::us_standard_1976(alt);
    const double mu = post2::environment::air_dynamic_viscosity_sutherland(atmo.temperature_k);
    const double v = std::max(mach, 0.05) * atmo.speed_of_sound_mps;
    double re = atmo.density_kgpm3 * v * geom.total_length_m / std::max(mu, 1e-9);
    re = std::max(re, 1.0e4);
    const double log10re = std::log10(re);
    const double cf_incompressible = 0.455 / std::pow(log10re, 2.58);
    const double cf = cf_incompressible / std::pow(1.0 + 0.144 * mach * mach, 0.65);
    const double area_ratio = geom.wetted_area_m2 / std::max(geom.ref_area_m2, 1e-9);
    return tuning.skin_friction_scale * cf * area_ratio;
}

// Transonic/supersonic shape function for nose wave drag, peaking near M~1.2.
double wave_drag_shape(double mach) {
    if (mach < 0.8) {
        return 0.0;
    }
    if (mach <= 1.2) {
        return std::sin((kPi / 2.0) * (mach - 0.8) / 0.4);  // 0 at 0.8 -> 1 at 1.2
    }
    const double denom = std::sqrt(std::max(mach * mach - 1.0, 1e-6));
    const double ref = std::sqrt(1.2 * 1.2 - 1.0);  // continuity at M=1.2
    return ref / denom;
}

double wave_drag_cd(const AeroGeometry& geom, const AeroModelTuning& tuning, double mach) {
    const double fineness = std::max(geom.nose_fineness(), 0.3);
    const double peak = tuning.wave_drag_coeff / fineness;  // slender nose -> less wave drag
    return peak * wave_drag_shape(mach);
}

double base_drag_cd(const AeroGeometry& geom, const AeroModelTuning& tuning, double mach) {
    double mach_factor;
    if (mach <= 0.8) {
        mach_factor = 1.0;
    } else if (mach <= 1.2) {
        mach_factor = 1.0 + 0.4 * (mach - 0.8) / 0.4;  // up to 1.4 transonically
    } else {
        mach_factor = 1.4 * 1.2 / mach;  // decays supersonically
    }
    const double area_ratio = geom.base_area_m2 / std::max(geom.ref_area_m2, 1e-9);
    double cd_base = tuning.base_drag_coeff * area_ratio * mach_factor;
    if (geom.power_on) {
        cd_base *= tuning.power_on_base_factor;  // plume fills the base while thrusting
    }
    return cd_base;
}

double normal_force_slope(const AeroModelTuning& tuning, double mach) {
    // Slender-body potential slope ~ constant, with a mild transonic bump.
    const double beta = std::sqrt(std::fabs(1.0 - mach * mach));
    const double comp = mach < 1.0 ? 1.0 / clampd(beta, 0.7, 1.0) : 1.0;
    return tuning.cn_alpha_per_rad * comp;
}

} // namespace

double AeroGeometry::nose_fineness() const {
    return nose_length_m / std::max(ref_diameter_m, 1e-6);
}

double AeroGeometry::body_fineness() const {
    return total_length_m / std::max(ref_diameter_m, 1e-6);
}

void finalize_geometry(AeroGeometry* geometry) {
    if (geometry == nullptr) {
        return;
    }
    AeroGeometry& g = *geometry;
    g.ref_diameter_m = std::max(g.ref_diameter_m, 1e-3);
    g.total_length_m = std::max(g.total_length_m, g.ref_diameter_m);
    g.nose_length_m = clampd(g.nose_length_m, 0.0, g.total_length_m);
    g.base_diameter_m = std::max(g.base_diameter_m, 1e-3);

    g.ref_area_m2 = 0.25 * kPi * g.ref_diameter_m * g.ref_diameter_m;
    g.base_area_m2 = 0.25 * kPi * g.base_diameter_m * g.base_diameter_m;
    g.planform_area_m2 = g.ref_diameter_m * g.total_length_m;

    const double cyl_length = std::max(g.total_length_m - g.nose_length_m, 0.0);
    const double cyl_wetted = kPi * g.ref_diameter_m * cyl_length;
    const double radius = 0.5 * g.ref_diameter_m;
    const double slant = std::sqrt(g.nose_length_m * g.nose_length_m + radius * radius);
    const double nose_wetted = kPi * radius * slant;  // conical approximation
    g.wetted_area_m2 = cyl_wetted + nose_wetted;
}

AeroCoefficients aero_coefficients(const AeroGeometry& geometry,
                                   const AeroModelTuning& tuning,
                                   double mach,
                                   double alpha_rad) {
    const double a = std::fabs(alpha_rad);
    const double sa = std::sin(a);
    const double ca = std::cos(a);

    // Axial force coefficient (== CD0 at alpha = 0).
    const double cd0 = skin_friction_cd(geometry, tuning, mach) +
                       wave_drag_cd(geometry, tuning, mach) +
                       base_drag_cd(geometry, tuning, mach);

    // Normal force: potential (Barrowman) + viscous crossflow (Allen-Perkins).
    const double cn_alpha = normal_force_slope(tuning, mach);
    const double cn_potential = cn_alpha * sa * ca;
    const double planform_ratio = geometry.planform_area_m2 / std::max(geometry.ref_area_m2, 1e-9);
    const double cn_crossflow = tuning.crossflow_efficiency * tuning.crossflow_drag_coeff *
                                planform_ratio * sa * sa;
    const double cn = cn_potential + cn_crossflow;

    AeroCoefficients out;
    out.ca = cd0;
    out.cn = cn;
    out.cd = cd0 * ca + cn * sa;   // wind-axis drag
    out.cl = cn * ca - cd0 * sa;   // wind-axis lift
    return out;
}

AeroTable generate_aero_table(const AeroGeometry& geometry,
                              const AeroModelTuning& tuning,
                              const AeroTableGridSpec& grid) {
    AeroGeometry geom = geometry;
    finalize_geometry(&geom);

    AeroTable table;
    table.mach = grid.mach;
    table.alpha_deg = grid.alpha_deg;
    if (table.mach.empty()) {
        table.mach = {0.0, 0.2, 0.4, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1,
                      1.2, 1.3, 1.5, 1.75, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0};
    }
    if (table.alpha_deg.empty()) {
        table.alpha_deg = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0};
    }
    table.reference_area_m2 = geom.ref_area_m2;

    const std::size_t na = table.alpha_deg.size();
    table.cd.assign(table.mach.size() * na, 0.0);
    table.cl.assign(table.mach.size() * na, 0.0);
    const double deg2rad = kPi / 180.0;
    for (std::size_t i = 0; i < table.mach.size(); ++i) {
        for (std::size_t j = 0; j < na; ++j) {
            const AeroCoefficients c =
                aero_coefficients(geom, tuning, table.mach[i], table.alpha_deg[j] * deg2rad);
            table.cd[i * na + j] = c.cd;
            table.cl[i * na + j] = c.cl;
        }
    }
    return table;
}

} // namespace post2::aero
