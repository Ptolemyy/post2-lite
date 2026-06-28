#pragma once

#include <vector>

#include "post2/aero/aero_table.hpp"

namespace post2::aero {

// Axisymmetric launch-vehicle geometry needed by the semi-empirical model.
// Lengths in metres. Areas are filled by finalize_geometry().
struct AeroGeometry {
    double ref_diameter_m = 3.66;    // max body/fairing diameter (sets S_ref)
    double total_length_m = 70.0;    // nose tip to base
    double nose_length_m = 13.1;     // ogive/cone length (fairing or interstage cap)
    double base_diameter_m = 3.66;   // diameter at the base plane
    bool power_on = true;            // engine firing -> base drag largely suppressed

    double ref_area_m2 = 0.0;        // pi/4 * ref_diameter^2 (computed)
    double base_area_m2 = 0.0;       // pi/4 * base_diameter^2 (computed)
    double wetted_area_m2 = 0.0;     // skin area for friction drag (computed)
    double planform_area_m2 = 0.0;   // ref_diameter * length, for crossflow (computed)

    double nose_fineness() const;    // nose_length / ref_diameter
    double body_fineness() const;    // total_length / ref_diameter
};

// Fills ref_area_m2, base_area_m2, wetted_area_m2, planform_area_m2 from the
// linear dimensions. Call after setting diameters/lengths.
void finalize_geometry(AeroGeometry* geometry);

// Tunable coefficients of the semi-empirical model. Defaults are reasonable for
// a clean slender launch vehicle and can be calibrated against a few CFD/FAR
// points without regenerating any code.
struct AeroModelTuning {
    double wave_drag_coeff = 0.55;       // scales nose transonic/supersonic wave drag
    double base_drag_coeff = 0.13;       // subsonic base drag (|Cp_base|)
    double power_on_base_factor = 0.10;  // fraction of base drag kept while thrusting
    double skin_friction_scale = 1.05;   // roughness/excrescence multiplier on Cf
    double cn_alpha_per_rad = 2.0;       // potential normal-force slope (Barrowman nose)
    double crossflow_drag_coeff = 1.2;   // cylinder crossflow drag coefficient
    double crossflow_efficiency = 0.65;  // finite-length crossflow efficiency (eta)
};

// Grid the table is generated on. Empty vectors fall back to sensible defaults
// (dense around the transonic region, alpha 0..20 deg).
struct AeroTableGridSpec {
    std::vector<double> mach;
    std::vector<double> alpha_deg;
};

// Deployed grid fins on a returning booster (lattice control surfaces). count is
// the number of fins; area_per_fin_m2 is the planform reference area of one fin.
// A zero count disables the grid-fin contribution.
struct GridFinSpec {
    int count = 0;
    double area_per_fin_m2 = 0.0;

    double total_area_m2() const {
        return count > 0 ? count * area_per_fin_m2 : 0.0;
    }
};

// Wind-axis coefficients at one operating point (alpha in radians, >= 0).
struct AeroCoefficients {
    double cd = 0.0;  // along -v_rel
    double cl = 0.0;  // perpendicular to v_rel, toward the body axis
    double ca = 0.0;  // axial (body x) force coefficient
    double cn = 0.0;  // normal (body) force coefficient
};

AeroCoefficients aero_coefficients(const AeroGeometry& geometry,
                                   const AeroModelTuning& tuning,
                                   double mach,
                                   double alpha_rad);

// Grid-fin axial drag coefficient (referenced to one fin's planform area) vs
// Mach. Lattice fins choke transonically (a sharp drag rise/peak near M~1.1),
// then unchoke supersonically. Semi-empirical.
double grid_fin_drag_coefficient(double mach);

// Grid-fin normal-force slope per radian vs Mach (control authority; drops
// transonically with the choking, partially recovers supersonically).
double grid_fin_normal_force_slope(double mach);

// Wind-axis grid-fin coefficients (cd along -v_rel, cl perpendicular), referenced
// to ref_area_m2, for the whole fin set. The axial drag is ~angle-independent
// (the lattice presents its frontal area in axial flow); the normal force is the
// classic sin*cos law. Returns zeros when the spec is empty.
AeroCoefficients grid_fin_coefficients(const GridFinSpec& fins,
                                       double mach,
                                       double alpha_rad,
                                       double ref_area_m2);

// Sutton-Graves stagnation-point convective heat flux [W/m^2]:
//   q = K * sqrt(rho / R_n) * V^3,  K = 1.7415e-4 (SI, yields W/m^2).
// density in kg/m^3, speed in m/s, nose_radius in m. Returns 0 for any
// non-positive input (a sharp/zero-radius nose has no defined stagnation point).
double stagnation_heat_flux_wpm2(double density_kgpm3,
                                 double speed_mps,
                                 double nose_radius_m);

// Effective stagnation nose radius [m] used by the heat-flux model. Returns the
// configured radius when positive; otherwise estimates a slender-launcher tip as
// 10% of the reference diameter, falling back to 0.5 m when the diameter is unset.
double effective_nose_radius_m(double configured_nose_radius_m,
                               double ref_diameter_m);

// Generates a (Mach x alpha) table of wind-axis CD/CL referenced to
// geometry.ref_area_m2. When fins.count > 0 the deployed grid-fin contribution
// is added to every cell, so the generated table already includes the fins (used
// for the booster-only recovery configuration).
AeroTable generate_aero_table(const AeroGeometry& geometry,
                              const AeroModelTuning& tuning = {},
                              const AeroTableGridSpec& grid = {},
                              const GridFinSpec& fins = {});

} // namespace post2::aero
