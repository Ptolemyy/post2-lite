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

// Generates a (Mach x alpha) table of wind-axis CD/CL referenced to
// geometry.ref_area_m2.
AeroTable generate_aero_table(const AeroGeometry& geometry,
                              const AeroModelTuning& tuning = {},
                              const AeroTableGridSpec& grid = {});

} // namespace post2::aero
