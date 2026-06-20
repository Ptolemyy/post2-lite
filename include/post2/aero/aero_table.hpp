#pragma once

#include <string>
#include <vector>

namespace post2::aero {

// Tabulated aerodynamic coefficients on a (Mach, angle-of-attack) grid.
// cd/cl are stored row-major: index = i_mach * alpha_deg.size() + j_alpha.
// Coefficients are wind-axis: drag is along -v_rel, lift perpendicular to it,
// both referenced to AeroGeometry::ref_area_m2.
struct AeroTable {
    std::vector<double> mach;        // strictly ascending
    std::vector<double> alpha_deg;   // strictly ascending (>= 0)
    std::vector<double> cd;          // size mach.size() * alpha_deg.size()
    std::vector<double> cl;          // same layout as cd
    double reference_area_m2 = 0.0;  // S_ref the coefficients are normalized to

    bool empty() const {
        return mach.empty() || alpha_deg.empty() ||
               cd.size() != mach.size() * alpha_deg.size() ||
               cl.size() != mach.size() * alpha_deg.size();
    }

    // Bilinear interpolation with edge clamping. alpha is treated as |alpha|
    // (symmetric body). Safe to call on an empty table (returns 0/0).
    void lookup(double mach, double alpha_deg, double* cd_out, double* cl_out) const;
};

// Writes a "post2-lite aero table v1" CSV (mach,alpha_deg,cd,cl long format).
bool write_aero_table_csv(const std::string& path, const AeroTable& table, std::string* error);

// Reads the CSV written by write_aero_table_csv. Rebuilds the grid axes from
// the rows; rows must form a complete rectangular (Mach x alpha) grid.
bool read_aero_table_csv(const std::string& path, AeroTable* table, std::string* error);

} // namespace post2::aero
