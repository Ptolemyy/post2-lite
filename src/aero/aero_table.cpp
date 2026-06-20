#include "post2/aero/aero_table.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace post2::aero {

namespace {

// Returns lower index i such that axis[i] <= x <= axis[i+1], plus the fraction
// t in [0,1] between them. Clamps to the axis ends. Assumes axis.size() >= 1.
std::size_t bracket(const std::vector<double>& axis, double x, double* t) {
    if (axis.size() == 1) {
        *t = 0.0;
        return 0;
    }
    if (x <= axis.front()) {
        *t = 0.0;
        return 0;
    }
    if (x >= axis.back()) {
        *t = 0.0;
        return axis.size() - 2;
    }
    const auto it = std::upper_bound(axis.begin(), axis.end(), x);
    const std::size_t hi = static_cast<std::size_t>(it - axis.begin());
    const std::size_t lo = hi - 1;
    const double span = axis[hi] - axis[lo];
    *t = span > 0.0 ? (x - axis[lo]) / span : 0.0;
    return lo;
}

} // namespace

void AeroTable::lookup(double mach_query, double alpha_query_deg,
                       double* cd_out, double* cl_out) const {
    if (cd_out) {
        *cd_out = 0.0;
    }
    if (cl_out) {
        *cl_out = 0.0;
    }
    if (empty()) {
        return;
    }
    const double a = std::fabs(alpha_query_deg);
    double tm = 0.0;
    double ta = 0.0;
    const std::size_t im = bracket(mach, mach_query, &tm);
    const std::size_t ia = bracket(alpha_deg, a, &ta);
    const std::size_t na = alpha_deg.size();
    const std::size_t im1 = std::min(im + 1, mach.size() - 1);
    const std::size_t ia1 = std::min(ia + 1, na - 1);

    auto sample = [&](const std::vector<double>& grid) {
        const double c00 = grid[im * na + ia];
        const double c01 = grid[im * na + ia1];
        const double c10 = grid[im1 * na + ia];
        const double c11 = grid[im1 * na + ia1];
        const double c0 = c00 + (c01 - c00) * ta;
        const double c1 = c10 + (c11 - c10) * ta;
        return c0 + (c1 - c0) * tm;
    };

    if (cd_out) {
        *cd_out = sample(cd);
    }
    if (cl_out) {
        // CL is odd in alpha for a symmetric body; restore the queried sign.
        const double sign = alpha_query_deg < 0.0 ? -1.0 : 1.0;
        *cl_out = sign * sample(cl);
    }
}

bool write_aero_table_csv(const std::string& path, const AeroTable& table, std::string* error) {
    if (table.empty()) {
        if (error) {
            *error = "aero table is empty";
        }
        return false;
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        if (error) {
            *error = "could not open aero table for writing: " + path;
        }
        return false;
    }
    out << "# post2-lite aero table v1\n";
    out << "# reference_area_m2," << table.reference_area_m2 << "\n";
    out << "mach,alpha_deg,cd,cl\n";
    const std::size_t na = table.alpha_deg.size();
    out.setf(std::ios::fixed);
    out.precision(6);
    for (std::size_t i = 0; i < table.mach.size(); ++i) {
        for (std::size_t j = 0; j < na; ++j) {
            out << table.mach[i] << ',' << table.alpha_deg[j] << ','
                << table.cd[i * na + j] << ',' << table.cl[i * na + j] << '\n';
        }
    }
    if (!out) {
        if (error) {
            *error = "write failed for aero table: " + path;
        }
        return false;
    }
    return true;
}

bool read_aero_table_csv(const std::string& path, AeroTable* table, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) {
            *error = "could not open aero table for reading: " + path;
        }
        return false;
    }
    AeroTable parsed;
    std::vector<double> machs;   // per-row
    std::vector<double> alphas;  // per-row
    std::vector<double> cds;
    std::vector<double> cls;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            const std::string key = "reference_area_m2,";
            const auto pos = line.find(key);
            if (pos != std::string::npos) {
                parsed.reference_area_m2 = std::strtod(line.c_str() + pos + key.size(), nullptr);
            }
            continue;
        }
        if (line.rfind("mach", 0) == 0) {
            continue;  // header
        }
        std::stringstream ss(line);
        std::string tok;
        double vals[4] = {0, 0, 0, 0};
        int n = 0;
        while (n < 4 && std::getline(ss, tok, ',')) {
            vals[n++] = std::strtod(tok.c_str(), nullptr);
        }
        if (n < 4) {
            continue;
        }
        machs.push_back(vals[0]);
        alphas.push_back(vals[1]);
        cds.push_back(vals[2]);
        cls.push_back(vals[3]);
    }
    if (machs.empty()) {
        if (error) {
            *error = "aero table contained no data rows: " + path;
        }
        return false;
    }
    // Rebuild sorted unique axes.
    auto unique_axis = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end(),
                            [](double a, double b) { return std::fabs(a - b) < 1e-9; }),
                v.end());
        return v;
    };
    parsed.mach = unique_axis(machs);
    parsed.alpha_deg = unique_axis(alphas);
    const std::size_t nm = parsed.mach.size();
    const std::size_t na = parsed.alpha_deg.size();
    parsed.cd.assign(nm * na, 0.0);
    parsed.cl.assign(nm * na, 0.0);
    auto axis_index = [](const std::vector<double>& axis, double x) -> std::size_t {
        std::size_t best = 0;
        double bestd = std::fabs(axis[0] - x);
        for (std::size_t k = 1; k < axis.size(); ++k) {
            const double d = std::fabs(axis[k] - x);
            if (d < bestd) {
                bestd = d;
                best = k;
            }
        }
        return best;
    };
    for (std::size_t r = 0; r < machs.size(); ++r) {
        const std::size_t i = axis_index(parsed.mach, machs[r]);
        const std::size_t j = axis_index(parsed.alpha_deg, alphas[r]);
        parsed.cd[i * na + j] = cds[r];
        parsed.cl[i * na + j] = cls[r];
    }
    if (parsed.empty()) {
        if (error) {
            *error = "aero table did not form a complete grid: " + path;
        }
        return false;
    }
    *table = parsed;
    return true;
}

} // namespace post2::aero
