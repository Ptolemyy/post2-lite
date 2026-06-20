// Verifies the per-staging-configuration aero-table generator and the
// max_attached_stage descriptor round-trips through case JSON.
#include "post2/core/aero_table_build.hpp"
#include "post2/core/case_config_io.hpp"
#include "post2/core/ksp_vehicle_site_import.hpp"
#include "post2/core/types.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void check(const char* what, bool ok)
{
    if (!ok) {
        std::cerr << "FAIL: " << what << '\n';
        ++g_failures;
    }
}

post2::vehicle::StageConfig make_stage(const std::string& name, double dry_kg,
                                       double prop_kg)
{
    post2::vehicle::StageConfig s;
    s.name = name;
    s.dry_mass_kg = dry_kg;
    s.engine.enabled = true;
    s.engine.thrust_vac_n = 1.0e6;
    s.engine.isp_vac_s = 300.0;
    post2::vehicle::TankConfig tank;
    tank.capacity_kg = prop_kg;
    tank.initial_kg = prop_kg;
    s.tanks = {tank};
    return s;
}

// Counts tables in `tables` whose (lo, max) descriptor equals (lo, max).
int count_config(const std::vector<post2::vehicle::AeroStageTable>& tables, int lo,
                 int max_attached)
{
    int n = 0;
    for (const auto& t : tables) {
        if (t.activate_at_min_attached_stage == lo &&
            t.max_attached_stage == max_attached) {
            ++n;
        }
    }
    return n;
}

void test_three_stage_ladder()
{
    post2::core::CaseConfig cfg;
    cfg.vehicle.aero.body_length_m = 60.0;
    cfg.vehicle.aero.ref_diameter_m = 5.0;
    cfg.vehicle.aero.base_diameter_m = 3.6;
    cfg.vehicle.stages = {
        make_stage("booster", 20000.0, 400000.0),
        make_stage("second", 4000.0, 90000.0),
        make_stage("third", 1500.0, 20000.0),
    };
    cfg.phases.push_back(post2::core::PhaseConfig{});

    post2::core::KspVehicleSiteImport imported;  // no measured lengths -> estimates
    std::string error;
    const bool ok = post2::core::generate_case_aero_tables(&cfg, imported, "", &error);
    check("generation succeeds", ok);
    if (!ok) {
        std::cerr << "  error: " << error << '\n';
        return;
    }

    const auto& tables = cfg.vehicle.aero.stage_tables;
    // 3 stages (top index 2): full[0,top] + upper[1,top] + upper[2,top]
    //                         + alone[0,0] + alone[1,1] = 5 tables.
    check("five tables generated", tables.size() == 5);
    check("full stack [0,top]", count_config(tables, 0, -1) == 1);
    check("upper after stage1 [1,top]", count_config(tables, 1, -1) == 1);
    check("last stage [2,top]", count_config(tables, 2, -1) == 1);
    check("stage1 alone [0,0]", count_config(tables, 0, 0) == 1);
    check("stage2 alone [1,1]", count_config(tables, 1, 1) == 1);

    check("use_table enabled", cfg.vehicle.aero.use_table);
    check("first_stage_table mirrors stage1-alone",
          cfg.vehicle.aero.first_stage_table.max_attached_stage == 0 &&
              cfg.vehicle.aero.first_stage_table.activate_at_min_attached_stage == 0);

    // The full stack must reference the longest body; single stages are shorter.
    const post2::vehicle::AeroStageTable* full = nullptr;
    for (const auto& t : tables) {
        if (t.activate_at_min_attached_stage == 0 && t.max_attached_stage < 0) {
            full = &t;
        }
    }
    check("full stack found", full != nullptr);
    if (full) {
        check("full stack uses fairing diameter", full->ref_diameter_m == 5.0);
    }
}

void test_single_stage_only_full()
{
    post2::core::CaseConfig cfg;
    cfg.vehicle.aero.body_length_m = 30.0;
    cfg.vehicle.stages = {make_stage("only", 5000.0, 100000.0)};
    post2::core::KspVehicleSiteImport imported;
    std::string error;
    const bool ok = post2::core::generate_case_aero_tables(&cfg, imported, "", &error);
    check("single-stage generation succeeds", ok);
    check("single stage -> one full table only",
          cfg.vehicle.aero.stage_tables.size() == 1 &&
              cfg.vehicle.aero.stage_tables.front().max_attached_stage == -1);
}

void test_max_attached_stage_roundtrip()
{
    post2::core::CaseConfig cfg;
    cfg.vehicle.aero.use_table = true;
    cfg.vehicle.aero.stage_tables = {
        post2::vehicle::AeroStageTable{0, "full.csv", 20.0, 5.0, 60.0, 10.0, 3.6, -1},
        post2::vehicle::AeroStageTable{0, "s1_only.csv", 10.0, 3.6, 40.0, 7.0, 3.6, 0},
        post2::vehicle::AeroStageTable{1, "upper.csv", 20.0, 5.0, 20.0, 10.0, 3.6, -1},
    };
    cfg.phases.push_back(post2::core::PhaseConfig{});

    const std::string json = post2::core::case_config_to_json(cfg);
    post2::core::CaseConfig loaded;
    std::string error;
    const bool ok = post2::core::case_config_from_json(json, &loaded, &error);
    check("case JSON round-trip parses", ok);
    if (!ok) {
        std::cerr << "  error: " << error << '\n';
        return;
    }
    const auto& t = loaded.vehicle.aero.stage_tables;
    check("round-trip preserves three tables", t.size() == 3);
    if (t.size() == 3) {
        check("full open-top preserved", t[0].max_attached_stage == -1);
        check("single-stage bound preserved", t[1].max_attached_stage == 0);
        check("upper open-top preserved", t[2].max_attached_stage == -1);
    }
}

}  // namespace

int main()
{
    test_three_stage_ladder();
    test_single_stage_only_full();
    test_max_attached_stage_roundtrip();
    if (g_failures == 0) {
        std::cout << "aero stage table test passed\n";
        return 0;
    }
    std::cerr << g_failures << " check(s) failed\n";
    return 1;
}
