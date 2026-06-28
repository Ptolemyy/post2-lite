#include "post2/core/control_models.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {
int g_fail = 0;
void check(const char* what, bool ok)
{
    if (!ok) {
        std::cerr << "FAIL: " << what << '\n';
        ++g_fail;
    }
}
}  // namespace

int main()
{
    using post2::core::select_ignited_engines;
    const double t1 = 1.0e6;  // per-engine thrust
    const std::vector<int> options = {1, 3, 9};
    int count = -1;
    double throttle = 0.0;

    // Within the 1-engine band -> 1 engine at a partial throttle.
    select_ignited_engines(0.7e6, t1, options, 0.4, &count, &throttle);
    check("0.7 -> 1 engine", count == 1);
    check("0.7 -> throttle 0.7", std::fabs(throttle - 0.7) < 1e-6);

    // Needs 3 engines (1.5 MN exceeds one engine at 100%).
    select_ignited_engines(1.5e6, t1, options, 0.4, &count, &throttle);
    check("1.5 -> 3 engines", count == 3);
    check("1.5 -> throttle 0.5", std::fabs(throttle - 0.5) < 1e-6);

    // Gap between 1-engine max (1.0) and 3-engine min (1.2): snap to nearest,
    // tie broken toward fewer engines -> 1 engine at 100%.
    select_ignited_engines(1.1e6, t1, options, 0.4, &count, &throttle);
    check("1.1 gap -> 1 engine @1.0", count == 1 && std::fabs(throttle - 1.0) < 1e-6);

    // Below the 1-engine min (0.4) -> 1 engine at min throttle.
    select_ignited_engines(0.2e6, t1, options, 0.4, &count, &throttle);
    check("0.2 -> 1 engine @0.4", count == 1 && std::fabs(throttle - 0.4) < 1e-6);

    // Above 3 engines -> 9 engines.
    select_ignited_engines(5.0e6, t1, options, 0.4, &count, &throttle);
    check("5.0 -> 9 engines", count == 9);

    // Empty options -> no discrete restriction (count 0 = caller falls back).
    select_ignited_engines(1.0e6, t1, {}, 0.4, &count, &throttle);
    check("empty options -> count 0", count == 0);

    // The entry_burn throttle model registers and is inert with no case config.
    post2::core::ThrottleModelConfig cfg;
    cfg.type = "entry_burn";
    auto model = post2::core::make_throttle_model(cfg);
    check("entry_burn model built", model != nullptr);
    if (model) {
        check("inert throttle without case", model->throttle(0.0, {}, {}) == 0.0);
        check("inert engine count -1", model->ignited_engine_count(0.0, {}, {}) == -1);
    }

    if (g_fail != 0) {
        std::cerr << g_fail << " engine-selection checks failed\n";
        return 1;
    }
    std::cout << "engine selection test passed\n";
    return 0;
}
