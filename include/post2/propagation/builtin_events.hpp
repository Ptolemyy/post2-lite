#pragma once

#include "post2/integrators/integrator.hpp"

namespace post2::propagation {

// Geodetic altitude is rotation-invariant about the z-axis, so events use
// ECI position directly without an ECI->ECEF transform.

// Triggers when WGS84 geodetic altitude crosses zero (ground impact).
post2::integrators::EventFunction altitude_zero_event(bool terminating = true);

// Triggers when geodetic altitude crosses a given threshold (m).
// Useful for atmosphere entry/exit and orbit-altitude checkpoints.
post2::integrators::EventFunction altitude_threshold_event(
    double altitude_m, bool terminating = false);

// Triggers when r . v changes sign (radial velocity == 0). periapsis fires
// at the descent->ascent transition (g goes from negative to positive),
// apoapsis at the ascent->descent transition (g goes from positive to
// negative). Either event detects both transitions; consumers can filter
// by inspecting r.v signs.
post2::integrators::EventFunction periapsis_event(bool terminating = false);
post2::integrators::EventFunction apoapsis_event(bool terminating = false);

} // namespace post2::propagation
