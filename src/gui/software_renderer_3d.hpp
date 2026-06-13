#pragma once

#include <windows.h>

#include "post2/core/state_log.hpp"
#include "post2/core/types.hpp"

namespace post2::gui {

struct ProjectedPoint3D {
    POINT screen = {};
    double depth_m = 0.0;
    bool valid = false;
    bool near_side = false;
};

class Camera3D {
public:
    void reset(double scene_radius_m);
    void set_scene_radius(double scene_radius_m);
    void set_viewport(RECT viewport);
    void rotate_pixels(int dx, int dy);
    void zoom_wheel(int wheel_delta);

    double scene_radius_m() const;
    RECT viewport() const;
    bool has_viewport() const;

    ProjectedPoint3D project(const post2::core::Vec3& position_m) const;
    post2::core::Vec3 eye_direction() const;
    post2::core::Vec3 eye_position() const;
    post2::core::Vec3 ray_direction_for_pixel(double screen_x, double screen_y) const;

private:
    double min_distance_m() const;
    double max_distance_m() const;

    double yaw_deg_ = 35.0;
    double pitch_deg_ = 22.0;
    double distance_m_ = 1.0;
    double scene_radius_m_ = 1.0;
    double fov_deg_ = 45.0;
    RECT viewport_ = {};
};

double compute_scene_radius_m(const post2::core::StateLog& state_log);

class SoftwareRenderer3D {
public:
    void draw(HDC hdc, const Camera3D& camera, const post2::core::StateLog& state_log) const;

private:
    void draw_trajectory_segments(
        HDC hdc,
        const Camera3D& camera,
        const post2::core::StateLog& state_log,
        bool near_side,
        COLORREF color,
        int width) const;
    void draw_earth(HDC hdc, const Camera3D& camera) const;
    void draw_earth_axis(HDC hdc, const Camera3D& camera) const;
    void draw_marker(
        HDC hdc,
        const Camera3D& camera,
        const post2::core::Vec3& position_m,
        COLORREF color,
        bool near_side) const;
};

} // namespace post2::gui
