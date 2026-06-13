#pragma once

#include <array>

#include <windows.h>

#include "post2/core/state_log.hpp"
#include "post2/core/types.hpp"

namespace post2::gui {

struct Matrix4D {
    std::array<double, 16> values = {};

    const double* data() const
    {
        return values.data();
    }
};

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
    Matrix4D view_matrix() const;
    Matrix4D projection_matrix() const;

private:
    double min_distance_m() const;
    double max_distance_m() const;
    double near_clip_m() const;
    double far_clip_m() const;

    double yaw_deg_ = 35.0;
    double pitch_deg_ = 22.0;
    double distance_m_ = 1.0;
    double scene_radius_m_ = 1.0;
    double fov_deg_ = 45.0;
    RECT viewport_ = {};
};

double compute_scene_radius_m(const post2::core::StateLog& state_log);

class OpenGLSceneRenderer {
public:
    OpenGLSceneRenderer() = default;
    ~OpenGLSceneRenderer();

    OpenGLSceneRenderer(const OpenGLSceneRenderer&) = delete;
    OpenGLSceneRenderer& operator=(const OpenGLSceneRenderer&) = delete;

    bool initialize(HWND hwnd);
    void destroy();
    void resize(int width, int height);
    void render(
        const Camera3D& camera,
        const post2::core::StateLog& state_log,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        bool earth_fixed_view);

private:
    bool make_current() const;
    bool load_earth_texture();
    void build_earth_mesh();
    void draw_scene(
        const Camera3D& camera,
        const post2::core::StateLog& state_log,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        bool earth_fixed_view);
    void draw_earth() const;
    void draw_axis() const;
    void draw_trajectory(
        const post2::core::StateLog& state_log,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        bool earth_fixed_view) const;
    void draw_markers(
        const post2::core::StateLog& state_log,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        bool earth_fixed_view) const;
    void draw_border() const;

    HWND hwnd_ = nullptr;
    HDC hdc_ = nullptr;
    HGLRC glrc_ = nullptr;
    unsigned int earth_texture_ = 0;
    unsigned int earth_display_list_ = 0;
    int width_ = 1;
    int height_ = 1;
    bool texture_loaded_ = false;
};

} // namespace post2::gui
