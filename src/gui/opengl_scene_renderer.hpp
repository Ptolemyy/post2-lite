#pragma once

#include <array>
#include <cstdint>
#include <vector>

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
    void reset(double scene_radius_m, const post2::core::Vec3& target_m);
    void set_scene_radius(double scene_radius_m);
    void set_target(const post2::core::Vec3& target_m);
    void set_eye_direction(const post2::core::Vec3& direction_from_target);
    void set_viewport(RECT viewport);
    void rotate_pixels(int dx, int dy);
    void zoom_wheel(int wheel_delta);

    double scene_radius_m() const;
    post2::core::Vec3 target() const;
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
    post2::core::Vec3 target_m_ = {0.0, 0.0, 0.0};
    RECT viewport_ = {};
};

double compute_scene_radius_m(const post2::core::StateLog& state_log);
double compute_scene_radius_m(
    const post2::core::StateLog& state_log,
    const std::vector<post2::core::PredictedTrajectoryPath>& predicted_paths);

struct SceneRenderOptions {
    bool earth_fixed_view = false;
    bool overview_mode = false;
    std::vector<int> primary_phase_indices;
};

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
        const std::vector<post2::core::PredictedTrajectoryPath>& predicted_paths,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        const SceneRenderOptions& options);

private:
    bool make_current() const;
    bool load_earth_texture();
    void build_earth_mesh();
    void draw_scene(
        const Camera3D& camera,
        const post2::core::StateLog& state_log,
        const std::vector<post2::core::PredictedTrajectoryPath>& predicted_paths,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        const SceneRenderOptions& options);
    void draw_earth(double rotation_rad, bool overview_mode) const;
    void draw_axis() const;
    void draw_trajectory(
        const post2::core::StateLog& state_log,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        const SceneRenderOptions& options) const;
    void draw_predicted_path(
        const post2::core::PredictedTrajectoryPath& predicted_path,
        double earth_rotation_at_epoch_rad,
        double earth_rotation_rad_per_s,
        const SceneRenderOptions& options,
        bool draw_apsis_markers) const;
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
    unsigned int overview_earth_texture_ = 0;
    unsigned int earth_display_list_ = 0;
    unsigned int overview_earth_display_list_ = 0;
    unsigned int coastline_display_list_ = 0;
    int earth_texture_width_ = 0;
    int earth_texture_height_ = 0;
    std::vector<std::uint8_t> earth_texture_rgb_;
    int width_ = 1;
    int height_ = 1;
    bool texture_loaded_ = false;
};

} // namespace post2::gui
