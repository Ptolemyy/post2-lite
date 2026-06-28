#include "opengl_scene_renderer.hpp"

#include "resource.h"

#include "post2/core/frames.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gl/GL.h>

namespace post2::gui {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr int kEarthSlices = 192;
constexpr int kEarthStacks = 96;
constexpr int kCoastlineLonSteps = 1440;
constexpr int kCoastlineLatSteps = 720;
constexpr std::size_t kMaxDrawnSegments = 3000;
constexpr double kTrajectorySurfaceLiftM = 1500.0;
constexpr double kCoastlineSurfaceLiftM = 3500.0;

using post2::core::Vec3;

Vec3 cross(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

Vec3 normalized_or(const Vec3& value, const Vec3& fallback)
{
    const double length = post2::vehicle::norm(value);
    if (length <= 1.0e-12) {
        return fallback;
    }
    return value / length;
}

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

int rect_width(const RECT& rect)
{
    return std::max(0L, rect.right - rect.left);
}

int rect_height(const RECT& rect)
{
    return std::max(0L, rect.bottom - rect.top);
}

bool point_visible_from_camera(const Vec3& eye_m, const Vec3& position_m)
{
    const Vec3 to_point_m = position_m - eye_m;
    const double a = post2::vehicle::dot(to_point_m, to_point_m);
    if (a <= 1.0e-12) {
        return true;
    }

    const double earth_radius_m = post2::core::kEarthRadiusM;
    const double b = 2.0 * post2::vehicle::dot(eye_m, to_point_m);
    const double c = post2::vehicle::dot(eye_m, eye_m) - earth_radius_m * earth_radius_m;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant <= 0.0) {
        return true;
    }

    const double first_hit_t = (-b - std::sqrt(discriminant)) / (2.0 * a);
    return first_hit_t <= 1.0e-6 || first_hit_t >= 1.0 - 1.0e-6;
}

std::size_t trajectory_draw_stride(std::size_t point_count)
{
    if (point_count <= kMaxDrawnSegments + 1) {
        return 1;
    }
    return std::max<std::size_t>(1, (point_count - 1 + kMaxDrawnSegments - 1) / kMaxDrawnSegments);
}

Vec3 lifted_for_display(const Vec3& position_m)
{
    const double length = post2::vehicle::norm(position_m);
    if (length <= 1.0e-9) {
        return position_m;
    }

    // Keeps launch points exactly on the textured sphere from flickering with the depth buffer.
    return position_m * ((length + kTrajectorySurfaceLiftM) / length);
}

Vec3 lifted_for_display(const Vec3& position_m, double lift_m)
{
    const double length = post2::vehicle::norm(position_m);
    if (length <= 1.0e-9) {
        return position_m;
    }
    return position_m * ((length + lift_m) / length);
}

Vec3 earth_fixed_position_for_display(
    const post2::core::LaunchVehicleStateLogEntry& entry,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s)
{
    const double theta_rad = post2::core::frames::earth_rotation_angle_rad(
        earth_rotation_at_epoch_rad,
        earth_rotation_rad_per_s,
        entry.time_s);
    return post2::core::frames::eci_to_ecef_position(entry.state.position_m, theta_rad);
}

Vec3 position_for_display(
    const post2::core::LaunchVehicleStateLogEntry& entry,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s,
    bool earth_fixed_view)
{
    if (!earth_fixed_view) {
        return entry.state.position_m;
    }
    return earth_fixed_position_for_display(
        entry,
        earth_rotation_at_epoch_rad,
        earth_rotation_rad_per_s);
}

double takeoff_time_s_for_display(const post2::core::StateLog& state_log)
{
    if (state_log.empty()) {
        return 0.0;
    }
    for (const auto& entry : state_log.entries()) {
        if (!entry.hold_down_clamp_active) {
            return entry.time_s;
        }
    }
    return state_log.front().time_s;
}

bool phase_is_primary(const SceneRenderOptions& options, int phase_index)
{
    return options.primary_phase_indices.empty() ||
        std::find(
            options.primary_phase_indices.begin(),
            options.primary_phase_indices.end(),
            phase_index) != options.primary_phase_indices.end();
}

struct EarthTexture {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgb;

    bool valid() const
    {
        return width > 0 && height > 0 && rgb.size() == static_cast<std::size_t>(width) * height * 3U;
    }
};

struct RgbSample {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
};

std::uint16_t read_u16(const std::uint8_t* value)
{
    return static_cast<std::uint16_t>(value[0] | (value[1] << 8));
}

std::uint32_t read_u32(const std::uint8_t* value)
{
    return static_cast<std::uint32_t>(
        value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24));
}

std::int32_t read_i32(const std::uint8_t* value)
{
    return static_cast<std::int32_t>(read_u32(value));
}

EarthTexture decode_bmp_texture(const void* raw_data, std::size_t raw_size)
{
    EarthTexture texture;
    if (!raw_data || raw_size < 54) {
        return texture;
    }

    const auto* data = static_cast<const std::uint8_t*>(raw_data);
    if (data[0] != 'B' || data[1] != 'M') {
        return texture;
    }

    const std::uint32_t pixel_offset = read_u32(data + 10);
    const std::uint32_t dib_header_size = read_u32(data + 14);
    if (dib_header_size < 40 || pixel_offset >= raw_size) {
        return texture;
    }

    const std::int32_t width = read_i32(data + 18);
    const std::int32_t signed_height = read_i32(data + 22);
    const std::uint16_t planes = read_u16(data + 26);
    const std::uint16_t bits_per_pixel = read_u16(data + 28);
    const std::uint32_t compression = read_u32(data + 30);
    if (width <= 0 || signed_height == 0 || planes != 1 || (bits_per_pixel != 24 && bits_per_pixel != 32) ||
        compression != BI_RGB) {
        return texture;
    }

    const int height = signed_height < 0 ? -signed_height : signed_height;
    const bool top_down = signed_height < 0;
    const std::size_t bytes_per_pixel = bits_per_pixel / 8;
    const std::size_t row_stride =
        ((static_cast<std::size_t>(width) * bytes_per_pixel + 3U) / 4U) * 4U;
    const std::size_t pixel_bytes = row_stride * static_cast<std::size_t>(height);
    if (pixel_offset > raw_size || pixel_bytes > raw_size - pixel_offset) {
        return texture;
    }

    texture.width = width;
    texture.height = height;
    texture.rgb.resize(static_cast<std::size_t>(width) * height * 3U);

    for (int y = 0; y < height; ++y) {
        const int source_y = top_down ? y : (height - 1 - y);
        const auto* row = data + pixel_offset + row_stride * static_cast<std::size_t>(source_y);
        for (int x = 0; x < width; ++x) {
            const auto* pixel = row + static_cast<std::size_t>(x) * bytes_per_pixel;
            const std::size_t target = (static_cast<std::size_t>(y) * width + x) * 3U;
            texture.rgb[target + 0] = pixel[2];
            texture.rgb[target + 1] = pixel[1];
            texture.rgb[target + 2] = pixel[0];
        }
    }

    return texture;
}

RgbSample texture_rgb_at(const std::vector<std::uint8_t>& rgb, int width, int height, double u, double v)
{
    if (width <= 0 || height <= 0 ||
        rgb.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U) {
        return {};
    }

    u -= std::floor(u);
    v = clamp(v, 0.0, 1.0);

    const double fx = u * static_cast<double>(width);
    const double fy = v * static_cast<double>(height - 1);
    const int x0 = ((static_cast<int>(std::floor(fx)) % width) + width) % width;
    const int x1 = (x0 + 1) % width;
    const int y0 = std::clamp(static_cast<int>(std::floor(fy)), 0, height - 1);
    const int y1 = std::clamp(y0 + 1, 0, height - 1);
    const double tx = fx - std::floor(fx);
    const double ty = fy - std::floor(fy);

    auto sample = [&](int x, int y) {
        const std::size_t index =
            (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(x)) * 3U;
        return RgbSample{
            static_cast<double>(rgb[index + 0]),
            static_cast<double>(rgb[index + 1]),
            static_cast<double>(rgb[index + 2]),
        };
    };
    const RgbSample c00 = sample(x0, y0);
    const RgbSample c10 = sample(x1, y0);
    const RgbSample c01 = sample(x0, y1);
    const RgbSample c11 = sample(x1, y1);
    const auto mix = [](double a, double b, double t) {
        return a + (b - a) * t;
    };
    return {
        mix(mix(c00.r, c10.r, tx), mix(c01.r, c11.r, tx), ty),
        mix(mix(c00.g, c10.g, tx), mix(c01.g, c11.g, tx), ty),
        mix(mix(c00.b, c10.b, tx), mix(c01.b, c11.b, tx), ty),
    };
}

double texture_water_score_at(const std::vector<std::uint8_t>& rgb, int width, int height, double u, double v)
{
    const RgbSample c = texture_rgb_at(rgb, width, height, u, v);
    const double max_rg = std::max(c.r, c.g);
    const double min_rgb = std::min({c.r, c.g, c.b});
    const double blue_margin = c.b - max_rg;
    double score = 0.5 + blue_margin / 55.0;
    score += (c.b - min_rgb) / 420.0;
    if (c.b < 45.0) {
        score -= (45.0 - c.b) / 90.0;
    }
    if (c.r > 185.0 && c.g > 185.0 && c.b > 185.0) {
        score -= 0.35;  // clouds/ice should not punch ocean holes.
    }
    return clamp(score, 0.0, 1.0);
}

bool texture_land_at(const std::vector<std::uint8_t>& rgb, int width, int height, double u, double v)
{
    return texture_water_score_at(rgb, width, height, u, v) < 0.5;
}

Vec3 ellipsoid_surface_point(double lat_rad, double lon_rad, double lift_m = 0.0)
{
    const double a_m = post2::core::frames::Wgs84::a_m + lift_m;
    const double b_m = post2::core::frames::Wgs84::b_m + lift_m;
    const double cos_lat = std::cos(lat_rad);
    return {
        a_m * cos_lat * std::cos(lon_rad),
        a_m * cos_lat * std::sin(lon_rad),
        b_m * std::sin(lat_rad),
    };
}

EarthTexture load_earth_texture_from_resource()
{
    const HMODULE module = GetModuleHandleW(nullptr);
    if (!module) {
        return {};
    }

    const HRSRC resource =
        FindResourceW(module, MAKEINTRESOURCEW(IDR_EARTH_TEXTURE_BMP), MAKEINTRESOURCEW(10));
    if (!resource) {
        return {};
    }

    const DWORD size = SizeofResource(module, resource);
    const HGLOBAL handle = LoadResource(module, resource);
    if (!handle || size == 0) {
        return {};
    }

    const void* data = LockResource(handle);
    return decode_bmp_texture(data, static_cast<std::size_t>(size));
}

void emit_earth_sphere_mesh()
{
    // WGS84 oblate ellipsoid: x,y use a (equatorial radius), z uses b (polar).
    // The texture's lat/lon UV mapping is unchanged - parametric lat is fed to
    // the same cos/sin and the geometry is then flattened along z.
    const double a_m = post2::core::frames::Wgs84::a_m;
    const double b_m = post2::core::frames::Wgs84::b_m;
    for (int stack = 0; stack < kEarthStacks; ++stack) {
        const double stack0 = static_cast<double>(stack) / static_cast<double>(kEarthStacks);
        const double stack1 = static_cast<double>(stack + 1) / static_cast<double>(kEarthStacks);
        const double lat0 = -0.5 * kPi + stack0 * kPi;
        const double lat1 = -0.5 * kPi + stack1 * kPi;

        glBegin(GL_TRIANGLE_STRIP);
        for (int slice = 0; slice <= kEarthSlices; ++slice) {
            const double unit_slice = static_cast<double>(slice) / static_cast<double>(kEarthSlices);
            const double lon = -kPi + unit_slice * (2.0 * kPi);
            const double cos_lon = std::cos(lon);
            const double sin_lon = std::sin(lon);

            auto emit_vertex = [&](double lat) {
                const double cos_lat = std::cos(lat);
                const double sin_lat = std::sin(lat);
                const double u = lon / (2.0 * kPi) + 0.5;
                const double v = 0.5 - lat / kPi;
                // Outward normal of the ellipsoid (x/a^2, y/a^2, z/b^2),
                // simplified to (cos*cos/a, cos*sin/a, sin/b) before
                // normalization. Renormalize so OpenGL gets a unit vector.
                const double nx = cos_lat * cos_lon / a_m;
                const double ny = cos_lat * sin_lon / a_m;
                const double nz = sin_lat / b_m;
                const double n_inv = 1.0 / std::sqrt(nx * nx + ny * ny + nz * nz);
                glTexCoord2d(u, v);
                glNormal3d(nx * n_inv, ny * n_inv, nz * n_inv);
                glVertex3d(a_m * cos_lat * cos_lon, a_m * cos_lat * sin_lon, b_m * sin_lat);
            };

            emit_vertex(lat0);
            emit_vertex(lat1);
        }
        glEnd();
    }
}

void emit_overview_earth_mesh(const std::vector<std::uint8_t>& rgb, int width, int height)
{
    for (int stack = 0; stack < kEarthStacks; ++stack) {
        const double stack0 = static_cast<double>(stack) / static_cast<double>(kEarthStacks);
        const double stack1 = static_cast<double>(stack + 1) / static_cast<double>(kEarthStacks);
        const double lat0 = -0.5 * kPi + stack0 * kPi;
        const double lat1 = -0.5 * kPi + stack1 * kPi;

        glBegin(GL_TRIANGLE_STRIP);
        for (int slice = 0; slice <= kEarthSlices; ++slice) {
            const double unit_slice = static_cast<double>(slice) / static_cast<double>(kEarthSlices);
            const double lon = -kPi + unit_slice * (2.0 * kPi);
            const double u = lon / (2.0 * kPi) + 0.5;

            auto emit_vertex = [&](double lat) {
                const double v = 0.5 - lat / kPi;
                glTexCoord2d(u, v);

                const double cos_lat = std::cos(lat);
                const double sin_lat = std::sin(lat);
                const double cos_lon = std::cos(lon);
                const double sin_lon = std::sin(lon);
                const double a_m = post2::core::frames::Wgs84::a_m;
                const double b_m = post2::core::frames::Wgs84::b_m;
                const double nx = cos_lat * cos_lon / a_m;
                const double ny = cos_lat * sin_lon / a_m;
                const double nz = sin_lat / b_m;
                const double n_inv = 1.0 / std::sqrt(nx * nx + ny * ny + nz * nz);
                glNormal3d(nx * n_inv, ny * n_inv, nz * n_inv);
                glVertex3d(a_m * cos_lat * cos_lon, a_m * cos_lat * sin_lon, b_m * sin_lat);
            };

            emit_vertex(lat0);
            emit_vertex(lat1);
        }
        glEnd();
    }
}

void emit_coastline_segments(const std::vector<std::uint8_t>& rgb, int width, int height)
{
    if (width <= 0 || height <= 0 || rgb.empty()) {
        return;
    }

    const int lon_nodes = kCoastlineLonSteps + 1;
    const int lat_nodes = kCoastlineLatSteps + 1;
    std::vector<double> water_score(
        static_cast<std::size_t>(lon_nodes) * static_cast<std::size_t>(lat_nodes));
    for (int y = 0; y < lat_nodes; ++y) {
        const double lat = -0.5 * kPi + static_cast<double>(y) * kPi /
            static_cast<double>(kCoastlineLatSteps);
        const double v = 0.5 - lat / kPi;
        for (int x = 0; x < lon_nodes; ++x) {
            const double lon = -kPi + static_cast<double>(x) * (2.0 * kPi) /
                static_cast<double>(kCoastlineLonSteps);
            const double u = lon / (2.0 * kPi) + 0.5;
            water_score[static_cast<std::size_t>(y) * lon_nodes + x] =
                texture_water_score_at(rgb, width, height, u, v);
        }
    }

    auto score = [&](int x, int y) {
        x = std::clamp(x, 0, lon_nodes - 1);
        y = std::clamp(y, 0, lat_nodes - 1);
        return water_score[static_cast<std::size_t>(y) * lon_nodes + x];
    };
    auto edge_t = [](double a, double b) {
        const double denom = b - a;
        if (std::abs(denom) <= 1.0e-9) {
            return 0.5;
        }
        return clamp((0.5 - a) / denom, 0.0, 1.0);
    };
    auto emit_segment = [&](double lat_a, double lon_a, double lat_b, double lon_b) {
        const Vec3 a = ellipsoid_surface_point(lat_a, lon_a, kCoastlineSurfaceLiftM);
        const Vec3 b = ellipsoid_surface_point(lat_b, lon_b, kCoastlineSurfaceLiftM);
        glVertex3d(a.x, a.y, a.z);
        glVertex3d(b.x, b.y, b.z);
    };

    glBegin(GL_LINES);
    for (int y = 0; y < kCoastlineLatSteps; ++y) {
        const double lat0 = -0.5 * kPi + static_cast<double>(y) * kPi /
            static_cast<double>(kCoastlineLatSteps);
        const double lat1 = -0.5 * kPi + static_cast<double>(y + 1) * kPi /
            static_cast<double>(kCoastlineLatSteps);
        for (int x = 0; x < kCoastlineLonSteps; ++x) {
            const double lon0 = -kPi + static_cast<double>(x) * (2.0 * kPi) /
                static_cast<double>(kCoastlineLonSteps);
            const double lon1 = -kPi + static_cast<double>(x + 1) * (2.0 * kPi) /
                static_cast<double>(kCoastlineLonSteps);
            const double s00 = score(x, y);
            const double s10 = score(x + 1, y);
            const double s11 = score(x + 1, y + 1);
            const double s01 = score(x, y + 1);
            const bool b00 = s00 >= 0.5;
            const bool b10 = s10 >= 0.5;
            const bool b11 = s11 >= 0.5;
            const bool b01 = s01 >= 0.5;
            const int crossings =
                (b00 != b10 ? 1 : 0) +
                (b10 != b11 ? 1 : 0) +
                (b11 != b01 ? 1 : 0) +
                (b01 != b00 ? 1 : 0);
            if (crossings == 0) {
                continue;
            }

            struct Crossing {
                int edge = 0;
                double lat = 0.0;
                double lon = 0.0;
            };
            Crossing c[4];
            int count = 0;
            if (b00 != b10) {
                const double t = edge_t(s00, s10);
                c[count++] = Crossing{0, lat0, lon0 + (lon1 - lon0) * t};
            }
            if (b10 != b11) {
                const double t = edge_t(s10, s11);
                c[count++] = Crossing{1, lat0 + (lat1 - lat0) * t, lon1};
            }
            if (b11 != b01) {
                const double t = edge_t(s11, s01);
                c[count++] = Crossing{2, lat1, lon1 + (lon0 - lon1) * t};
            }
            if (b01 != b00) {
                const double t = edge_t(s01, s00);
                c[count++] = Crossing{3, lat1 + (lat0 - lat1) * t, lon0};
            }

            if (count == 2) {
                emit_segment(c[0].lat, c[0].lon, c[1].lat, c[1].lon);
            } else if (count == 4) {
                const double center = 0.25 * (s00 + s10 + s11 + s01);
                if (center >= 0.5) {
                    emit_segment(c[0].lat, c[0].lon, c[1].lat, c[1].lon);
                    emit_segment(c[2].lat, c[2].lon, c[3].lat, c[3].lon);
                } else {
                    emit_segment(c[0].lat, c[0].lon, c[3].lat, c[3].lon);
                    emit_segment(c[1].lat, c[1].lon, c[2].lat, c[2].lon);
                }
            }
        }
    }
    glEnd();
}

} // namespace

void Camera3D::reset(double scene_radius_m)
{
    reset(scene_radius_m, target_m_);
}

void Camera3D::reset(double scene_radius_m, const Vec3& target_m)
{
    yaw_deg_ = 35.0;
    pitch_deg_ = 22.0;
    target_m_ = target_m;
    set_scene_radius(scene_radius_m);
    distance_m_ = 3.2 * scene_radius_m_;
}

void Camera3D::set_scene_radius(double scene_radius_m)
{
    scene_radius_m_ = std::max(1.0, scene_radius_m);
    if (distance_m_ <= 0.0) {
        distance_m_ = 3.2 * scene_radius_m_;
    }
    distance_m_ = clamp(distance_m_, min_distance_m(), max_distance_m());
}

void Camera3D::set_target(const Vec3& target_m)
{
    target_m_ = target_m;
}

void Camera3D::set_eye_direction(const Vec3& direction_from_target)
{
    const Vec3 direction = normalized_or(direction_from_target, {1.0, 0.0, 0.0});
    yaw_deg_ = std::atan2(direction.y, direction.x) / kDegToRad;
    pitch_deg_ = std::asin(clamp(direction.z, -1.0, 1.0)) / kDegToRad;
    pitch_deg_ = clamp(pitch_deg_, -89.0, 89.0);
}

void Camera3D::set_viewport(RECT viewport)
{
    viewport_ = viewport;
}

void Camera3D::rotate_pixels(int dx, int dy)
{
    yaw_deg_ -= static_cast<double>(dx) * 0.35;
    pitch_deg_ += static_cast<double>(dy) * 0.35;
    pitch_deg_ = clamp(pitch_deg_, -89.0, 89.0);
}

void Camera3D::zoom_wheel(int wheel_delta)
{
    const double wheel_steps = static_cast<double>(wheel_delta) / static_cast<double>(WHEEL_DELTA);
    distance_m_ *= std::pow(0.88, wheel_steps);
    distance_m_ = clamp(distance_m_, min_distance_m(), max_distance_m());
}

double Camera3D::scene_radius_m() const
{
    return scene_radius_m_;
}

Vec3 Camera3D::target() const
{
    return target_m_;
}

RECT Camera3D::viewport() const
{
    return viewport_;
}

bool Camera3D::has_viewport() const
{
    return rect_width(viewport_) > 20 && rect_height(viewport_) > 20;
}

ProjectedPoint3D Camera3D::project(const Vec3& position_m) const
{
    if (!has_viewport()) {
        return {};
    }

    const Vec3 eye = eye_position();
    const Vec3 forward = normalized_or(target_m_ - eye, {-1.0, 0.0, 0.0});
    const Vec3 right = normalized_or(cross(forward, {0.0, 0.0, 1.0}), {0.0, 1.0, 0.0});
    const Vec3 up = normalized_or(cross(right, forward), {0.0, 0.0, 1.0});
    const Vec3 from_eye = position_m - eye;

    const double depth_m = post2::vehicle::dot(from_eye, forward);
    if (depth_m <= 1.0) {
        return {};
    }

    const double camera_x_m = post2::vehicle::dot(from_eye, right);
    const double camera_y_m = post2::vehicle::dot(from_eye, up);
    const double min_extent_px = static_cast<double>(std::min(rect_width(viewport_), rect_height(viewport_)));
    const double focal_px = (min_extent_px * 0.5) / std::tan((fov_deg_ * kDegToRad) * 0.5);
    const double center_x = static_cast<double>(viewport_.left + viewport_.right) * 0.5;
    const double center_y = static_cast<double>(viewport_.top + viewport_.bottom) * 0.5;

    return {
        {
            static_cast<LONG>(std::lround(center_x + camera_x_m * focal_px / depth_m)),
            static_cast<LONG>(std::lround(center_y - camera_y_m * focal_px / depth_m)),
        },
        depth_m,
        true,
        point_visible_from_camera(eye, position_m),
    };
}

Vec3 Camera3D::eye_direction() const
{
    return normalized_or(eye_position() - target_m_, {1.0, 0.0, 0.0});
}

Vec3 Camera3D::ray_direction_for_pixel(double screen_x, double screen_y) const
{
    const Vec3 eye = eye_position();
    const Vec3 forward = normalized_or(target_m_ - eye, {-1.0, 0.0, 0.0});
    const Vec3 right = normalized_or(cross(forward, {0.0, 0.0, 1.0}), {0.0, 1.0, 0.0});
    const Vec3 up = normalized_or(cross(right, forward), {0.0, 0.0, 1.0});
    const double min_extent_px = static_cast<double>(std::min(rect_width(viewport_), rect_height(viewport_)));
    const double focal_px = (min_extent_px * 0.5) / std::tan((fov_deg_ * kDegToRad) * 0.5);
    const double center_x = static_cast<double>(viewport_.left + viewport_.right) * 0.5;
    const double center_y = static_cast<double>(viewport_.top + viewport_.bottom) * 0.5;
    const Vec3 direction =
        forward +
        right * ((screen_x - center_x) / focal_px) +
        up * ((center_y - screen_y) / focal_px);
    return normalized_or(direction, forward);
}

Matrix4D Camera3D::view_matrix() const
{
    const Vec3 eye = eye_position();
    const Vec3 forward = normalized_or(target_m_ - eye, {-1.0, 0.0, 0.0});
    const Vec3 right = normalized_or(cross(forward, {0.0, 0.0, 1.0}), {0.0, 1.0, 0.0});
    const Vec3 up = normalized_or(cross(right, forward), {0.0, 0.0, 1.0});

    Matrix4D matrix;
    matrix.values = {
        right.x,
        up.x,
        -forward.x,
        0.0,
        right.y,
        up.y,
        -forward.y,
        0.0,
        right.z,
        up.z,
        -forward.z,
        0.0,
        -post2::vehicle::dot(right, eye),
        -post2::vehicle::dot(up, eye),
        post2::vehicle::dot(forward, eye),
        1.0,
    };
    return matrix;
}

Matrix4D Camera3D::projection_matrix() const
{
    const int width = std::max(1, rect_width(viewport_));
    const int height = std::max(1, rect_height(viewport_));
    const int min_extent = std::max(1, std::min(width, height));
    const double near_m = near_clip_m();
    const double far_m = far_clip_m();
    const double half_fov = std::tan((fov_deg_ * kDegToRad) * 0.5);
    const double right_m = near_m * half_fov * (static_cast<double>(width) / static_cast<double>(min_extent));
    const double top_m = near_m * half_fov * (static_cast<double>(height) / static_cast<double>(min_extent));

    Matrix4D matrix;
    matrix.values = {
        near_m / right_m,
        0.0,
        0.0,
        0.0,
        0.0,
        near_m / top_m,
        0.0,
        0.0,
        0.0,
        0.0,
        -(far_m + near_m) / (far_m - near_m),
        -1.0,
        0.0,
        0.0,
        -(2.0 * far_m * near_m) / (far_m - near_m),
        0.0,
    };
    return matrix;
}

double Camera3D::min_distance_m() const
{
    return 1.4 * scene_radius_m_;
}

double Camera3D::max_distance_m() const
{
    return 12.0 * scene_radius_m_;
}

double Camera3D::near_clip_m() const
{
    const double world_radius_m = post2::vehicle::norm(target_m_) + 1.30 * scene_radius_m_;
    return std::max(10.0, distance_m_ - world_radius_m);
}

double Camera3D::far_clip_m() const
{
    const double world_radius_m = post2::vehicle::norm(target_m_) + 1.30 * scene_radius_m_;
    return std::max(near_clip_m() + 1000.0, distance_m_ + world_radius_m);
}

Vec3 Camera3D::eye_position() const
{
    const double yaw = yaw_deg_ * kDegToRad;
    const double pitch = pitch_deg_ * kDegToRad;
    const double cp = std::cos(pitch);
    return target_m_ + Vec3{
        distance_m_ * cp * std::cos(yaw),
        distance_m_ * cp * std::sin(yaw),
        distance_m_ * std::sin(pitch),
    };
}

double compute_scene_radius_m(const post2::core::StateLog& state_log)
{
    double radius_m = post2::core::kEarthRadiusM;
    for (const auto& entry : state_log.entries()) {
        radius_m = std::max(radius_m, post2::vehicle::norm(entry.state.position_m));
    }
    return radius_m;
}

double compute_scene_radius_m(
    const post2::core::StateLog& state_log,
    const std::vector<post2::core::PredictedTrajectoryPath>& predicted_paths)
{
    double radius_m = compute_scene_radius_m(state_log);
    for (const auto& predicted : predicted_paths) {
        for (const auto& entry : predicted.state_log.entries()) {
            radius_m = std::max(radius_m, post2::vehicle::norm(entry.state.position_m));
        }
    }
    return radius_m;
}

OpenGLSceneRenderer::~OpenGLSceneRenderer()
{
    destroy();
}

bool OpenGLSceneRenderer::initialize(HWND hwnd)
{
    if (glrc_) {
        return true;
    }

    hwnd_ = hwnd;
    hdc_ = GetDC(hwnd_);
    if (!hdc_) {
        return false;
    }

    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    const int pixel_format = ChoosePixelFormat(hdc_, &pfd);
    if (pixel_format == 0 || !SetPixelFormat(hdc_, pixel_format, &pfd)) {
        destroy();
        return false;
    }

    glrc_ = wglCreateContext(hdc_);
    if (!glrc_ || !make_current()) {
        destroy();
        return false;
    }

    glClearColor(244.0f / 255.0f, 247.0f / 255.0f, 251.0f / 255.0f, 1.0f);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_LIGHTING);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    texture_loaded_ = load_earth_texture();
    build_earth_mesh();

    RECT client = {};
    GetClientRect(hwnd_, &client);
    resize(rect_width(client), rect_height(client));
    return true;
}

void OpenGLSceneRenderer::destroy()
{
    if (glrc_) {
        wglMakeCurrent(hdc_, glrc_);
        if (earth_texture_ != 0) {
            glDeleteTextures(1, &earth_texture_);
            earth_texture_ = 0;
        }
        if (overview_earth_texture_ != 0) {
            glDeleteTextures(1, &overview_earth_texture_);
            overview_earth_texture_ = 0;
        }
        if (earth_display_list_ != 0) {
            glDeleteLists(earth_display_list_, 1);
            earth_display_list_ = 0;
        }
        if (overview_earth_display_list_ != 0) {
            glDeleteLists(overview_earth_display_list_, 1);
            overview_earth_display_list_ = 0;
        }
        if (coastline_display_list_ != 0) {
            glDeleteLists(coastline_display_list_, 1);
            coastline_display_list_ = 0;
        }
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(glrc_);
        glrc_ = nullptr;
    }

    if (hwnd_ && hdc_) {
        ReleaseDC(hwnd_, hdc_);
        hdc_ = nullptr;
    }
    hwnd_ = nullptr;
    texture_loaded_ = false;
    earth_texture_width_ = 0;
    earth_texture_height_ = 0;
    earth_texture_rgb_.clear();
}

void OpenGLSceneRenderer::resize(int width, int height)
{
    width_ = std::max(1, width);
    height_ = std::max(1, height);
    if (make_current()) {
        glViewport(0, 0, width_, height_);
    }
}

void OpenGLSceneRenderer::render(
    const Camera3D& camera,
    const post2::core::StateLog& state_log,
    const std::vector<post2::core::PredictedTrajectoryPath>& predicted_paths,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s,
    const SceneRenderOptions& options)
{
    if (!make_current()) {
        return;
    }

    RECT client = {};
    if (hwnd_) {
        GetClientRect(hwnd_, &client);
        resize(rect_width(client), rect_height(client));
    }

    draw_scene(camera, state_log, predicted_paths, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options);
    SwapBuffers(hdc_);
}

bool OpenGLSceneRenderer::make_current() const
{
    return hdc_ && glrc_ && wglMakeCurrent(hdc_, glrc_) == TRUE;
}

bool OpenGLSceneRenderer::load_earth_texture()
{
    const EarthTexture texture = load_earth_texture_from_resource();
    if (!texture.valid()) {
        return false;
    }

    earth_texture_width_ = texture.width;
    earth_texture_height_ = texture.height;
    earth_texture_rgb_ = texture.rgb;

    glGenTextures(1, &earth_texture_);
    if (earth_texture_ == 0) {
        return false;
    }

    glBindTexture(GL_TEXTURE_2D, earth_texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB,
        texture.width,
        texture.height,
        0,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        texture.rgb.data());
    if (glGetError() != GL_NO_ERROR) {
        return false;
    }

    std::vector<std::uint8_t> overview_rgb(
        static_cast<std::size_t>(texture.width) * static_cast<std::size_t>(texture.height) * 3U);
    for (int y = 0; y < texture.height; ++y) {
        const double v = (static_cast<double>(y) + 0.5) / static_cast<double>(texture.height);
        for (int x = 0; x < texture.width; ++x) {
            const double u = (static_cast<double>(x) + 0.5) / static_cast<double>(texture.width);
            const double water = texture_water_score_at(texture.rgb, texture.width, texture.height, u, v);
            const double t = clamp((water - 0.44) / 0.12, 0.0, 1.0);
            const double land_r = 186.0;
            const double land_g = 194.0;
            const double land_b = 164.0;
            const double ocean_r = 86.0;
            const double ocean_g = 141.0;
            const double ocean_b = 188.0;
            const std::size_t index =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(texture.width) +
                 static_cast<std::size_t>(x)) * 3U;
            overview_rgb[index + 0] = static_cast<std::uint8_t>(
                std::lround(land_r + (ocean_r - land_r) * t));
            overview_rgb[index + 1] = static_cast<std::uint8_t>(
                std::lround(land_g + (ocean_g - land_g) * t));
            overview_rgb[index + 2] = static_cast<std::uint8_t>(
                std::lround(land_b + (ocean_b - land_b) * t));
        }
    }

    glGenTextures(1, &overview_earth_texture_);
    if (overview_earth_texture_ == 0) {
        return false;
    }
    glBindTexture(GL_TEXTURE_2D, overview_earth_texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB,
        texture.width,
        texture.height,
        0,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        overview_rgb.data());
    return glGetError() == GL_NO_ERROR;
}

void OpenGLSceneRenderer::build_earth_mesh()
{
    earth_display_list_ = glGenLists(1);
    if (earth_display_list_ == 0) {
        return;
    }

    glNewList(earth_display_list_, GL_COMPILE);
    emit_earth_sphere_mesh();
    glEndList();

    overview_earth_display_list_ = glGenLists(1);
    if (overview_earth_display_list_ != 0) {
        glNewList(overview_earth_display_list_, GL_COMPILE);
        emit_overview_earth_mesh(earth_texture_rgb_, earth_texture_width_, earth_texture_height_);
        glEndList();
    }

    coastline_display_list_ = glGenLists(1);
    if (coastline_display_list_ != 0) {
        glNewList(coastline_display_list_, GL_COMPILE);
        emit_coastline_segments(earth_texture_rgb_, earth_texture_width_, earth_texture_height_);
        glEndList();
    }
}

void OpenGLSceneRenderer::draw_scene(
    const Camera3D& camera,
    const post2::core::StateLog& state_log,
    const std::vector<post2::core::PredictedTrajectoryPath>& predicted_paths,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s,
    const SceneRenderOptions& options)
{
    glViewport(0, 0, width_, height_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.has_viewport() && !state_log.empty()) {
        const Matrix4D projection = camera.projection_matrix();
        const Matrix4D view = camera.view_matrix();
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixd(projection.data());
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(view.data());

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_TRUE);
        const double earth_rotation_for_display_rad = options.earth_fixed_view
            ? 0.0
            : post2::core::frames::earth_rotation_angle_rad(
                earth_rotation_at_epoch_rad,
                earth_rotation_rad_per_s,
                takeoff_time_s_for_display(state_log));
        draw_earth(earth_rotation_for_display_rad, options.overview_mode);
        if (!options.overview_mode) {
            draw_axis();
        }
        const post2::core::PredictedTrajectoryPath* marker_prediction = nullptr;
        for (const auto& predicted_path : predicted_paths) {
            if (!predicted_path.state_log.empty()) {
                marker_prediction = &predicted_path;
            }
        }
        for (const auto& predicted_path : predicted_paths) {
            draw_predicted_path(
                predicted_path,
                earth_rotation_at_epoch_rad,
                earth_rotation_rad_per_s,
                options,
                !options.overview_mode && &predicted_path == marker_prediction);
        }
        draw_trajectory(state_log, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options);
        draw_markers(state_log, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options.earth_fixed_view);
    }

    draw_border();
}

void OpenGLSceneRenderer::draw_earth(double rotation_rad, bool overview_mode) const
{
    glPushMatrix();
    glRotated(rotation_rad / kDegToRad, 0.0, 0.0, 1.0);

    glDepthMask(GL_TRUE);
    glColor3f(1.0f, 1.0f, 1.0f);
    if (overview_mode && overview_earth_texture_ != 0) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, overview_earth_texture_);
    } else if (!overview_mode && texture_loaded_ && earth_texture_ != 0) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, earth_texture_);
    } else {
        glDisable(GL_TEXTURE_2D);
        glColor3ub(92, 148, 214);
    }

    if (overview_mode && overview_earth_display_list_ != 0) {
        glCallList(overview_earth_display_list_);
    } else if (earth_display_list_ != 0) {
        glCallList(earth_display_list_);
    } else {
        emit_earth_sphere_mesh();
    }
    glDisable(GL_TEXTURE_2D);
    if (overview_mode && coastline_display_list_ != 0) {
        glDepthMask(GL_FALSE);
        glLineWidth(1.0f);
        glColor3ub(31, 41, 55);
        glCallList(coastline_display_list_);
        glDepthMask(GL_TRUE);
    }
    glPopMatrix();
}

void OpenGLSceneRenderer::draw_axis() const
{
    const double radius_m = post2::core::kEarthRadiusM;
    const double inner_m = radius_m * 1.085;
    const double outer_m = radius_m * 1.24;

    glDisable(GL_TEXTURE_2D);
    auto draw_axis_lines = [&] {
        glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, inner_m);
        glVertex3d(0.0, 0.0, outer_m);
        glVertex3d(0.0, 0.0, -inner_m);
        glVertex3d(0.0, 0.0, -outer_m);
        glEnd();
    };

    glLineWidth(3.0f);
    glColor3ub(255, 128, 0);
    draw_axis_lines();
    glLineWidth(1.0f);
}

void OpenGLSceneRenderer::draw_trajectory(
    const post2::core::StateLog& state_log,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s,
    const SceneRenderOptions& options) const
{
    const auto& entries = state_log.entries();
    if (entries.size() < 2) {
        return;
    }

    glDisable(GL_TEXTURE_2D);
    glDepthMask(GL_FALSE);

    const std::size_t stride = trajectory_draw_stride(entries.size());
    const auto draw_pass = [&](bool primary_pass) {
        if (primary_pass) {
            glLineWidth(options.overview_mode ? 3.0f : 2.5f);
            glColor4ub(220, 38, 38, 255);
        } else {
            glLineWidth(1.5f);
            glColor4ub(100, 116, 139, 120);
        }

        bool strip_open = false;
        double prev_time = 0.0;
        const auto close_strip = [&]() {
            if (strip_open) {
                glEnd();
                strip_open = false;
            }
        };
        const auto emit = [&](const auto& entry) {
            const bool primary = phase_is_primary(options, entry.phase_index);
            if (primary != primary_pass) {
                close_strip();
                return;
            }

            // Break the strip at a backward time jump: a recovery branch rewinds
            // the clock to its stage's separation time, so connecting across it
            // would draw a straight line from the orbit point back to MECO.
            if (strip_open && entry.time_s < prev_time) {
                close_strip();
            }
            if (!strip_open) {
                glBegin(GL_LINE_STRIP);
                strip_open = true;
            }

            const Vec3 position = lifted_for_display(position_for_display(
                entry, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options.earth_fixed_view));
            glVertex3d(position.x, position.y, position.z);
            prev_time = entry.time_s;
        };
        for (std::size_t index = 0; index < entries.size(); index += stride) {
            emit(entries[index]);
        }
        if ((entries.size() - 1) % stride != 0) {
            emit(entries.back());
        }
        close_strip();
    };

    if (options.overview_mode && !options.primary_phase_indices.empty()) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        draw_pass(false);
        draw_pass(true);
        glDisable(GL_BLEND);
    } else {
        draw_pass(true);
    }

    glLineWidth(1.0f);
    glDepthMask(GL_TRUE);
}

void OpenGLSceneRenderer::draw_markers(
    const post2::core::StateLog& state_log,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s,
    bool earth_fixed_view) const
{
    if (state_log.empty()) {
        return;
    }

    glDisable(GL_TEXTURE_2D);
    glDepthMask(GL_FALSE);
    glPointSize(7.0f);
    glBegin(GL_POINTS);
    Vec3 position = lifted_for_display(position_for_display(
        state_log.front(), earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, earth_fixed_view));
    glColor3ub(22, 163, 74);
    glVertex3d(position.x, position.y, position.z);
    position = lifted_for_display(position_for_display(
        state_log.back(), earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, earth_fixed_view));
    glColor3ub(17, 24, 39);
    glVertex3d(position.x, position.y, position.z);
    glEnd();
    glPointSize(1.0f);
    glDepthMask(GL_TRUE);
}

void OpenGLSceneRenderer::draw_predicted_path(
    const post2::core::PredictedTrajectoryPath& predicted_path,
    double earth_rotation_at_epoch_rad,
    double earth_rotation_rad_per_s,
    const SceneRenderOptions& options,
    bool draw_apsis_markers) const
{
    const auto& entries = predicted_path.state_log.entries();
    if (entries.size() < 2) {
        return;
    }

    glDisable(GL_TEXTURE_2D);
    glDepthMask(GL_FALSE);
    if (options.overview_mode) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // Dashed integrated prediction path, distinct from the solid red ascent.
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0x00FF);
    glLineWidth(options.overview_mode ? 1.0f : 2.0f);
    if (options.overview_mode) {
        glColor4ub(100, 116, 139, 78);
    } else {
        glColor3ub(
            static_cast<GLubyte>(std::clamp(predicted_path.color.red, 0, 255)),
            static_cast<GLubyte>(std::clamp(predicted_path.color.green, 0, 255)),
            static_cast<GLubyte>(std::clamp(predicted_path.color.blue, 0, 255)));
    }
    glBegin(GL_LINE_STRIP);
    for (const auto& entry : entries) {
        const Vec3 position = lifted_for_display(position_for_display(
            entry, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options.earth_fixed_view));
        glVertex3d(position.x, position.y, position.z);
    }
    glEnd();
    glDisable(GL_LINE_STIPPLE);

    if (!draw_apsis_markers) {
        glLineWidth(1.0f);
        glDepthMask(GL_TRUE);
        if (options.overview_mode) {
            glDisable(GL_BLEND);
        }
        return;
    }

    // Apoapsis (max altitude) and periapsis (min altitude) markers.
    const post2::core::LaunchVehicleStateLogEntry* apo = &entries.front();
    const post2::core::LaunchVehicleStateLogEntry* peri = &entries.front();
    for (const auto& entry : entries) {
        if (entry.altitude_m > apo->altitude_m) {
            apo = &entry;
        }
        if (entry.altitude_m < peri->altitude_m) {
            peri = &entry;
        }
    }
    glPointSize(9.0f);
    glBegin(GL_POINTS);
    const Vec3 apo_pos = lifted_for_display(position_for_display(
        *apo, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options.earth_fixed_view));
    glColor3ub(37, 99, 235);   // blue = apoapsis (A)
    glVertex3d(apo_pos.x, apo_pos.y, apo_pos.z);
    const Vec3 peri_pos = lifted_for_display(position_for_display(
        *peri, earth_rotation_at_epoch_rad, earth_rotation_rad_per_s, options.earth_fixed_view));
    glColor3ub(219, 39, 119);  // magenta = periapsis (P)
    glVertex3d(peri_pos.x, peri_pos.y, peri_pos.z);
    glEnd();
    glPointSize(1.0f);

    glLineWidth(1.0f);
    glDepthMask(GL_TRUE);
    if (options.overview_mode) {
        glDisable(GL_BLEND);
    }
}

void OpenGLSceneRenderer::draw_border() const
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, static_cast<double>(width_), static_cast<double>(height_), 0.0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor3ub(226, 232, 240);
    glBegin(GL_LINE_LOOP);
    glVertex2d(0.5, 0.5);
    glVertex2d(static_cast<double>(width_) - 0.5, 0.5);
    glVertex2d(static_cast<double>(width_) - 0.5, static_cast<double>(height_) - 0.5);
    glVertex2d(0.5, static_cast<double>(height_) - 0.5);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

} // namespace post2::gui
