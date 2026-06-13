#include "software_renderer_3d.hpp"

#include "resource.h"

#include "post2/core/frames.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace post2::gui {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;

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

void select_and_delete_pen(HDC hdc, HPEN old_pen, HPEN pen)
{
    SelectObject(hdc, old_pen);
    DeleteObject(pen);
}

void select_and_delete_brush(HDC hdc, HBRUSH old_brush, HBRUSH brush)
{
    SelectObject(hdc, old_brush);
    DeleteObject(brush);
}

int projected_earth_radius_px(const Camera3D& camera, double center_depth_m)
{
    const RECT viewport = camera.viewport();
    const int min_extent = std::min(rect_width(viewport), rect_height(viewport));
    const double focal_px = (static_cast<double>(min_extent) * 0.5) / std::tan(45.0 * kDegToRad * 0.5);
    const double earth_radius_m = post2::core::kEarthRadiusM;
    const double tangent_depth_m =
        std::sqrt(std::max(1.0, center_depth_m * center_depth_m - earth_radius_m * earth_radius_m));
    return static_cast<int>(
        std::max(4.0, static_cast<double>(std::lround(focal_px * earth_radius_m / tangent_depth_m))));
}

std::size_t trajectory_draw_stride(std::size_t point_count)
{
    constexpr std::size_t kMaxDrawnSegments = 3000;
    if (point_count <= kMaxDrawnSegments + 1) {
        return 1;
    }
    return std::max<std::size_t>(1, (point_count - 1 + kMaxDrawnSegments - 1) / kMaxDrawnSegments);
}

struct EarthTexture {
    int width = 0;
    int height = 0;
    std::vector<std::uint32_t> pixels;

    bool valid() const
    {
        return width > 0 && height > 0 && pixels.size() == static_cast<std::size_t>(width) * height;
    }
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
    texture.pixels.resize(static_cast<std::size_t>(width) * height);

    for (int y = 0; y < height; ++y) {
        const int source_y = top_down ? y : (height - 1 - y);
        const auto* row = data + pixel_offset + row_stride * static_cast<std::size_t>(source_y);
        for (int x = 0; x < width; ++x) {
            const auto* pixel = row + static_cast<std::size_t>(x) * bytes_per_pixel;
            const std::uint32_t blue = pixel[0];
            const std::uint32_t green = pixel[1];
            const std::uint32_t red = pixel[2];
            texture.pixels[static_cast<std::size_t>(y) * width + x] = (red << 16) | (green << 8) | blue;
        }
    }

    return texture;
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

const EarthTexture& earth_texture()
{
    static const EarthTexture texture = load_earth_texture_from_resource();
    return texture;
}

std::uint32_t sample_texture_bilinear(const EarthTexture& texture, double u, double v)
{
    u -= std::floor(u);
    v = clamp(v, 0.0, 1.0);

    const double x = u * static_cast<double>(texture.width - 1);
    const double y = v * static_cast<double>(texture.height - 1);
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = (x0 + 1) % texture.width;
    const int y1 = std::min(y0 + 1, texture.height - 1);
    const double tx = x - static_cast<double>(x0);
    const double ty = y - static_cast<double>(y0);

    auto channel = [&](int pixel_index, int shift) {
        return static_cast<double>((texture.pixels[static_cast<std::size_t>(pixel_index)] >> shift) & 0xffU);
    };

    const int p00 = y0 * texture.width + x0;
    const int p10 = y0 * texture.width + x1;
    const int p01 = y1 * texture.width + x0;
    const int p11 = y1 * texture.width + x1;

    auto interpolate = [&](int shift) {
        const double top = channel(p00, shift) * (1.0 - tx) + channel(p10, shift) * tx;
        const double bottom = channel(p01, shift) * (1.0 - tx) + channel(p11, shift) * tx;
        return static_cast<std::uint32_t>(std::lround(top * (1.0 - ty) + bottom * ty));
    };

    const std::uint32_t red = interpolate(16);
    const std::uint32_t green = interpolate(8);
    const std::uint32_t blue = interpolate(0);
    return (red << 16) | (green << 8) | blue;
}

std::uint32_t apply_limb_shading(std::uint32_t color, double limb)
{
    const double shade = 0.68 + 0.32 * std::sqrt(clamp(limb, 0.0, 1.0));
    const auto shaded = [shade](std::uint32_t value) {
        return static_cast<std::uint32_t>(clamp(std::lround(static_cast<double>(value) * shade), 0.0, 255.0));
    };

    const std::uint32_t red = shaded((color >> 16) & 0xffU);
    const std::uint32_t green = shaded((color >> 8) & 0xffU);
    const std::uint32_t blue = shaded(color & 0xffU);
    return (red << 16) | (green << 8) | blue;
}

void draw_fallback_earth(HDC hdc, POINT center, int radius_px)
{
    HBRUSH earth_brush = CreateSolidBrush(RGB(172, 205, 244));
    HPEN earth_pen = CreatePen(PS_SOLID, 2, RGB(37, 99, 235));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, earth_pen));
    HBRUSH old_brush = static_cast<HBRUSH>(SelectObject(hdc, earth_brush));
    Ellipse(
        hdc,
        center.x - radius_px,
        center.y - radius_px,
        center.x + radius_px,
        center.y + radius_px);
    select_and_delete_brush(hdc, old_brush, earth_brush);
    select_and_delete_pen(hdc, old_pen, earth_pen);

    earth_pen = CreatePen(PS_SOLID, 2, RGB(37, 99, 235));
    old_pen = static_cast<HPEN>(SelectObject(hdc, earth_pen));
    SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Ellipse(
        hdc,
        center.x - radius_px,
        center.y - radius_px,
        center.x + radius_px,
        center.y + radius_px);
    select_and_delete_pen(hdc, old_pen, earth_pen);
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

bool intersect_sphere_nearest(
    const Vec3& ray_origin_m,
    const Vec3& ray_direction,
    double sphere_radius_m,
    Vec3* hit_m)
{
    const double a = post2::vehicle::dot(ray_direction, ray_direction);
    if (a <= 1.0e-12) {
        return false;
    }

    const double b = 2.0 * post2::vehicle::dot(ray_origin_m, ray_direction);
    const double c =
        post2::vehicle::dot(ray_origin_m, ray_origin_m) - sphere_radius_m * sphere_radius_m;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
        return false;
    }

    const double root = std::sqrt(discriminant);
    double t = (-b - root) / (2.0 * a);
    if (t <= 1.0e-6) {
        t = (-b + root) / (2.0 * a);
    }
    if (t <= 1.0e-6) {
        return false;
    }

    *hit_m = ray_origin_m + ray_direction * t;
    return true;
}

// WGS84 ellipsoid ray intersection. Re-uses the sphere solver after scaling
// the problem by (1/a, 1/a, 1/b) into a unit-sphere space.
bool intersect_wgs84_ellipsoid_nearest(
    const Vec3& ray_origin_m,
    const Vec3& ray_direction,
    Vec3* hit_m)
{
    const double a_m = post2::core::frames::Wgs84::a_m;
    const double b_m = post2::core::frames::Wgs84::b_m;
    const Vec3 scaled_origin{ray_origin_m.x / a_m, ray_origin_m.y / a_m, ray_origin_m.z / b_m};
    const Vec3 scaled_direction{ray_direction.x / a_m, ray_direction.y / a_m, ray_direction.z / b_m};

    Vec3 scaled_hit = {};
    if (!intersect_sphere_nearest(scaled_origin, scaled_direction, 1.0, &scaled_hit)) {
        return false;
    }
    *hit_m = {scaled_hit.x * a_m, scaled_hit.y * a_m, scaled_hit.z * b_m};
    return true;
}

} // namespace

void Camera3D::reset(double scene_radius_m)
{
    yaw_deg_ = 35.0;
    pitch_deg_ = 22.0;
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
    const Vec3 forward = normalized_or(Vec3{0.0, 0.0, 0.0} - eye, {-1.0, 0.0, 0.0});
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
    const Vec3 eye_dir = eye_direction();

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
    return normalized_or(eye_position(), {1.0, 0.0, 0.0});
}

Vec3 Camera3D::ray_direction_for_pixel(double screen_x, double screen_y) const
{
    const Vec3 eye = eye_position();
    const Vec3 forward = normalized_or(Vec3{0.0, 0.0, 0.0} - eye, {-1.0, 0.0, 0.0});
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

double Camera3D::min_distance_m() const
{
    return 1.4 * scene_radius_m_;
}

double Camera3D::max_distance_m() const
{
    return 12.0 * scene_radius_m_;
}

Vec3 Camera3D::eye_position() const
{
    const double yaw = yaw_deg_ * kDegToRad;
    const double pitch = pitch_deg_ * kDegToRad;
    const double cp = std::cos(pitch);
    return {
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

void SoftwareRenderer3D::draw(HDC hdc, const Camera3D& camera, const post2::core::StateLog& state_log) const
{
    if (!camera.has_viewport() || state_log.empty()) {
        return;
    }

    const RECT viewport = camera.viewport();
    const int saved_dc = SaveDC(hdc);
    IntersectClipRect(hdc, viewport.left, viewport.top, viewport.right, viewport.bottom);

    HBRUSH background = CreateSolidBrush(RGB(244, 247, 251));
    FillRect(hdc, &viewport, background);
    DeleteObject(background);

    HPEN border_pen = CreatePen(PS_SOLID, 1, RGB(226, 232, 240));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, border_pen));
    HBRUSH old_brush = static_cast<HBRUSH>(SelectObject(hdc, GetStockObject(NULL_BRUSH)));
    Rectangle(hdc, viewport.left, viewport.top, viewport.right, viewport.bottom);
    SelectObject(hdc, old_brush);
    select_and_delete_pen(hdc, old_pen, border_pen);

    draw_trajectory_segments(hdc, camera, state_log, false, RGB(248, 178, 178), 2);
    draw_marker(hdc, camera, state_log.front().state.position_m, RGB(122, 211, 151), false);
    draw_marker(hdc, camera, state_log.back().state.position_m, RGB(148, 163, 184), false);

    draw_earth(hdc, camera);
    draw_earth_axis(hdc, camera);

    draw_trajectory_segments(hdc, camera, state_log, true, RGB(220, 38, 38), 3);
    draw_marker(hdc, camera, state_log.front().state.position_m, RGB(22, 163, 74), true);
    draw_marker(hdc, camera, state_log.back().state.position_m, RGB(17, 24, 39), true);

    RestoreDC(hdc, saved_dc);
}

void SoftwareRenderer3D::draw_trajectory_segments(
    HDC hdc,
    const Camera3D& camera,
    const post2::core::StateLog& state_log,
    bool near_side,
    COLORREF color,
    int width) const
{
    const auto& entries = state_log.entries();
    if (entries.size() < 2) {
        return;
    }

    HPEN pen = CreatePen(PS_SOLID, width, color);
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, pen));

    const std::size_t stride = trajectory_draw_stride(entries.size());
    std::size_t previous_index = 0;
    while (previous_index + 1 < entries.size()) {
        const std::size_t next_index = std::min(previous_index + stride, entries.size() - 1);
        const ProjectedPoint3D a = camera.project(entries[previous_index].state.position_m);
        const ProjectedPoint3D b = camera.project(entries[next_index].state.position_m);
        if (!a.valid || !b.valid) {
            previous_index = next_index;
            continue;
        }

        if (a.near_side != b.near_side || a.near_side != near_side) {
            previous_index = next_index;
            continue;
        }

        MoveToEx(hdc, a.screen.x, a.screen.y, nullptr);
        LineTo(hdc, b.screen.x, b.screen.y);
        previous_index = next_index;
    }

    select_and_delete_pen(hdc, old_pen, pen);
}

void SoftwareRenderer3D::draw_earth(HDC hdc, const Camera3D& camera) const
{
    const ProjectedPoint3D center = camera.project({0.0, 0.0, 0.0});
    if (!center.valid) {
        return;
    }

    const int radius_px = projected_earth_radius_px(camera, center.depth_m);
    const EarthTexture& texture = earth_texture();
    if (!texture.valid()) {
        draw_fallback_earth(hdc, center.screen, radius_px);
        return;
    }

    const RECT viewport = camera.viewport();
    const int left = std::max(viewport.left, center.screen.x - radius_px);
    const int top = std::max(viewport.top, center.screen.y - radius_px);
    const int right_edge = std::min(viewport.right, center.screen.x + radius_px + 1);
    const int bottom = std::min(viewport.bottom, center.screen.y + radius_px + 1);
    const int width = right_edge - left;
    const int height = bottom - top;
    if (width <= 0 || height <= 0) {
        return;
    }

    const std::uint32_t background_color = (244U << 16) | (247U << 8) | 251U;
    std::vector<std::uint32_t> pixels(static_cast<std::size_t>(width) * height, background_color);
    const Vec3 eye = camera.eye_position();

    const double a_m = post2::core::frames::Wgs84::a_m;
    const double b_m = post2::core::frames::Wgs84::b_m;
    const double inv_a2 = 1.0 / (a_m * a_m);
    const double inv_b2 = 1.0 / (b_m * b_m);

    for (int y = 0; y < height; ++y) {
        const double screen_y = static_cast<double>(top + y) + 0.5;
        for (int x = 0; x < width; ++x) {
            const double screen_x = static_cast<double>(left + x) + 0.5;
            const Vec3 ray_direction = camera.ray_direction_for_pixel(screen_x, screen_y);
            Vec3 hit = {};
            if (!intersect_wgs84_ellipsoid_nearest(eye, ray_direction, &hit)) {
                continue;
            }

            // Outward normal of the ellipsoid is (x/a^2, y/a^2, z/b^2)
            // normalized. Texture uses WGS84 geodetic lat/lon.
            Vec3 normal{hit.x * inv_a2, hit.y * inv_a2, hit.z * inv_b2};
            const double normal_inv = 1.0 / post2::vehicle::norm(normal);
            normal = normal * normal_inv;
            const Vec3 to_eye = normalized_or(eye - hit, {normal.x, normal.y, normal.z});
            const double limb = clamp(post2::vehicle::dot(normal, to_eye), 0.0, 1.0);
            const post2::core::frames::Geodetic geo =
                post2::core::frames::ecef_to_geodetic(hit);
            const double u = geo.longitude_rad / (2.0 * kPi) + 0.5;
            const double v = 0.5 - geo.latitude_rad / kPi;
            pixels[static_cast<std::size_t>(y) * width + x] =
                apply_limb_shading(sample_texture_bilinear(texture, u, v), limb);
        }
    }

    BITMAPINFO bitmap_info = {};
    bitmap_info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bitmap_info.bmiHeader.biWidth = width;
    bitmap_info.bmiHeader.biHeight = -height;
    bitmap_info.bmiHeader.biPlanes = 1;
    bitmap_info.bmiHeader.biBitCount = 32;
    bitmap_info.bmiHeader.biCompression = BI_RGB;

    const int saved_dc = SaveDC(hdc);
    HRGN earth_region = CreateEllipticRgn(
        center.screen.x - radius_px,
        center.screen.y - radius_px,
        center.screen.x + radius_px + 1,
        center.screen.y + radius_px + 1);
    ExtSelectClipRgn(hdc, earth_region, RGN_AND);
    SetDIBitsToDevice(
        hdc,
        left,
        top,
        static_cast<DWORD>(width),
        static_cast<DWORD>(height),
        0,
        0,
        0,
        static_cast<UINT>(height),
        pixels.data(),
        &bitmap_info,
        DIB_RGB_COLORS);
    DeleteObject(earth_region);
    RestoreDC(hdc, saved_dc);

    HPEN earth_pen = CreatePen(PS_SOLID, 1, RGB(52, 83, 116));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, earth_pen));
    SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Ellipse(
        hdc,
        center.screen.x - radius_px,
        center.screen.y - radius_px,
        center.screen.x + radius_px,
        center.screen.y + radius_px);
    select_and_delete_pen(hdc, old_pen, earth_pen);
}

void SoftwareRenderer3D::draw_earth_axis(HDC hdc, const Camera3D& camera) const
{
    const double radius_m = post2::core::kEarthRadiusM;
    const double axis_extent_m = radius_m * 1.16;
    HPEN outline_pen = CreatePen(PS_SOLID, 2, RGB(15, 23, 42));
    HPEN axis_pen = CreatePen(PS_SOLID, 1, RGB(226, 232, 240));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, outline_pen));

    auto draw_axis_stub = [&](const Vec3& surface_m, const Vec3& outer_m) {
        const ProjectedPoint3D surface = camera.project(surface_m);
        const ProjectedPoint3D outer = camera.project(outer_m);
        if (!surface.valid || !outer.valid || !surface.near_side || !outer.near_side) {
            return;
        }

        SelectObject(hdc, outline_pen);
        MoveToEx(hdc, surface.screen.x, surface.screen.y, nullptr);
        LineTo(hdc, outer.screen.x, outer.screen.y);

        SelectObject(hdc, axis_pen);
        MoveToEx(hdc, surface.screen.x, surface.screen.y, nullptr);
        LineTo(hdc, outer.screen.x, outer.screen.y);
    };

    // Poles sit at z = +/-b on the WGS84 ellipsoid, not +/-a.
    const double polar_radius_m = post2::core::frames::Wgs84::b_m;
    draw_axis_stub({0.0, 0.0, polar_radius_m}, {0.0, 0.0, axis_extent_m});
    draw_axis_stub({0.0, 0.0, -polar_radius_m}, {0.0, 0.0, -axis_extent_m});

    SelectObject(hdc, old_pen);
    DeleteObject(axis_pen);
    DeleteObject(outline_pen);
}

void SoftwareRenderer3D::draw_marker(
    HDC hdc,
    const Camera3D& camera,
    const Vec3& position_m,
    COLORREF color,
    bool near_side) const
{
    const ProjectedPoint3D projected = camera.project(position_m);
    if (!projected.valid || projected.near_side != near_side) {
        return;
    }

    const int radius_px = projected.near_side ? 5 : 4;
    HBRUSH brush = CreateSolidBrush(color);
    HPEN pen = CreatePen(PS_SOLID, 1, RGB(15, 23, 42));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, pen));
    HBRUSH old_brush = static_cast<HBRUSH>(SelectObject(hdc, brush));
    Ellipse(
        hdc,
        projected.screen.x - radius_px,
        projected.screen.y - radius_px,
        projected.screen.x + radius_px,
        projected.screen.y + radius_px);
    select_and_delete_brush(hdc, old_brush, brush);
    select_and_delete_pen(hdc, old_pen, pen);
}

} // namespace post2::gui
