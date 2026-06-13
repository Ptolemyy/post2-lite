#include "chart_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace post2::gui {

namespace {

constexpr const wchar_t* kChartPanelClassName = L"Post2LiteChartPanel";
constexpr int kMarginLeft = 70;
constexpr int kMarginRight = 24;
constexpr int kMarginTop = 36;
constexpr int kMarginBottom = 48;
constexpr int kAxisTickCount = 5;

std::wstring format_value(double value)
{
    wchar_t buffer[64] = {};
    const double abs_v = std::abs(value);
    if (abs_v == 0.0) {
        return L"0";
    }
    if (abs_v >= 1.0e5 || abs_v < 1.0e-2) {
        std::swprintf(buffer, 64, L"%.2e", value);
    } else if (abs_v >= 100.0) {
        std::swprintf(buffer, 64, L"%.0f", value);
    } else if (abs_v >= 1.0) {
        std::swprintf(buffer, 64, L"%.2f", value);
    } else {
        std::swprintf(buffer, 64, L"%.3f", value);
    }
    return buffer;
}

double nice_step(double range, int target_ticks)
{
    if (range <= 0.0 || target_ticks <= 0) {
        return 1.0;
    }
    const double rough = range / target_ticks;
    const double exponent = std::floor(std::log10(rough));
    const double base = std::pow(10.0, exponent);
    const double mantissa = rough / base;
    double nice_mantissa = 10.0;
    if (mantissa < 1.5) nice_mantissa = 1.0;
    else if (mantissa < 3.0) nice_mantissa = 2.0;
    else if (mantissa < 7.0) nice_mantissa = 5.0;
    return nice_mantissa * base;
}

} // namespace

bool register_chart_panel_class(HINSTANCE instance)
{
    WNDCLASSW wc = {};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = ChartPanel::static_window_proc;
    wc.hInstance = instance;
    wc.lpszClassName = kChartPanelClassName;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    if (RegisterClassW(&wc)) {
        return true;
    }
    return GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
}

ChartPanel::~ChartPanel()
{
    destroy();
}

bool ChartPanel::initialize(HWND parent, RECT rect)
{
    if (hwnd_) {
        return true;
    }
    const HINSTANCE instance =
        reinterpret_cast<HINSTANCE>(GetWindowLongPtrW(parent, GWLP_HINSTANCE));
    if (!register_chart_panel_class(instance)) {
        return false;
    }
    width_ = std::max(0L, rect.right - rect.left);
    height_ = std::max(0L, rect.bottom - rect.top);
    hwnd_ = CreateWindowExW(
        0,
        kChartPanelClassName,
        L"",
        WS_CHILD | WS_CLIPSIBLINGS,
        rect.left,
        rect.top,
        width_,
        height_,
        parent,
        nullptr,
        instance,
        this);
    if (!hwnd_) {
        return false;
    }
    SetWindowLongPtrW(hwnd_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
    return true;
}

void ChartPanel::destroy()
{
    if (hwnd_) {
        DestroyWindow(hwnd_);
        hwnd_ = nullptr;
    }
}

void ChartPanel::resize(int width, int height)
{
    width_ = std::max(0, width);
    height_ = std::max(0, height);
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

void ChartPanel::set_data(ChartConfig config)
{
    config_ = std::move(config);
    if (hwnd_) {
        InvalidateRect(hwnd_, nullptr, FALSE);
    }
}

void ChartPanel::show()
{
    if (hwnd_) {
        ShowWindow(hwnd_, SW_SHOW);
    }
}

void ChartPanel::hide()
{
    if (hwnd_) {
        ShowWindow(hwnd_, SW_HIDE);
    }
}

LRESULT CALLBACK ChartPanel::static_window_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    if (message == WM_NCCREATE) {
        const auto* cs = reinterpret_cast<CREATESTRUCT*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    }
    auto* self = reinterpret_cast<ChartPanel*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (self) {
        return self->window_proc(hwnd, message, wparam, lparam);
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

LRESULT ChartPanel::window_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    switch (message) {
    case WM_SIZE: {
        width_ = LOWORD(lparam);
        height_ = HIWORD(lparam);
        return 0;
    }
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps;
        const HDC hdc = BeginPaint(hwnd, &ps);
        paint(hdc);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_DESTROY:
        if (hwnd_ == hwnd) {
            hwnd_ = nullptr;
        }
        return 0;
    default:
        break;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void ChartPanel::paint(HDC hdc)
{
    // Background.
    RECT client{0, 0, width_, height_};
    HBRUSH background = CreateSolidBrush(RGB(252, 253, 255));
    FillRect(hdc, &client, background);
    DeleteObject(background);

    if (config_.data.x.empty() || config_.data.y.empty() ||
        config_.data.x.size() != config_.data.y.size()) {
        // Empty state placeholder.
        SetBkMode(hdc, TRANSPARENT);
        SetTextColor(hdc, RGB(100, 116, 139));
        HFONT font = CreateFontW(15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
            OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
        HFONT old_font = static_cast<HFONT>(SelectObject(hdc, font));
        const std::wstring text = config_.title.empty() ? L"No data" : (config_.title + L"\n(no data)");
        RECT rect{0, 0, width_, height_};
        DrawTextW(hdc, text.c_str(), -1, &rect,
            DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        SelectObject(hdc, old_font);
        DeleteObject(font);
        return;
    }

    // Compute data bounds.
    double x_min = config_.data.x.front();
    double x_max = config_.data.x.front();
    double y_min = config_.data.y.front();
    double y_max = config_.data.y.front();
    std::size_t peak_index = 0;
    for (std::size_t i = 0; i < config_.data.x.size(); ++i) {
        const double x = config_.data.x[i];
        const double y = config_.data.y[i];
        x_min = std::min(x_min, x);
        x_max = std::max(x_max, x);
        y_min = std::min(y_min, y);
        if (y > y_max) {
            y_max = y;
            peak_index = i;
        }
    }
    if (x_max <= x_min) x_max = x_min + 1.0;
    // Pad y range so the line isn't glued to the plot edges.
    const double y_span = y_max - y_min;
    if (y_span <= 0.0) {
        y_max = y_min + 1.0;
    } else {
        y_min -= 0.05 * y_span;
        y_max += 0.05 * y_span;
    }

    const int plot_left = kMarginLeft;
    const int plot_top = kMarginTop;
    const int plot_right = std::max(plot_left + 1, width_ - kMarginRight);
    const int plot_bottom = std::max(plot_top + 1, height_ - kMarginBottom);
    const int plot_width = plot_right - plot_left;
    const int plot_height = plot_bottom - plot_top;

    auto to_pixel_x = [&](double x) {
        const double t = (x - x_min) / (x_max - x_min);
        return plot_left + static_cast<int>(t * plot_width);
    };
    auto to_pixel_y = [&](double y) {
        const double t = (y - y_min) / (y_max - y_min);
        return plot_bottom - static_cast<int>(t * plot_height);
    };

    SetBkMode(hdc, TRANSPARENT);
    HFONT label_font = CreateFontW(13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    HFONT title_font = CreateFontW(16, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");

    // Grid lines.
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(226, 232, 240));
    HPEN axis_pen = CreatePen(PS_SOLID, 1, RGB(100, 116, 139));
    HPEN data_pen = CreatePen(PS_SOLID, 2, RGB(220, 38, 38));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, grid_pen));
    HFONT old_font = static_cast<HFONT>(SelectObject(hdc, label_font));
    SetTextColor(hdc, RGB(71, 85, 105));

    // Y grid lines + labels
    const double y_step = nice_step(y_max - y_min, kAxisTickCount);
    const double y_start = std::ceil(y_min / y_step) * y_step;
    for (double y = y_start; y <= y_max; y += y_step) {
        const int py = to_pixel_y(y);
        SelectObject(hdc, grid_pen);
        MoveToEx(hdc, plot_left, py, nullptr);
        LineTo(hdc, plot_right, py);
        const std::wstring label = format_value(y);
        RECT label_rect{0, py - 8, plot_left - 4, py + 8};
        DrawTextW(hdc, label.c_str(), -1, &label_rect,
            DT_RIGHT | DT_SINGLELINE | DT_VCENTER);
    }

    // X grid lines + labels
    const double x_step = nice_step(x_max - x_min, kAxisTickCount);
    const double x_start = std::ceil(x_min / x_step) * x_step;
    for (double x = x_start; x <= x_max; x += x_step) {
        const int px = to_pixel_x(x);
        SelectObject(hdc, grid_pen);
        MoveToEx(hdc, px, plot_top, nullptr);
        LineTo(hdc, px, plot_bottom);
        const std::wstring label = format_value(x);
        RECT label_rect{px - 40, plot_bottom + 4, px + 40, plot_bottom + 20};
        DrawTextW(hdc, label.c_str(), -1, &label_rect,
            DT_CENTER | DT_SINGLELINE | DT_TOP);
    }

    // Axes (bottom + left)
    SelectObject(hdc, axis_pen);
    MoveToEx(hdc, plot_left, plot_top, nullptr);
    LineTo(hdc, plot_left, plot_bottom);
    LineTo(hdc, plot_right, plot_bottom);

    // Data polyline.
    SelectObject(hdc, data_pen);
    bool started = false;
    for (std::size_t i = 0; i < config_.data.x.size(); ++i) {
        const int px = to_pixel_x(config_.data.x[i]);
        const int py = to_pixel_y(config_.data.y[i]);
        if (!started) {
            MoveToEx(hdc, px, py, nullptr);
            started = true;
        } else {
            LineTo(hdc, px, py);
        }
    }

    // Peak marker.
    if (config_.mark_peak && peak_index < config_.data.x.size()) {
        const int px = to_pixel_x(config_.data.x[peak_index]);
        const int py = to_pixel_y(config_.data.y[peak_index]);
        HPEN peak_pen = CreatePen(PS_SOLID, 2, RGB(37, 99, 235));
        SelectObject(hdc, peak_pen);
        Ellipse(hdc, px - 5, py - 5, px + 5, py + 5);
        wchar_t buffer[128] = {};
        std::swprintf(buffer, 128, L"peak: x=%s, y=%s",
            format_value(config_.data.x[peak_index]).c_str(),
            format_value(config_.data.y[peak_index]).c_str());
        SetTextColor(hdc, RGB(37, 99, 235));
        const int label_x = std::min(plot_right - 220, px + 8);
        const int label_y = std::max(plot_top + 4, py - 18);
        TextOutW(hdc, label_x, label_y, buffer, lstrlenW(buffer));
        SetTextColor(hdc, RGB(71, 85, 105));
        SelectObject(hdc, data_pen);
        DeleteObject(peak_pen);
    }

    // Title at top.
    SelectObject(hdc, title_font);
    SetTextColor(hdc, RGB(15, 23, 42));
    RECT title_rect{plot_left, 4, plot_right, kMarginTop - 4};
    DrawTextW(hdc, config_.title.c_str(), -1, &title_rect,
        DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    // X axis label (bottom center).
    SelectObject(hdc, label_font);
    SetTextColor(hdc, RGB(71, 85, 105));
    RECT xlabel_rect{plot_left, height_ - 22, plot_right, height_ - 4};
    DrawTextW(hdc, config_.x_label.c_str(), -1, &xlabel_rect,
        DT_CENTER | DT_VCENTER | DT_SINGLELINE);

    // Y axis label (rotated; we cheat and stack at left edge).
    const std::wstring y_axis_text = config_.y_unit.empty()
        ? config_.y_label
        : config_.y_label + L" [" + config_.y_unit + L"]";
    RECT ylabel_rect{2, plot_top - 18, plot_left + 10, plot_top - 2};
    DrawTextW(hdc, y_axis_text.c_str(), -1, &ylabel_rect,
        DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    // Cleanup.
    SelectObject(hdc, old_pen);
    SelectObject(hdc, old_font);
    DeleteObject(grid_pen);
    DeleteObject(axis_pen);
    DeleteObject(data_pen);
    DeleteObject(label_font);
    DeleteObject(title_font);
}

} // namespace post2::gui
