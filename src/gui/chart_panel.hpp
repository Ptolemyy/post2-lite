#pragma once

#include <string>
#include <vector>

#include <windows.h>

namespace post2::gui {

struct ChartSeries {
    std::vector<double> x;
    std::vector<double> y;
};

struct ChartConfig {
    std::wstring title;
    std::wstring x_label;
    std::wstring y_label;
    std::wstring y_unit;
    ChartSeries data;
    // Optional secondary y-axis-tinted highlight (e.g. peak marker).
    bool mark_peak = false;
};

// A lightweight HWND child that renders a 2D line chart via GDI. Supports
// time series and arbitrary x-y data. Auto-scales axes to the data range
// and draws grid lines, ticks, and labels.
class ChartPanel {
public:
    ChartPanel() = default;
    ~ChartPanel();

    ChartPanel(const ChartPanel&) = delete;
    ChartPanel& operator=(const ChartPanel&) = delete;

    bool initialize(HWND parent, RECT rect);
    void destroy();
    void resize(int width, int height);
    void set_data(ChartConfig config);
    void show();
    void hide();
    HWND hwnd() const { return hwnd_; }

    // Public so the free-function class registrar can hand it to RegisterClass.
    static LRESULT CALLBACK static_window_proc(HWND, UINT, WPARAM, LPARAM);

private:
    LRESULT window_proc(HWND, UINT, WPARAM, LPARAM);
    void paint(HDC hdc);

    HWND hwnd_ = nullptr;
    ChartConfig config_;
    int width_ = 0;
    int height_ = 0;
};

// Registers the shared window class for ChartPanel. Idempotent.
bool register_chart_panel_class(HINSTANCE instance);

} // namespace post2::gui
