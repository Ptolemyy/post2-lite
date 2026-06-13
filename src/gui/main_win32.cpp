#include "post2/core/case_config_io.hpp"
#include "post2/core/io.hpp"
#include "post2/core/optimization.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"
#include "opengl_scene_renderer.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <commdlg.h>
#include <windows.h>
#include <windowsx.h>

namespace {

using post2::core::CoreMode;
using post2::core::CaseConfig;
using post2::core::SimulationConfig;
using post2::core::SimulationResult;

constexpr int kMenuRefresh = 1001;
constexpr int kMenuExportCsv = 1002;
constexpr int kMenuExportSvg = 1003;
constexpr int kMenuExit = 1004;
constexpr int kMenuCaseLoad = 1005;
constexpr int kMenuCaseSave = 1006;
constexpr int kMenuModeLocal = 1101;
constexpr int kMenuModeRemote = 1102;
constexpr int kMenuRemoteEndpoint = 1103;
constexpr int kMenuVehicleEdit = 1301;
constexpr int kMenuRunTrajectory = 1501;
constexpr int kMenuOptimizationSettings = 1502;
constexpr int kMenuOptimizationExecute = 1503;
constexpr int kMenuLaunchSettings = 1601;
constexpr int kMenuCasePhases = 1801;
constexpr int kPhaseList = 1901;
constexpr int kPhaseAdd = 1902;
constexpr int kPhaseDelete = 1903;
constexpr int kPhaseNameEdit = 1904;
constexpr int kPhaseDurationEdit = 1905;
constexpr int kPhaseInheritInitial = 1906;
constexpr int kPhaseHoldDownInitial = 1907;
constexpr int kPhaseGravity = 1908;
constexpr int kPhaseThrust = 1909;
constexpr int kPhaseNormalForce = 1910;
constexpr int kPhaseOptimizeEnabled = 1919;
constexpr int kPhaseThrottleType = 1911;
constexpr int kPhaseThrottleC0 = 1912;
constexpr int kPhaseThrottleC1 = 1913;
constexpr int kPhaseThrottleC2 = 1914;
constexpr int kPhaseThrottleT2W = 1915;
constexpr int kPhaseSteeringType = 1916;
constexpr int kPhaseSteeringAzimuth = 1917;
constexpr int kPhaseSteeringElevation = 1918;
constexpr int kPhaseApply = 1920;
constexpr int kPhaseJson = 1921;
constexpr int kPhaseActionList = 1922;
constexpr int kPhaseActionAdd = 1923;
constexpr int kPhaseActionDelete = 1924;
constexpr int kPhaseActionTimeEdit = 1925;
constexpr int kPhaseActionType = 1926;
constexpr int kPhaseActionValue = 1927;
constexpr int kPhaseActionStage = 1928;
constexpr int kPhaseScrollPane = 1930;
constexpr int kSidebarWidth = 326;
constexpr int kRemoteHostEdit = 1201;
constexpr int kRemotePortEdit = 1202;
constexpr int kRemoteOk = IDOK;
constexpr int kRemoteCancel = IDCANCEL;
constexpr int kVehicleNameEdit = 1401;
constexpr int kVehicleDryMassEdit = 1402;
constexpr int kVehicleEngineEnabled = 1403;
constexpr int kVehicleEngineThrustEdit = 1404;
constexpr int kVehicleEngineIspEdit = 1405;
constexpr int kVehicleEngineDirXEdit = 1406;
constexpr int kVehicleEngineDirYEdit = 1407;
constexpr int kVehicleEngineDirZEdit = 1408;
constexpr int kVehicleTankNameEdit = 1409;
constexpr int kVehicleTankPropellantEdit = 1410;
constexpr int kVehicleTankCapacityEdit = 1411;
constexpr int kVehicleTankInitialEdit = 1412;
constexpr int kVehicleStageList = 1413;
constexpr int kVehicleStageAdd = 1414;
constexpr int kVehicleStageDelete = 1415;
constexpr int kVehicleStageEdit = 1416;
constexpr int kVehicleStageActive = 1417;
constexpr int kLaunchLatitudeEdit = 1701;
constexpr int kLaunchLongitudeEdit = 1702;
constexpr int kLaunchAltitudeEdit = 1703;
constexpr int kLaunchHoldDownClampEnabled = 1704;
constexpr int kLaunchHoldDownClampReleaseEdit = 1705;
constexpr int kLaunchNormalForceEnabled = 1706;
constexpr int kOutputsEdit = 2001;
constexpr int kOptModeCombo = 2101;
constexpr int kOptOptimizerCombo = 2102;
constexpr int kOptMaxIterationsEdit = 2103;
constexpr int kOptToleranceEdit = 2104;
constexpr int kOptStepFractionEdit = 2105;
constexpr int kOptTargetList = 2106;
constexpr int kOptTargetAdd = 2107;
constexpr int kOptTargetDelete = 2108;
constexpr int kOptTargetMetricCombo = 2109;
constexpr int kOptTargetModeCombo = 2110;
constexpr int kOptTargetValueEdit = 2111;
constexpr int kOptTargetMinEdit = 2112;
constexpr int kOptTargetMaxEdit = 2113;
constexpr int kOptTargetWeightEdit = 2114;
constexpr int kOptObjectiveEnabled = 2115;
constexpr int kOptObjectiveMetricCombo = 2116;
constexpr int kOptObjectiveDirectionCombo = 2117;
constexpr int kOptObjectiveWeightEdit = 2118;
constexpr const wchar_t* kSceneWindowClassName = L"Post2LiteOpenGLScene";

HINSTANCE g_instance = nullptr;
CoreMode g_mode = CoreMode::Local;
SimulationConfig g_config;
CaseConfig g_case;
SimulationResult g_result;
std::string g_status;
std::string g_remote_host = "127.0.0.1";
int g_remote_port = 5050;
post2::gui::Camera3D g_camera;
post2::gui::OpenGLSceneRenderer g_scene_renderer;
HWND g_scene_hwnd = nullptr;
bool g_camera_initialized = false;
bool g_case_initialized = false;
bool g_dragging = false;
POINT g_last_mouse = {};
int g_selected_phase_index = 0;
HWND g_phase_list = nullptr;
HWND g_phase_add_button = nullptr;
HWND g_phase_delete_button = nullptr;
HWND g_phase_name_edit = nullptr;
HWND g_phase_duration_edit = nullptr;
HWND g_phase_inherit_initial = nullptr;
HWND g_phase_hold_down_initial = nullptr;
HWND g_phase_optimize_enabled = nullptr;
HWND g_phase_gravity = nullptr;
HWND g_phase_thrust = nullptr;
HWND g_phase_normal_force = nullptr;
HWND g_phase_throttle_type = nullptr;
HWND g_phase_throttle_c0 = nullptr;
HWND g_phase_throttle_c1 = nullptr;
HWND g_phase_throttle_c2 = nullptr;
HWND g_phase_throttle_t2w = nullptr;
HWND g_phase_steering_type = nullptr;
HWND g_phase_steering_azimuth = nullptr;
HWND g_phase_steering_elevation = nullptr;
HWND g_phase_apply_button = nullptr;
HWND g_phase_json_button = nullptr;
HWND g_phase_action_list = nullptr;
HWND g_phase_action_add_button = nullptr;
HWND g_phase_action_delete_button = nullptr;
HWND g_phase_action_time_edit = nullptr;
HWND g_phase_action_type = nullptr;
HWND g_phase_action_value = nullptr;
HWND g_phase_action_stage = nullptr;
HWND g_phase_scroll_pane = nullptr;
HWND g_outputs_edit = nullptr;
int g_selected_action_index = 0;
std::string g_outputs_text;

struct NumericBindingRow {
    std::string path;
    std::string label;
    HWND value_edit = nullptr;
    HWND opt_check = nullptr;
    HWND min_edit = nullptr;
    HWND max_edit = nullptr;
};

std::vector<NumericBindingRow> g_phase_numeric_rows;
int g_phase_scroll_pos = 0;
int g_phase_scroll_content_height = 0;
int g_phase_scroll_page_height = 0;
WNDPROC g_phase_scroll_old_proc = nullptr;

void refresh_phase_list();
void load_selected_phase_controls();
void create_label(HWND parent, int x, int y, int width, const wchar_t* text, HFONT font);
HWND create_edit(HWND parent, int id, int x, int y, int width, const std::wstring& text, HFONT font);
HWND create_button(HWND parent, int id, int x, int y, int width, int height, const wchar_t* text, HFONT font);
HWND create_checkbox(HWND parent, int id, int x, int y, int width, const wchar_t* text, HFONT font);
HWND create_combo(HWND parent, int id, int x, int y, int width, HFONT font);
void add_combo_item(HWND combo, const wchar_t* text);
void select_combo_text(HWND combo, const std::string& text);
std::string get_combo_text(HWND combo);
void sync_scene_window(HWND hwnd);
void invalidate_scene_window();

bool window_is_live(HWND hwnd)
{
    return hwnd && IsWindow(hwnd);
}

void clear_phase_editor_handles()
{
    g_phase_name_edit = nullptr;
    g_phase_duration_edit = nullptr;
    g_phase_inherit_initial = nullptr;
    g_phase_hold_down_initial = nullptr;
    g_phase_optimize_enabled = nullptr;
    g_phase_gravity = nullptr;
    g_phase_thrust = nullptr;
    g_phase_normal_force = nullptr;
    g_phase_throttle_type = nullptr;
    g_phase_throttle_c0 = nullptr;
    g_phase_throttle_c1 = nullptr;
    g_phase_throttle_c2 = nullptr;
    g_phase_throttle_t2w = nullptr;
    g_phase_steering_type = nullptr;
    g_phase_steering_azimuth = nullptr;
    g_phase_steering_elevation = nullptr;
    g_phase_action_list = nullptr;
    g_phase_action_add_button = nullptr;
    g_phase_action_delete_button = nullptr;
    g_phase_action_time_edit = nullptr;
    g_phase_action_type = nullptr;
    g_phase_action_value = nullptr;
    g_phase_action_stage = nullptr;
    g_phase_scroll_pane = nullptr;
    g_phase_numeric_rows.clear();
    g_phase_scroll_pos = 0;
    g_phase_scroll_content_height = 0;
    g_phase_scroll_page_height = 0;
    g_phase_scroll_old_proc = nullptr;
    g_selected_action_index = 0;
}

struct RemoteSettingsDialogState {
    HWND hwnd = nullptr;
    HWND host_edit = nullptr;
    HWND port_edit = nullptr;
    HWND parent = nullptr;
    std::string host;
    int port = 5050;
    bool accepted = false;
};

struct VehicleSettingsDialogState {
    HWND hwnd = nullptr;
    HWND name_edit = nullptr;
    HWND dry_mass_edit = nullptr;
    HWND stage_list = nullptr;
    post2::vehicle::VehicleConfig config;
    post2::core::OptimizationConfig optimization;
    int selected_stage_index = 0;
    bool accepted = false;
};

struct StageEditorDialogState {
    HWND hwnd = nullptr;
    HWND name_edit = nullptr;
    HWND active_check = nullptr;
    HWND attached_check = nullptr;
    HWND dry_mass_edit = nullptr;
    HWND dry_mass_opt_check = nullptr;
    HWND dry_mass_min_edit = nullptr;
    HWND dry_mass_max_edit = nullptr;
    HWND engine_enabled = nullptr;
    HWND engine_thrust_edit = nullptr;
    HWND engine_isp_edit = nullptr;
    HWND engine_dir_x_edit = nullptr;
    HWND engine_dir_y_edit = nullptr;
    HWND engine_dir_z_edit = nullptr;
    HWND tank_name_edit = nullptr;
    HWND tank_propellant_edit = nullptr;
    HWND tank_capacity_edit = nullptr;
    HWND tank_initial_edit = nullptr;
    post2::vehicle::StageConfig stage;
    post2::core::OptimizationConfig* optimization = nullptr;
    int stage_index = -1;
    bool accepted = false;
};

struct OptimizationSettingsDialogState {
    HWND hwnd = nullptr;
    HWND mode_combo = nullptr;
    HWND optimizer_combo = nullptr;
    HWND max_iterations_edit = nullptr;
    HWND tolerance_edit = nullptr;
    HWND step_fraction_edit = nullptr;
    HWND target_list = nullptr;
    HWND target_metric_combo = nullptr;
    HWND target_mode_combo = nullptr;
    HWND target_value_edit = nullptr;
    HWND target_min_edit = nullptr;
    HWND target_max_edit = nullptr;
    HWND target_weight_edit = nullptr;
    HWND objective_enabled = nullptr;
    HWND objective_metric_combo = nullptr;
    HWND objective_direction_combo = nullptr;
    HWND objective_weight_edit = nullptr;
    post2::core::OptimizationConfig config;
    int selected_target_index = 0;
    bool accepted = false;
};

struct LaunchSettingsDialogState {
    HWND hwnd = nullptr;
    HWND latitude_edit = nullptr;
    HWND longitude_edit = nullptr;
    HWND altitude_edit = nullptr;
    HWND hold_down_clamp_enabled = nullptr;
    HWND hold_down_clamp_release_edit = nullptr;
    HWND normal_force_enabled = nullptr;
    SimulationConfig config;
    bool accepted = false;
};

struct CaseJsonDialogState {
    HWND hwnd = nullptr;
    HWND json_edit = nullptr;
    CaseConfig config;
    bool accepted = false;
};

struct PhaseEditorDialogState {
    HWND hwnd = nullptr;
    HWND parent = nullptr;
    post2::core::PhaseConfig original_phase;
    bool accepted = false;
};

std::wstring widen(const std::string& text)
{
    if (text.empty()) {
        return L"";
    }
    const int needed = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, nullptr, 0);
    if (needed <= 0) {
        return L"";
    }
    std::wstring wide(static_cast<std::size_t>(needed - 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, wide.data(), needed);
    return wide;
}

std::string narrow(const std::wstring& text)
{
    if (text.empty()) {
        return "";
    }
    const int needed = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (needed <= 0) {
        return "";
    }
    std::string narrow_text(static_cast<std::size_t>(needed - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), -1, narrow_text.data(), needed, nullptr, nullptr);
    return narrow_text;
}

std::wstring get_window_text(HWND hwnd)
{
    const int length = GetWindowTextLengthW(hwnd);
    std::wstring text(static_cast<std::size_t>(length + 1), L'\0');
    GetWindowTextW(hwnd, text.data(), length + 1);
    text.resize(static_cast<std::size_t>(length));
    return text;
}

std::string remote_endpoint_text()
{
    return g_remote_host + ":" + std::to_string(g_remote_port);
}

void ensure_case_initialized()
{
    if (!g_case_initialized || g_case.phases.empty()) {
        g_case = post2::core::case_from_simulation_config(g_config);
        g_case_initialized = true;
    }
}

void sync_legacy_from_case()
{
    ensure_case_initialized();
    g_config = post2::core::simulation_config_from_case(g_case);
}

void sync_case_surface_from_legacy()
{
    ensure_case_initialized();
    g_case.vehicle = g_config.vehicle;
    g_case.launch_site = g_config.launch_site;
    g_case.step_s = g_config.step_s;
    if (!g_case.phases.empty()) {
        auto& phase = g_case.phases.front();
        phase.duration_s = g_config.duration_s;
        phase.force_models.normal_force = g_config.normal_force.enabled;
        phase.hold_down_clamp_initial_active =
            g_config.hold_down_clamp.enabled && g_config.hold_down_clamp.release_time_s > 0.0;
        if (g_config.hold_down_clamp.enabled) {
            bool updated = false;
            for (auto& action : phase.actions) {
                if (action.type == "set_hold_down_clamp_active" && !action.value) {
                    action.time_s = g_config.hold_down_clamp.release_time_s;
                    updated = true;
                    break;
                }
            }
            if (!updated) {
                phase.actions.push_back({g_config.hold_down_clamp.release_time_s, "set_hold_down_clamp_active", false});
            }
        }
    }
}

void update_status()
{
    if (!g_result.ok) {
        g_status = std::string("mode: ") + post2::core::core_mode_name(g_mode) +
            " | remote: " + remote_endpoint_text() +
            " | " + g_result.error;
        return;
    }

    const auto& last = g_result.state_log.back();
    g_status =
        std::string("mode: ") + post2::core::core_mode_name(g_mode) +
        " | remote: " + remote_endpoint_text() +
        " | state log entries: " + std::to_string(g_result.state_log.size()) +
        " | end altitude: " + std::to_string(last.altitude_m / 1000.0) +
        " km | end speed: " + std::to_string(last.speed_mps) +
        " m/s | HDC: " + (last.hold_down_clamp_active ? "active" : "inactive");
}

void set_outputs_text(const std::string& text)
{
    g_outputs_text = text;
    if (window_is_live(g_outputs_edit)) {
        SetWindowTextW(g_outputs_edit, widen(g_outputs_text).c_str());
    }
}

std::string format_metric_lines(const std::vector<post2::core::OptimizationMetricValue>& metrics)
{
    std::ostringstream output;
    for (const auto& metric : metrics) {
        output << metric.metric << ": " << metric.value << '\n';
    }
    return output.str();
}

std::string format_run_outputs(const SimulationResult& result)
{
    if (!result.ok) {
        return "Run failed: " + result.error;
    }
    if (result.state_log.empty()) {
        return "Run failed: empty StateLog";
    }

    const auto& last = result.state_log.back();
    std::ostringstream output;
    output
        << "Run trajectory\n"
        << "State entries: " << result.state_log.size() << '\n'
        << "End time (s): " << last.time_s << '\n'
        << "Terminal altitude (m): " << last.altitude_m << '\n'
        << "Terminal speed (m/s): " << last.speed_mps << '\n'
        << "Payload mass (kg): " << post2::vehicle::payload_stage_dry_mass_kg(g_case.vehicle) << '\n';
    output << format_metric_lines(post2::core::evaluate_trajectory_metrics(result.state_log, g_case));
    return output.str();
}

std::string format_optimization_outputs(const post2::core::OptimizationResult& result)
{
    std::ostringstream output;
    output
        << "Optimize\n"
        << "OK: " << (result.ok ? "true" : "false") << '\n'
        << "Iterations: " << result.iterations << '\n'
        << "Evaluations: " << result.evaluations << '\n'
        << "Best score: " << result.best_score << '\n'
        << "Payload mass (kg): " << post2::vehicle::payload_stage_dry_mass_kg(g_case.vehicle) << '\n';
    if (!result.error.empty()) {
        output << "Error: " << result.error << '\n';
    }
    for (const auto& change : result.variable_changes) {
        output << change.path << ": " << change.old_value << " -> " << change.new_value << '\n';
    }
    output << format_metric_lines(result.final_metrics);
    for (const auto& message : result.messages) {
        output << "Message: " << message << '\n';
    }
    return output.str();
}

void run_simulation(HWND hwnd)
{
    ensure_case_initialized();
    const auto service = post2::core::make_trajectory_service(g_mode, g_remote_host, g_remote_port);
    g_result = service->simulate(g_case);
    if (g_result.ok && !g_result.state_log.empty()) {
        const double scene_radius_m = post2::gui::compute_scene_radius_m(g_result.state_log);
        if (!g_camera_initialized) {
            g_camera.reset(scene_radius_m);
            g_camera_initialized = true;
        } else {
            g_camera.set_scene_radius(scene_radius_m);
        }
    }
    update_status();
    set_outputs_text(format_run_outputs(g_result));
    sync_scene_window(hwnd);
    InvalidateRect(hwnd, nullptr, TRUE);
}

void execute_optimization(HWND hwnd)
{
    ensure_case_initialized();
    const auto service = post2::core::make_trajectory_service(g_mode, g_remote_host, g_remote_port);
    const auto result = post2::core::optimize_case(&g_case, *service);
    set_outputs_text(format_optimization_outputs(result));
    if (!result.ok) {
        g_result = result.final_simulation;
        g_status = "optimize failed: " + result.error;
        sync_scene_window(hwnd);
        InvalidateRect(hwnd, nullptr, TRUE);
        MessageBoxW(hwnd, widen(result.error).c_str(), L"Optimize failed", MB_ICONERROR);
        return;
    }

    g_result = result.final_simulation;
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
    if (g_result.ok && !g_result.state_log.empty()) {
        const double scene_radius_m = post2::gui::compute_scene_radius_m(g_result.state_log);
        if (!g_camera_initialized) {
            g_camera.reset(scene_radius_m);
            g_camera_initialized = true;
        } else {
            g_camera.set_scene_radius(scene_radius_m);
        }
    }
    update_status();
    sync_scene_window(hwnd);
    InvalidateRect(hwnd, nullptr, TRUE);
}

RECT scene_rect(HWND hwnd)
{
    RECT client;
    GetClientRect(hwnd, &client);
    client.left += kSidebarWidth + 24;
    client.right -= 24;
    client.top += 104;
    client.bottom -= 24;
    if (client.right < client.left) {
        client.right = client.left;
    }
    if (client.bottom < client.top) {
        client.bottom = client.top;
    }
    return client;
}

void update_camera_viewport(HWND hwnd)
{
    const RECT rect = scene_rect(hwnd);
    RECT viewport = {};
    viewport.right = std::max(0L, rect.right - rect.left);
    viewport.bottom = std::max(0L, rect.bottom - rect.top);
    g_camera.set_viewport(viewport);
}

bool scene_has_state()
{
    return g_result.ok && !g_result.state_log.empty();
}

void invalidate_scene_window()
{
    if (window_is_live(g_scene_hwnd)) {
        InvalidateRect(g_scene_hwnd, nullptr, FALSE);
    }
}

void sync_scene_window(HWND hwnd)
{
    if (!window_is_live(g_scene_hwnd)) {
        update_camera_viewport(hwnd);
        return;
    }

    const RECT rect = scene_rect(hwnd);
    const int width = std::max(0L, rect.right - rect.left);
    const int height = std::max(0L, rect.bottom - rect.top);
    MoveWindow(g_scene_hwnd, rect.left, rect.top, width, height, TRUE);
    update_camera_viewport(hwnd);
    ShowWindow(g_scene_hwnd, scene_has_state() ? SW_SHOWNA : SW_HIDE);
    invalidate_scene_window();
}

void draw_text_line(HDC hdc, int x, int y, const std::string& text)
{
    const std::wstring wide = widen(text);
    TextOutW(hdc, x, y, wide.c_str(), static_cast<int>(wide.size()));
}

bool parse_port_text(const std::wstring& text, int* port)
{
    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size() || parsed < 1 || parsed > 65535) {
            return false;
        }
        *port = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_double_text(const std::wstring& text, double* value)
{
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::wstring format_double(double value)
{
    return widen(std::to_string(value));
}

void set_child_font(HWND hwnd, HFONT font)
{
    SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
}

HMENU control_id(int id)
{
    return reinterpret_cast<HMENU>(static_cast<INT_PTR>(id));
}

void create_remote_dialog_controls(RemoteSettingsDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    HWND host_label = CreateWindowExW(
        0, L"STATIC", L"Host", WS_CHILD | WS_VISIBLE,
        18, 22, 70, 20, state->hwnd, nullptr, g_instance, nullptr);
    set_child_font(host_label, font);

    state->host_edit = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"EDIT", widen(state->host).c_str(),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
        88, 18, 250, 24, state->hwnd, control_id(kRemoteHostEdit), g_instance, nullptr);
    set_child_font(state->host_edit, font);

    HWND port_label = CreateWindowExW(
        0, L"STATIC", L"Port", WS_CHILD | WS_VISIBLE,
        18, 58, 70, 20, state->hwnd, nullptr, g_instance, nullptr);
    set_child_font(port_label, font);

    state->port_edit = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"EDIT", std::to_wstring(state->port).c_str(),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL | ES_NUMBER,
        88, 54, 110, 24, state->hwnd, control_id(kRemotePortEdit), g_instance, nullptr);
    set_child_font(state->port_edit, font);

    HWND ok_button = CreateWindowExW(
        0, L"BUTTON", L"OK", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
        176, 100, 76, 28, state->hwnd, control_id(kRemoteOk), g_instance, nullptr);
    set_child_font(ok_button, font);

    HWND cancel_button = CreateWindowExW(
        0, L"BUTTON", L"Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        262, 100, 76, 28, state->hwnd, control_id(kRemoteCancel), g_instance, nullptr);
    set_child_font(cancel_button, font);
}

bool accept_remote_dialog(RemoteSettingsDialogState* state)
{
    const std::wstring host_text = get_window_text(state->host_edit);
    const std::wstring port_text = get_window_text(state->port_edit);
    const std::string host = narrow(host_text);
    int port = 0;

    if (host.empty()) {
        MessageBoxW(state->hwnd, L"Host cannot be empty.", L"Remote endpoint", MB_ICONWARNING);
        SetFocus(state->host_edit);
        return false;
    }
    if (!parse_port_text(port_text, &port)) {
        MessageBoxW(state->hwnd, L"Port must be an integer from 1 to 65535.", L"Remote endpoint", MB_ICONWARNING);
        SetFocus(state->port_edit);
        return false;
    }

    state->host = host;
    state->port = port;
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK remote_settings_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<RemoteSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }

    case WM_CREATE:
        state = reinterpret_cast<RemoteSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_remote_dialog_controls(state);
        return 0;

    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case kRemoteOk:
            accept_remote_dialog(state);
            return 0;
        case kRemoteCancel:
            DestroyWindow(hwnd);
            return 0;
        default:
            break;
        }
        break;

    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;

    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_remote_settings_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = remote_settings_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2RemoteSettingsWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_remote_settings_dialog(HWND parent)
{
    register_remote_settings_class();

    RemoteSettingsDialogState state;
    state.parent = parent;
    state.host = g_remote_host;
    state.port = g_remote_port;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 370;
    constexpr int dialog_height = 176;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2RemoteSettingsWindow",
        L"Remote endpoint",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create remote settings dialog.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.host_edit);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (state.accepted) {
        g_remote_host = state.host;
        g_remote_port = state.port;
    }

    return state.accepted;
}

post2::vehicle::TankConfig& first_tank(post2::vehicle::VehicleConfig& config)
{
    if (config.tanks.empty()) {
        config.tanks.push_back(post2::vehicle::TankConfig{});
    }
    return config.tanks.front();
}

std::vector<post2::vehicle::StageConfig>& ensure_vehicle_stages(post2::vehicle::VehicleConfig& config)
{
    if (config.stages.empty()) {
        config.stages = post2::vehicle::effective_stage_configs(config);
    }
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&config);
    return config.stages;
}

void create_label(HWND parent, int x, int y, int width, const wchar_t* text, HFONT font)
{
    HWND label = CreateWindowExW(0, L"STATIC", text, WS_CHILD | WS_VISIBLE, x, y, width, 20, parent, nullptr, g_instance, nullptr);
    set_child_font(label, font);
}

HWND create_edit(HWND parent, int id, int x, int y, int width, const std::wstring& text, HFONT font)
{
    HWND edit = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"EDIT",
        text.c_str(),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
        x,
        y,
        width,
        24,
        parent,
        control_id(id),
        g_instance,
        nullptr);
    set_child_font(edit, font);
    return edit;
}

void create_vehicle_dialog_controls(VehicleSettingsDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    ensure_vehicle_stages(state->config);

    create_label(state->hwnd, 18, 18, 90, L"Name", font);
    state->name_edit = create_edit(state->hwnd, kVehicleNameEdit, 118, 14, 332, widen(state->config.name), font);

    create_label(state->hwnd, 18, 52, 90, L"Dry mass kg", font);
    state->dry_mass_edit = create_edit(state->hwnd, kVehicleDryMassEdit, 118, 48, 120, format_double(state->config.dry_mass_kg), font);

    create_label(state->hwnd, 18, 92, 90, L"Stages", font);
    state->stage_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18,
        116,
        432,
        150,
        state->hwnd,
        control_id(kVehicleStageList),
        g_instance,
        nullptr);
    set_child_font(state->stage_list, font);
    for (std::size_t i = 0; i < state->config.stages.size(); ++i) {
        const auto& stage = state->config.stages[i];
        const std::string label = std::to_string(i + 1) + ": " + stage.name +
            (stage.active ? " | active" : " | inactive") +
            (stage.attached ? " | attached" : " | detached");
        SendMessageW(state->stage_list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(label).c_str()));
    }
    SendMessageW(state->stage_list, LB_SETCURSEL, static_cast<WPARAM>(state->selected_stage_index), 0);

    HWND add_button = CreateWindowExW(
        0, L"BUTTON", L"Add", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        18, 278, 76, 28, state->hwnd, control_id(kVehicleStageAdd), g_instance, nullptr);
    set_child_font(add_button, font);
    HWND edit_button = CreateWindowExW(
        0, L"BUTTON", L"Edit", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        108, 278, 76, 28, state->hwnd, control_id(kVehicleStageEdit), g_instance, nullptr);
    set_child_font(edit_button, font);
    HWND delete_button = CreateWindowExW(
        0, L"BUTTON", L"Delete", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        198, 278, 76, 28, state->hwnd, control_id(kVehicleStageDelete), g_instance, nullptr);
    set_child_font(delete_button, font);

    HWND ok_button = CreateWindowExW(
        0, L"BUTTON", L"OK", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
        284, 278, 76, 28, state->hwnd, control_id(IDOK), g_instance, nullptr);
    set_child_font(ok_button, font);

    HWND cancel_button = CreateWindowExW(
        0, L"BUTTON", L"Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        374, 278, 76, 28, state->hwnd, control_id(IDCANCEL), g_instance, nullptr);
    set_child_font(cancel_button, font);
}

bool read_double_field(HWND dialog, HWND edit, const wchar_t* label, const wchar_t* title, double* value)
{
    if (!parse_double_text(get_window_text(edit), value)) {
        MessageBoxW(dialog, (std::wstring(label) + L" must be a number.").c_str(), title, MB_ICONWARNING);
        SetFocus(edit);
        return false;
    }
    return true;
}

bool read_double_field(HWND dialog, HWND edit, const wchar_t* label, double* value)
{
    return read_double_field(dialog, edit, label, L"Vehicle", value);
}

bool read_int_field(HWND dialog, HWND edit, const wchar_t* label, const wchar_t* title, int* value)
{
    try {
        const std::wstring text = get_window_text(edit);
        std::size_t consumed = 0;
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            throw std::invalid_argument("trailing characters");
        }
        *value = parsed;
        return true;
    } catch (const std::exception&) {
        MessageBoxW(dialog, (std::wstring(label) + L" must be an integer.").c_str(), title, MB_ICONWARNING);
        SetFocus(edit);
        return false;
    }
}

HWND create_dynamic_edit(HWND parent, int x, int y, int width, const std::wstring& text, HFONT font)
{
    HWND edit = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"EDIT",
        text.c_str(),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
        x,
        y,
        width,
        24,
        parent,
        nullptr,
        g_instance,
        nullptr);
    set_child_font(edit, font);
    return edit;
}

HWND create_dynamic_checkbox(HWND parent, int x, int y, int width, const wchar_t* text, HFONT font)
{
    HWND checkbox = CreateWindowExW(
        0,
        L"BUTTON",
        text,
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        x,
        y,
        width,
        22,
        parent,
        nullptr,
        g_instance,
        nullptr);
    set_child_font(checkbox, font);
    return checkbox;
}

post2::core::OptimizationVariableConfig* find_optimization_variable(
    post2::core::OptimizationConfig* optimization,
    const std::string& path)
{
    if (!optimization) {
        return nullptr;
    }
    const auto found = std::find_if(
        optimization->variables.begin(),
        optimization->variables.end(),
        [&](const post2::core::OptimizationVariableConfig& variable) {
            return variable.path == path;
        });
    return found == optimization->variables.end() ? nullptr : &(*found);
}

const post2::core::OptimizationVariableConfig* find_optimization_variable(
    const post2::core::OptimizationConfig& optimization,
    const std::string& path)
{
    const auto found = std::find_if(
        optimization.variables.begin(),
        optimization.variables.end(),
        [&](const post2::core::OptimizationVariableConfig& variable) {
            return variable.path == path;
        });
    return found == optimization.variables.end() ? nullptr : &(*found);
}

void load_numeric_binding_row(
    const NumericBindingRow& row,
    const post2::core::OptimizationConfig& optimization,
    double value)
{
    SetWindowTextW(row.value_edit, format_double(value).c_str());
    if (const auto* variable = find_optimization_variable(optimization, row.path)) {
        Button_SetCheck(row.opt_check, variable->enabled ? BST_CHECKED : BST_UNCHECKED);
        SetWindowTextW(row.min_edit, format_double(variable->min_value).c_str());
        SetWindowTextW(row.max_edit, format_double(variable->max_value).c_str());
    } else {
        Button_SetCheck(row.opt_check, BST_UNCHECKED);
        SetWindowTextW(row.min_edit, L"");
        SetWindowTextW(row.max_edit, L"");
    }
}

NumericBindingRow create_numeric_binding_row(
    HWND parent,
    int y,
    const std::string& label,
    const std::string& path,
    double value,
    const post2::core::OptimizationConfig& optimization,
    HFONT font)
{
    create_label(parent, 18, y + 4, 246, widen(label).c_str(), font);
    NumericBindingRow row;
    row.path = path;
    row.label = label;
    row.value_edit = create_dynamic_edit(parent, 278, y, 100, format_double(value), font);
    row.opt_check = create_dynamic_checkbox(parent, 396, y + 1, 48, L"Opt", font);
    row.min_edit = create_dynamic_edit(parent, 456, y, 92, L"", font);
    row.max_edit = create_dynamic_edit(parent, 564, y, 92, L"", font);
    load_numeric_binding_row(row, optimization, value);
    return row;
}

bool apply_numeric_variable_controls(
    HWND dialog,
    const NumericBindingRow& row,
    double value,
    post2::core::OptimizationConfig* optimization,
    const wchar_t* title)
{
    const bool opt_enabled = Button_GetCheck(row.opt_check) == BST_CHECKED;
    auto* existing = find_optimization_variable(optimization, row.path);

    double min_value = 0.0;
    double max_value = 0.0;
    const std::wstring min_text = get_window_text(row.min_edit);
    const std::wstring max_text = get_window_text(row.max_edit);
    const bool has_minmax = !min_text.empty() || !max_text.empty();

    if (opt_enabled || has_minmax || existing) {
        if (!parse_double_text(min_text, &min_value)) {
            MessageBoxW(dialog, widen(row.label + " Min must be a number.").c_str(), title, MB_ICONWARNING);
            SetFocus(row.min_edit);
            return false;
        }
        if (!parse_double_text(max_text, &max_value)) {
            MessageBoxW(dialog, widen(row.label + " Max must be a number.").c_str(), title, MB_ICONWARNING);
            SetFocus(row.max_edit);
            return false;
        }
        if (min_value > max_value) {
            MessageBoxW(dialog, widen(row.label + " Min cannot exceed Max.").c_str(), title, MB_ICONWARNING);
            SetFocus(row.min_edit);
            return false;
        }
        if (opt_enabled && (value < min_value || value > max_value)) {
            MessageBoxW(dialog, widen(row.label + " value must be inside Min/Max.").c_str(), title, MB_ICONWARNING);
            SetFocus(row.value_edit);
            return false;
        }
    }

    if (opt_enabled || existing) {
        if (!existing) {
            optimization->variables.push_back({row.path, opt_enabled, min_value, max_value});
        } else {
            existing->enabled = opt_enabled;
            existing->min_value = min_value;
            existing->max_value = max_value;
        }
    }
    return true;
}

void refresh_vehicle_stage_list(VehicleSettingsDialogState* state)
{
    if (!state || !state->stage_list) {
        return;
    }
    ensure_vehicle_stages(state->config);
    SendMessageW(state->stage_list, LB_RESETCONTENT, 0, 0);
    for (std::size_t i = 0; i < state->config.stages.size(); ++i) {
        const auto& stage = state->config.stages[i];
        const std::string label = std::to_string(i + 1) + ": " + stage.name +
            (stage.active ? " | active" : " | inactive") +
            (stage.attached ? " | attached" : " | detached");
        SendMessageW(state->stage_list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(label).c_str()));
    }
    if (state->selected_stage_index < 0 ||
        static_cast<std::size_t>(state->selected_stage_index) >= state->config.stages.size()) {
        state->selected_stage_index = state->config.stages.empty() ? -1 : 0;
    }
    if (state->selected_stage_index >= 0) {
        SendMessageW(state->stage_list, LB_SETCURSEL, static_cast<WPARAM>(state->selected_stage_index), 0);
    }
}

void create_stage_editor_controls(StageEditorDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    const auto& engine = state->stage.engine;
    const auto& tank = state->stage.tanks.empty() ? post2::vehicle::TankConfig{} : state->stage.tanks.front();
    const std::string dry_mass_path = "vehicle.stages[" + std::to_string(state->stage_index) + "].dry_mass_kg";

    create_label(state->hwnd, 18, 18, 90, L"Stage name", font);
    state->name_edit = create_edit(state->hwnd, kVehicleNameEdit, 118, 14, 520, widen(state->stage.name), font);

    state->active_check = CreateWindowExW(
        0,
        L"BUTTON",
        L"Stage active",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        18,
        52,
        160,
        22,
        state->hwnd,
        control_id(kVehicleStageActive),
        g_instance,
        nullptr);
    set_child_font(state->active_check, font);
    Button_SetCheck(state->active_check, state->stage.active ? BST_CHECKED : BST_UNCHECKED);

    state->attached_check = CreateWindowExW(
        0,
        L"BUTTON",
        L"Stage attached",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        180,
        52,
        160,
        22,
        state->hwnd,
        nullptr,
        g_instance,
        nullptr);
    set_child_font(state->attached_check, font);
    Button_SetCheck(state->attached_check, state->stage.attached ? BST_CHECKED : BST_UNCHECKED);

    create_label(state->hwnd, 278, 82, 70, L"Value", font);
    create_label(state->hwnd, 456, 82, 70, L"Min", font);
    create_label(state->hwnd, 564, 82, 70, L"Max", font);
    NumericBindingRow dry_mass_row = create_numeric_binding_row(
        state->hwnd,
        104,
        "Dry mass kg",
        dry_mass_path,
        state->stage.dry_mass_kg,
        state->optimization ? *state->optimization : post2::core::OptimizationConfig{},
        font);
    state->dry_mass_edit = dry_mass_row.value_edit;
    state->dry_mass_opt_check = dry_mass_row.opt_check;
    state->dry_mass_min_edit = dry_mass_row.min_edit;
    state->dry_mass_max_edit = dry_mass_row.max_edit;

    state->engine_enabled = CreateWindowExW(
        0,
        L"BUTTON",
        L"Engine enabled",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        18,
        146,
        160,
        22,
        state->hwnd,
        control_id(kVehicleEngineEnabled),
        g_instance,
        nullptr);
    set_child_font(state->engine_enabled, font);
    Button_SetCheck(state->engine_enabled, engine.enabled ? BST_CHECKED : BST_UNCHECKED);

    create_label(state->hwnd, 18, 180, 90, L"Max thrust N", font);
    state->engine_thrust_edit = create_edit(state->hwnd, kVehicleEngineThrustEdit, 118, 176, 120, format_double(engine.max_thrust_n), font);

    create_label(state->hwnd, 258, 180, 55, L"Isp s", font);
    state->engine_isp_edit = create_edit(state->hwnd, kVehicleEngineIspEdit, 318, 176, 100, format_double(engine.isp_s), font);

    create_label(state->hwnd, 18, 214, 90, L"Engine dir", font);
    state->engine_dir_x_edit = create_edit(state->hwnd, kVehicleEngineDirXEdit, 118, 210, 70, format_double(engine.direction_body.x), font);
    state->engine_dir_y_edit = create_edit(state->hwnd, kVehicleEngineDirYEdit, 198, 210, 70, format_double(engine.direction_body.y), font);
    state->engine_dir_z_edit = create_edit(state->hwnd, kVehicleEngineDirZEdit, 278, 210, 70, format_double(engine.direction_body.z), font);

    create_label(state->hwnd, 18, 256, 90, L"Tank name", font);
    state->tank_name_edit = create_edit(state->hwnd, kVehicleTankNameEdit, 118, 252, 120, widen(tank.name), font);

    create_label(state->hwnd, 258, 256, 70, L"Propellant", font);
    state->tank_propellant_edit = create_edit(state->hwnd, kVehicleTankPropellantEdit, 338, 252, 80, widen(tank.propellant), font);

    create_label(state->hwnd, 18, 290, 90, L"Capacity kg", font);
    state->tank_capacity_edit = create_edit(state->hwnd, kVehicleTankCapacityEdit, 118, 286, 120, format_double(tank.capacity_kg), font);

    create_label(state->hwnd, 258, 290, 70, L"Initial kg", font);
    state->tank_initial_edit = create_edit(state->hwnd, kVehicleTankInitialEdit, 338, 286, 80, format_double(tank.initial_kg), font);

    HWND ok_button = CreateWindowExW(
        0, L"BUTTON", L"OK", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
        472, 334, 76, 28, state->hwnd, control_id(IDOK), g_instance, nullptr);
    set_child_font(ok_button, font);

    HWND cancel_button = CreateWindowExW(
        0, L"BUTTON", L"Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        562, 334, 76, 28, state->hwnd, control_id(IDCANCEL), g_instance, nullptr);
    set_child_font(cancel_button, font);
}

bool accept_stage_editor_dialog(StageEditorDialogState* state)
{
    post2::vehicle::StageConfig stage = state->stage;
    stage.name = narrow(get_window_text(state->name_edit));
    stage.active = Button_GetCheck(state->active_check) == BST_CHECKED;
    stage.attached = Button_GetCheck(state->attached_check) == BST_CHECKED;
    stage.engine.enabled = Button_GetCheck(state->engine_enabled) == BST_CHECKED;

    if (!read_double_field(state->hwnd, state->dry_mass_edit, L"Dry mass", &stage.dry_mass_kg) ||
        !read_double_field(state->hwnd, state->engine_thrust_edit, L"Max thrust", &stage.engine.max_thrust_n) ||
        !read_double_field(state->hwnd, state->engine_isp_edit, L"Isp", &stage.engine.isp_s) ||
        !read_double_field(state->hwnd, state->engine_dir_x_edit, L"Engine dir X", &stage.engine.direction_body.x) ||
        !read_double_field(state->hwnd, state->engine_dir_y_edit, L"Engine dir Y", &stage.engine.direction_body.y) ||
        !read_double_field(state->hwnd, state->engine_dir_z_edit, L"Engine dir Z", &stage.engine.direction_body.z)) {
        return false;
    }
    if (stage.dry_mass_kg < 0.0) {
        MessageBoxW(state->hwnd, L"Dry mass cannot be negative.", L"Stage", MB_ICONWARNING);
        SetFocus(state->dry_mass_edit);
        return false;
    }
    NumericBindingRow dry_mass_row;
    dry_mass_row.path = "vehicle.stages[" + std::to_string(state->stage_index) + "].dry_mass_kg";
    dry_mass_row.label = "Dry mass kg";
    dry_mass_row.value_edit = state->dry_mass_edit;
    dry_mass_row.opt_check = state->dry_mass_opt_check;
    dry_mass_row.min_edit = state->dry_mass_min_edit;
    dry_mass_row.max_edit = state->dry_mass_max_edit;
    if (!apply_numeric_variable_controls(state->hwnd, dry_mass_row, stage.dry_mass_kg, state->optimization, L"Stage")) {
        return false;
    }

    if (stage.tanks.empty()) {
        stage.tanks.push_back(post2::vehicle::TankConfig{});
    }
    auto& tank = stage.tanks.front();
    tank.name = narrow(get_window_text(state->tank_name_edit));
    tank.propellant = narrow(get_window_text(state->tank_propellant_edit));
    if (!read_double_field(state->hwnd, state->tank_capacity_edit, L"Tank capacity", &tank.capacity_kg) ||
        !read_double_field(state->hwnd, state->tank_initial_edit, L"Tank initial", &tank.initial_kg)) {
        return false;
    }

    post2::vehicle::VehicleConfig probe = post2::vehicle::default_vehicle_config();
    probe.stages = {stage};
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&probe);
    std::string error;
    if (!post2::vehicle::validate_vehicle_config(probe, &error)) {
        MessageBoxW(state->hwnd, widen(error).c_str(), L"Stage", MB_ICONWARNING);
        return false;
    }

    state->stage = std::move(stage);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK stage_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<StageEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<StageEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_stage_editor_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_stage_editor_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_stage_editor_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = stage_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2StageEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_stage_editor_dialog(
    HWND parent,
    post2::vehicle::StageConfig* stage,
    post2::core::OptimizationConfig* optimization,
    int stage_index)
{
    register_stage_editor_class();

    StageEditorDialogState state;
    state.stage = *stage;
    state.optimization = optimization;
    state.stage_index = stage_index;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 690;
    constexpr int dialog_height = 420;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2StageEditorWindow",
        L"Edit stage",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create stage editor.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.name_edit);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (state.accepted) {
        *stage = std::move(state.stage);
    }

    return state.accepted;
}

void create_launch_settings_controls(LaunchSettingsDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 18, 100, L"Latitude deg", font);
    state->latitude_edit = create_edit(
        state->hwnd,
        kLaunchLatitudeEdit,
        138,
        14,
        120,
        format_double(state->config.launch_site.latitude_deg),
        font);

    create_label(state->hwnd, 18, 52, 100, L"Longitude deg", font);
    state->longitude_edit = create_edit(
        state->hwnd,
        kLaunchLongitudeEdit,
        138,
        48,
        120,
        format_double(state->config.launch_site.longitude_deg),
        font);

    create_label(state->hwnd, 18, 86, 100, L"Altitude m", font);
    state->altitude_edit = create_edit(
        state->hwnd,
        kLaunchAltitudeEdit,
        138,
        82,
        120,
        format_double(state->config.launch_site.altitude_m),
        font);

    state->hold_down_clamp_enabled = CreateWindowExW(
        0,
        L"BUTTON",
        L"Hold Down Clamp enabled",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        18,
        124,
        210,
        22,
        state->hwnd,
        control_id(kLaunchHoldDownClampEnabled),
        g_instance,
        nullptr);
    set_child_font(state->hold_down_clamp_enabled, font);
    Button_SetCheck(
        state->hold_down_clamp_enabled,
        state->config.hold_down_clamp.enabled ? BST_CHECKED : BST_UNCHECKED);

    create_label(state->hwnd, 18, 158, 110, L"Release time s", font);
    state->hold_down_clamp_release_edit = create_edit(
        state->hwnd,
        kLaunchHoldDownClampReleaseEdit,
        138,
        154,
        120,
        format_double(state->config.hold_down_clamp.release_time_s),
        font);

    state->normal_force_enabled = CreateWindowExW(
        0,
        L"BUTTON",
        L"Normal force enabled",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        18,
        196,
        190,
        22,
        state->hwnd,
        control_id(kLaunchNormalForceEnabled),
        g_instance,
        nullptr);
    set_child_font(state->normal_force_enabled, font);
    Button_SetCheck(state->normal_force_enabled, state->config.normal_force.enabled ? BST_CHECKED : BST_UNCHECKED);

    HWND ok_button = CreateWindowExW(
        0, L"BUTTON", L"OK", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
        226, 238, 76, 28, state->hwnd, control_id(IDOK), g_instance, nullptr);
    set_child_font(ok_button, font);

    HWND cancel_button = CreateWindowExW(
        0, L"BUTTON", L"Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        316, 238, 76, 28, state->hwnd, control_id(IDCANCEL), g_instance, nullptr);
    set_child_font(cancel_button, font);
}

bool validate_launch_settings(HWND dialog, const SimulationConfig& config)
{
    if (config.launch_site.latitude_deg < -90.0 || config.launch_site.latitude_deg > 90.0) {
        MessageBoxW(dialog, L"Latitude must be in [-90, 90].", L"Launch site", MB_ICONWARNING);
        return false;
    }
    if (config.launch_site.altitude_m <= -config.earth_radius_m) {
        MessageBoxW(dialog, L"Altitude is below the planet center.", L"Launch site", MB_ICONWARNING);
        return false;
    }
    if (config.hold_down_clamp.release_time_s < 0.0) {
        MessageBoxW(dialog, L"Release time cannot be negative.", L"Launch site", MB_ICONWARNING);
        return false;
    }
    return true;
}

bool accept_launch_settings_dialog(LaunchSettingsDialogState* state)
{
    SimulationConfig config = state->config;
    if (!read_double_field(state->hwnd, state->latitude_edit, L"Latitude", L"Launch site", &config.launch_site.latitude_deg) ||
        !read_double_field(state->hwnd, state->longitude_edit, L"Longitude", L"Launch site", &config.launch_site.longitude_deg) ||
        !read_double_field(state->hwnd, state->altitude_edit, L"Altitude", L"Launch site", &config.launch_site.altitude_m) ||
        !read_double_field(
            state->hwnd,
            state->hold_down_clamp_release_edit,
            L"Release time",
            L"Launch site",
            &config.hold_down_clamp.release_time_s)) {
        return false;
    }

    config.hold_down_clamp.enabled = Button_GetCheck(state->hold_down_clamp_enabled) == BST_CHECKED;
    config.normal_force.enabled = Button_GetCheck(state->normal_force_enabled) == BST_CHECKED;

    if (!validate_launch_settings(state->hwnd, config)) {
        return false;
    }

    state->config = config;
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK launch_settings_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<LaunchSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<LaunchSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_launch_settings_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_launch_settings_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_launch_settings_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = launch_settings_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2LaunchSettingsWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_launch_settings_dialog(HWND parent)
{
    sync_legacy_from_case();
    register_launch_settings_class();

    LaunchSettingsDialogState state;
    state.config = g_config;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 430;
    constexpr int dialog_height = 320;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2LaunchSettingsWindow",
        L"Launch site",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create launch settings dialog.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.latitude_edit);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (state.accepted) {
        g_config.launch_site = state.config.launch_site;
        g_config.hold_down_clamp = state.config.hold_down_clamp;
        g_config.normal_force = state.config.normal_force;
        sync_case_surface_from_legacy();
    }

    return state.accepted;
}

bool accept_vehicle_dialog(VehicleSettingsDialogState* state)
{
    post2::vehicle::VehicleConfig config = state->config;
    config.name = narrow(get_window_text(state->name_edit));
    if (!read_double_field(state->hwnd, state->dry_mass_edit, L"Dry mass", &config.dry_mass_kg)) {
        return false;
    }
    ensure_vehicle_stages(config);
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&config);

    std::string error;
    if (!post2::vehicle::validate_vehicle_config(config, &error)) {
        MessageBoxW(state->hwnd, widen(error).c_str(), L"Vehicle", MB_ICONWARNING);
        return false;
    }

    state->config = std::move(config);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

void add_vehicle_stage(VehicleSettingsDialogState* state)
{
    auto& stages = ensure_vehicle_stages(state->config);
    post2::vehicle::StageConfig stage;
    stage.name = "stage " + std::to_string(stages.size() + 1);
    stage.active = stages.empty();
    if (!stages.empty()) {
        stage.engine = stages.back().engine;
        stage.tanks = stages.back().tanks;
    }
    const int stage_index = static_cast<int>(stages.size());
    if (!show_stage_editor_dialog(state->hwnd, &stage, &state->optimization, stage_index)) {
        return;
    }
    stages.push_back(std::move(stage));
    state->selected_stage_index = static_cast<int>(stages.size() - 1);
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&state->config);
    refresh_vehicle_stage_list(state);
}

void edit_vehicle_stage(VehicleSettingsDialogState* state)
{
    auto& stages = ensure_vehicle_stages(state->config);
    if (state->selected_stage_index < 0 ||
        static_cast<std::size_t>(state->selected_stage_index) >= stages.size()) {
        return;
    }
    if (show_stage_editor_dialog(
            state->hwnd,
            &stages[static_cast<std::size_t>(state->selected_stage_index)],
            &state->optimization,
            state->selected_stage_index)) {
        post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&state->config);
        refresh_vehicle_stage_list(state);
    }
}

void delete_vehicle_stage(VehicleSettingsDialogState* state)
{
    auto& stages = ensure_vehicle_stages(state->config);
    if (stages.size() <= 1) {
        MessageBoxW(state->hwnd, L"A vehicle must keep at least one stage.", L"Vehicle", MB_ICONWARNING);
        return;
    }
    if (state->selected_stage_index < 0 ||
        static_cast<std::size_t>(state->selected_stage_index) >= stages.size()) {
        return;
    }
    stages.erase(stages.begin() + state->selected_stage_index);
    if (static_cast<std::size_t>(state->selected_stage_index) >= stages.size()) {
        state->selected_stage_index = static_cast<int>(stages.size() - 1);
    }
    post2::vehicle::sync_legacy_vehicle_fields_from_first_stage(&state->config);
    refresh_vehicle_stage_list(state);
}

LRESULT CALLBACK vehicle_settings_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<VehicleSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<VehicleSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_vehicle_dialog_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_vehicle_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kVehicleStageList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const LRESULT selected = SendMessageW(state->stage_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    state->selected_stage_index = static_cast<int>(selected);
                }
                return 0;
            }
            break;
        case kVehicleStageAdd:
            add_vehicle_stage(state);
            return 0;
        case kVehicleStageEdit:
            edit_vehicle_stage(state);
            return 0;
        case kVehicleStageDelete:
            delete_vehicle_stage(state);
            return 0;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_vehicle_settings_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = vehicle_settings_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2VehicleSettingsWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_vehicle_settings_dialog(HWND parent)
{
    ensure_case_initialized();
    register_vehicle_settings_class();

    VehicleSettingsDialogState state;
    state.config = g_case.vehicle;
    state.optimization = g_case.optimization;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 462;
    constexpr int dialog_height = 360;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2VehicleSettingsWindow",
        L"Vehicle config",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create vehicle dialog.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.name_edit);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (state.accepted) {
        g_config.vehicle = state.config;
        g_case.vehicle = state.config;
        g_case.optimization = state.optimization;
    }

    return state.accepted;
}

bool choose_case_config_file(HWND parent, bool save, std::string* path)
{
    std::array<wchar_t, MAX_PATH> buffer = {};
    wcscpy_s(buffer.data(), buffer.size(), L"case.json");

    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = parent;
    ofn.lpstrFilter = L"Case JSON (*.json)\0*.json\0All files (*.*)\0*.*\0";
    ofn.lpstrFile = buffer.data();
    ofn.nMaxFile = static_cast<DWORD>(buffer.size());
    ofn.lpstrDefExt = L"json";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | (save ? OFN_OVERWRITEPROMPT : OFN_FILEMUSTEXIST);

    const BOOL ok = save ? GetSaveFileNameW(&ofn) : GetOpenFileNameW(&ofn);
    if (!ok) {
        return false;
    }

    *path = narrow(buffer.data());
    return true;
}

void load_case_config(HWND hwnd)
{
    std::string path;
    if (!choose_case_config_file(hwnd, false, &path)) {
        return;
    }

    CaseConfig config;
    std::string error;
    if (!post2::core::load_case_config_file(path, &config, &error)) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Case load failed", MB_ICONERROR);
        return;
    }

    g_case = std::move(config);
    g_case_initialized = true;
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
    run_simulation(hwnd);
}

void save_case_config(HWND hwnd)
{
    ensure_case_initialized();
    std::string path;
    if (!choose_case_config_file(hwnd, true, &path)) {
        return;
    }

    std::string error;
    if (!post2::core::save_case_config_file(path, g_case, &error)) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Case save failed", MB_ICONERROR);
        return;
    }

    MessageBoxW(hwnd, widen("Wrote " + path).c_str(), L"POST2 Lite", MB_ICONINFORMATION);
}

void create_case_json_controls(CaseJsonDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    const std::wstring json_text = widen(post2::core::case_config_to_json(state->config));

    state->json_edit = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"EDIT",
        json_text.c_str(),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_MULTILINE | ES_AUTOVSCROLL | ES_AUTOHSCROLL |
            WS_VSCROLL | WS_HSCROLL,
        18,
        18,
        742,
        456,
        state->hwnd,
        nullptr,
        g_instance,
        nullptr);
    set_child_font(state->json_edit, font);

    HWND ok_button = CreateWindowExW(
        0, L"BUTTON", L"OK", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,
        594, 492, 76, 28, state->hwnd, control_id(IDOK), g_instance, nullptr);
    set_child_font(ok_button, font);

    HWND cancel_button = CreateWindowExW(
        0, L"BUTTON", L"Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        684, 492, 76, 28, state->hwnd, control_id(IDCANCEL), g_instance, nullptr);
    set_child_font(cancel_button, font);
}

bool accept_case_json_dialog(CaseJsonDialogState* state)
{
    CaseConfig parsed;
    std::string error;
    if (!post2::core::case_config_from_json(narrow(get_window_text(state->json_edit)), &parsed, &error)) {
        MessageBoxW(state->hwnd, widen(error).c_str(), L"Case JSON", MB_ICONWARNING);
        SetFocus(state->json_edit);
        return false;
    }

    state->config = std::move(parsed);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK case_json_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<CaseJsonDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<CaseJsonDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_case_json_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_case_json_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_case_json_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = case_json_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2CaseJsonWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_case_phases_dialog(HWND parent)
{
    ensure_case_initialized();
    register_case_json_class();

    CaseJsonDialogState state;
    state.config = g_case;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 800;
    constexpr int dialog_height = 580;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2CaseJsonWindow",
        L"Case / Phases",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create case dialog.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.json_edit);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (state.accepted) {
        g_case = std::move(state.config);
        g_case_initialized = true;
        sync_legacy_from_case();
    }

    return state.accepted;
}

void add_metric_items(HWND combo)
{
    add_combo_item(combo, L"terminal_altitude_m");
    add_combo_item(combo, L"terminal_speed_mps");
    add_combo_item(combo, L"inclination_deg");
    add_combo_item(combo, L"periapsis_altitude_m");
    add_combo_item(combo, L"payload_mass_kg");
    if (!g_case_initialized) {
        return;
    }
    const char* phase_metrics[] = {
        "terminal_altitude_m",
        "terminal_speed_mps",
        "inclination_deg",
        "periapsis_altitude_m",
    };
    for (std::size_t phase_index = 0; phase_index < g_case.phases.size(); ++phase_index) {
        for (const char* metric : phase_metrics) {
            std::ostringstream item;
            item << "phases[" << phase_index << "]." << metric;
            const std::wstring wide = widen(item.str());
            add_combo_item(combo, wide.c_str());
        }
    }
}

void refresh_optimization_target_list(OptimizationSettingsDialogState* state)
{
    SendMessageW(state->target_list, LB_RESETCONTENT, 0, 0);
    for (std::size_t i = 0; i < state->config.targets.size(); ++i) {
        const auto& target = state->config.targets[i];
        std::ostringstream label;
        label << i + 1 << ": " << target.metric << " " << target.mode;
        SendMessageW(state->target_list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(label.str()).c_str()));
    }
    if (state->config.targets.empty()) {
        state->selected_target_index = -1;
    } else {
        if (state->selected_target_index < 0 ||
            static_cast<std::size_t>(state->selected_target_index) >= state->config.targets.size()) {
            state->selected_target_index = 0;
        }
        SendMessageW(state->target_list, LB_SETCURSEL, static_cast<WPARAM>(state->selected_target_index), 0);
    }
}

void load_selected_optimization_target(OptimizationSettingsDialogState* state)
{
    const bool has_target =
        state->selected_target_index >= 0 &&
        static_cast<std::size_t>(state->selected_target_index) < state->config.targets.size();
    EnableWindow(state->target_metric_combo, has_target);
    EnableWindow(state->target_mode_combo, has_target);
    EnableWindow(state->target_value_edit, has_target);
    EnableWindow(state->target_min_edit, has_target);
    EnableWindow(state->target_max_edit, has_target);
    EnableWindow(state->target_weight_edit, has_target);
    if (!has_target) {
        SendMessageW(state->target_metric_combo, CB_SETCURSEL, 0, 0);
        SendMessageW(state->target_mode_combo, CB_SETCURSEL, 0, 0);
        SetWindowTextW(state->target_value_edit, L"");
        SetWindowTextW(state->target_min_edit, L"");
        SetWindowTextW(state->target_max_edit, L"");
        SetWindowTextW(state->target_weight_edit, L"");
        return;
    }

    const auto& target = state->config.targets[static_cast<std::size_t>(state->selected_target_index)];
    select_combo_text(state->target_metric_combo, target.metric);
    select_combo_text(state->target_mode_combo, target.mode);
    SetWindowTextW(state->target_value_edit, format_double(target.value).c_str());
    SetWindowTextW(state->target_min_edit, format_double(target.min_value).c_str());
    SetWindowTextW(state->target_max_edit, format_double(target.max_value).c_str());
    SetWindowTextW(state->target_weight_edit, format_double(target.weight).c_str());
}

bool save_selected_optimization_target(OptimizationSettingsDialogState* state)
{
    if (state->selected_target_index < 0 ||
        static_cast<std::size_t>(state->selected_target_index) >= state->config.targets.size()) {
        return true;
    }

    auto target = state->config.targets[static_cast<std::size_t>(state->selected_target_index)];
    target.metric = get_combo_text(state->target_metric_combo);
    target.mode = get_combo_text(state->target_mode_combo);
    if (!read_double_field(state->hwnd, state->target_value_edit, L"Target value", L"Optimize", &target.value) ||
        !read_double_field(state->hwnd, state->target_min_edit, L"Target min", L"Optimize", &target.min_value) ||
        !read_double_field(state->hwnd, state->target_max_edit, L"Target max", L"Optimize", &target.max_value) ||
        !read_double_field(state->hwnd, state->target_weight_edit, L"Target weight", L"Optimize", &target.weight)) {
        return false;
    }
    if (target.mode == "range" && target.min_value > target.max_value) {
        MessageBoxW(state->hwnd, L"Target min cannot exceed max.", L"Optimize", MB_ICONWARNING);
        SetFocus(state->target_min_edit);
        return false;
    }
    state->config.targets[static_cast<std::size_t>(state->selected_target_index)] = std::move(target);
    return true;
}

void create_optimization_settings_controls(OptimizationSettingsDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 18, 52, L"Mode", font);
    state->mode_combo = create_combo(state->hwnd, kOptModeCombo, 82, 14, 122, font);
    add_combo_item(state->mode_combo, L"target");
    add_combo_item(state->mode_combo, L"optimize");
    select_combo_text(state->mode_combo, state->config.mode);

    create_label(state->hwnd, 230, 18, 70, L"Optimizer", font);
    state->optimizer_combo = create_combo(state->hwnd, kOptOptimizerCombo, 312, 14, 170, font);
    add_combo_item(state->optimizer_combo, L"fmincon");
    add_combo_item(state->optimizer_combo, L"sqp");
    select_combo_text(state->optimizer_combo, state->config.optimizer);

    create_label(state->hwnd, 18, 58, 90, L"Max iter", font);
    state->max_iterations_edit = create_edit(
        state->hwnd, kOptMaxIterationsEdit, 112, 54, 92, widen(std::to_string(state->config.max_iterations)), font);
    create_label(state->hwnd, 230, 58, 74, L"Tolerance", font);
    state->tolerance_edit = create_edit(
        state->hwnd, kOptToleranceEdit, 312, 54, 92, format_double(state->config.tolerance), font);
    create_label(state->hwnd, 430, 58, 92, L"Step frac", font);
    state->step_fraction_edit = create_edit(
        state->hwnd, kOptStepFractionEdit, 526, 54, 92, format_double(state->config.initial_step_fraction), font);

    create_label(state->hwnd, 18, 102, 80, L"Targets", font);
    state->target_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18,
        128,
        252,
        190,
        state->hwnd,
        control_id(kOptTargetList),
        g_instance,
        nullptr);
    set_child_font(state->target_list, font);
    create_button(state->hwnd, kOptTargetAdd, 18, 330, 70, 26, L"Add", font);
    create_button(state->hwnd, kOptTargetDelete, 102, 330, 70, 26, L"Delete", font);

    create_label(state->hwnd, 298, 132, 60, L"Metric", font);
    state->target_metric_combo = create_combo(state->hwnd, kOptTargetMetricCombo, 384, 128, 204, font);
    add_metric_items(state->target_metric_combo);
    create_label(state->hwnd, 298, 170, 60, L"Mode", font);
    state->target_mode_combo = create_combo(state->hwnd, kOptTargetModeCombo, 384, 166, 120, font);
    add_combo_item(state->target_mode_combo, L"equal");
    add_combo_item(state->target_mode_combo, L"range");

    create_label(state->hwnd, 298, 208, 60, L"Value", font);
    state->target_value_edit = create_edit(state->hwnd, kOptTargetValueEdit, 384, 204, 120, L"", font);
    create_label(state->hwnd, 298, 246, 60, L"Min", font);
    state->target_min_edit = create_edit(state->hwnd, kOptTargetMinEdit, 384, 242, 120, L"", font);
    create_label(state->hwnd, 526, 246, 42, L"Max", font);
    state->target_max_edit = create_edit(state->hwnd, kOptTargetMaxEdit, 574, 242, 120, L"", font);
    create_label(state->hwnd, 298, 284, 60, L"Weight", font);
    state->target_weight_edit = create_edit(state->hwnd, kOptTargetWeightEdit, 384, 280, 120, L"", font);

    state->objective_enabled = create_checkbox(state->hwnd, kOptObjectiveEnabled, 18, 388, 120, L"Objective", font);
    Button_SetCheck(state->objective_enabled, state->config.objective.enabled ? BST_CHECKED : BST_UNCHECKED);
    create_label(state->hwnd, 158, 390, 54, L"Metric", font);
    state->objective_metric_combo = create_combo(state->hwnd, kOptObjectiveMetricCombo, 218, 386, 190, font);
    add_metric_items(state->objective_metric_combo);
    select_combo_text(state->objective_metric_combo, state->config.objective.metric);
    create_label(state->hwnd, 430, 390, 70, L"Direction", font);
    state->objective_direction_combo = create_combo(state->hwnd, kOptObjectiveDirectionCombo, 506, 386, 120, font);
    add_combo_item(state->objective_direction_combo, L"minimize");
    add_combo_item(state->objective_direction_combo, L"maximize");
    select_combo_text(state->objective_direction_combo, state->config.objective.direction);
    create_label(state->hwnd, 18, 430, 54, L"Weight", font);
    state->objective_weight_edit = create_edit(
        state->hwnd, kOptObjectiveWeightEdit, 82, 426, 120, format_double(state->config.objective.weight), font);

    HWND ok_button = create_button(state->hwnd, IDOK, 542, 478, 76, 28, L"OK", font);
    SendMessageW(ok_button, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 632, 478, 76, 28, L"Cancel", font);

    refresh_optimization_target_list(state);
    load_selected_optimization_target(state);
}

bool accept_optimization_settings_dialog(OptimizationSettingsDialogState* state)
{
    if (!save_selected_optimization_target(state)) {
        return false;
    }

    state->config.mode = get_combo_text(state->mode_combo);
    state->config.optimizer = get_combo_text(state->optimizer_combo);
    if (!read_int_field(state->hwnd, state->max_iterations_edit, L"Max iterations", L"Optimize", &state->config.max_iterations) ||
        !read_double_field(state->hwnd, state->tolerance_edit, L"Tolerance", L"Optimize", &state->config.tolerance) ||
        !read_double_field(
            state->hwnd,
            state->step_fraction_edit,
            L"Initial step fraction",
            L"Optimize",
            &state->config.initial_step_fraction) ||
        !read_double_field(
            state->hwnd,
            state->objective_weight_edit,
            L"Objective weight",
            L"Optimize",
            &state->config.objective.weight)) {
        return false;
    }
    state->config.objective.enabled = Button_GetCheck(state->objective_enabled) == BST_CHECKED;
    state->config.objective.metric = get_combo_text(state->objective_metric_combo);
    state->config.objective.direction = get_combo_text(state->objective_direction_combo);
    if (state->config.max_iterations <= 0) {
        MessageBoxW(state->hwnd, L"Max iterations must be positive.", L"Optimize", MB_ICONWARNING);
        return false;
    }
    if (state->config.tolerance <= 0.0 || state->config.initial_step_fraction <= 0.0) {
        MessageBoxW(state->hwnd, L"Tolerance and step fraction must be positive.", L"Optimize", MB_ICONWARNING);
        return false;
    }

    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK optimization_settings_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<OptimizationSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<OptimizationSettingsDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_optimization_settings_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_optimization_settings_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kOptTargetList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const int previous = state->selected_target_index;
                const LRESULT selected = SendMessageW(state->target_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    if (!save_selected_optimization_target(state)) {
                        SendMessageW(state->target_list, LB_SETCURSEL, static_cast<WPARAM>(previous), 0);
                        return 0;
                    }
                    state->selected_target_index = static_cast<int>(selected);
                    load_selected_optimization_target(state);
                }
                return 0;
            }
            break;
        case kOptTargetAdd:
            if (!save_selected_optimization_target(state)) {
                return 0;
            }
            state->config.targets.push_back({"terminal_altitude_m", "equal", 200000.0, 0.0, 0.0, 1.0});
            state->selected_target_index = static_cast<int>(state->config.targets.size() - 1);
            refresh_optimization_target_list(state);
            load_selected_optimization_target(state);
            return 0;
        case kOptTargetDelete:
            if (state->selected_target_index >= 0 &&
                static_cast<std::size_t>(state->selected_target_index) < state->config.targets.size()) {
                state->config.targets.erase(state->config.targets.begin() + state->selected_target_index);
                if (static_cast<std::size_t>(state->selected_target_index) >= state->config.targets.size()) {
                    state->selected_target_index = static_cast<int>(state->config.targets.size()) - 1;
                }
                refresh_optimization_target_list(state);
                load_selected_optimization_target(state);
            }
            return 0;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_optimization_settings_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = optimization_settings_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2OptimizationSettingsWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_optimization_settings_dialog(HWND parent)
{
    ensure_case_initialized();
    register_optimization_settings_class();

    OptimizationSettingsDialogState state;
    state.config = g_case.optimization;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 740;
    constexpr int dialog_height = 560;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2OptimizationSettingsWindow",
        L"Optimization Settings",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create optimization settings dialog.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.mode_combo);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (state.accepted) {
        g_case.optimization = std::move(state.config);
    }

    return state.accepted;
}

std::string trim_ascii(std::string text)
{
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

HWND create_button(HWND parent, int id, int x, int y, int width, int height, const wchar_t* text, HFONT font)
{
    HWND button = CreateWindowExW(
        0,
        L"BUTTON",
        text,
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
        x,
        y,
        width,
        height,
        parent,
        control_id(id),
        g_instance,
        nullptr);
    set_child_font(button, font);
    return button;
}

HWND create_checkbox(HWND parent, int id, int x, int y, int width, const wchar_t* text, HFONT font)
{
    HWND checkbox = CreateWindowExW(
        0,
        L"BUTTON",
        text,
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
        x,
        y,
        width,
        22,
        parent,
        control_id(id),
        g_instance,
        nullptr);
    set_child_font(checkbox, font);
    return checkbox;
}

HWND create_combo(HWND parent, int id, int x, int y, int width, HFONT font)
{
    HWND combo = CreateWindowExW(
        0,
        L"COMBOBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST,
        x,
        y,
        width,
        160,
        parent,
        control_id(id),
        g_instance,
        nullptr);
    set_child_font(combo, font);
    return combo;
}

void add_combo_item(HWND combo, const wchar_t* text)
{
    SendMessageW(combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(text));
}

void select_combo_text(HWND combo, const std::string& text)
{
    const std::wstring wide = widen(text);
    const LRESULT index = SendMessageW(combo, CB_FINDSTRINGEXACT, static_cast<WPARAM>(-1), reinterpret_cast<LPARAM>(wide.c_str()));
    SendMessageW(combo, CB_SETCURSEL, index == CB_ERR ? 0 : index, 0);
}

std::string get_combo_text(HWND combo)
{
    const LRESULT index = SendMessageW(combo, CB_GETCURSEL, 0, 0);
    if (index == CB_ERR) {
        return "";
    }
    const LRESULT length = SendMessageW(combo, CB_GETLBTEXTLEN, static_cast<WPARAM>(index), 0);
    if (length <= 0) {
        return "";
    }
    std::wstring text(static_cast<std::size_t>(length + 1), L'\0');
    SendMessageW(combo, CB_GETLBTEXT, static_cast<WPARAM>(index), reinterpret_cast<LPARAM>(text.data()));
    text.resize(static_cast<std::size_t>(length));
    return narrow(text);
}

std::string action_type_display_label(const std::string& type)
{
    if (type == "set_engine_enabled") {
        return "Engine enabled";
    }
    if (type == "set_hold_down_clamp_active") {
        return "Hold-down clamp active";
    }
    if (type == "set_stage_active") {
        return "Stage active";
    }
    if (type == "set_stage_attached") {
        return "Stage attached";
    }
    return type;
}

std::string action_type_from_display_label(const std::string& label)
{
    if (label == "Engine enabled" || label == "set_engine_enabled") {
        return "set_engine_enabled";
    }
    if (label == "Hold-down clamp active" || label == "set_hold_down_clamp_active") {
        return "set_hold_down_clamp_active";
    }
    if (label == "Stage active" || label == "set_stage_active") {
        return "set_stage_active";
    }
    if (label == "Stage attached" || label == "set_stage_attached") {
        return "set_stage_attached";
    }
    return label;
}

std::string action_state_display_label(const std::string& type, bool value)
{
    if (type == "set_engine_enabled") {
        return value ? "enabled" : "disabled";
    }
    return value ? "active" : "inactive";
}

bool action_state_from_display_label(const std::string& type, const std::string& label)
{
    std::string normalized = label;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (normalized == "enabled" ||
        normalized == "active" ||
        normalized == "true" ||
        normalized == "on" ||
        normalized == "1") {
        return true;
    }
    if (normalized == "disabled" ||
        normalized == "inactive" ||
        normalized == "false" ||
        normalized == "off" ||
        normalized == "0") {
        return false;
    }
    return type == "set_engine_enabled" ? label != "disabled" : label != "inactive";
}

void populate_action_value_combo(const std::string& type, bool value)
{
    if (!window_is_live(g_phase_action_value)) {
        return;
    }
    SendMessageW(g_phase_action_value, CB_RESETCONTENT, 0, 0);
    if (type == "set_engine_enabled") {
        add_combo_item(g_phase_action_value, L"enabled");
        add_combo_item(g_phase_action_value, L"disabled");
    } else {
        add_combo_item(g_phase_action_value, L"active");
        add_combo_item(g_phase_action_value, L"inactive");
    }
    select_combo_text(g_phase_action_value, action_state_display_label(type, value));
}

post2::core::PhaseConfig* selected_phase()
{
    ensure_case_initialized();
    if (g_case.phases.empty()) {
        g_case.phases.push_back(post2::core::PhaseConfig{});
    }
    if (g_selected_phase_index < 0 || static_cast<std::size_t>(g_selected_phase_index) >= g_case.phases.size()) {
        g_selected_phase_index = 0;
    }
    return &g_case.phases[static_cast<std::size_t>(g_selected_phase_index)];
}

std::string action_label(const post2::core::PhaseAction& action)
{
    std::string name = action.type == "set_engine_enabled"
        ? "engine"
        : (action.type == "set_stage_active"
                ? "stage"
                : (action.type == "set_stage_attached" ? "stage attach" : "hold-down clamp"));
    if (action.type == "set_stage_active" || action.type == "set_stage_attached") {
        const std::string stage = action.stage_name.empty()
            ? ("#" + std::to_string(action.stage_index + 1))
            : action.stage_name;
        name += " " + stage;
    }
    return "t=" + std::to_string(action.time_s) + "  " + name + " -> " +
        action_state_display_label(action.type, action.value);
}

void populate_action_stage_combo()
{
    if (!window_is_live(g_phase_action_stage)) {
        return;
    }
    SendMessageW(g_phase_action_stage, CB_RESETCONTENT, 0, 0);
    const auto stages = post2::vehicle::effective_stage_configs(g_case.vehicle);
    for (std::size_t i = 0; i < stages.size(); ++i) {
        const std::string label = std::to_string(i + 1) + ": " + stages[i].name;
        SendMessageW(g_phase_action_stage, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(label).c_str()));
    }
    if (!stages.empty()) {
        SendMessageW(g_phase_action_stage, CB_SETCURSEL, 0, 0);
    }
}

void refresh_action_list()
{
    if (!window_is_live(g_phase_action_list)) {
        return;
    }

    const auto* phase = selected_phase();
    SendMessageW(g_phase_action_list, LB_RESETCONTENT, 0, 0);
    for (const auto& action : phase->actions) {
        SendMessageW(g_phase_action_list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(action_label(action)).c_str()));
    }
    if (phase->actions.empty()) {
        g_selected_action_index = -1;
    } else {
        if (g_selected_action_index < 0 ||
            static_cast<std::size_t>(g_selected_action_index) >= phase->actions.size()) {
            g_selected_action_index = 0;
        }
        SendMessageW(g_phase_action_list, LB_SETCURSEL, static_cast<WPARAM>(g_selected_action_index), 0);
    }
}

void load_selected_action_controls()
{
    if (!window_is_live(g_phase_action_type)) {
        return;
    }

    const auto* phase = selected_phase();
    const bool has_action =
        g_selected_action_index >= 0 &&
        static_cast<std::size_t>(g_selected_action_index) < phase->actions.size();

    EnableWindow(g_phase_action_delete_button, has_action);
    EnableWindow(g_phase_action_type, has_action);
    EnableWindow(g_phase_action_value, has_action);
    EnableWindow(g_phase_action_stage, false);

    if (!has_action) {
        SendMessageW(g_phase_action_type, CB_SETCURSEL, 0, 0);
        populate_action_value_combo("set_stage_active", true);
        SendMessageW(g_phase_action_stage, CB_SETCURSEL, 0, 0);
        return;
    }

    const auto& action = phase->actions[static_cast<std::size_t>(g_selected_action_index)];
    select_combo_text(g_phase_action_type, action_type_display_label(action.type));
    populate_action_value_combo(action.type, action.value);
    EnableWindow(g_phase_action_stage, action.type == "set_stage_active" || action.type == "set_stage_attached");
    const int stage_index = action.stage_index >= 0 ? action.stage_index : 0;
    SendMessageW(g_phase_action_stage, CB_SETCURSEL, static_cast<WPARAM>(stage_index), 0);
}

bool update_selected_action_from_controls(HWND hwnd, post2::core::PhaseConfig* phase)
{
    if (!window_is_live(g_phase_action_type) ||
        g_selected_action_index < 0 ||
        static_cast<std::size_t>(g_selected_action_index) >= phase->actions.size()) {
        return true;
    }

    post2::core::PhaseAction action = phase->actions[static_cast<std::size_t>(g_selected_action_index)];
    action.type = action_type_from_display_label(get_combo_text(g_phase_action_type));
    action.value = action_state_from_display_label(action.type, get_combo_text(g_phase_action_value));
    action.stage_index = -1;
    action.stage_name.clear();
    if (action.type == "set_stage_active" || action.type == "set_stage_attached") {
        const LRESULT selected_stage = SendMessageW(g_phase_action_stage, CB_GETCURSEL, 0, 0);
        if (selected_stage == CB_ERR) {
            MessageBoxW(hwnd, L"Select a target stage.", L"Phase action", MB_ICONWARNING);
            return false;
        }
        action.stage_index = static_cast<int>(selected_stage);
        const auto stages = post2::vehicle::effective_stage_configs(g_case.vehicle);
        if (static_cast<std::size_t>(action.stage_index) < stages.size()) {
            action.stage_name = stages[static_cast<std::size_t>(action.stage_index)].name;
        }
    }
    if (action.type != "set_engine_enabled" &&
        action.type != "set_hold_down_clamp_active" &&
        action.type != "set_stage_active" &&
        action.type != "set_stage_attached") {
        MessageBoxW(hwnd, L"Unsupported action type.", L"Phase action", MB_ICONWARNING);
        return false;
    }

    phase->actions[static_cast<std::size_t>(g_selected_action_index)] = std::move(action);
    return true;
}

bool add_action_from_sidebar(HWND hwnd)
{
    auto* phase = selected_phase();
    if (!update_selected_action_from_controls(hwnd, phase)) {
        return false;
    }

    post2::core::PhaseAction action;
    action.time_s = 0.0;
    action.type = "set_stage_active";
    action.value = true;
    action.stage_index = 0;
    const auto stages = post2::vehicle::effective_stage_configs(g_case.vehicle);
    if (!stages.empty()) {
        action.stage_name = stages.front().name;
    }
    phase->actions.push_back(std::move(action));
    g_selected_action_index = static_cast<int>(phase->actions.size() - 1);
    refresh_action_list();
    load_selected_action_controls();
    return true;
}

bool delete_action_from_sidebar(HWND hwnd)
{
    auto* phase = selected_phase();
    if (phase->actions.empty()) {
        return true;
    }
    if (g_selected_action_index < 0 ||
        static_cast<std::size_t>(g_selected_action_index) >= phase->actions.size()) {
        return true;
    }

    phase->actions.erase(phase->actions.begin() + g_selected_action_index);
    if (static_cast<std::size_t>(g_selected_action_index) >= phase->actions.size()) {
        g_selected_action_index = static_cast<int>(phase->actions.size()) - 1;
    }
    refresh_action_list();
    load_selected_action_controls();
    return update_selected_action_from_controls(hwnd, phase);
}

std::string phase_path_prefix()
{
    return "phases[" + std::to_string(g_selected_phase_index) + "]";
}

void update_phase_scrollbar()
{
    if (!window_is_live(g_phase_scroll_pane)) {
        return;
    }
    const int max_pos = std::max(0, g_phase_scroll_content_height - g_phase_scroll_page_height);
    g_phase_scroll_pos = std::clamp(g_phase_scroll_pos, 0, max_pos);

    SCROLLINFO info = {};
    info.cbSize = sizeof(info);
    info.fMask = SIF_RANGE | SIF_PAGE | SIF_POS;
    info.nMin = 0;
    info.nMax = std::max(0, g_phase_scroll_content_height - 1);
    info.nPage = static_cast<UINT>(std::max(1, g_phase_scroll_page_height));
    info.nPos = g_phase_scroll_pos;
    SetScrollInfo(g_phase_scroll_pane, SB_VERT, &info, TRUE);
}

void scroll_phase_pane_to(int new_pos)
{
    if (!window_is_live(g_phase_scroll_pane)) {
        return;
    }
    const int max_pos = std::max(0, g_phase_scroll_content_height - g_phase_scroll_page_height);
    new_pos = std::clamp(new_pos, 0, max_pos);
    if (new_pos == g_phase_scroll_pos) {
        update_phase_scrollbar();
        return;
    }

    const int delta_y = g_phase_scroll_pos - new_pos;
    g_phase_scroll_pos = new_pos;
    ScrollWindowEx(
        g_phase_scroll_pane,
        0,
        delta_y,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        SW_SCROLLCHILDREN | SW_INVALIDATE | SW_ERASE);
    update_phase_scrollbar();
    UpdateWindow(g_phase_scroll_pane);
}

LRESULT CALLBACK phase_scroll_pane_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    switch (message) {
    case WM_VSCROLL: {
        SCROLLINFO info = {};
        info.cbSize = sizeof(info);
        info.fMask = SIF_ALL;
        GetScrollInfo(hwnd, SB_VERT, &info);
        int new_pos = g_phase_scroll_pos;
        switch (LOWORD(wparam)) {
        case SB_LINEUP:
            new_pos -= 24;
            break;
        case SB_LINEDOWN:
            new_pos += 24;
            break;
        case SB_PAGEUP:
            new_pos -= std::max(24, g_phase_scroll_page_height - 24);
            break;
        case SB_PAGEDOWN:
            new_pos += std::max(24, g_phase_scroll_page_height - 24);
            break;
        case SB_THUMBPOSITION:
        case SB_THUMBTRACK:
            new_pos = info.nTrackPos;
            break;
        case SB_TOP:
            new_pos = 0;
            break;
        case SB_BOTTOM:
            new_pos = g_phase_scroll_content_height;
            break;
        default:
            break;
        }
        scroll_phase_pane_to(new_pos);
        return 0;
    }
    case WM_MOUSEWHEEL: {
        const int wheel_delta = GET_WHEEL_DELTA_WPARAM(wparam);
        scroll_phase_pane_to(g_phase_scroll_pos - (wheel_delta / WHEEL_DELTA) * 72);
        return 0;
    }
    default:
        break;
    }
    return g_phase_scroll_old_proc
        ? CallWindowProcW(g_phase_scroll_old_proc, hwnd, message, wparam, lparam)
        : DefWindowProcW(hwnd, message, wparam, lparam);
}

void create_phase_section(HWND parent, int* y, const wchar_t* title, HFONT font)
{
    *y += 10;
    create_label(parent, 18, *y, 220, title, font);
    *y += 28;
}

void create_phase_numeric_headers(HWND parent, int y, HFONT font)
{
    create_label(parent, 278, y, 70, L"Value", font);
    create_label(parent, 456, y, 70, L"Min", font);
    create_label(parent, 564, y, 70, L"Max", font);
}

NumericBindingRow add_phase_numeric_row(
    HWND parent,
    int* y,
    const std::string& label,
    const std::string& path,
    double value,
    HFONT font)
{
    NumericBindingRow row = create_numeric_binding_row(parent, *y, label, path, value, g_case.optimization, font);
    g_phase_numeric_rows.push_back(row);
    *y += 34;
    return row;
}

void add_phase_poly_rows(
    HWND parent,
    int* y,
    const std::string& label,
    const std::string& path,
    const post2::core::Poly2Config& poly,
    HFONT font)
{
    add_phase_numeric_row(parent, y, label + " c0", path + ".c0", poly.c0, font);
    add_phase_numeric_row(parent, y, label + " c1", path + ".c1", poly.c1, font);
    add_phase_numeric_row(parent, y, label + " c2", path + ".c2", poly.c2, font);
}

void add_phase_throttle_rows(
    HWND parent,
    int* y,
    const post2::core::ThrottleModelConfig& throttle,
    const std::string& path,
    HFONT font)
{
    NumericBindingRow row = add_phase_numeric_row(parent, y, "Throttle c0", path + ".c0", throttle.c0, font);
    g_phase_throttle_c0 = row.value_edit;
    row = add_phase_numeric_row(parent, y, "Throttle c1", path + ".c1", throttle.c1, font);
    g_phase_throttle_c1 = row.value_edit;
    row = add_phase_numeric_row(parent, y, "Throttle c2", path + ".c2", throttle.c2, font);
    g_phase_throttle_c2 = row.value_edit;
    row = add_phase_numeric_row(parent, y, "Target T2W", path + ".target_t2w", throttle.target_t2w, font);
    g_phase_throttle_t2w = row.value_edit;

    for (std::size_t i = 0; i < throttle.points.size(); ++i) {
        const std::string item = "Point " + std::to_string(i + 1) + " ";
        const std::string point_path = path + ".points[" + std::to_string(i) + "]";
        add_phase_numeric_row(parent, y, item + "time", point_path + ".time_s", throttle.points[i].time_s, font);
        add_phase_numeric_row(parent, y, item + "throttle", point_path + ".throttle", throttle.points[i].throttle, font);
    }
}

void add_phase_steering_rows(
    HWND parent,
    int* y,
    const post2::core::SteeringModelConfig& steering,
    const std::string& path,
    const std::string& label_prefix,
    HFONT font)
{
    add_phase_poly_rows(parent, y, label_prefix + "Roll", path + ".roll", steering.roll_deg, font);
    add_phase_poly_rows(parent, y, label_prefix + "Pitch", path + ".pitch", steering.pitch_deg, font);
    add_phase_poly_rows(parent, y, label_prefix + "Yaw", path + ".yaw", steering.yaw_deg, font);
    NumericBindingRow row =
        add_phase_numeric_row(parent, y, label_prefix + "Azimuth c0", path + ".azimuth.c0", steering.azimuth_deg.c0, font);
    if (label_prefix.empty()) {
        g_phase_steering_azimuth = row.value_edit;
    }
    add_phase_numeric_row(parent, y, label_prefix + "Azimuth c1", path + ".azimuth.c1", steering.azimuth_deg.c1, font);
    add_phase_numeric_row(parent, y, label_prefix + "Azimuth c2", path + ".azimuth.c2", steering.azimuth_deg.c2, font);
    row = add_phase_numeric_row(parent, y, label_prefix + "Elevation c0", path + ".elevation.c0", steering.elevation_deg.c0, font);
    if (label_prefix.empty()) {
        g_phase_steering_elevation = row.value_edit;
    }
    add_phase_numeric_row(parent, y, label_prefix + "Elevation c1", path + ".elevation.c1", steering.elevation_deg.c1, font);
    add_phase_numeric_row(parent, y, label_prefix + "Elevation c2", path + ".elevation.c2", steering.elevation_deg.c2, font);

    add_phase_numeric_row(parent, y, label_prefix + "Fixed ECI x", path + ".fixed_direction_eci.x", steering.fixed_direction_eci.x, font);
    add_phase_numeric_row(parent, y, label_prefix + "Fixed ECI y", path + ".fixed_direction_eci.y", steering.fixed_direction_eci.y, font);
    add_phase_numeric_row(parent, y, label_prefix + "Fixed ECI z", path + ".fixed_direction_eci.z", steering.fixed_direction_eci.z, font);

    for (std::size_t i = 0; i < steering.points.size(); ++i) {
        const std::string item = label_prefix + "Quat " + std::to_string(i + 1) + " ";
        const std::string point_path = path + ".points[" + std::to_string(i) + "]";
        add_phase_numeric_row(parent, y, item + "time", point_path + ".time_s", steering.points[i].time_s, font);
        add_phase_numeric_row(parent, y, item + "w", point_path + ".quat.w", steering.points[i].quat.w, font);
        add_phase_numeric_row(parent, y, item + "x", point_path + ".quat.x", steering.points[i].quat.x, font);
        add_phase_numeric_row(parent, y, item + "y", point_path + ".quat.y", steering.points[i].quat.y, font);
        add_phase_numeric_row(parent, y, item + "z", point_path + ".quat.z", steering.points[i].quat.z, font);
    }

    for (std::size_t i = 0; i < steering.segments.size(); ++i) {
        const std::string item = label_prefix + "Segment " + std::to_string(i + 1) + " ";
        const std::string segment_path = path + ".segments[" + std::to_string(i) + "]";
        add_phase_numeric_row(parent, y, item + "start", segment_path + ".start_time_s", steering.segments[i].start_time_s, font);
        if (steering.segments[i].model) {
            add_phase_steering_rows(
                parent,
                y,
                *steering.segments[i].model,
                segment_path + ".model",
                item,
                font);
        }
    }
}

void load_phase_numeric_rows_from_case()
{
    for (const auto& row : g_phase_numeric_rows) {
        double value = 0.0;
        std::string error;
        if (post2::core::read_optimization_variable(g_case, row.path, &value, &error)) {
            load_numeric_binding_row(row, g_case.optimization, value);
        }
    }
}

void refresh_phase_list()
{
    if (!g_phase_list) {
        return;
    }

    ensure_case_initialized();
    SendMessageW(g_phase_list, LB_RESETCONTENT, 0, 0);
    for (std::size_t i = 0; i < g_case.phases.size(); ++i) {
        const std::string label = std::to_string(i + 1) + ": " + g_case.phases[i].name +
            (g_case.phases[i].optimize_enabled ? "" : " (opt off)");
        SendMessageW(g_phase_list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(label).c_str()));
    }
    if (g_case.phases.empty()) {
        g_selected_phase_index = -1;
    } else {
        if (g_selected_phase_index < 0 ||
            static_cast<std::size_t>(g_selected_phase_index) >= g_case.phases.size()) {
            g_selected_phase_index = 0;
        }
        SendMessageW(g_phase_list, LB_SETCURSEL, static_cast<WPARAM>(g_selected_phase_index), 0);
    }
}

void load_selected_phase_controls()
{
    if (!window_is_live(g_phase_scroll_pane)) {
        return;
    }

    const auto* phase = selected_phase();
    SetWindowTextW(g_phase_name_edit, widen(phase->name).c_str());
    Button_SetCheck(g_phase_inherit_initial, phase->inherit_initial_state ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(g_phase_hold_down_initial, phase->hold_down_clamp_initial_active ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(g_phase_optimize_enabled, phase->optimize_enabled ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(g_phase_gravity, phase->force_models.gravity ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(g_phase_thrust, phase->force_models.thrust ? BST_CHECKED : BST_UNCHECKED);
    Button_SetCheck(g_phase_normal_force, phase->force_models.normal_force ? BST_CHECKED : BST_UNCHECKED);

    select_combo_text(g_phase_throttle_type, phase->throttle_model.type);
    select_combo_text(g_phase_steering_type, phase->steering_model.type);
    load_phase_numeric_rows_from_case();
    refresh_action_list();
    load_selected_action_controls();
}

bool apply_phase_controls(HWND hwnd)
{
    ensure_case_initialized();
    if (g_selected_phase_index < 0 ||
        static_cast<std::size_t>(g_selected_phase_index) >= g_case.phases.size()) {
        return false;
    }

    CaseConfig candidate = g_case;
    post2::core::PhaseConfig edited = candidate.phases[static_cast<std::size_t>(g_selected_phase_index)];
    edited.name = narrow(get_window_text(g_phase_name_edit));
    if (edited.name.empty()) {
        MessageBoxW(hwnd, L"Phase name cannot be empty.", L"Phase", MB_ICONWARNING);
        SetFocus(g_phase_name_edit);
        return false;
    }

    edited.inherit_initial_state = Button_GetCheck(g_phase_inherit_initial) == BST_CHECKED;
    edited.hold_down_clamp_initial_active = Button_GetCheck(g_phase_hold_down_initial) == BST_CHECKED;
    edited.optimize_enabled = Button_GetCheck(g_phase_optimize_enabled) == BST_CHECKED;
    edited.force_models.gravity = Button_GetCheck(g_phase_gravity) == BST_CHECKED;
    edited.force_models.thrust = Button_GetCheck(g_phase_thrust) == BST_CHECKED;
    edited.force_models.normal_force = Button_GetCheck(g_phase_normal_force) == BST_CHECKED;

    edited.throttle_model.type = get_combo_text(g_phase_throttle_type);
    edited.steering_model.type = get_combo_text(g_phase_steering_type);

    if (!update_selected_action_from_controls(hwnd, &edited)) {
        return false;
    }

    candidate.phases[static_cast<std::size_t>(g_selected_phase_index)] = edited;

    for (const auto& row : g_phase_numeric_rows) {
        const std::wstring text = get_window_text(row.value_edit);
        double value = 0.0;
        if (!parse_double_text(text, &value)) {
            MessageBoxW(hwnd, widen(row.label + " must be a number.").c_str(), L"Phase", MB_ICONWARNING);
            SetFocus(row.value_edit);
            return false;
        }

        std::string path_error;
        if (!post2::core::write_optimization_variable(&candidate, row.path, value, &path_error)) {
            if (path_error.find("out of range") == std::string::npos) {
                MessageBoxW(hwnd, widen(path_error).c_str(), L"Phase", MB_ICONWARNING);
                SetFocus(row.value_edit);
                return false;
            }
            continue;
        }
        if (!apply_numeric_variable_controls(hwnd, row, value, &candidate.optimization, L"Phase")) {
            return false;
        }
    }

    const auto& final_phase = candidate.phases[static_cast<std::size_t>(g_selected_phase_index)];
    if (final_phase.duration_s <= 0.0) {
        MessageBoxW(hwnd, L"Duration must be positive.", L"Phase", MB_ICONWARNING);
        SetFocus(g_phase_duration_edit);
        return false;
    }
    for (const auto& action : final_phase.actions) {
        if (action.time_s < 0.0 || action.time_s > final_phase.duration_s) {
            MessageBoxW(hwnd, L"Action time must be inside the phase duration.", L"Phase", MB_ICONWARNING);
            return false;
        }
    }

    g_case = std::move(candidate);
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
    return true;
}

void add_phase_from_sidebar()
{
    ensure_case_initialized();
    post2::core::PhaseConfig phase;
    phase.name = "phase " + std::to_string(g_case.phases.size() + 1);
    phase.duration_s = g_case.phases.empty() ? g_config.duration_s : 60.0;
    phase.inherit_initial_state = !g_case.phases.empty();
    g_case.phases.push_back(std::move(phase));
    g_selected_phase_index = static_cast<int>(g_case.phases.size() - 1);
    refresh_phase_list();
    load_selected_phase_controls();
}

void delete_selected_phase_from_sidebar()
{
    ensure_case_initialized();
    if (g_case.phases.size() <= 1) {
        MessageBoxW(GetParent(g_phase_list), L"A case must keep at least one phase.", L"Phase", MB_ICONWARNING);
        return;
    }
    if (g_selected_phase_index < 0 || static_cast<std::size_t>(g_selected_phase_index) >= g_case.phases.size()) {
        return;
    }
    g_case.phases.erase(g_case.phases.begin() + g_selected_phase_index);
    if (static_cast<std::size_t>(g_selected_phase_index) >= g_case.phases.size()) {
        g_selected_phase_index = static_cast<int>(g_case.phases.size() - 1);
    }
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
}

void create_phase_editor_controls(HWND hwnd)
{
    clear_phase_editor_handles();
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    const auto* phase = selected_phase();
    const std::string prefix = phase_path_prefix();

    g_phase_scroll_pane = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"STATIC",
        L"",
        WS_CHILD | WS_VISIBLE | WS_VSCROLL | WS_CLIPCHILDREN,
        18,
        18,
        730,
        642,
        hwnd,
        control_id(kPhaseScrollPane),
        g_instance,
        nullptr);
    set_child_font(g_phase_scroll_pane, font);
    g_phase_scroll_old_proc = reinterpret_cast<WNDPROC>(
        SetWindowLongPtrW(g_phase_scroll_pane, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(phase_scroll_pane_proc)));

    int y = 16;
    create_label(g_phase_scroll_pane, 18, y + 4, 70, L"Name", font);
    g_phase_name_edit = create_edit(g_phase_scroll_pane, kPhaseNameEdit, 278, y, 386, widen(phase->name), font);
    y += 38;

    g_phase_inherit_initial = create_checkbox(g_phase_scroll_pane, kPhaseInheritInitial, 18, y, 138, L"Inherit state", font);
    g_phase_hold_down_initial = create_checkbox(g_phase_scroll_pane, kPhaseHoldDownInitial, 180, y, 144, L"HDC active", font);
    g_phase_optimize_enabled = create_checkbox(g_phase_scroll_pane, kPhaseOptimizeEnabled, 332, y, 160, L"Optimize this phase", font);
    y += 34;

    create_label(g_phase_scroll_pane, 18, y + 4, 70, L"Forces", font);
    g_phase_gravity = create_checkbox(g_phase_scroll_pane, kPhaseGravity, 118, y, 86, L"Gravity", font);
    g_phase_thrust = create_checkbox(g_phase_scroll_pane, kPhaseThrust, 220, y, 78, L"Thrust", font);
    g_phase_normal_force = create_checkbox(g_phase_scroll_pane, kPhaseNormalForce, 314, y, 110, L"Normal", font);
    y += 40;

    create_phase_numeric_headers(g_phase_scroll_pane, y - 18, font);
    NumericBindingRow duration_row =
        add_phase_numeric_row(g_phase_scroll_pane, &y, "Duration", prefix + ".duration_s", phase->duration_s, font);
    g_phase_duration_edit = duration_row.value_edit;

    create_phase_section(g_phase_scroll_pane, &y, L"Throttle", font);
    create_label(g_phase_scroll_pane, 18, y + 4, 86, L"Type", font);
    g_phase_throttle_type = create_combo(g_phase_scroll_pane, kPhaseThrottleType, 118, y, 154, font);
    add_combo_item(g_phase_throttle_type, L"poly");
    add_combo_item(g_phase_throttle_type, L"t2w");
    add_combo_item(g_phase_throttle_type, L"interpolated");
    y += 38;
    add_phase_throttle_rows(g_phase_scroll_pane, &y, phase->throttle_model, prefix + ".throttle_model", font);

    create_phase_section(g_phase_scroll_pane, &y, L"Steering", font);
    create_label(g_phase_scroll_pane, 18, y + 4, 86, L"Type", font);
    g_phase_steering_type = create_combo(g_phase_scroll_pane, kPhaseSteeringType, 118, y, 210, font);
    add_combo_item(g_phase_steering_type, L"generic_poly");
    add_combo_item(g_phase_steering_type, L"rpy_poly");
    add_combo_item(g_phase_steering_type, L"fixed_eci");
    add_combo_item(g_phase_steering_type, L"generic_quat_interp");
    add_combo_item(g_phase_steering_type, L"generic_selectable");
    y += 38;
    add_phase_steering_rows(g_phase_scroll_pane, &y, phase->steering_model, prefix + ".steering_model", "", font);

    create_phase_section(g_phase_scroll_pane, &y, L"Actions", font);
    g_phase_action_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18,
        y,
        646,
        118,
        g_phase_scroll_pane,
        control_id(kPhaseActionList),
        g_instance,
        nullptr);
    set_child_font(g_phase_action_list, font);
    y += 128;

    g_phase_action_add_button = create_button(g_phase_scroll_pane, kPhaseActionAdd, 18, y, 62, 26, L"Add", font);
    g_phase_action_delete_button = create_button(g_phase_scroll_pane, kPhaseActionDelete, 88, y, 62, 26, L"Delete", font);
    y += 38;

    create_label(g_phase_scroll_pane, 18, y + 4, 70, L"Action", font);
    g_phase_action_type = create_combo(g_phase_scroll_pane, kPhaseActionType, 118, y, 190, font);
    add_combo_item(g_phase_action_type, L"Engine enabled");
    add_combo_item(g_phase_action_type, L"Hold-down clamp active");
    add_combo_item(g_phase_action_type, L"Stage active");
    add_combo_item(g_phase_action_type, L"Stage attached");
    create_label(g_phase_scroll_pane, 330, y + 4, 38, L"State", font);
    g_phase_action_value = create_combo(g_phase_scroll_pane, kPhaseActionValue, 378, y, 90, font);
    create_label(g_phase_scroll_pane, 494, y + 4, 38, L"Stage", font);
    g_phase_action_stage = create_combo(g_phase_scroll_pane, kPhaseActionStage, 542, y, 122, font);
    populate_action_stage_combo();
    y += 38;

    for (std::size_t i = 0; i < phase->actions.size(); ++i) {
        const std::string label = "Action " + std::to_string(i + 1) + " time";
        const std::string path = prefix + ".actions[" + std::to_string(i) + "].time_s";
        add_phase_numeric_row(g_phase_scroll_pane, &y, label, path, phase->actions[i].time_s, font);
    }

    RECT pane_client = {};
    GetClientRect(g_phase_scroll_pane, &pane_client);
    g_phase_scroll_content_height = y + 20;
    g_phase_scroll_page_height = pane_client.bottom - pane_client.top;
    update_phase_scrollbar();

    HWND ok_button = create_button(hwnd, IDOK, 554, 682, 76, 28, L"OK", font);
    SendMessageW(ok_button, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(hwnd, IDCANCEL, 644, 682, 76, 28, L"Cancel", font);

    load_selected_phase_controls();
}

void create_phase_sidebar_controls(HWND hwnd)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(hwnd, 18, 18, 70, L"Phases", font);
    g_phase_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18,
        42,
        290,
        356,
        hwnd,
        control_id(kPhaseList),
        g_instance,
        nullptr);
    set_child_font(g_phase_list, font);

    g_phase_add_button = create_button(hwnd, kPhaseAdd, 18, 412, 62, 26, L"Add", font);
    g_phase_delete_button = create_button(hwnd, kPhaseDelete, 88, 412, 62, 26, L"Delete", font);
    g_phase_apply_button = create_button(hwnd, kPhaseApply, 168, 412, 62, 26, L"Edit", font);
    g_phase_json_button = create_button(hwnd, kPhaseJson, 238, 412, 70, 26, L"JSON", font);

    create_label(hwnd, 18, 460, 70, L"Outputs", font);
    g_outputs_edit = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"EDIT",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL | WS_VSCROLL,
        18,
        486,
        290,
        176,
        hwnd,
        control_id(kOutputsEdit),
        g_instance,
        nullptr);
    set_child_font(g_outputs_edit, font);
    SetWindowTextW(g_outputs_edit, widen(g_outputs_text).c_str());

    refresh_phase_list();
}

LRESULT CALLBACK phase_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<PhaseEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<PhaseEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_phase_editor_controls(hwnd);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            if (apply_phase_controls(hwnd)) {
                state->accepted = true;
                DestroyWindow(hwnd);
            }
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kPhaseActionList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const int previous = g_selected_action_index;
                const LRESULT selected = SendMessageW(g_phase_action_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    if (!update_selected_action_from_controls(hwnd, selected_phase())) {
                        SendMessageW(g_phase_action_list, LB_SETCURSEL, static_cast<WPARAM>(previous), 0);
                        return 0;
                    }
                    refresh_action_list();
                    g_selected_action_index = static_cast<int>(selected);
                    SendMessageW(g_phase_action_list, LB_SETCURSEL, static_cast<WPARAM>(g_selected_action_index), 0);
                    load_selected_action_controls();
                }
                return 0;
            }
            break;
        case kPhaseActionAdd:
            add_action_from_sidebar(hwnd);
            return 0;
        case kPhaseActionDelete:
            delete_action_from_sidebar(hwnd);
            return 0;
        case kPhaseActionType:
            if (HIWORD(wparam) == CBN_SELCHANGE) {
                const std::string type = action_type_from_display_label(get_combo_text(g_phase_action_type));
                const bool value = action_state_from_display_label(type, get_combo_text(g_phase_action_value));
                populate_action_value_combo(type, value);
                EnableWindow(g_phase_action_stage, type == "set_stage_active" || type == "set_stage_attached");
                return 0;
            }
            break;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    case WM_MOUSEWHEEL:
        scroll_phase_pane_to(g_phase_scroll_pos - (GET_WHEEL_DELTA_WPARAM(wparam) / WHEEL_DELTA) * 72);
        return 0;
    case WM_NCDESTROY:
        clear_phase_editor_handles();
        break;
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_phase_editor_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = phase_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2PhaseEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_phase_editor_dialog(HWND parent)
{
    ensure_case_initialized();
    register_phase_editor_class();

    PhaseEditorDialogState state;
    state.parent = parent;
    state.original_phase = *selected_phase();

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 790;
    constexpr int dialog_height = 770;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2PhaseEditorWindow",
        L"Edit phase",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x,
        y,
        dialog_width,
        dialog_height,
        parent,
        nullptr,
        g_instance,
        &state);

    if (!dialog) {
        MessageBoxW(parent, L"Failed to create phase editor.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(g_phase_name_edit);

    MSG msg;
    while (IsWindow(dialog) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dialog, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);

    if (!state.accepted) {
        *selected_phase() = std::move(state.original_phase);
    }
    clear_phase_editor_handles();
    refresh_phase_list();
    return state.accepted;
}

void paint_scene(HWND hwnd, HDC hdc)
{
    RECT client;
    GetClientRect(hwnd, &client);

    HBRUSH background = CreateSolidBrush(RGB(248, 250, 252));
    FillRect(hdc, &client, background);
    DeleteObject(background);

    RECT sidebar = client;
    sidebar.right = kSidebarWidth;
    HBRUSH sidebar_background = CreateSolidBrush(RGB(241, 245, 249));
    FillRect(hdc, &sidebar, sidebar_background);
    DeleteObject(sidebar_background);
    HPEN separator = CreatePen(PS_SOLID, 1, RGB(203, 213, 225));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, separator));
    MoveToEx(hdc, kSidebarWidth, 0, nullptr);
    LineTo(hdc, kSidebarWidth, client.bottom);
    SelectObject(hdc, old_pen);
    DeleteObject(separator);

    const int content_x = kSidebarWidth + 24;

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(15, 23, 42));

    HFONT title_font = CreateFontW(22, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    HFONT old_font = static_cast<HFONT>(SelectObject(hdc, title_font));
    draw_text_line(hdc, content_x, 20, "POST2 Lite trajectory");
    SelectObject(hdc, old_font);
    DeleteObject(title_font);

    HFONT body_font = CreateFontW(15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    old_font = static_cast<HFONT>(SelectObject(hdc, body_font));
    draw_text_line(hdc, content_x, 52, g_status);
    draw_text_line(hdc, content_x, 74, "3D inertial view. Remote endpoint: " + remote_endpoint_text() + ".");

    if (!g_result.ok || g_result.state_log.empty()) {
        SelectObject(hdc, old_font);
        DeleteObject(body_font);
        return;
    }

    SelectObject(hdc, old_font);
    DeleteObject(body_font);
}

void update_mode_menu(HWND hwnd)
{
    HMENU menu = GetMenu(hwnd);
    CheckMenuItem(menu, kMenuModeLocal, MF_BYCOMMAND | (g_mode == CoreMode::Local ? MF_CHECKED : MF_UNCHECKED));
    CheckMenuItem(menu, kMenuModeRemote, MF_BYCOMMAND | (g_mode == CoreMode::Remote ? MF_CHECKED : MF_UNCHECKED));
}

HMENU create_main_menu()
{
    HMENU menu = CreateMenu();
    HMENU file_menu = CreatePopupMenu();
    HMENU mode_menu = CreatePopupMenu();
    HMENU run_menu = CreatePopupMenu();
    HMENU optimize_menu = CreatePopupMenu();
    HMENU case_menu = CreatePopupMenu();
    HMENU launch_menu = CreatePopupMenu();
    HMENU vehicle_menu = CreatePopupMenu();

    AppendMenuW(file_menu, MF_STRING, kMenuRefresh, L"Refresh");
    AppendMenuW(file_menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(file_menu, MF_STRING, kMenuCaseLoad, L"Load case...");
    AppendMenuW(file_menu, MF_STRING, kMenuCaseSave, L"Save case...");
    AppendMenuW(file_menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(file_menu, MF_STRING, kMenuExportCsv, L"Export CSV");
    AppendMenuW(file_menu, MF_STRING, kMenuExportSvg, L"Export SVG");
    AppendMenuW(file_menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(file_menu, MF_STRING, kMenuExit, L"Exit");

    AppendMenuW(mode_menu, MF_STRING, kMenuModeLocal, L"Local core");
    AppendMenuW(mode_menu, MF_STRING, kMenuModeRemote, L"Remote core");
    AppendMenuW(mode_menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(mode_menu, MF_STRING, kMenuRemoteEndpoint, L"Remote endpoint...");

    AppendMenuW(run_menu, MF_STRING, kMenuRunTrajectory, L"Run trajectory");

    AppendMenuW(optimize_menu, MF_STRING, kMenuOptimizationSettings, L"Settings...");
    AppendMenuW(optimize_menu, MF_STRING, kMenuOptimizationExecute, L"Execute optimize");

    AppendMenuW(case_menu, MF_STRING, kMenuCasePhases, L"Phases...");

    AppendMenuW(launch_menu, MF_STRING, kMenuLaunchSettings, L"Launch site...");

    AppendMenuW(vehicle_menu, MF_STRING, kMenuVehicleEdit, L"Edit vehicle...");

    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(file_menu), L"File");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(mode_menu), L"Mode");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(run_menu), L"Run");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(optimize_menu), L"Optimize");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(case_menu), L"Case");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(launch_menu), L"Launch");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(vehicle_menu), L"Vehicle");
    return menu;
}

void export_current(HWND hwnd, bool svg)
{
    if (!g_result.ok || g_result.state_log.empty()) {
        MessageBoxW(hwnd, L"No StateLog is available to export.", L"POST2 Lite", MB_ICONWARNING);
        return;
    }

    std::string error;
    const std::string path = svg ? "gui_trajectory.svg" : "gui_trajectory.csv";
    const bool ok = svg
        ? post2::core::write_svg_file(path, g_result.state_log, &error)
        : post2::core::write_csv_file(path, g_result.state_log, &error);

    if (!ok) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Export failed", MB_ICONERROR);
        return;
    }

    MessageBoxW(hwnd, widen("Wrote " + path).c_str(), L"POST2 Lite", MB_ICONINFORMATION);
}

LRESULT CALLBACK scene_window_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    switch (message) {
    case WM_CREATE:
        return g_scene_renderer.initialize(hwnd) ? 0 : -1;

    case WM_SIZE: {
        const int width = LOWORD(lparam);
        const int height = HIWORD(lparam);
        RECT viewport = {};
        viewport.right = width;
        viewport.bottom = height;
        g_camera.set_viewport(viewport);
        g_scene_renderer.resize(width, height);
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;
    }

    case WM_LBUTTONDOWN:
        SetFocus(hwnd);
        g_dragging = true;
        g_last_mouse = {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
        SetCapture(hwnd);
        return 0;

    case WM_MOUSEMOVE:
        if (g_dragging && (wparam & MK_LBUTTON)) {
            const POINT current = {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
            g_camera.rotate_pixels(current.x - g_last_mouse.x, current.y - g_last_mouse.y);
            g_last_mouse = current;
            invalidate_scene_window();
            return 0;
        }
        break;

    case WM_LBUTTONUP:
        if (g_dragging) {
            g_dragging = false;
            ReleaseCapture();
            return 0;
        }
        break;

    case WM_CAPTURECHANGED:
        g_dragging = false;
        return 0;

    case WM_LBUTTONDBLCLK:
        g_camera.reset(g_camera.scene_radius_m());
        invalidate_scene_window();
        return 0;

    case WM_MOUSEWHEEL:
        g_camera.zoom_wheel(GET_WHEEL_DELTA_WPARAM(wparam));
        invalidate_scene_window();
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        BeginPaint(hwnd, &ps);
        g_scene_renderer.render(g_camera, g_result.state_log);
        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_DESTROY:
        g_scene_renderer.destroy();
        if (g_scene_hwnd == hwnd) {
            g_scene_hwnd = nullptr;
        }
        return 0;

    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

bool register_scene_window_class(HINSTANCE instance)
{
    WNDCLASSW wc = {};
    wc.style = CS_OWNDC | CS_DBLCLKS;
    wc.lpfnWndProc = scene_window_proc;
    wc.hInstance = instance;
    wc.lpszClassName = kSceneWindowClassName;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;

    if (RegisterClassW(&wc)) {
        return true;
    }
    return GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
}

void create_scene_window(HWND parent)
{
    const RECT rect = scene_rect(parent);
    g_scene_hwnd = CreateWindowExW(
        0,
        kSceneWindowClassName,
        L"",
        WS_CHILD | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
        rect.left,
        rect.top,
        std::max(0L, rect.right - rect.left),
        std::max(0L, rect.bottom - rect.top),
        parent,
        nullptr,
        g_instance,
        nullptr);
    sync_scene_window(parent);
}

LRESULT CALLBACK window_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    switch (message) {
    case WM_CREATE:
        SetMenu(hwnd, create_main_menu());
        update_mode_menu(hwnd);
        ensure_case_initialized();
        create_phase_sidebar_controls(hwnd);
        create_scene_window(hwnd);
        run_simulation(hwnd);
        return 0;

    case WM_LBUTTONDOWN:
        update_camera_viewport(hwnd);
        g_dragging = true;
        g_last_mouse = {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
        SetCapture(hwnd);
        return 0;

    case WM_MOUSEMOVE:
        if (g_dragging && (wparam & MK_LBUTTON)) {
            const POINT current = {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
            g_camera.rotate_pixels(current.x - g_last_mouse.x, current.y - g_last_mouse.y);
            g_last_mouse = current;
            invalidate_scene_window();
            return 0;
        }
        break;

    case WM_LBUTTONUP:
        if (g_dragging) {
            g_dragging = false;
            ReleaseCapture();
            return 0;
        }
        break;

    case WM_CAPTURECHANGED:
        g_dragging = false;
        return 0;

    case WM_LBUTTONDBLCLK:
        g_camera.reset(g_camera.scene_radius_m());
        invalidate_scene_window();
        return 0;

    case WM_MOUSEWHEEL:
        g_camera.zoom_wheel(GET_WHEEL_DELTA_WPARAM(wparam));
        invalidate_scene_window();
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case kMenuRefresh:
            run_simulation(hwnd);
            return 0;
        case kMenuRunTrajectory:
            run_simulation(hwnd);
            return 0;
        case kMenuOptimizationSettings:
            if (show_optimization_settings_dialog(hwnd)) {
                refresh_phase_list();
                load_selected_phase_controls();
                set_outputs_text("Optimization settings updated.");
                InvalidateRect(hwnd, nullptr, TRUE);
            }
            return 0;
        case kMenuOptimizationExecute:
            execute_optimization(hwnd);
            return 0;
        case kMenuExportCsv:
            export_current(hwnd, false);
            return 0;
        case kMenuExportSvg:
            export_current(hwnd, true);
            return 0;
        case kMenuCaseLoad:
            load_case_config(hwnd);
            return 0;
        case kMenuCaseSave:
            save_case_config(hwnd);
            return 0;
        case kMenuModeLocal:
            g_mode = CoreMode::Local;
            update_mode_menu(hwnd);
            run_simulation(hwnd);
            return 0;
        case kMenuModeRemote:
            g_mode = CoreMode::Remote;
            update_mode_menu(hwnd);
            run_simulation(hwnd);
            return 0;
        case kMenuRemoteEndpoint:
            if (show_remote_settings_dialog(hwnd)) {
                if (g_mode == CoreMode::Remote) {
                    run_simulation(hwnd);
                } else {
                    update_status();
                    InvalidateRect(hwnd, nullptr, TRUE);
                }
            }
            return 0;
        case kMenuCasePhases:
            if (show_case_phases_dialog(hwnd)) {
                refresh_phase_list();
                load_selected_phase_controls();
                run_simulation(hwnd);
            }
            return 0;
        case kPhaseList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const LRESULT selected = SendMessageW(g_phase_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    g_selected_phase_index = static_cast<int>(selected);
                    g_selected_action_index = 0;
                }
                return 0;
            }
            break;
        case kPhaseAdd:
            add_phase_from_sidebar();
            run_simulation(hwnd);
            return 0;
        case kPhaseDelete:
            delete_selected_phase_from_sidebar();
            run_simulation(hwnd);
            return 0;
        case kPhaseApply:
            if (show_phase_editor_dialog(hwnd)) {
                run_simulation(hwnd);
            }
            return 0;
        case kPhaseJson:
            if (show_case_phases_dialog(hwnd)) {
                refresh_phase_list();
                load_selected_phase_controls();
                run_simulation(hwnd);
            }
            return 0;
        case kMenuLaunchSettings:
            if (show_launch_settings_dialog(hwnd)) {
                refresh_phase_list();
                load_selected_phase_controls();
                run_simulation(hwnd);
            }
            return 0;
        case kMenuVehicleEdit:
            if (show_vehicle_settings_dialog(hwnd)) {
                run_simulation(hwnd);
            }
            return 0;
        case kMenuExit:
            DestroyWindow(hwnd);
            return 0;
        default:
            break;
        }
        break;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);

        RECT client;
        GetClientRect(hwnd, &client);
        const int width = client.right - client.left;
        const int height = client.bottom - client.top;
        if (width > 0 && height > 0) {
            HDC memory_dc = CreateCompatibleDC(hdc);
            HBITMAP bitmap = CreateCompatibleBitmap(hdc, width, height);
            HBITMAP old_bitmap = static_cast<HBITMAP>(SelectObject(memory_dc, bitmap));
            paint_scene(hwnd, memory_dc);
            BitBlt(hdc, 0, 0, width, height, memory_dc, 0, 0, SRCCOPY);
            SelectObject(memory_dc, old_bitmap);
            DeleteObject(bitmap);
            DeleteDC(memory_dc);
        } else {
            paint_scene(hwnd, hdc);
        }

        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_SIZE:
        sync_scene_window(hwnd);
        InvalidateRect(hwnd, nullptr, TRUE);
        return 0;

    case WM_DESTROY:
        g_scene_renderer.destroy();
        PostQuitMessage(0);
        return 0;

    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show_command)
{
    g_instance = instance;
    const wchar_t* class_name = L"Post2LiteWindow";

    if (!register_scene_window_class(instance)) {
        return 1;
    }

    WNDCLASSW wc = {};
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;
    wc.lpfnWndProc = window_proc;
    wc.hInstance = instance;
    wc.lpszClassName = class_name;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);

    RegisterClassW(&wc);

    HWND hwnd = CreateWindowExW(
        0,
        class_name,
        L"POST2 Lite",
        WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1100,
        820,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (!hwnd) {
        return 1;
    }

    ShowWindow(hwnd, show_command);
    UpdateWindow(hwnd);

    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    return static_cast<int>(msg.wParam);
}
