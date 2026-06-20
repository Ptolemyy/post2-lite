#include "post2/aero/aero_model.hpp"
#include "post2/aero/aero_table.hpp"
#include "post2/core/aero_table_build.hpp"
#include "post2/core/case_config_io.hpp"
#include "post2/core/frames.hpp"
#include "post2/core/io.hpp"
#include "post2/core/ksp_vehicle_site_import.hpp"
#include "post2/core/optimization.hpp"
#include "post2/core/simulation_driver.hpp"
#include "post2/core/trajectory_service.hpp"
#include "post2/vehicle/runtime_state.hpp"
#include "post2/vehicle/vehicle_config_io.hpp"
#include "chart_panel.hpp"
#include "opengl_scene_renderer.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <commdlg.h>
#include <windows.h>
#include <windowsx.h>
#include <GL/gl.h>

namespace {

using post2::core::CoreMode;
using post2::core::CaseConfig;
using post2::core::OptimizationResult;
using post2::core::SimulationConfig;
using post2::core::SimulationResult;
using post2::core::StateLog;

constexpr int kMenuRefresh = 1001;
constexpr int kMenuExportCsv = 1002;
constexpr int kMenuExportSvg = 1003;
constexpr int kMenuExit = 1004;
constexpr int kMenuCaseLoad = 1005;
constexpr int kMenuCaseSave = 1006;
constexpr int kMenuCaseSaveAs = 1007;
constexpr int kMenuExportKosCsv = 1008;
constexpr int kMenuModeLocal = 1101;
constexpr int kMenuModeRemote = 1102;
constexpr int kMenuRemoteEndpoint = 1103;
constexpr int kMenuVehicleEdit = 1301;
constexpr int kMenuVehicleImportKsp = 1302;
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
constexpr int kPhaseAero = 1929;
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
constexpr int kPhaseAtmosphereType = 1931;
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
constexpr int kVehicleAeroEnabled = 1418;
constexpr int kVehicleAeroTableList = 1419;
constexpr int kVehicleAeroTableImport = 1420;
constexpr int kVehicleAeroTableView = 1421;
constexpr int kVehicleAeroTableEditStage = 1422;
constexpr int kVehicleAeroTableDelete = 1423;
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
constexpr int kOptTargetEdit = 2124;
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
constexpr int kOptContinuationStrategyCombo = 2119;
constexpr int kOptContinuationVariableCombo = 2120;
constexpr int kOptContinuationDirectionCombo = 2121;
constexpr int kOptContinuationStepsEdit = 2122;
constexpr int kOptContinuationStartsEdit = 2123;
constexpr int kOptObjectiveList = 2126;
constexpr int kOptObjectiveAdd = 2127;
constexpr int kOptObjectiveEdit = 2128;
constexpr int kOptObjectiveDelete = 2129;
constexpr int kOptEnvelopeSamplesEdit = 2130;
constexpr UINT kOptimizationFinishedMessage = WM_APP + 1;

// Sub-dialog control IDs for the phase-action / trigger-condition / event
// modal editors. They live in their own range to avoid colliding with the
// inline phase-action controls (1922-1928 above) which still exist while we
// migrate callers to the modal pattern.
constexpr int kPhaseActionEditTime = 2401;
constexpr int kPhaseActionEditType = 2402;
constexpr int kPhaseActionEditValue = 2403;
constexpr int kPhaseActionEditStage = 2404;
constexpr int kTriggerEditType = 2410;
constexpr int kTriggerEditComparison = 2411;
constexpr int kTriggerEditValue = 2412;
constexpr int kEventEditName = 2420;
constexpr int kEventEditEnabled = 2421;
constexpr int kEventEditTriggerButton = 2422;
constexpr int kEventEditActionList = 2423;
constexpr int kEventEditActionAdd = 2424;
constexpr int kEventEditActionEdit = 2425;
constexpr int kEventEditActionDelete = 2426;
// Top-level case-events pane controls.
constexpr int kCaseEventsList = 2430;
constexpr int kCaseEventsAdd = 2431;
constexpr int kCaseEventsEdit = 2432;
constexpr int kCaseEventsDelete = 2433;
// Phase termination Edit button.
constexpr int kPhaseTerminationEditButton = 2440;
constexpr int kPhaseActionEdit = 2441;
constexpr const wchar_t* kSceneWindowClassName = L"Post2LiteOpenGLScene";

// Middle column holding pre-takeoff and final vehicle stats.
constexpr int kVehicleColumnX = kSidebarWidth + 24;       // 350
constexpr int kVehicleColumnWidth = 280;
constexpr int kVehicleColumnRight = kVehicleColumnX + kVehicleColumnWidth;  // 630
constexpr int kContentColumnX = kVehicleColumnRight + 24; // 654

// View selector buttons.
constexpr int kViewButton3D = 2500;
constexpr int kViewButtonProfile = 2501;
constexpr int kViewButtonQ = 2502;
constexpr int kViewButtonThrottle = 2503;
constexpr int kViewButtonSpeed = 2504;
constexpr int kViewButtonMass = 2505;
constexpr int kViewButtonCount = 6;
constexpr int kViewButtonHeight = 30;
constexpr int kViewButtonGap = 4;
constexpr int kViewButtonTop = 84;

enum class ViewKind {
    Scene3D = 0,
    Profile2D = 1,
    DynamicPressure = 2,
    Throttle = 3,
    Speed = 4,
    Mass = 5,
    Count = 6,
};

HINSTANCE g_instance = nullptr;
CoreMode g_mode = CoreMode::Local;
SimulationConfig g_config;
CaseConfig g_case;
SimulationResult g_result;
// Integrated predicted orbit from the final state, recomputed whenever g_result
// changes; drawn (teal, with A/P markers) in the 3D scene and SVG export.
StateLog g_predicted_orbit;
std::string g_status;
std::string g_remote_host = "127.0.0.1";
int g_remote_port = 5050;
std::string g_case_path;
post2::gui::Camera3D g_camera;
post2::gui::OpenGLSceneRenderer g_scene_renderer;
HWND g_scene_hwnd = nullptr;
ViewKind g_active_view = ViewKind::Scene3D;
std::array<HWND, kViewButtonCount> g_view_buttons = {};
post2::gui::ChartPanel g_chart_profile;
post2::gui::ChartPanel g_chart_q;
post2::gui::ChartPanel g_chart_throttle;
post2::gui::ChartPanel g_chart_speed;
post2::gui::ChartPanel g_chart_mass;

struct PendingOptimizationResult {
    CaseConfig case_config;
    OptimizationResult result;
};

std::mutex g_optimization_mutex;
bool g_optimization_running = false;
std::optional<PendingOptimizationResult> g_pending_optimization;

void populate_charts();
void switch_view(HWND hwnd, ViewKind view);
bool g_camera_initialized = false;
bool g_case_initialized = false;
bool g_dragging = false;
POINT g_last_mouse = {};
int g_selected_phase_index = 0;
HWND g_phase_list = nullptr;
HWND g_phase_add_button = nullptr;
HWND g_phase_delete_button = nullptr;
HWND g_phase_name_edit = nullptr;
HWND g_phase_termination_label = nullptr;
HWND g_phase_inherit_initial = nullptr;
HWND g_phase_hold_down_initial = nullptr;
HWND g_phase_optimize_enabled = nullptr;
HWND g_phase_gravity = nullptr;
HWND g_phase_thrust = nullptr;
HWND g_phase_normal_force = nullptr;
HWND g_phase_aero = nullptr;
HWND g_phase_atmosphere_type = nullptr;
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

// Phase-boundary continuity checkboxes. Bound to bool paths ending in
// `.continuity` (steering angle polys and the throttle model) and round-tripped
// through read/write_optimization_flag like the numeric rows above.
struct ContinuityBindingRow {
    std::string path;
    HWND check = nullptr;
};

std::vector<ContinuityBindingRow> g_phase_continuity_rows;
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
std::string action_label(const post2::core::PhaseAction& action);
std::string action_type_display_label(const std::string& type);
std::string action_type_from_display_label(const std::string& label);
std::string action_state_display_label(const std::string& type, bool value);
bool action_state_from_display_label(const std::string& type, const std::string& label);
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
    g_phase_termination_label = nullptr;
    g_phase_inherit_initial = nullptr;
    g_phase_hold_down_initial = nullptr;
    g_phase_optimize_enabled = nullptr;
    g_phase_gravity = nullptr;
    g_phase_thrust = nullptr;
    g_phase_normal_force = nullptr;
    g_phase_aero = nullptr;
    g_phase_atmosphere_type = nullptr;
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
    g_phase_continuity_rows.clear();
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
    HWND aero_enabled = nullptr;
    HWND aero_table_list = nullptr;
    HWND gl_preview = nullptr;  // embedded OpenGL vehicle preview child window
    HWND stage_list = nullptr;
    post2::vehicle::VehicleConfig config;
    post2::core::OptimizationConfig optimization;
    int selected_stage_index = 0;
    int selected_aero_table_index = 0;
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
    HWND target_add_button = nullptr;
    HWND target_edit_button = nullptr;
    HWND target_delete_button = nullptr;
    HWND objective_list = nullptr;
    HWND objective_add_button = nullptr;
    HWND objective_edit_button = nullptr;
    HWND objective_delete_button = nullptr;
    HWND continuation_strategy_combo = nullptr;
    HWND continuation_variable_combo = nullptr;
    HWND continuation_direction_combo = nullptr;
    HWND continuation_steps_edit = nullptr;
    HWND continuation_starts_edit = nullptr;
    HWND envelope_samples_edit = nullptr;
    post2::core::OptimizationConfig config;
    int selected_target_index = 0;
    int selected_objective_index = 0;
    bool accepted = false;
};

struct OptimizationTargetEditorDialogState {
    HWND hwnd = nullptr;
    HWND metric_combo = nullptr;
    HWND mode_combo = nullptr;
    HWND value_edit = nullptr;
    HWND min_edit = nullptr;
    HWND max_edit = nullptr;
    HWND weight_edit = nullptr;
    post2::core::OptimizationTargetConfig target;
    bool accepted = false;
};

struct PhaseActionEditorDialogState {
    HWND hwnd = nullptr;
    HWND time_edit = nullptr;
    HWND type_combo = nullptr;
    HWND value_combo = nullptr;
    HWND stage_combo = nullptr;
    post2::core::PhaseAction action;
    const post2::vehicle::VehicleConfig* vehicle = nullptr;
    bool accepted = false;
};

struct TriggerConditionEditorDialogState {
    HWND hwnd = nullptr;
    HWND type_combo = nullptr;
    HWND comparison_combo = nullptr;
    HWND value_edit = nullptr;
    HWND opt_check = nullptr;
    HWND min_edit = nullptr;
    HWND max_edit = nullptr;
    post2::core::TriggerCondition trigger;
    // When non-null + non-empty variable_path, the editor exposes an Opt /
    // Min / Max row that registers the trigger value as an optimization
    // variable in *optimization. nullptr/empty disables the Opt row.
    post2::core::OptimizationConfig* optimization = nullptr;
    std::string variable_path;
    bool accepted = false;
};

struct EventEditorDialogState {
    HWND hwnd = nullptr;
    HWND name_edit = nullptr;
    HWND enabled_check = nullptr;
    HWND trigger_summary_label = nullptr;
    HWND trigger_edit_button = nullptr;
    HWND action_list = nullptr;
    HWND action_add_button = nullptr;
    HWND action_edit_button = nullptr;
    HWND action_delete_button = nullptr;
    post2::core::EventConfig event;
    const post2::vehicle::VehicleConfig* vehicle = nullptr;
    int selected_action_index = -1;
    bool accepted = false;
};

struct CaseEventsManagerDialogState {
    HWND hwnd = nullptr;
    HWND event_list = nullptr;
    HWND add_button = nullptr;
    HWND edit_button = nullptr;
    HWND delete_button = nullptr;
    std::vector<post2::core::EventConfig> events;
    const post2::vehicle::VehicleConfig* vehicle = nullptr;
    int selected_index = -1;
    bool accepted = false;
};

struct OptimizationObjectiveEditorDialogState {
    HWND hwnd = nullptr;
    HWND enabled_check = nullptr;
    HWND metric_combo = nullptr;
    HWND direction_combo = nullptr;
    HWND weight_edit = nullptr;
    post2::core::OptimizationObjectiveConfig objective;
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

std::string lowercase(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
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
        if (phase.termination.type == "time") {
            phase.termination.value = g_config.duration_s;
        }
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

bool optimization_is_running()
{
    std::lock_guard<std::mutex> lock(g_optimization_mutex);
    return g_optimization_running;
}

std::string optimizing_status_text()
{
    return std::string("optimizing... | mode: ") + post2::core::core_mode_name(g_mode) +
        " | remote: " + remote_endpoint_text();
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
        << "Found feasible: " << (result.found_feasible ? "true" : "false") << '\n'
        << "Iterations: " << result.iterations << '\n'
        << "Evaluations: " << result.evaluations << '\n'
        << "Best score: " << result.best_score << '\n'
        << "Max violation: " << result.max_constraint_violation << '\n'
        << "L1 violation: " << result.l1_constraint_violation << '\n'
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
    g_predicted_orbit = (g_result.ok && !g_result.state_log.empty())
        ? post2::core::predict_orbit_path(g_case, g_result.state_log)
        : StateLog{};
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
    populate_charts();
    sync_scene_window(hwnd);
    InvalidateRect(hwnd, nullptr, TRUE);
}

void execute_optimization(HWND hwnd)
{
    ensure_case_initialized();

    bool already_running = false;
    {
        std::lock_guard<std::mutex> lock(g_optimization_mutex);
        if (g_optimization_running) {
            already_running = true;
        } else {
            g_optimization_running = true;
            g_pending_optimization.reset();
        }
    }
    if (already_running) {
        MessageBoxW(hwnd, L"Optimization is already running.", L"POST2 Lite", MB_ICONINFORMATION);
        return;
    }

    CaseConfig working_case = g_case;
    const CoreMode mode = g_mode;
    const std::string remote_host = g_remote_host;
    const int remote_port = g_remote_port;

    g_status = optimizing_status_text();
    set_outputs_text("Optimizing...");
    InvalidateRect(hwnd, nullptr, TRUE);

    std::thread([hwnd,
                 working_case = std::move(working_case),
                 mode,
                 remote_host,
                 remote_port]() mutable {
        OptimizationResult result;
        try {
            const auto service =
                post2::core::make_trajectory_service(mode, remote_host, remote_port);
            result = post2::core::optimize_case(&working_case, *service);
        } catch (const std::exception& ex) {
            result.error = ex.what();
        } catch (...) {
            result.error = "unknown optimization error";
        }

        {
            std::lock_guard<std::mutex> lock(g_optimization_mutex);
            g_pending_optimization =
                PendingOptimizationResult{std::move(working_case), std::move(result)};
        }
        PostMessageW(hwnd, kOptimizationFinishedMessage, 0, 0);
    }).detach();
}

void finish_optimization(HWND hwnd)
{
    std::optional<PendingOptimizationResult> pending;
    {
        std::lock_guard<std::mutex> lock(g_optimization_mutex);
        pending = std::move(g_pending_optimization);
        g_pending_optimization.reset();
        g_optimization_running = false;
    }
    if (!pending.has_value()) {
        return;
    }

    g_case = std::move(pending->case_config);
    const OptimizationResult& result = pending->result;
    set_outputs_text(format_optimization_outputs(result));
    if (!result.ok) {
        g_result = result.final_simulation;
        g_status = "optimize failed: " +
            (result.error.empty() ? std::string("optimization failed") : result.error);
        sync_scene_window(hwnd);
        InvalidateRect(hwnd, nullptr, TRUE);
        MessageBoxW(
            hwnd,
            widen(result.error.empty() ? "Optimization failed." : result.error).c_str(),
            L"Optimize failed",
            MB_ICONERROR);
        return;
    }

    g_result = result.final_simulation;
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
    g_predicted_orbit = (g_result.ok && !g_result.state_log.empty())
        ? post2::core::predict_orbit_path(g_case, g_result.state_log)
        : StateLog{};
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
    populate_charts();
    sync_scene_window(hwnd);
    InvalidateRect(hwnd, nullptr, TRUE);
}

RECT scene_rect(HWND hwnd)
{
    RECT client;
    GetClientRect(hwnd, &client);
    client.left = kContentColumnX;
    client.right -= 24;
    // button top + button height + 8 px gap
    client.top += kViewButtonTop + kViewButtonHeight + 8;
    client.bottom -= 24;
    if (client.right < client.left) {
        client.right = client.left;
    }
    if (client.bottom < client.top) {
        client.bottom = client.top;
    }
    return client;
}

post2::gui::ChartPanel* chart_for_view(ViewKind view)
{
    switch (view) {
    case ViewKind::Profile2D:        return &g_chart_profile;
    case ViewKind::DynamicPressure:  return &g_chart_q;
    case ViewKind::Throttle:         return &g_chart_throttle;
    case ViewKind::Speed:            return &g_chart_speed;
    case ViewKind::Mass:             return &g_chart_mass;
    default:                          return nullptr;
    }
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

void apply_active_view_visibility()
{
    const bool has_data = scene_has_state();
    const bool show_scene = has_data && g_active_view == ViewKind::Scene3D;
    if (window_is_live(g_scene_hwnd)) {
        ShowWindow(g_scene_hwnd, show_scene ? SW_SHOWNA : SW_HIDE);
    }
    auto sync_chart = [&](post2::gui::ChartPanel& panel, ViewKind v) {
        const bool show_chart = has_data && g_active_view == v;
        if (show_chart) panel.show(); else panel.hide();
    };
    sync_chart(g_chart_profile, ViewKind::Profile2D);
    sync_chart(g_chart_q, ViewKind::DynamicPressure);
    sync_chart(g_chart_throttle, ViewKind::Throttle);
    sync_chart(g_chart_speed, ViewKind::Speed);
    sync_chart(g_chart_mass, ViewKind::Mass);
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
    auto move_chart = [&](post2::gui::ChartPanel& panel) {
        if (panel.hwnd()) {
            MoveWindow(panel.hwnd(), rect.left, rect.top, width, height, TRUE);
            panel.resize(width, height);
        }
    };
    move_chart(g_chart_profile);
    move_chart(g_chart_q);
    move_chart(g_chart_throttle);
    move_chart(g_chart_speed);
    move_chart(g_chart_mass);
    apply_active_view_visibility();
    invalidate_scene_window();
}

void switch_view(HWND hwnd, ViewKind view)
{
    g_active_view = view;
    for (int i = 0; i < kViewButtonCount; ++i) {
        if (g_view_buttons[i]) {
            const bool pressed = static_cast<int>(view) == i;
            Button_SetCheck(g_view_buttons[i], pressed ? BST_CHECKED : BST_UNCHECKED);
        }
    }
    apply_active_view_visibility();
    InvalidateRect(hwnd, nullptr, FALSE);
}

double great_circle_arc_m(double lat1_rad, double lon1_rad, double lat2_rad, double lon2_rad)
{
    const double dlon = lon2_rad - lon1_rad;
    const double cos_arg = std::sin(lat1_rad) * std::sin(lat2_rad)
        + std::cos(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon);
    const double clamped = std::max(-1.0, std::min(1.0, cos_arg));
    return post2::core::frames::Wgs84::a_m * std::acos(clamped);
}

void populate_charts()
{
    using post2::gui::ChartConfig;
    using post2::gui::ChartSeries;
    if (!g_result.ok || g_result.state_log.empty()) {
        ChartConfig empty;
        g_chart_profile.set_data(empty);
        g_chart_q.set_data(empty);
        g_chart_throttle.set_data(empty);
        g_chart_speed.set_data(empty);
        g_chart_mass.set_data(empty);
        return;
    }
    const auto& entries = g_result.state_log.entries();
    constexpr double kDegToRadLocal = 3.141592653589793238462643383279502884 / 180.0;
    const double launch_lat = g_case.launch_site.latitude_deg * kDegToRadLocal;
    const double launch_lon = g_case.launch_site.longitude_deg * kDegToRadLocal;

    // 2D profile range: from takeoff (first non-clamped entry) to last entry
    // with engine_thrust_n > 0 (all-engines-off cutoff).
    std::size_t profile_start = 0;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        if (!entries[i].hold_down_clamp_active) {
            profile_start = i;
            break;
        }
    }
    std::size_t profile_end = profile_start;
    bool found_thrust = false;
    for (std::size_t i = profile_start; i < entries.size(); ++i) {
        if (entries[i].engine_thrust_n > 0.0) {
            profile_end = i;
            found_thrust = true;
        }
    }
    if (!found_thrust) {
        profile_end = entries.size() - 1;
    }

    ChartSeries profile_s, q_s, throttle_s, speed_s, mass_s;
    const std::size_t powered_count = profile_end - profile_start + 1;
    profile_s.x.reserve(powered_count);
    profile_s.y.reserve(powered_count);
    q_s.x.reserve(powered_count);
    q_s.y.reserve(powered_count);
    throttle_s.x.reserve(powered_count);
    throttle_s.y.reserve(powered_count);
    speed_s.x.reserve(powered_count);
    speed_s.y.reserve(powered_count);
    mass_s.x.reserve(powered_count);
    mass_s.y.reserve(powered_count);

    for (std::size_t i = profile_start; i <= profile_end; ++i) {
        const auto& entry = entries[i];
        const post2::core::frames::Geodetic geo =
            post2::core::frames::ecef_to_geodetic(entry.state.position_m);
        const double downrange_m = great_circle_arc_m(
            launch_lat, launch_lon, geo.latitude_rad, geo.longitude_rad);
        profile_s.x.push_back(downrange_m / 1000.0);
        profile_s.y.push_back(entry.altitude_m / 1000.0);
        q_s.x.push_back(entry.time_s);
        q_s.y.push_back(entry.dynamic_pressure_pa / 1000.0);
        throttle_s.x.push_back(entry.time_s);
        throttle_s.y.push_back(entry.throttle);
        speed_s.x.push_back(entry.time_s);
        speed_s.y.push_back(entry.speed_mps);
        mass_s.x.push_back(entry.time_s);
        mass_s.y.push_back(entry.total_mass_kg);
    }

    ChartConfig profile_cfg;
    profile_cfg.title = L"Launch profile (liftoff to engine cutoff)";
    profile_cfg.x_label = L"Downrange [km]";
    profile_cfg.y_label = L"Altitude";
    profile_cfg.y_unit = L"km";
    profile_cfg.data = std::move(profile_s);
    g_chart_profile.set_data(std::move(profile_cfg));

    ChartConfig q_cfg;
    q_cfg.title = L"Dynamic pressure";
    q_cfg.x_label = L"Time [s]";
    q_cfg.y_label = L"Q";
    q_cfg.y_unit = L"kPa";
    q_cfg.data = std::move(q_s);
    q_cfg.mark_peak = true;
    g_chart_q.set_data(std::move(q_cfg));

    ChartConfig throttle_cfg;
    throttle_cfg.title = L"Throttle command";
    throttle_cfg.x_label = L"Time [s]";
    throttle_cfg.y_label = L"Throttle";
    throttle_cfg.y_unit = L"0-1";
    throttle_cfg.data = std::move(throttle_s);
    g_chart_throttle.set_data(std::move(throttle_cfg));

    ChartConfig speed_cfg;
    speed_cfg.title = L"Speed (inertial)";
    speed_cfg.x_label = L"Time [s]";
    speed_cfg.y_label = L"Speed";
    speed_cfg.y_unit = L"m/s";
    speed_cfg.data = std::move(speed_s);
    g_chart_speed.set_data(std::move(speed_cfg));

    ChartConfig mass_cfg;
    mass_cfg.title = L"Vehicle total mass";
    mass_cfg.x_label = L"Time [s]";
    mass_cfg.y_label = L"Mass";
    mass_cfg.y_unit = L"kg";
    mass_cfg.data = std::move(mass_s);
    g_chart_mass.set_data(std::move(mass_cfg));
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

// Short human label for one aero table row: its activation level (which staging
// configuration it applies to), reference area, and the table file name.
std::wstring format_aero_table_label(const post2::vehicle::AeroStageTable& table)
{
    const auto slash = table.table_path.find_last_of("/\\");
    const std::string file = slash != std::string::npos
        ? table.table_path.substr(slash + 1) : table.table_path;
    std::wostringstream label;
    label << L"L" << table.activate_at_min_attached_stage << L"  S="
          << std::fixed << std::setprecision(2) << table.reference_area_m2 << L" m2  "
          << widen(file.empty() ? std::string("(no file)") : file);
    return label.str();
}

// Repopulate the aero-table listbox from config.aero.stage_tables and restore the
// selection. Tables are kept ascending by activation level.
void refresh_vehicle_aero_table_list(VehicleSettingsDialogState* state)
{
    if (!state || !state->aero_table_list) {
        return;
    }
    auto& tables = state->config.aero.stage_tables;
    std::sort(tables.begin(), tables.end(),
              [](const post2::vehicle::AeroStageTable& a, const post2::vehicle::AeroStageTable& b) {
                  return a.activate_at_min_attached_stage < b.activate_at_min_attached_stage;
              });
    SendMessageW(state->aero_table_list, LB_RESETCONTENT, 0, 0);
    for (const auto& table : tables) {
        SendMessageW(state->aero_table_list, LB_ADDSTRING, 0,
                     reinterpret_cast<LPARAM>(format_aero_table_label(table).c_str()));
    }
    if (!tables.empty()) {
        if (state->selected_aero_table_index < 0 ||
            static_cast<std::size_t>(state->selected_aero_table_index) >= tables.size()) {
            state->selected_aero_table_index = 0;
        }
        SendMessageW(state->aero_table_list, LB_SETCURSEL,
                     static_cast<WPARAM>(state->selected_aero_table_index), 0);
    }
    if (state->gl_preview) {
        InvalidateRect(state->gl_preview, nullptr, FALSE);
    }
}

// ---- Increment 3: embedded OpenGL vehicle preview ---------------------------
//
// A child window with its own GL context that draws the whole rocket as a stack
// of immediate-mode primitives (a core cylinder + a fairing/nose cone) sized from
// the aero table geometry. It is read-only: it reflects whatever geometry the
// aero tables carry and is repainted when the table list changes.

constexpr double kPreviewPi = 3.14159265358979323846;

struct VehiclePreviewState {
    HGLRC glrc = nullptr;
    const post2::vehicle::AeroConfig* aero = nullptr;  // owned by the dialog state
    double yaw_deg = 30.0;
    double pitch_deg = 12.0;
    double zoom = 1.0;
    bool dragging = false;
    POINT last = {};
};

// Pick the geometry to draw: the full-stack table (lowest activation level) that
// carries a positive length, falling back to the legacy aero geometry fields.
// Returns false when nothing has usable dimensions.
bool resolve_preview_geometry(const post2::vehicle::AeroConfig& aero,
                              double* length, double* nose, double* core_r, double* fairing_r)
{
    const post2::vehicle::AeroStageTable* best = nullptr;
    for (const auto& t : aero.stage_tables) {
        if (t.body_length_m <= 0.0) {
            continue;
        }
        if (!best || t.activate_at_min_attached_stage < best->activate_at_min_attached_stage) {
            best = &t;
        }
    }
    double len = best ? best->body_length_m : aero.body_length_m;
    double nose_len = best ? best->nose_length_m : aero.nose_length_m;
    double core_d = best ? best->base_diameter_m : aero.base_diameter_m;
    double fairing_d = best ? best->ref_diameter_m : aero.ref_diameter_m;
    if (len <= 0.0) {
        // No table / geometry: fall back to a representative body sized from the
        // reference area (diameter = sqrt(4 S / pi)) with a default slenderness,
        // so a constant-CD case still shows its caliber instead of a blank view.
        if (aero.reference_area_m2 > 0.0) {
            const double diameter = std::sqrt(4.0 * aero.reference_area_m2 / kPreviewPi);
            core_d = diameter;
            fairing_d = diameter;
            len = 12.0 * diameter;
            nose_len = 0.0;  // filled in below
        } else {
            return false;
        }
    }
    if (nose_len <= 0.0 || nose_len >= len) {
        nose_len = 0.15 * len;
    }
    if (core_d <= 0.0) {
        core_d = fairing_d > 0.0 ? fairing_d : 0.1 * len;
    }
    if (fairing_d <= 0.0) {
        fairing_d = core_d;
    }
    *length = len;
    *nose = nose_len;
    *core_r = 0.5 * core_d;
    *fairing_r = 0.5 * fairing_d;
    return true;
}

// Truncated cone (frustum) from radius r0 at y0 to radius r1 at y1, about the Y
// axis. r1 == r0 gives a cylinder; r1 == 0 gives a cone. Normals point outward
// with the correct slope (rely on GL_NORMALIZE so they need not be unit length).
void preview_draw_frustum(double r0, double r1, double y0, double y1, int segments)
{
    const double dy = y1 - y0;
    const double dr = r0 - r1;
    const double slope = std::sqrt(dy * dy + dr * dr) > 1.0e-9 ? dr / std::abs(dy) : 0.0;
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        const double a = 2.0 * kPreviewPi * i / segments;
        const double c = std::cos(a);
        const double s = std::sin(a);
        glNormal3d(c, slope, s);
        glVertex3d(r0 * c, y0, r0 * s);
        glVertex3d(r1 * c, y1, r1 * s);
    }
    glEnd();
}

void preview_draw_annulus(double r_inner, double r_outer, double y, double normal_y, int segments)
{
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        const double a = 2.0 * kPreviewPi * i / segments;
        const double c = std::cos(a);
        const double s = std::sin(a);
        glNormal3d(0.0, normal_y, 0.0);
        glVertex3d(r_inner * c, y, r_inner * s);
        glVertex3d(r_outer * c, y, r_outer * s);
    }
    glEnd();
}

void render_vehicle_preview(HWND hwnd, VehiclePreviewState* state)
{
    if (!state || !state->glrc) {
        return;
    }
    RECT rc;
    GetClientRect(hwnd, &rc);
    const int w = std::max<int>(1, rc.right);
    const int h = std::max<int>(1, rc.bottom);

    HDC dc = GetDC(hwnd);
    wglMakeCurrent(dc, state->glrc);

    glViewport(0, 0, w, h);
    glClearColor(0.10f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    double length = 0.0, nose = 0.0, core_r = 0.0, fairing_r = 0.0;
    const bool ok = state->aero &&
        resolve_preview_geometry(*state->aero, &length, &nose, &core_r, &fairing_r);

    const double aspect = static_cast<double>(w) / static_cast<double>(h);
    const double fov = 40.0 * kPreviewPi / 180.0;
    const double near_p = 0.1;
    const double top = near_p * std::tan(0.5 * fov);
    const double right = top * aspect;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (ok) {
        const double radius = std::max(fairing_r, 0.5 * length);
        const double dist = length * 1.2 / std::max(0.2, state->zoom) + radius;
        const double far_p = dist + 3.0 * length + 10.0;
        glFrustum(-right, right, -top, top, near_p, far_p);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_NORMALIZE);
        const GLfloat light_pos[4] = {0.4f, 1.0f, 0.8f, 0.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

        glTranslated(0.0, 0.0, -dist);
        glRotated(state->pitch_deg, 1.0, 0.0, 0.0);
        glRotated(state->yaw_deg, 0.0, 1.0, 0.0);
        glTranslated(0.0, -0.5 * length, 0.0);  // centre the rocket

        const int seg = 36;
        const double y_core_top = length - nose;  // where the nose / fairing begins
        glColor3d(0.80, 0.82, 0.86);
        preview_draw_frustum(core_r, core_r, 0.0, y_core_top, seg);  // core body
        preview_draw_annulus(0.0, core_r, 0.0, -1.0, seg);           // bottom cap

        glColor3d(0.93, 0.94, 0.97);
        if (fairing_r > core_r + 1.0e-6) {
            // Distinct fairing wider than the core: a tapered shoulder, a straight
            // fairing barrel, then the ogive-like nose cap.
            const double shoulder = 0.25 * nose;
            const double barrel = 0.40 * nose;
            const double y1 = y_core_top + shoulder;
            const double y2 = y1 + barrel;
            preview_draw_frustum(core_r, fairing_r, y_core_top, y1, seg);  // shoulder
            preview_draw_frustum(fairing_r, fairing_r, y1, y2, seg);       // barrel
            preview_draw_frustum(fairing_r, 0.0, y2, length, seg);         // nose cap
        } else {
            // No distinct fairing: a plain nose cone over the core.
            preview_draw_frustum(core_r, 0.0, y_core_top, length, seg);
        }
        glDisable(GL_LIGHTING);
    } else {
        glFrustum(-right, right, -top, top, near_p, 100.0);
    }

    SwapBuffers(dc);
    wglMakeCurrent(nullptr, nullptr);
    ReleaseDC(hwnd, dc);
}

LRESULT CALLBACK vehicle_preview_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<VehiclePreviewState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE: {
        HDC dc = GetDC(hwnd);
        PIXELFORMATDESCRIPTOR pfd = {};
        pfd.nSize = sizeof(pfd);
        pfd.nVersion = 1;
        pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
        pfd.iPixelType = PFD_TYPE_RGBA;
        pfd.cColorBits = 24;
        pfd.cDepthBits = 24;
        pfd.iLayerType = PFD_MAIN_PLANE;
        const int pf = ChoosePixelFormat(dc, &pfd);
        SetPixelFormat(dc, pf, &pfd);
        state->glrc = wglCreateContext(dc);
        ReleaseDC(hwnd, dc);
        return 0;
    }
    case WM_ERASEBKGND:
        return 1;  // GL clears the surface; skip the flicker-prone default erase
    case WM_PAINT: {
        PAINTSTRUCT ps;
        BeginPaint(hwnd, &ps);
        render_vehicle_preview(hwnd, state);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_LBUTTONDOWN:
        state->dragging = true;
        state->last.x = GET_X_LPARAM(lparam);
        state->last.y = GET_Y_LPARAM(lparam);
        SetCapture(hwnd);
        return 0;
    case WM_MOUSEMOVE:
        if (state->dragging) {
            const int x = GET_X_LPARAM(lparam);
            const int y = GET_Y_LPARAM(lparam);
            state->yaw_deg += 0.5 * (x - state->last.x);
            state->pitch_deg += 0.5 * (y - state->last.y);
            state->pitch_deg = std::max(-89.0, std::min(89.0, state->pitch_deg));
            state->last.x = x;
            state->last.y = y;
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case WM_LBUTTONUP:
        state->dragging = false;
        ReleaseCapture();
        return 0;
    case WM_MOUSEWHEEL: {
        const int delta = GET_WHEEL_DELTA_WPARAM(wparam);
        state->zoom *= delta > 0 ? 1.1 : 1.0 / 1.1;
        state->zoom = std::max(0.25, std::min(4.0, state->zoom));
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;
    }
    case WM_NCDESTROY:
        if (state) {
            if (state->glrc) {
                wglMakeCurrent(nullptr, nullptr);
                wglDeleteContext(state->glrc);
            }
            delete state;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
        }
        return 0;
    default:
        break;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_vehicle_preview_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    WNDCLASSW wc = {};
    wc.lpfnWndProc = vehicle_preview_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2VehiclePreviewWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.style = CS_OWNDC;  // a stable DC for the GL context
    RegisterClassW(&wc);
    registered = true;
}

HWND create_vehicle_preview_window(HWND parent, post2::vehicle::AeroConfig* aero,
                                   int x, int y, int width, int height)
{
    register_vehicle_preview_class();
    auto* state = new VehiclePreviewState();
    state->aero = aero;
    HWND preview = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"Post2VehiclePreviewWindow",
        L"",
        WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
        x, y, width, height,
        parent,
        nullptr,
        g_instance,
        state);
    if (!preview) {
        delete state;
    }
    return preview;
}

void create_vehicle_dialog_controls(VehicleSettingsDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    ensure_vehicle_stages(state->config);

    create_label(state->hwnd, 18, 18, 90, L"Name", font);
    state->name_edit = create_edit(state->hwnd, kVehicleNameEdit, 118, 14, 332, widen(state->config.name), font);

    create_label(state->hwnd, 18, 52, 90, L"Dry mass kg", font);
    state->dry_mass_edit = create_edit(state->hwnd, kVehicleDryMassEdit, 118, 48, 120, format_double(state->config.dry_mass_kg), font);

    state->aero_enabled = create_checkbox(
        state->hwnd,
        kVehicleAeroEnabled,
        18,
        84,
        130,
        L"Aero drag",
        font);

    Button_SetCheck(state->aero_enabled, state->config.aero.enabled ? BST_CHECKED : BST_UNCHECKED);

    // Aero tables: one CD/CL(Mach, alpha) table per staging configuration. The
    // reference area is a property of each table's geometry, so there is no longer
    // a standalone area / Cd / Cl field -- those came from the table set instead.
    create_label(state->hwnd, 18, 116, 200, L"Aero tables (per staging config)", font);
    state->aero_table_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL | WS_HSCROLL,
        18,
        140,
        482,
        92,
        state->hwnd,
        control_id(kVehicleAeroTableList),
        g_instance,
        nullptr);
    set_child_font(state->aero_table_list, font);
    // Migrate a legacy single-table config (only aero_table_path set, no
    // stage_tables) into one full-stack (L0) entry so it shows in the list.
    if (state->config.aero.stage_tables.empty() && !state->config.aero.aero_table_path.empty()) {
        post2::vehicle::AeroStageTable legacy;
        legacy.activate_at_min_attached_stage = 0;
        legacy.table_path = state->config.aero.aero_table_path;
        legacy.reference_area_m2 = state->config.aero.reference_area_m2;
        legacy.ref_diameter_m = state->config.aero.ref_diameter_m;
        legacy.body_length_m = state->config.aero.body_length_m;
        legacy.nose_length_m = state->config.aero.nose_length_m;
        legacy.base_diameter_m = state->config.aero.base_diameter_m;
        state->config.aero.stage_tables.push_back(legacy);
    }
    refresh_vehicle_aero_table_list(state);

    create_button(state->hwnd, kVehicleAeroTableImport, 18, 240, 90, 26, L"Import...", font);
    create_button(state->hwnd, kVehicleAeroTableView, 116, 240, 72, 26, L"View", font);
    create_button(state->hwnd, kVehicleAeroTableEditStage, 196, 240, 110, 26, L"Edit stage", font);
    create_button(state->hwnd, kVehicleAeroTableDelete, 314, 240, 72, 26, L"Delete", font);

    create_label(state->hwnd, 18, 280, 90, L"Stages", font);
    state->stage_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18,
        304,
        482,
        104,
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

    create_button(state->hwnd, kVehicleStageAdd, 18, 416, 76, 28, L"Add", font);
    create_button(state->hwnd, kVehicleStageEdit, 108, 416, 76, 28, L"Edit", font);
    create_button(state->hwnd, kVehicleStageDelete, 198, 416, 76, 28, L"Delete", font);

    // Embedded 3D vehicle preview (right column). The OpenGL child window is
    // created and rendered in create_vehicle_preview_window; it draws the stacked
    // rocket assembled from the aero tables' geometry.
    create_label(state->hwnd, 520, 14, 220, L"Vehicle preview", font);
    state->gl_preview = create_vehicle_preview_window(state->hwnd, &state->config.aero,
                                                      520, 36, 222, 408);

    create_button(state->hwnd, IDOK, 560, 462, 76, 28, L"OK", font);
    create_button(state->hwnd, IDCANCEL, 650, 462, 76, 28, L"Cancel", font);
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
        // Accept values sitting on a bound to within floating-point rounding.
        // Optimized results routinely land exactly on a Min/Max and pick up a
        // few ULPs of error when written to / read back from the case JSON, so a
        // strict compare would spuriously reject an otherwise valid case; only
        // flag genuine violations.
        const double bound_tol = 1.0e-9 * (1.0 + std::abs(min_value) + std::abs(max_value));
        if (opt_enabled && (value < min_value - bound_tol || value > max_value + bound_tol)) {
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
    state->engine_thrust_edit = create_edit(state->hwnd, kVehicleEngineThrustEdit, 118, 176, 120, format_double(engine.thrust_vac_n), font);

    create_label(state->hwnd, 258, 180, 55, L"Isp s", font);
    state->engine_isp_edit = create_edit(state->hwnd, kVehicleEngineIspEdit, 318, 176, 100, format_double(engine.isp_vac_s), font);

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
        !read_double_field(state->hwnd, state->engine_thrust_edit, L"Max thrust", &stage.engine.thrust_vac_n) ||
        !read_double_field(state->hwnd, state->engine_isp_edit, L"Isp", &stage.engine.isp_vac_s) ||
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
    config.aero.enabled = Button_GetCheck(state->aero_enabled) == BST_CHECKED;
    // The aero table set (config.aero.stage_tables) is edited in place by the
    // Import / Edit stage / Delete handlers; reference area / Cd / Cl no longer
    // have dialog fields (they derive from the tables). Keep the legacy primary
    // pointer and reference area in sync with the level-0 (full-stack) table.
    config.aero.use_table = !config.aero.stage_tables.empty();
    for (const auto& table : config.aero.stage_tables) {
        if (table.activate_at_min_attached_stage == 0) {
            config.aero.aero_table_path = table.table_path;
            config.aero.reference_area_m2 = table.reference_area_m2;
            config.aero.ref_diameter_m = table.ref_diameter_m;
            config.aero.body_length_m = table.body_length_m;
            config.aero.nose_length_m = table.nose_length_m;
            config.aero.base_diameter_m = table.base_diameter_m;
            break;
        }
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

// ---- Small modal integer prompt (used to edit an aero table's stage level) ----

struct IntPromptState {
    HWND edit = nullptr;
    int value = 0;
    bool accepted = false;
};

LRESULT CALLBACK int_prompt_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<IntPromptState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK: {
            const std::wstring text = get_window_text(state->edit);
            try {
                std::size_t consumed = 0;
                const int parsed = std::stoi(text, &consumed);
                if (consumed != text.size()) {
                    throw std::invalid_argument("trailing characters");
                }
                state->value = parsed;
                state->accepted = true;
                DestroyWindow(hwnd);
            } catch (const std::exception&) {
                MessageBoxW(hwnd, L"Enter an integer.", L"Edit", MB_ICONWARNING);
            }
            return 0;
        }
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

bool prompt_integer(HWND parent, const wchar_t* title, const wchar_t* label, int* value)
{
    static bool registered = false;
    if (!registered) {
        WNDCLASSW wc = {};
        wc.lpfnWndProc = int_prompt_proc;
        wc.hInstance = g_instance;
        wc.lpszClassName = L"Post2IntPromptWindow";
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
        RegisterClassW(&wc);
        registered = true;
    }

    IntPromptState state;
    state.value = *value;

    RECT pr;
    GetWindowRect(parent, &pr);
    constexpr int w = 320;
    constexpr int h = 156;
    const int x = pr.left + ((pr.right - pr.left) - w) / 2;
    const int y = pr.top + ((pr.bottom - pr.top) - h) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2IntPromptWindow",
        title,
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x, y, w, h, parent, nullptr, g_instance, &state);
    if (!dialog) {
        return false;
    }

    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    create_label(dialog, 18, 16, 280, label, font);
    state.edit = create_edit(dialog, 0, 18, 58, 280, std::to_wstring(*value), font);
    create_button(dialog, IDOK, 140, 90, 70, 26, L"OK", font);
    create_button(dialog, IDCANCEL, 220, 90, 70, 26, L"Cancel", font);

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.edit);

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
        *value = state.value;
    }
    return state.accepted;
}

bool choose_open_file(HWND parent, const wchar_t* title, const wchar_t* filter, std::string* path)
{
    std::vector<wchar_t> buffer(1024, 0);
    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = parent;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = buffer.data();
    ofn.nMaxFile = static_cast<DWORD>(buffer.size());
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (!GetOpenFileNameW(&ofn)) {
        return false;
    }
    *path = narrow(buffer.data());
    return true;
}

// ---- Aero table list handlers ----

// Defined below (Increment 2): in-GUI CD/CL(Mach, alpha) table viewer.
void show_aero_table_viewer(HWND parent, const std::string& path);

// Find and select the table at the given activation level after a re-sort.
void select_aero_table_at_level(VehicleSettingsDialogState* state, int activate)
{
    const auto& tables = state->config.aero.stage_tables;
    for (std::size_t i = 0; i < tables.size(); ++i) {
        if (tables[i].activate_at_min_attached_stage == activate) {
            state->selected_aero_table_index = static_cast<int>(i);
            SendMessageW(state->aero_table_list, LB_SETCURSEL, static_cast<WPARAM>(i), 0);
            return;
        }
    }
}

void import_aero_table(VehicleSettingsDialogState* state)
{
    std::string path;
    if (!choose_open_file(state->hwnd, L"Import aero table CSV",
            L"Aero table CSV (*.csv)\0*.csv\0All files\0*.*\0\0", &path)) {
        return;
    }
    post2::aero::AeroTable table;
    std::string err;
    if (!post2::aero::read_aero_table_csv(path, &table, &err)) {
        MessageBoxW(state->hwnd, widen(err).c_str(), L"Import aero table", MB_ICONWARNING);
        return;
    }

    auto& tables = state->config.aero.stage_tables;
    int activate = 0;
    for (const auto& t : tables) {
        activate = std::max(activate, t.activate_at_min_attached_stage + 1);
    }
    if (!prompt_integer(state->hwnd, L"Aero table stage",
            L"Activates after this many lower stages separate (0 = full stack):", &activate)) {
        return;
    }
    if (activate < 0) {
        activate = 0;
    }

    post2::vehicle::AeroStageTable entry;
    entry.activate_at_min_attached_stage = activate;
    entry.table_path = path;
    entry.reference_area_m2 = table.reference_area_m2;
    // Replace any existing table that already claims this activation level.
    tables.erase(std::remove_if(tables.begin(), tables.end(),
        [&](const post2::vehicle::AeroStageTable& t) {
            return t.activate_at_min_attached_stage == activate;
        }), tables.end());
    tables.push_back(entry);

    state->config.aero.use_table = true;
    state->config.aero.enabled = true;
    Button_SetCheck(state->aero_enabled, BST_CHECKED);
    refresh_vehicle_aero_table_list(state);
    select_aero_table_at_level(state, activate);
}

void edit_aero_table_stage(VehicleSettingsDialogState* state)
{
    auto& tables = state->config.aero.stage_tables;
    const int idx = state->selected_aero_table_index;
    if (idx < 0 || static_cast<std::size_t>(idx) >= tables.size()) {
        return;
    }
    int activate = tables[static_cast<std::size_t>(idx)].activate_at_min_attached_stage;
    if (!prompt_integer(state->hwnd, L"Aero table stage",
            L"Activates after this many lower stages separate (0 = full stack):", &activate)) {
        return;
    }
    if (activate < 0) {
        activate = 0;
    }
    tables[static_cast<std::size_t>(idx)].activate_at_min_attached_stage = activate;
    refresh_vehicle_aero_table_list(state);
    select_aero_table_at_level(state, activate);
}

void delete_aero_table(VehicleSettingsDialogState* state)
{
    auto& tables = state->config.aero.stage_tables;
    const int idx = state->selected_aero_table_index;
    if (idx < 0 || static_cast<std::size_t>(idx) >= tables.size()) {
        return;
    }
    tables.erase(tables.begin() + idx);
    if (state->selected_aero_table_index > 0 &&
        static_cast<std::size_t>(state->selected_aero_table_index) >= tables.size()) {
        state->selected_aero_table_index = static_cast<int>(tables.size()) - 1;
    }
    if (tables.empty()) {
        state->config.aero.use_table = false;
    }
    refresh_vehicle_aero_table_list(state);
}

void view_aero_table(VehicleSettingsDialogState* state)
{
    auto& tables = state->config.aero.stage_tables;
    const int idx = state->selected_aero_table_index;
    if (idx < 0 || static_cast<std::size_t>(idx) >= tables.size()) {
        MessageBoxW(state->hwnd, L"Select an aero table first.", L"View aero table", MB_ICONINFORMATION);
        return;
    }
    show_aero_table_viewer(state->hwnd, tables[static_cast<std::size_t>(idx)].table_path);
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
        case kVehicleAeroTableList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const LRESULT selected = SendMessageW(state->aero_table_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    state->selected_aero_table_index = static_cast<int>(selected);
                }
                return 0;
            }
            break;
        case kVehicleAeroTableImport:
            import_aero_table(state);
            return 0;
        case kVehicleAeroTableView:
            view_aero_table(state);
            return 0;
        case kVehicleAeroTableEditStage:
            edit_aero_table_stage(state);
            return 0;
        case kVehicleAeroTableDelete:
            delete_aero_table(state);
            return 0;
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

// ---- Increment 2: in-GUI aero table viewer ----------------------------------

struct AeroViewerState {
    HWND edit = nullptr;
};

LRESULT CALLBACK aero_table_viewer_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<AeroViewerState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_SIZE:
        if (state && state->edit) {
            RECT rc;
            GetClientRect(hwnd, &rc);
            MoveWindow(state->edit, 0, 0, rc.right, rc.bottom, TRUE);
        }
        return 0;
    case WM_COMMAND:
        if (LOWORD(wparam) == IDCANCEL) {
            DestroyWindow(hwnd);
            return 0;
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

// Render one CD/CL(Mach, alpha) table as a fixed-width text grid.
std::wstring format_aero_table_text(const std::string& path, const post2::aero::AeroTable& table)
{
    std::wostringstream out;
    out << L"File: " << widen(path) << L"\r\n";
    out << L"Reference area: " << std::fixed << std::setprecision(3)
        << table.reference_area_m2 << L" m2\r\n";
    out << L"Grid: " << table.mach.size() << L" Mach x " << table.alpha_deg.size()
        << L" alpha\r\n\r\n";

    const auto block = [&](const wchar_t* name, const std::vector<double>& vals) {
        out << name << L"  (rows = Mach, cols = alpha deg)\r\n";
        out << L"  Mach \\ a |";
        for (double a : table.alpha_deg) {
            out << std::setw(8) << std::setprecision(1) << std::fixed << a;
        }
        out << L"\r\n";
        for (std::size_t i = 0; i < table.mach.size(); ++i) {
            out << std::setw(9) << std::setprecision(3) << std::fixed << table.mach[i] << L" |";
            for (std::size_t j = 0; j < table.alpha_deg.size(); ++j) {
                const std::size_t k = i * table.alpha_deg.size() + j;
                out << std::setw(8) << std::setprecision(4) << std::fixed
                    << (k < vals.size() ? vals[k] : 0.0);
            }
            out << L"\r\n";
        }
        out << L"\r\n";
    };
    block(L"CD", table.cd);
    block(L"CL", table.cl);
    return out.str();
}

void show_aero_table_viewer(HWND parent, const std::string& path)
{
    if (path.empty()) {
        MessageBoxW(parent, L"This table has no file to view.", L"Aero table", MB_ICONINFORMATION);
        return;
    }
    post2::aero::AeroTable table;
    std::string err;
    if (!post2::aero::read_aero_table_csv(path, &table, &err)) {
        MessageBoxW(parent, widen(err).c_str(), L"Aero table", MB_ICONWARNING);
        return;
    }

    static bool registered = false;
    if (!registered) {
        WNDCLASSW wc = {};
        wc.lpfnWndProc = aero_table_viewer_proc;
        wc.hInstance = g_instance;
        wc.lpszClassName = L"Post2AeroTableViewerWindow";
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
        RegisterClassW(&wc);
        registered = true;
    }

    AeroViewerState state;
    RECT pr;
    GetWindowRect(parent, &pr);
    constexpr int w = 660;
    constexpr int h = 540;
    const int x = pr.left + ((pr.right - pr.left) - w) / 2;
    const int y = pr.top + ((pr.bottom - pr.top) - h) / 2;

    HWND viewer = CreateWindowExW(
        WS_EX_WINDOWEDGE,
        L"Post2AeroTableViewerWindow",
        L"Aero table",
        WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_POPUP,
        x, y, w, h, parent, nullptr, g_instance, &state);
    if (!viewer) {
        return;
    }

    state.edit = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"EDIT",
        format_aero_table_text(path, table).c_str(),
        WS_CHILD | WS_VISIBLE | WS_VSCROLL | WS_HSCROLL |
            ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL | ES_AUTOHSCROLL,
        0, 0, w, h, viewer, nullptr, g_instance, nullptr);
    // Fixed-width font so the grid columns line up.
    static HFONT mono = CreateFontW(-13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
        FIXED_PITCH | FF_MODERN, L"Consolas");
    SendMessageW(state.edit, WM_SETFONT, reinterpret_cast<WPARAM>(mono), TRUE);

    RECT rc;
    GetClientRect(viewer, &rc);
    MoveWindow(state.edit, 0, 0, rc.right, rc.bottom, TRUE);

    EnableWindow(parent, FALSE);
    ShowWindow(viewer, SW_SHOW);

    MSG msg;
    while (IsWindow(viewer) && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(viewer, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }
    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);
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
    constexpr int dialog_width = 760;
    constexpr int dialog_height = 540;
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
    g_case_path = path;
    g_case_initialized = true;
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
    run_simulation(hwnd);
}

void save_case_config(HWND hwnd)
{
    ensure_case_initialized();
    std::string path = g_case_path;
    if (g_case_path.empty()) {
        if (!choose_case_config_file(hwnd, true, &path)) {
            return;
        }
    }

    std::string error;
    if (!post2::core::save_case_config_file(path, g_case, &error)) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Case save failed", MB_ICONERROR);
        return;
    }

    g_case_path = path;
    MessageBoxW(hwnd, widen("Wrote " + path).c_str(), L"POST2 Lite", MB_ICONINFORMATION);
}

void save_case_config_as(HWND hwnd)
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

    g_case_path = path;
    MessageBoxW(hwnd, widen("Wrote " + path).c_str(), L"POST2 Lite", MB_ICONINFORMATION);
}

bool choose_ksp_vehicle_site_file(HWND parent, std::string* path)
{
    std::array<wchar_t, MAX_PATH> buffer = {};
    wcscpy_s(buffer.data(), buffer.size(), L"vehicle_launchsite.json");

    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = parent;
    ofn.lpstrFilter = L"KSP vehicle/site JSON (*.json)\0*.json\0All files (*.*)\0*.*\0";
    ofn.lpstrFile = buffer.data();
    ofn.nMaxFile = static_cast<DWORD>(buffer.size());
    ofn.lpstrDefExt = L"json";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | OFN_FILEMUSTEXIST;

    if (!GetOpenFileNameW(&ofn)) {
        return false;
    }

    *path = narrow(buffer.data());
    return true;
}

// Builds a semi-empirical CD/CL(Mach, alpha) table from the imported geometry,
// writes it next to the source JSON, and points the case aero/atmosphere models
// at the real lift/drag path. Geometry falls back to real Falcon-9 dimensions
// when fields are unset; users can edit the dimensions and re-import to refine.
void generate_aero_table_for_case(HWND hwnd,
                                  const post2::core::KspVehicleSiteImport& imported,
                                  const std::string& source_path)
{
    // Table CSVs are written next to the source JSON. Delegates to the shared
    // core routine so the GUI and the post2_regen_aero CLI produce identical output.
    const auto slash = source_path.find_last_of("/\\");
    const std::string dir =
        slash != std::string::npos ? source_path.substr(0, slash + 1) : std::string();
    std::string error;
    if (!post2::core::generate_case_aero_tables(&g_case, imported, dir, &error)) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Aero table generation failed", MB_ICONWARNING);
    }
}

void import_ksp_vehicle_site(HWND hwnd)
{
    ensure_case_initialized();
    std::string path;
    if (!choose_ksp_vehicle_site_file(hwnd, &path)) {
        return;
    }

    post2::core::KspVehicleSiteImport imported;
    std::string error;
    if (!post2::core::load_ksp_vehicle_site_import_file(
            path,
            g_case.vehicle.aero,
            &imported,
            &error)) {
        MessageBoxW(hwnd, widen(error).c_str(), L"KSP import failed", MB_ICONERROR);
        return;
    }

    post2::core::apply_ksp_vehicle_site_import(&g_case, imported);
    generate_aero_table_for_case(hwnd, imported, path);
    sync_legacy_from_case();
    refresh_phase_list();
    load_selected_phase_controls();
    run_simulation(hwnd);
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
    add_combo_item(combo, L"apoapsis_altitude_m");
    add_combo_item(combo, L"eccentricity");
    add_combo_item(combo, L"flight_path_angle_deg");
    add_combo_item(combo, L"downrange_m");
    add_combo_item(combo, L"latitude_deg");
    add_combo_item(combo, L"longitude_deg");
    add_combo_item(combo, L"payload_mass_kg");
    add_combo_item(combo, L"propellant_remaining_kg");
    add_combo_item(combo, L"max_q_pa");
    add_combo_item(combo, L"max_dynamic_pressure_pa");
    add_combo_item(combo, L"max_accel_mps2");
    add_combo_item(combo, L"min_altitude_m");
    add_combo_item(combo, L"min_throttle");
    add_combo_item(combo, L"max_throttle");
    if (!g_case_initialized) {
        return;
    }
    const char* phase_metrics[] = {
        "terminal_altitude_m",
        "terminal_speed_mps",
        "inclination_deg",
        "periapsis_altitude_m",
        "apoapsis_altitude_m",
        "eccentricity",
        "flight_path_angle_deg",
        "duration_s",
        "propellant_remaining_kg",
        "max_q_pa",
        "max_dynamic_pressure_pa",
        "max_accel_mps2",
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

bool is_stage_dry_mass_variable(const std::string& path)
{
    return path.find("vehicle.stages[") == 0 &&
        path.rfind(".dry_mass_kg") != std::string::npos;
}

std::string default_continuation_variable_path(
    const post2::core::OptimizationConfig& optimization)
{
    std::string first_enabled;
    std::string last_stage_dry_mass;
    for (const auto& variable : optimization.variables) {
        if (!variable.enabled) {
            continue;
        }
        if (first_enabled.empty()) {
            first_enabled = variable.path;
        }
        std::string lower = variable.path;
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        if (lower.find("payload") != std::string::npos) {
            return variable.path;
        }
        if (is_stage_dry_mass_variable(variable.path)) {
            last_stage_dry_mass = variable.path;
        }
    }
    return last_stage_dry_mass.empty() ? first_enabled : last_stage_dry_mass;
}

bool optimization_has_enabled_variable(
    const post2::core::OptimizationConfig& optimization,
    const std::string& path)
{
    return std::any_of(
        optimization.variables.begin(),
        optimization.variables.end(),
        [&](const post2::core::OptimizationVariableConfig& variable) {
            return variable.enabled && variable.path == path;
        });
}

void add_continuation_variable_items(
    HWND combo,
    const post2::core::OptimizationConfig& optimization)
{
    for (const auto& variable : optimization.variables) {
        if (variable.enabled) {
            const std::wstring wide = widen(variable.path);
            add_combo_item(combo, wide.c_str());
        }
    }
}

void sync_optimization_continuation_controls(OptimizationSettingsDialogState* state)
{
    if (!state || !state->continuation_strategy_combo) {
        return;
    }
    const std::string strategy = get_combo_text(state->continuation_strategy_combo);
    const bool variable_continuation =
        strategy == "continuation" || strategy == "continuation+multistart";
    const bool epsilon = strategy == "epsilon-constraint";
    const bool multistart = strategy == "continuation+multistart";
    const bool envelope = strategy == "envelope";
    EnableWindow(state->continuation_variable_combo, variable_continuation);
    EnableWindow(state->continuation_direction_combo, variable_continuation);
    EnableWindow(state->continuation_steps_edit, variable_continuation || epsilon);
    EnableWindow(state->continuation_starts_edit, variable_continuation && multistart);
    EnableWindow(state->envelope_samples_edit, envelope);
}

void refresh_optimization_target_list(OptimizationSettingsDialogState* state)
{
    SendMessageW(state->target_list, LB_RESETCONTENT, 0, 0);
    for (std::size_t i = 0; i < state->config.targets.size(); ++i) {
        const auto& target = state->config.targets[i];
        std::ostringstream label;
        label << i + 1 << ": " << target.metric << " ";
        if (target.mode == "equal" || target.mode == "equality") {
            label << "== " << target.value;
        } else if (target.mode == "range") {
            label << "[" << target.min_value << ", " << target.max_value << "]";
        } else if (target.mode == "upper" || target.mode == "upper_bound") {
            label << "<= " << target.max_value;
        } else if (target.mode == "lower" || target.mode == "lower_bound") {
            label << ">= " << target.min_value;
        } else {
            label << target.mode;
        }
        label << " | w " << target.weight;
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

post2::core::OptimizationTargetConfig default_optimization_target_config()
{
    return {"terminal_altitude_m", "equal", 200000.0, 0.0, 0.0, 1.0};
}

void sync_optimization_target_editor_mode_controls(OptimizationTargetEditorDialogState* state)
{
    if (!state) {
        return;
    }
    const std::string mode = get_combo_text(state->mode_combo);
    EnableWindow(state->value_edit, mode == "equal" || mode == "equality");
    EnableWindow(state->min_edit, mode == "range" || mode == "lower" || mode == "lower_bound");
    EnableWindow(state->max_edit, mode == "range" || mode == "upper" || mode == "upper_bound");
    EnableWindow(state->weight_edit, TRUE);
}

bool read_optional_double_field(
    HWND dialog,
    HWND edit,
    const wchar_t* label,
    const wchar_t* title,
    double* value)
{
    const std::wstring text = get_window_text(edit);
    if (text.empty()) {
        return true;
    }
    return read_double_field(dialog, edit, label, title, value);
}

void create_optimization_target_editor_controls(OptimizationTargetEditorDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 22, 70, L"Metric", font);
    state->metric_combo = create_combo(state->hwnd, kOptTargetMetricCombo, 104, 18, 290, font);
    add_metric_items(state->metric_combo);
    select_combo_text(state->metric_combo, state->target.metric);

    create_label(state->hwnd, 18, 62, 70, L"Mode", font);
    state->mode_combo = create_combo(state->hwnd, kOptTargetModeCombo, 104, 58, 150, font);
    add_combo_item(state->mode_combo, L"equal");
    add_combo_item(state->mode_combo, L"range");
    add_combo_item(state->mode_combo, L"upper");
    add_combo_item(state->mode_combo, L"lower");
    select_combo_text(state->mode_combo, state->target.mode);

    create_label(state->hwnd, 18, 104, 70, L"Value", font);
    state->value_edit = create_edit(
        state->hwnd, kOptTargetValueEdit, 104, 100, 130, format_double(state->target.value), font);

    create_label(state->hwnd, 18, 144, 70, L"Min", font);
    state->min_edit = create_edit(
        state->hwnd, kOptTargetMinEdit, 104, 140, 130, format_double(state->target.min_value), font);
    create_label(state->hwnd, 252, 144, 42, L"Max", font);
    state->max_edit = create_edit(
        state->hwnd, kOptTargetMaxEdit, 306, 140, 130, format_double(state->target.max_value), font);

    create_label(state->hwnd, 18, 184, 70, L"Weight", font);
    state->weight_edit = create_edit(
        state->hwnd, kOptTargetWeightEdit, 104, 180, 130, format_double(state->target.weight), font);

    HWND ok_button = create_button(state->hwnd, IDOK, 270, 230, 76, 28, L"OK", font);
    SendMessageW(ok_button, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 360, 230, 76, 28, L"Cancel", font);

    sync_optimization_target_editor_mode_controls(state);
}

bool accept_optimization_target_editor_dialog(OptimizationTargetEditorDialogState* state)
{
    auto target = state->target;
    target.metric = get_combo_text(state->metric_combo);
    target.mode = get_combo_text(state->mode_combo);
    if (!read_double_field(state->hwnd, state->weight_edit, L"Target weight", L"Target", &target.weight)) {
        return false;
    }
    if (target.mode == "equal" || target.mode == "equality") {
        if (!read_double_field(state->hwnd, state->value_edit, L"Target value", L"Target", &target.value) ||
            !read_optional_double_field(state->hwnd, state->min_edit, L"Target min", L"Target", &target.min_value) ||
            !read_optional_double_field(state->hwnd, state->max_edit, L"Target max", L"Target", &target.max_value)) {
            return false;
        }
    } else if (target.mode == "range") {
        if (!read_optional_double_field(state->hwnd, state->value_edit, L"Target value", L"Target", &target.value) ||
            !read_double_field(state->hwnd, state->min_edit, L"Target min", L"Target", &target.min_value) ||
            !read_double_field(state->hwnd, state->max_edit, L"Target max", L"Target", &target.max_value)) {
            return false;
        }
    } else if (target.mode == "upper" || target.mode == "upper_bound") {
        if (!read_optional_double_field(state->hwnd, state->value_edit, L"Target value", L"Target", &target.value) ||
            !read_optional_double_field(state->hwnd, state->min_edit, L"Target min", L"Target", &target.min_value) ||
            !read_double_field(state->hwnd, state->max_edit, L"Target max", L"Target", &target.max_value)) {
            return false;
        }
    } else if (target.mode == "lower" || target.mode == "lower_bound") {
        if (!read_optional_double_field(state->hwnd, state->value_edit, L"Target value", L"Target", &target.value) ||
            !read_double_field(state->hwnd, state->min_edit, L"Target min", L"Target", &target.min_value) ||
            !read_optional_double_field(state->hwnd, state->max_edit, L"Target max", L"Target", &target.max_value)) {
            return false;
        }
    } else {
        MessageBoxW(state->hwnd, L"Unsupported target mode.", L"Target", MB_ICONWARNING);
        SetFocus(state->mode_combo);
        return false;
    }
    if (target.mode == "range" && target.min_value > target.max_value) {
        MessageBoxW(state->hwnd, L"Target min cannot exceed max.", L"Target", MB_ICONWARNING);
        SetFocus(state->min_edit);
        return false;
    }

    state->target = std::move(target);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK optimization_target_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<OptimizationTargetEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<OptimizationTargetEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_optimization_target_editor_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_optimization_target_editor_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kOptTargetModeCombo:
            if (HIWORD(wparam) == CBN_SELCHANGE) {
                sync_optimization_target_editor_mode_controls(state);
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
    default:
        break;
    }

    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_optimization_target_editor_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = optimization_target_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2OptimizationTargetEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_optimization_target_editor_dialog(
    HWND parent,
    post2::core::OptimizationTargetConfig* target,
    const wchar_t* title)
{
    register_optimization_target_editor_class();

    OptimizationTargetEditorDialogState state;
    state.target = *target;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 480;
    constexpr int dialog_height = 320;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2OptimizationTargetEditorWindow",
        title,
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
        MessageBoxW(parent, L"Failed to create target editor.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.metric_combo);

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
        *target = std::move(state.target);
    }

    return state.accepted;
}

// ============================================================================
// Phase action editor (modal). Replaces the inline edit row that silently
// dropped time_s edits.
// ============================================================================

void populate_action_value_combo_for_hwnd(HWND combo, const std::string& type, bool value)
{
    SendMessageW(combo, CB_RESETCONTENT, 0, 0);
    if (type == "set_engine_enabled") {
        add_combo_item(combo, L"enabled");
        add_combo_item(combo, L"disabled");
    } else {
        add_combo_item(combo, L"active");
        add_combo_item(combo, L"inactive");
    }
    select_combo_text(combo, action_state_display_label(type, value));
}

void populate_action_stage_combo_for_hwnd(
    HWND combo, const post2::vehicle::VehicleConfig& vehicle, int selected_index)
{
    SendMessageW(combo, CB_RESETCONTENT, 0, 0);
    const auto stages = post2::vehicle::effective_stage_configs(vehicle);
    for (std::size_t i = 0; i < stages.size(); ++i) {
        const std::wstring label = widen(std::to_string(i + 1) + ": " + stages[i].name);
        SendMessageW(combo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
    }
    const int clamped = selected_index >= 0 && static_cast<std::size_t>(selected_index) < stages.size()
        ? selected_index : 0;
    SendMessageW(combo, CB_SETCURSEL, static_cast<WPARAM>(clamped), 0);
}

void sync_phase_action_editor_stage_enabled(PhaseActionEditorDialogState* state)
{
    if (!state) return;
    const std::string type = action_type_from_display_label(get_combo_text(state->type_combo));
    EnableWindow(state->stage_combo,
        (type == "set_stage_active" || type == "set_stage_attached") ? TRUE : FALSE);
}

void create_phase_action_editor_controls(PhaseActionEditorDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 22, 70, L"Time (s)", font);
    state->time_edit = create_edit(
        state->hwnd, kPhaseActionEditTime, 104, 18, 130, format_double(state->action.time_s), font);

    create_label(state->hwnd, 18, 62, 70, L"Action", font);
    state->type_combo = create_combo(state->hwnd, kPhaseActionEditType, 104, 58, 250, font);
    add_combo_item(state->type_combo, L"Engine enabled");
    add_combo_item(state->type_combo, L"Hold-down clamp active");
    add_combo_item(state->type_combo, L"Stage active");
    add_combo_item(state->type_combo, L"Stage attached");
    select_combo_text(state->type_combo, action_type_display_label(state->action.type));

    create_label(state->hwnd, 18, 102, 70, L"State", font);
    state->value_combo = create_combo(state->hwnd, kPhaseActionEditValue, 104, 98, 130, font);
    populate_action_value_combo_for_hwnd(state->value_combo, state->action.type, state->action.value);

    create_label(state->hwnd, 18, 142, 70, L"Stage", font);
    state->stage_combo = create_combo(state->hwnd, kPhaseActionEditStage, 104, 138, 250, font);
    if (state->vehicle) {
        populate_action_stage_combo_for_hwnd(state->stage_combo, *state->vehicle, state->action.stage_index);
    }
    sync_phase_action_editor_stage_enabled(state);

    HWND ok = create_button(state->hwnd, IDOK, 270, 200, 76, 28, L"OK", font);
    SendMessageW(ok, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 360, 200, 76, 28, L"Cancel", font);
}

bool accept_phase_action_editor_dialog(PhaseActionEditorDialogState* state)
{
    post2::core::PhaseAction action = state->action;
    if (!read_double_field(state->hwnd, state->time_edit, L"time_s", L"Phase action", &action.time_s)) {
        return false;
    }
    if (action.time_s < 0.0) {
        MessageBoxW(state->hwnd, L"time_s must be non-negative.", L"Phase action", MB_ICONWARNING);
        SetFocus(state->time_edit);
        return false;
    }
    action.type = action_type_from_display_label(get_combo_text(state->type_combo));
    action.value = action_state_from_display_label(action.type, get_combo_text(state->value_combo));
    action.stage_index = -1;
    action.stage_name.clear();
    if (action.type == "set_stage_active" || action.type == "set_stage_attached") {
        const LRESULT sel = SendMessageW(state->stage_combo, CB_GETCURSEL, 0, 0);
        if (sel == CB_ERR) {
            MessageBoxW(state->hwnd, L"Select a target stage.", L"Phase action", MB_ICONWARNING);
            SetFocus(state->stage_combo);
            return false;
        }
        action.stage_index = static_cast<int>(sel);
        if (state->vehicle) {
            const auto stages = post2::vehicle::effective_stage_configs(*state->vehicle);
            if (static_cast<std::size_t>(action.stage_index) < stages.size()) {
                action.stage_name = stages[static_cast<std::size_t>(action.stage_index)].name;
            }
        }
    }
    state->action = std::move(action);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK phase_action_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<PhaseActionEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<PhaseActionEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_phase_action_editor_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_phase_action_editor_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kPhaseActionEditType:
            if (HIWORD(wparam) == CBN_SELCHANGE) {
                const std::string type =
                    action_type_from_display_label(get_combo_text(state->type_combo));
                const bool current_value = action_state_from_display_label(
                    type, get_combo_text(state->value_combo));
                populate_action_value_combo_for_hwnd(state->value_combo, type, current_value);
                sync_phase_action_editor_stage_enabled(state);
                return 0;
            }
            break;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_phase_action_editor_class()
{
    static bool registered = false;
    if (registered) return;
    WNDCLASSW wc = {};
    wc.lpfnWndProc = phase_action_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2PhaseActionEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_phase_action_editor_dialog(
    HWND parent,
    post2::core::PhaseAction* action,
    const post2::vehicle::VehicleConfig& vehicle,
    const wchar_t* title)
{
    register_phase_action_editor_class();
    PhaseActionEditorDialogState state;
    state.action = *action;
    state.vehicle = &vehicle;
    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 480;
    constexpr int dialog_height = 280;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;
    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2PhaseActionEditorWindow",
        title,
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x, y, dialog_width, dialog_height,
        parent, nullptr, g_instance, &state);
    if (!dialog) {
        MessageBoxW(parent, L"Failed to create phase action editor.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }
    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.time_edit);
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
        *action = std::move(state.action);
    }
    return state.accepted;
}

// ============================================================================
// Trigger condition editor (modal). Shared by phase termination and event
// triggers.
// ============================================================================

std::string trigger_type_display_label(const std::string& type)
{
    if (type == "time") return "Time (s)";
    if (type == "altitude_m") return "Altitude (m, geodetic)";
    if (type == "velocity_mps") return "Velocity magnitude (m/s)";
    if (type == "total_mass_kg") return "Total vehicle mass (kg)";
    if (type == "propellant_mass_kg") return "Propellant mass (kg, active stage)";
    if (type == "thrust_fraction") return "Thrust established (fraction)";
    return type;
}

std::string trigger_type_from_display_label(const std::string& label)
{
    if (label.rfind("Time", 0) == 0) return "time";
    if (label.rfind("Altitude", 0) == 0) return "altitude_m";
    if (label.rfind("Velocity", 0) == 0) return "velocity_mps";
    if (label.rfind("Total vehicle mass", 0) == 0) return "total_mass_kg";
    if (label.rfind("Propellant mass", 0) == 0) return "propellant_mass_kg";
    if (label.rfind("Thrust established", 0) == 0) return "thrust_fraction";
    return label;
}

std::wstring trigger_summary(const post2::core::TriggerCondition& trigger)
{
    std::wstring summary = widen(trigger_type_display_label(trigger.type));
    summary += L" ";
    summary += widen(trigger.comparison);
    summary += L" ";
    summary += format_double(trigger.value);
    return summary;
}

void create_trigger_editor_controls(TriggerConditionEditorDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 22, 80, L"Quantity", font);
    state->type_combo = create_combo(state->hwnd, kTriggerEditType, 118, 18, 290, font);
    add_combo_item(state->type_combo, L"Time (s)");
    add_combo_item(state->type_combo, L"Altitude (m, geodetic)");
    add_combo_item(state->type_combo, L"Velocity magnitude (m/s)");
    add_combo_item(state->type_combo, L"Total vehicle mass (kg)");
    add_combo_item(state->type_combo, L"Propellant mass (kg, active stage)");
    add_combo_item(state->type_combo, L"Thrust established (fraction)");
    select_combo_text(state->type_combo, trigger_type_display_label(state->trigger.type));

    create_label(state->hwnd, 18, 62, 80, L"Comparison", font);
    state->comparison_combo = create_combo(state->hwnd, kTriggerEditComparison, 118, 58, 90, font);
    add_combo_item(state->comparison_combo, L">=");
    add_combo_item(state->comparison_combo, L"<=");
    select_combo_text(state->comparison_combo, state->trigger.comparison);

    create_label(state->hwnd, 18, 102, 80, L"Value", font);
    state->value_edit = create_edit(
        state->hwnd, kTriggerEditValue, 118, 98, 200, format_double(state->trigger.value), font);

    int dialog_h = 240;
    if (state->optimization && !state->variable_path.empty()) {
        // Look up an existing optimization variable for this path; if found,
        // pre-fill Opt / Min / Max from it.
        bool opt_enabled = false;
        double min_value = 0.0;
        double max_value = 0.0;
        if (const auto* variable = find_optimization_variable(*state->optimization, state->variable_path)) {
            opt_enabled = variable->enabled;
            min_value = variable->min_value;
            max_value = variable->max_value;
        }
        create_label(state->hwnd, 18, 142, 100, L"Optimize", font);
        state->opt_check = CreateWindowExW(
            0, L"BUTTON", L"Opt",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
            118, 140, 56, 24,
            state->hwnd, nullptr, g_instance, nullptr);
        set_child_font(state->opt_check, font);
        Button_SetCheck(state->opt_check, opt_enabled ? BST_CHECKED : BST_UNCHECKED);

        create_label(state->hwnd, 180, 142, 36, L"Min", font);
        state->min_edit = create_edit(
            state->hwnd, 0, 218, 138, 90,
            (opt_enabled || min_value != 0.0 || max_value != 0.0) ? format_double(min_value) : L"",
            font);
        create_label(state->hwnd, 314, 142, 36, L"Max", font);
        state->max_edit = create_edit(
            state->hwnd, 0, 352, 138, 90,
            (opt_enabled || min_value != 0.0 || max_value != 0.0) ? format_double(max_value) : L"",
            font);
        dialog_h = 280;
    }

    HWND ok = create_button(state->hwnd, IDOK, 250, dialog_h - 80, 76, 28, L"OK", font);
    SendMessageW(ok, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 340, dialog_h - 80, 76, 28, L"Cancel", font);
}

bool accept_trigger_editor_dialog(TriggerConditionEditorDialogState* state)
{
    post2::core::TriggerCondition trigger;
    trigger.type = trigger_type_from_display_label(get_combo_text(state->type_combo));
    trigger.comparison = get_combo_text(state->comparison_combo);
    if (trigger.comparison != ">=" && trigger.comparison != "<=") {
        MessageBoxW(state->hwnd, L"Comparison must be >= or <=.", L"Trigger", MB_ICONWARNING);
        return false;
    }
    if (!read_double_field(state->hwnd, state->value_edit, L"value", L"Trigger", &trigger.value)) {
        return false;
    }

    // Optimization binding. Same semantics as apply_numeric_variable_controls
    // but reading the controls embedded in this dialog.
    if (state->optimization && !state->variable_path.empty() &&
        window_is_live(state->opt_check)) {
        const bool opt_enabled = Button_GetCheck(state->opt_check) == BST_CHECKED;
        auto* existing = find_optimization_variable(state->optimization, state->variable_path);

        double min_value = 0.0;
        double max_value = 0.0;
        const std::wstring min_text = get_window_text(state->min_edit);
        const std::wstring max_text = get_window_text(state->max_edit);
        const bool has_minmax = !min_text.empty() || !max_text.empty();
        if (opt_enabled || has_minmax || existing) {
            if (!parse_double_text(min_text, &min_value)) {
                MessageBoxW(state->hwnd, L"Min must be a number.", L"Trigger", MB_ICONWARNING);
                SetFocus(state->min_edit);
                return false;
            }
            if (!parse_double_text(max_text, &max_value)) {
                MessageBoxW(state->hwnd, L"Max must be a number.", L"Trigger", MB_ICONWARNING);
                SetFocus(state->max_edit);
                return false;
            }
            if (min_value > max_value) {
                MessageBoxW(state->hwnd, L"Min cannot exceed Max.", L"Trigger", MB_ICONWARNING);
                SetFocus(state->min_edit);
                return false;
            }
            if (opt_enabled && (trigger.value < min_value || trigger.value > max_value)) {
                MessageBoxW(state->hwnd, L"Value must be inside Min/Max.", L"Trigger", MB_ICONWARNING);
                SetFocus(state->value_edit);
                return false;
            }
        }
        if (opt_enabled || existing) {
            if (!existing) {
                state->optimization->variables.push_back(
                    {state->variable_path, opt_enabled, min_value, max_value});
            } else {
                existing->enabled = opt_enabled;
                existing->min_value = min_value;
                existing->max_value = max_value;
            }
        }
    }

    state->trigger = std::move(trigger);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK trigger_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<TriggerConditionEditorDialogState*>(
        GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<TriggerConditionEditorDialogState*>(
            GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_trigger_editor_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_trigger_editor_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_trigger_editor_class()
{
    static bool registered = false;
    if (registered) return;
    WNDCLASSW wc = {};
    wc.lpfnWndProc = trigger_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2TriggerEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_trigger_condition_editor_dialog(
    HWND parent,
    post2::core::TriggerCondition* trigger,
    post2::core::OptimizationConfig* optimization,
    const std::string& variable_path,
    const wchar_t* title)
{
    register_trigger_editor_class();
    TriggerConditionEditorDialogState state;
    state.trigger = *trigger;
    state.optimization = optimization;
    state.variable_path = variable_path;
    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 460;
    const int dialog_height =
        (optimization && !variable_path.empty()) ? 280 : 240;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;
    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2TriggerEditorWindow",
        title,
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x, y, dialog_width, dialog_height,
        parent, nullptr, g_instance, &state);
    if (!dialog) return false;
    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.type_combo);
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
        *trigger = std::move(state.trigger);
    }
    return state.accepted;
}

// ============================================================================
// Event editor (modal). A name + enabled + trigger + inner action list.
// ============================================================================

void refresh_event_action_list(EventEditorDialogState* state)
{
    SendMessageW(state->action_list, LB_RESETCONTENT, 0, 0);
    for (const auto& action : state->event.actions) {
        SendMessageW(
            state->action_list, LB_ADDSTRING, 0,
            reinterpret_cast<LPARAM>(widen(action_label(action)).c_str()));
    }
    if (state->event.actions.empty()) {
        state->selected_action_index = -1;
    } else {
        if (state->selected_action_index < 0 ||
            static_cast<std::size_t>(state->selected_action_index) >= state->event.actions.size()) {
            state->selected_action_index = 0;
        }
        SendMessageW(
            state->action_list, LB_SETCURSEL,
            static_cast<WPARAM>(state->selected_action_index), 0);
    }
    EnableWindow(state->action_edit_button, state->selected_action_index >= 0);
    EnableWindow(state->action_delete_button, state->selected_action_index >= 0);
}

void refresh_event_trigger_summary(EventEditorDialogState* state)
{
    SetWindowTextW(state->trigger_summary_label, trigger_summary(state->event.trigger).c_str());
}

void create_event_editor_controls(EventEditorDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 22, 70, L"Name", font);
    state->name_edit = create_edit(
        state->hwnd, kEventEditName, 104, 18, 320, widen(state->event.name), font);

    state->enabled_check = create_checkbox(state->hwnd, kEventEditEnabled, 440, 20, 70, L"Enabled", font);
    Button_SetCheck(state->enabled_check, state->event.enabled ? BST_CHECKED : BST_UNCHECKED);

    create_label(state->hwnd, 18, 62, 70, L"Trigger", font);
    state->trigger_summary_label = CreateWindowExW(
        0, L"STATIC", L"", WS_CHILD | WS_VISIBLE,
        104, 64, 380, 20, state->hwnd, nullptr, g_instance, nullptr);
    set_child_font(state->trigger_summary_label, font);
    state->trigger_edit_button = create_button(
        state->hwnd, kEventEditTriggerButton, 488, 58, 76, 24, L"Edit...", font);
    refresh_event_trigger_summary(state);

    create_label(state->hwnd, 18, 102, 70, L"Actions", font);
    state->action_list = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"LISTBOX", L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18, 122, 546, 140,
        state->hwnd,
        reinterpret_cast<HMENU>(static_cast<LONG_PTR>(kEventEditActionList)),
        g_instance, nullptr);
    set_child_font(state->action_list, font);

    state->action_add_button = create_button(state->hwnd, kEventEditActionAdd, 18, 268, 76, 26, L"Add", font);
    state->action_edit_button = create_button(state->hwnd, kEventEditActionEdit, 102, 268, 76, 26, L"Edit", font);
    state->action_delete_button = create_button(state->hwnd, kEventEditActionDelete, 186, 268, 76, 26, L"Delete", font);

    HWND ok = create_button(state->hwnd, IDOK, 412, 320, 76, 28, L"OK", font);
    SendMessageW(ok, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 498, 320, 76, 28, L"Cancel", font);

    refresh_event_action_list(state);
}

bool accept_event_editor_dialog(EventEditorDialogState* state)
{
    state->event.name = narrow(get_window_text(state->name_edit));
    state->event.enabled = Button_GetCheck(state->enabled_check) == BST_CHECKED;
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK event_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<EventEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<EventEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_event_editor_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_event_editor_dialog(state);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kEventEditTriggerButton:
            if (show_trigger_condition_editor_dialog(
                    state->hwnd, &state->event.trigger, nullptr, std::string(), L"Edit Trigger")) {
                refresh_event_trigger_summary(state);
            }
            return 0;
        case kEventEditActionList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const LRESULT sel = SendMessageW(state->action_list, LB_GETCURSEL, 0, 0);
                if (sel != LB_ERR) {
                    state->selected_action_index = static_cast<int>(sel);
                    EnableWindow(state->action_edit_button, TRUE);
                    EnableWindow(state->action_delete_button, TRUE);
                }
                return 0;
            }
            if (HIWORD(wparam) == LBN_DBLCLK && state->selected_action_index >= 0 &&
                state->vehicle) {
                auto action = state->event.actions[static_cast<std::size_t>(state->selected_action_index)];
                if (show_phase_action_editor_dialog(state->hwnd, &action, *state->vehicle, L"Edit Action")) {
                    state->event.actions[static_cast<std::size_t>(state->selected_action_index)] = std::move(action);
                    refresh_event_action_list(state);
                }
                return 0;
            }
            break;
        case kEventEditActionAdd:
            if (state->vehicle) {
                post2::core::PhaseAction action;
                action.type = "set_stage_active";
                action.value = true;
                action.stage_index = 0;
                const auto stages = post2::vehicle::effective_stage_configs(*state->vehicle);
                if (!stages.empty()) action.stage_name = stages.front().name;
                if (show_phase_action_editor_dialog(state->hwnd, &action, *state->vehicle, L"Add Action")) {
                    state->event.actions.push_back(std::move(action));
                    state->selected_action_index = static_cast<int>(state->event.actions.size() - 1);
                    refresh_event_action_list(state);
                }
            }
            return 0;
        case kEventEditActionEdit:
            if (state->selected_action_index >= 0 &&
                static_cast<std::size_t>(state->selected_action_index) < state->event.actions.size() &&
                state->vehicle) {
                auto action = state->event.actions[static_cast<std::size_t>(state->selected_action_index)];
                if (show_phase_action_editor_dialog(state->hwnd, &action, *state->vehicle, L"Edit Action")) {
                    state->event.actions[static_cast<std::size_t>(state->selected_action_index)] = std::move(action);
                    refresh_event_action_list(state);
                }
            }
            return 0;
        case kEventEditActionDelete:
            if (state->selected_action_index >= 0 &&
                static_cast<std::size_t>(state->selected_action_index) < state->event.actions.size()) {
                state->event.actions.erase(
                    state->event.actions.begin() + state->selected_action_index);
                if (static_cast<std::size_t>(state->selected_action_index) >= state->event.actions.size()) {
                    state->selected_action_index = static_cast<int>(state->event.actions.size()) - 1;
                }
                refresh_event_action_list(state);
            }
            return 0;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_event_editor_class()
{
    static bool registered = false;
    if (registered) return;
    WNDCLASSW wc = {};
    wc.lpfnWndProc = event_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2EventEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_event_editor_dialog(
    HWND parent,
    post2::core::EventConfig* event,
    const post2::vehicle::VehicleConfig& vehicle,
    const wchar_t* title)
{
    register_event_editor_class();
    EventEditorDialogState state;
    state.event = *event;
    state.vehicle = &vehicle;
    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 620;
    constexpr int dialog_height = 400;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;
    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2EventEditorWindow",
        title,
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x, y, dialog_width, dialog_height,
        parent, nullptr, g_instance, &state);
    if (!dialog) return false;
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
        *event = std::move(state.event);
    }
    return state.accepted;
}

// ============================================================================
// Case events manager (modal). List + Add / Edit / Delete + OK / Cancel.
// ============================================================================

std::wstring event_summary_label(const post2::core::EventConfig& event)
{
    std::wstring text = event.enabled ? L"[on] " : L"[off] ";
    text += widen(event.name.empty() ? std::string("(unnamed)") : event.name);
    text += L"  |  ";
    text += trigger_summary(event.trigger);
    text += L"  |  ";
    text += widen(std::to_string(event.actions.size()) + " action(s)");
    return text;
}

void refresh_case_events_list(CaseEventsManagerDialogState* state)
{
    SendMessageW(state->event_list, LB_RESETCONTENT, 0, 0);
    for (const auto& event : state->events) {
        SendMessageW(state->event_list, LB_ADDSTRING, 0,
            reinterpret_cast<LPARAM>(event_summary_label(event).c_str()));
    }
    if (state->events.empty()) {
        state->selected_index = -1;
    } else {
        if (state->selected_index < 0 ||
            static_cast<std::size_t>(state->selected_index) >= state->events.size()) {
            state->selected_index = 0;
        }
        SendMessageW(state->event_list, LB_SETCURSEL,
            static_cast<WPARAM>(state->selected_index), 0);
    }
    EnableWindow(state->edit_button, state->selected_index >= 0);
    EnableWindow(state->delete_button, state->selected_index >= 0);
}

void create_case_events_manager_controls(CaseEventsManagerDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    create_label(state->hwnd, 18, 18, 200, L"Non-sequential events", font);
    state->event_list = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"LISTBOX", L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18, 44, 614, 240,
        state->hwnd,
        reinterpret_cast<HMENU>(static_cast<LONG_PTR>(kCaseEventsList)),
        g_instance, nullptr);
    set_child_font(state->event_list, font);

    state->add_button = create_button(state->hwnd, kCaseEventsAdd, 18, 296, 76, 26, L"Add", font);
    state->edit_button = create_button(state->hwnd, kCaseEventsEdit, 102, 296, 76, 26, L"Edit", font);
    state->delete_button = create_button(state->hwnd, kCaseEventsDelete, 186, 296, 76, 26, L"Delete", font);

    HWND ok = create_button(state->hwnd, IDOK, 478, 350, 76, 28, L"OK", font);
    SendMessageW(ok, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 564, 350, 76, 28, L"Cancel", font);

    refresh_case_events_list(state);
}

LRESULT CALLBACK case_events_manager_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<CaseEventsManagerDialogState*>(
        GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<CaseEventsManagerDialogState*>(
            GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_case_events_manager_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            state->accepted = true;
            DestroyWindow(hwnd);
            return 0;
        case IDCANCEL:
            DestroyWindow(hwnd);
            return 0;
        case kCaseEventsList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const LRESULT sel = SendMessageW(state->event_list, LB_GETCURSEL, 0, 0);
                if (sel != LB_ERR) {
                    state->selected_index = static_cast<int>(sel);
                    EnableWindow(state->edit_button, TRUE);
                    EnableWindow(state->delete_button, TRUE);
                }
                return 0;
            }
            if (HIWORD(wparam) == LBN_DBLCLK && state->selected_index >= 0 && state->vehicle) {
                auto event = state->events[static_cast<std::size_t>(state->selected_index)];
                if (show_event_editor_dialog(state->hwnd, &event, *state->vehicle, L"Edit Event")) {
                    state->events[static_cast<std::size_t>(state->selected_index)] = std::move(event);
                    refresh_case_events_list(state);
                }
                return 0;
            }
            break;
        case kCaseEventsAdd:
            if (state->vehicle) {
                post2::core::EventConfig event;
                event.name = "event";
                event.enabled = true;
                event.trigger = {"time", ">=", 0.0};
                if (show_event_editor_dialog(state->hwnd, &event, *state->vehicle, L"Add Event")) {
                    state->events.push_back(std::move(event));
                    state->selected_index = static_cast<int>(state->events.size() - 1);
                    refresh_case_events_list(state);
                }
            }
            return 0;
        case kCaseEventsEdit:
            if (state->selected_index >= 0 &&
                static_cast<std::size_t>(state->selected_index) < state->events.size() &&
                state->vehicle) {
                auto event = state->events[static_cast<std::size_t>(state->selected_index)];
                if (show_event_editor_dialog(state->hwnd, &event, *state->vehicle, L"Edit Event")) {
                    state->events[static_cast<std::size_t>(state->selected_index)] = std::move(event);
                    refresh_case_events_list(state);
                }
            }
            return 0;
        case kCaseEventsDelete:
            if (state->selected_index >= 0 &&
                static_cast<std::size_t>(state->selected_index) < state->events.size()) {
                state->events.erase(state->events.begin() + state->selected_index);
                if (static_cast<std::size_t>(state->selected_index) >= state->events.size()) {
                    state->selected_index = static_cast<int>(state->events.size()) - 1;
                }
                refresh_case_events_list(state);
            }
            return 0;
        }
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    }
    return DefWindowProcW(hwnd, message, wparam, lparam);
}

void register_case_events_manager_class()
{
    static bool registered = false;
    if (registered) return;
    WNDCLASSW wc = {};
    wc.lpfnWndProc = case_events_manager_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2CaseEventsManagerWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_case_events_manager_dialog(
    HWND parent,
    std::vector<post2::core::EventConfig>* events,
    const post2::vehicle::VehicleConfig& vehicle)
{
    register_case_events_manager_class();
    CaseEventsManagerDialogState state;
    state.events = *events;
    state.vehicle = &vehicle;
    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 680;
    constexpr int dialog_height = 430;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;
    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2CaseEventsManagerWindow",
        L"Case Events",
        WS_CAPTION | WS_SYSMENU | WS_POPUP,
        x, y, dialog_width, dialog_height,
        parent, nullptr, g_instance, &state);
    if (!dialog) return false;
    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.event_list);
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
        *events = std::move(state.events);
    }
    return state.accepted;
}

void normalize_optimization_objectives_for_ui(post2::core::OptimizationConfig* config)
{
    if (!config || !config->objectives.empty()) {
        return;
    }
    if (config->objective.enabled) {
        config->objectives.push_back(config->objective);
    }
}

post2::core::OptimizationObjectiveConfig default_optimization_objective_config()
{
    post2::core::OptimizationObjectiveConfig objective;
    objective.enabled = true;
    objective.metric = "payload_mass_kg";
    objective.direction = "maximize";
    objective.weight = 1.0;
    return objective;
}

void refresh_optimization_objective_list(OptimizationSettingsDialogState* state)
{
    SendMessageW(state->objective_list, LB_RESETCONTENT, 0, 0);
    for (std::size_t i = 0; i < state->config.objectives.size(); ++i) {
        const auto& objective = state->config.objectives[i];
        std::ostringstream label;
        label << i + 1 << ": " << (objective.enabled ? "on " : "off ")
              << objective.direction << ' ' << objective.metric
              << " | w " << objective.weight;
        SendMessageW(state->objective_list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(widen(label.str()).c_str()));
    }
    if (state->config.objectives.empty()) {
        state->selected_objective_index = -1;
    } else {
        if (state->selected_objective_index < 0 ||
            static_cast<std::size_t>(state->selected_objective_index) >= state->config.objectives.size()) {
            state->selected_objective_index = 0;
        }
        SendMessageW(state->objective_list, LB_SETCURSEL, static_cast<WPARAM>(state->selected_objective_index), 0);
    }
}

void create_optimization_objective_editor_controls(OptimizationObjectiveEditorDialogState* state)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    state->enabled_check = create_checkbox(state->hwnd, kOptObjectiveEnabled, 18, 20, 92, L"Enabled", font);
    Button_SetCheck(state->enabled_check, state->objective.enabled ? BST_CHECKED : BST_UNCHECKED);

    create_label(state->hwnd, 18, 62, 70, L"Metric", font);
    state->metric_combo = create_combo(state->hwnd, kOptObjectiveMetricCombo, 104, 58, 290, font);
    add_metric_items(state->metric_combo);
    select_combo_text(state->metric_combo, state->objective.metric);

    create_label(state->hwnd, 18, 102, 70, L"Direction", font);
    state->direction_combo = create_combo(state->hwnd, kOptObjectiveDirectionCombo, 104, 98, 150, font);
    add_combo_item(state->direction_combo, L"minimize");
    add_combo_item(state->direction_combo, L"maximize");
    select_combo_text(state->direction_combo, state->objective.direction);

    create_label(state->hwnd, 18, 142, 70, L"Weight", font);
    state->weight_edit = create_edit(
        state->hwnd, kOptObjectiveWeightEdit, 104, 138, 130, format_double(state->objective.weight), font);

    HWND ok_button = create_button(state->hwnd, IDOK, 270, 180, 76, 28, L"OK", font);
    SendMessageW(ok_button, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 360, 180, 76, 28, L"Cancel", font);
}

bool accept_optimization_objective_editor_dialog(OptimizationObjectiveEditorDialogState* state)
{
    auto objective = state->objective;
    objective.enabled = Button_GetCheck(state->enabled_check) == BST_CHECKED;
    objective.metric = get_combo_text(state->metric_combo);
    objective.direction = get_combo_text(state->direction_combo);
    if (!read_double_field(state->hwnd, state->weight_edit, L"Objective weight", L"Objective", &objective.weight)) {
        return false;
    }
    if (objective.direction != "minimize" && objective.direction != "maximize") {
        MessageBoxW(state->hwnd, L"Unsupported objective direction.", L"Objective", MB_ICONWARNING);
        SetFocus(state->direction_combo);
        return false;
    }
    state->objective = std::move(objective);
    state->accepted = true;
    DestroyWindow(state->hwnd);
    return true;
}

LRESULT CALLBACK optimization_objective_editor_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    auto* state = reinterpret_cast<OptimizationObjectiveEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    switch (message) {
    case WM_NCCREATE: {
        auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(create->lpCreateParams));
        return TRUE;
    }
    case WM_CREATE:
        state = reinterpret_cast<OptimizationObjectiveEditorDialogState*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        state->hwnd = hwnd;
        create_optimization_objective_editor_controls(state);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wparam)) {
        case IDOK:
            accept_optimization_objective_editor_dialog(state);
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

void register_optimization_objective_editor_class()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    WNDCLASSW wc = {};
    wc.lpfnWndProc = optimization_objective_editor_proc;
    wc.hInstance = g_instance;
    wc.lpszClassName = L"Post2OptimizationObjectiveEditorWindow";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&wc);
    registered = true;
}

bool show_optimization_objective_editor_dialog(
    HWND parent,
    post2::core::OptimizationObjectiveConfig* objective,
    const wchar_t* title)
{
    register_optimization_objective_editor_class();

    OptimizationObjectiveEditorDialogState state;
    state.objective = *objective;

    RECT parent_rect;
    GetWindowRect(parent, &parent_rect);
    constexpr int dialog_width = 480;
    constexpr int dialog_height = 260;
    const int x = parent_rect.left + ((parent_rect.right - parent_rect.left) - dialog_width) / 2;
    const int y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - dialog_height) / 2;

    HWND dialog = CreateWindowExW(
        WS_EX_DLGMODALFRAME | WS_EX_WINDOWEDGE,
        L"Post2OptimizationObjectiveEditorWindow",
        title,
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
        MessageBoxW(parent, L"Failed to create objective editor.", L"POST2 Lite", MB_ICONERROR);
        return false;
    }

    EnableWindow(parent, FALSE);
    ShowWindow(dialog, SW_SHOW);
    SetFocus(state.metric_combo);

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
        *objective = std::move(state.objective);
    }

    return state.accepted;
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

    normalize_optimization_objectives_for_ui(&state->config);

    create_label(state->hwnd, 18, 102, 90, L"Targets", font);
    state->target_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        18,
        132,
        324,
        196,
        state->hwnd,
        control_id(kOptTargetList),
        g_instance,
        nullptr);
    set_child_font(state->target_list, font);
    state->target_add_button = create_button(state->hwnd, kOptTargetAdd, 18, 340, 70, 26, L"Add", font);
    state->target_edit_button = create_button(state->hwnd, kOptTargetEdit, 102, 340, 70, 26, L"Edit", font);
    state->target_delete_button = create_button(state->hwnd, kOptTargetDelete, 186, 340, 70, 26, L"Delete", font);

    create_label(state->hwnd, 370, 102, 110, L"Objectives", font);
    state->objective_list = CreateWindowExW(
        WS_EX_CLIENTEDGE,
        L"LISTBOX",
        L"",
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | LBS_NOTIFY | WS_VSCROLL,
        370,
        132,
        324,
        196,
        state->hwnd,
        control_id(kOptObjectiveList),
        g_instance,
        nullptr);
    set_child_font(state->objective_list, font);
    state->objective_add_button = create_button(state->hwnd, kOptObjectiveAdd, 370, 340, 70, 26, L"Add", font);
    state->objective_edit_button = create_button(state->hwnd, kOptObjectiveEdit, 454, 340, 70, 26, L"Edit", font);
    state->objective_delete_button = create_button(state->hwnd, kOptObjectiveDelete, 538, 340, 70, 26, L"Delete", font);

    create_label(state->hwnd, 18, 482, 60, L"Strategy", font);
    state->continuation_strategy_combo =
        create_combo(state->hwnd, kOptContinuationStrategyCombo, 82, 478, 166, font);
    add_combo_item(state->continuation_strategy_combo, L"single");
    add_combo_item(state->continuation_strategy_combo, L"continuation");
    add_combo_item(state->continuation_strategy_combo, L"continuation+multistart");
    add_combo_item(state->continuation_strategy_combo, L"epsilon-constraint");
    add_combo_item(state->continuation_strategy_combo, L"envelope");
    select_combo_text(
        state->continuation_strategy_combo,
        state->config.envelope_search.enabled
            ? "envelope"
            : (state->config.continuation.enabled
            ? (state->config.continuation.mode == "objective"
                ? "epsilon-constraint"
                : (state->config.continuation.multistart_enabled
                    ? "continuation+multistart"
                    : "continuation"))
            : "single"));

    create_label(state->hwnd, 270, 482, 58, L"Cont var", font);
    state->continuation_variable_combo =
        create_combo(state->hwnd, kOptContinuationVariableCombo, 334, 478, 360, font);
    add_continuation_variable_items(state->continuation_variable_combo, state->config);
    const std::string continuation_variable =
        state->config.continuation.variable_path.empty()
            ? default_continuation_variable_path(state->config)
            : state->config.continuation.variable_path;
    select_combo_text(state->continuation_variable_combo, continuation_variable);

    create_label(state->hwnd, 18, 522, 60, L"Direction", font);
    state->continuation_direction_combo =
        create_combo(state->hwnd, kOptContinuationDirectionCombo, 82, 518, 120, font);
    add_combo_item(state->continuation_direction_combo, L"increase");
    add_combo_item(state->continuation_direction_combo, L"decrease");
    select_combo_text(state->continuation_direction_combo, state->config.continuation.direction);

    create_label(state->hwnd, 230, 522, 44, L"Steps", font);
    state->continuation_steps_edit = create_edit(
        state->hwnd,
        kOptContinuationStepsEdit,
        284,
        518,
        70,
        widen(std::to_string(state->config.continuation.steps)),
        font);
    create_label(state->hwnd, 384, 522, 44, L"Starts", font);
    state->continuation_starts_edit = create_edit(
        state->hwnd,
        kOptContinuationStartsEdit,
        442,
        518,
        70,
        widen(std::to_string(state->config.continuation.multistart_count)),
        font);
    create_label(state->hwnd, 540, 522, 58, L"Samples", font);
    state->envelope_samples_edit = create_edit(
        state->hwnd,
        kOptEnvelopeSamplesEdit,
        604,
        518,
        70,
        widen(std::to_string(state->config.envelope_search.sample_count)),
        font);

    HWND ok_button = create_button(state->hwnd, IDOK, 542, 560, 76, 28, L"OK", font);
    SendMessageW(ok_button, BM_SETSTYLE, BS_DEFPUSHBUTTON, TRUE);
    create_button(state->hwnd, IDCANCEL, 632, 560, 76, 28, L"Cancel", font);

    refresh_optimization_target_list(state);
    refresh_optimization_objective_list(state);
    sync_optimization_continuation_controls(state);
}

bool accept_optimization_settings_dialog(OptimizationSettingsDialogState* state)
{
    state->config.mode = get_combo_text(state->mode_combo);
    state->config.optimizer = get_combo_text(state->optimizer_combo);
    const std::string continuation_strategy =
        get_combo_text(state->continuation_strategy_combo);
    if (!read_int_field(state->hwnd, state->max_iterations_edit, L"Max iterations", L"Optimize", &state->config.max_iterations) ||
        !read_int_field(
            state->hwnd,
            state->continuation_steps_edit,
            L"Continuation steps",
            L"Optimize",
            &state->config.continuation.steps) ||
        !read_int_field(
            state->hwnd,
            state->continuation_starts_edit,
            L"Continuation starts",
            L"Optimize",
            &state->config.continuation.multistart_count) ||
        !read_int_field(
            state->hwnd,
            state->envelope_samples_edit,
            L"Envelope samples",
            L"Optimize",
            &state->config.envelope_search.sample_count) ||
        !read_double_field(state->hwnd, state->tolerance_edit, L"Tolerance", L"Optimize", &state->config.tolerance) ||
        !read_double_field(
            state->hwnd,
            state->step_fraction_edit,
            L"Initial step fraction",
            L"Optimize",
            &state->config.initial_step_fraction)) {
        return false;
    }
    state->config.objective = state->config.objectives.empty()
        ? post2::core::OptimizationObjectiveConfig{}
        : state->config.objectives.front();
    state->config.continuation.enabled =
        continuation_strategy == "continuation" ||
        continuation_strategy == "continuation+multistart" ||
        continuation_strategy == "epsilon-constraint";
    state->config.continuation.mode =
        continuation_strategy == "epsilon-constraint" ? "objective" : "variable";
    state->config.continuation.multistart_enabled =
        continuation_strategy == "continuation+multistart";
    state->config.envelope_search.enabled = continuation_strategy == "envelope";
    state->config.continuation.variable_path =
        get_combo_text(state->continuation_variable_combo);
    state->config.continuation.direction =
        get_combo_text(state->continuation_direction_combo);
    if (state->config.max_iterations <= 0) {
        MessageBoxW(state->hwnd, L"Max iterations must be positive.", L"Optimize", MB_ICONWARNING);
        return false;
    }
    if (state->config.continuation.steps <= 0 ||
        state->config.continuation.multistart_count <= 0) {
        MessageBoxW(state->hwnd, L"Continuation steps and starts must be positive.", L"Optimize", MB_ICONWARNING);
        return false;
    }
    if (state->config.envelope_search.sample_count <= 0) {
        MessageBoxW(state->hwnd, L"Envelope samples must be positive.", L"Optimize", MB_ICONWARNING);
        return false;
    }
    if (state->config.tolerance <= 0.0 || state->config.initial_step_fraction <= 0.0) {
        MessageBoxW(state->hwnd, L"Tolerance and step fraction must be positive.", L"Optimize", MB_ICONWARNING);
        return false;
    }
    if (state->config.continuation.enabled &&
        state->config.continuation.mode != "objective" &&
        !optimization_has_enabled_variable(
            state->config,
            state->config.continuation.variable_path)) {
        MessageBoxW(state->hwnd, L"Continuation variable must be an enabled optimization variable.", L"Optimize", MB_ICONWARNING);
        SetFocus(state->continuation_variable_combo);
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
                const LRESULT selected = SendMessageW(state->target_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    state->selected_target_index = static_cast<int>(selected);
                }
                return 0;
            }
            if (HIWORD(wparam) == LBN_DBLCLK) {
                const LRESULT selected = SendMessageW(state->target_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    state->selected_target_index = static_cast<int>(selected);
                    auto target = state->config.targets[static_cast<std::size_t>(state->selected_target_index)];
                    if (show_optimization_target_editor_dialog(state->hwnd, &target, L"Edit Target")) {
                        state->config.targets[static_cast<std::size_t>(state->selected_target_index)] = std::move(target);
                        refresh_optimization_target_list(state);
                    }
                }
                return 0;
            }
            break;
        case kOptTargetAdd:
        {
            auto target = default_optimization_target_config();
            if (show_optimization_target_editor_dialog(state->hwnd, &target, L"Add Target")) {
                state->config.targets.push_back(std::move(target));
                state->selected_target_index = static_cast<int>(state->config.targets.size() - 1);
                refresh_optimization_target_list(state);
            }
            return 0;
        }
        case kOptTargetEdit:
            if (state->selected_target_index >= 0 &&
                static_cast<std::size_t>(state->selected_target_index) < state->config.targets.size()) {
                auto target = state->config.targets[static_cast<std::size_t>(state->selected_target_index)];
                if (show_optimization_target_editor_dialog(state->hwnd, &target, L"Edit Target")) {
                    state->config.targets[static_cast<std::size_t>(state->selected_target_index)] = std::move(target);
                    refresh_optimization_target_list(state);
                }
            }
            return 0;
        case kOptTargetDelete:
            if (state->selected_target_index >= 0 &&
                static_cast<std::size_t>(state->selected_target_index) < state->config.targets.size()) {
                state->config.targets.erase(state->config.targets.begin() + state->selected_target_index);
                if (static_cast<std::size_t>(state->selected_target_index) >= state->config.targets.size()) {
                    state->selected_target_index = static_cast<int>(state->config.targets.size()) - 1;
                }
                refresh_optimization_target_list(state);
            }
            return 0;
        case kOptObjectiveList:
            if (HIWORD(wparam) == LBN_SELCHANGE) {
                const LRESULT selected = SendMessageW(state->objective_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    state->selected_objective_index = static_cast<int>(selected);
                }
                return 0;
            }
            if (HIWORD(wparam) == LBN_DBLCLK) {
                const LRESULT selected = SendMessageW(state->objective_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    state->selected_objective_index = static_cast<int>(selected);
                    auto objective = state->config.objectives[static_cast<std::size_t>(state->selected_objective_index)];
                    if (show_optimization_objective_editor_dialog(state->hwnd, &objective, L"Edit Objective")) {
                        state->config.objectives[static_cast<std::size_t>(state->selected_objective_index)] = std::move(objective);
                        refresh_optimization_objective_list(state);
                    }
                }
                return 0;
            }
            break;
        case kOptObjectiveAdd:
        {
            auto objective = default_optimization_objective_config();
            if (show_optimization_objective_editor_dialog(state->hwnd, &objective, L"Add Objective")) {
                state->config.objectives.push_back(std::move(objective));
                state->selected_objective_index = static_cast<int>(state->config.objectives.size() - 1);
                refresh_optimization_objective_list(state);
            }
            return 0;
        }
        case kOptObjectiveEdit:
            if (state->selected_objective_index >= 0 &&
                static_cast<std::size_t>(state->selected_objective_index) < state->config.objectives.size()) {
                auto objective = state->config.objectives[static_cast<std::size_t>(state->selected_objective_index)];
                if (show_optimization_objective_editor_dialog(state->hwnd, &objective, L"Edit Objective")) {
                    state->config.objectives[static_cast<std::size_t>(state->selected_objective_index)] = std::move(objective);
                    refresh_optimization_objective_list(state);
                }
            }
            return 0;
        case kOptObjectiveDelete:
            if (state->selected_objective_index >= 0 &&
                static_cast<std::size_t>(state->selected_objective_index) < state->config.objectives.size()) {
                state->config.objectives.erase(state->config.objectives.begin() + state->selected_objective_index);
                if (static_cast<std::size_t>(state->selected_objective_index) >= state->config.objectives.size()) {
                    state->selected_objective_index = static_cast<int>(state->config.objectives.size()) - 1;
                }
                refresh_optimization_objective_list(state);
            }
            return 0;
        case kOptContinuationStrategyCombo:
            if (HIWORD(wparam) == CBN_SELCHANGE) {
                sync_optimization_continuation_controls(state);
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
    constexpr int dialog_height = 630;
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
        WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_VSCROLL | CBS_DROPDOWNLIST,
        x,
        y,
        width,
        260,
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

    post2::core::PhaseAction action;
    action.time_s = 0.0;
    action.type = "set_stage_active";
    action.value = true;
    action.stage_index = 0;
    const auto stages = post2::vehicle::effective_stage_configs(g_case.vehicle);
    if (!stages.empty()) {
        action.stage_name = stages.front().name;
    }
    if (!show_phase_action_editor_dialog(hwnd, &action, g_case.vehicle, L"Add Action")) {
        return true;
    }
    phase->actions.push_back(std::move(action));
    g_selected_action_index = static_cast<int>(phase->actions.size() - 1);
    refresh_action_list();
    load_selected_action_controls();
    return true;
}

bool edit_action_from_sidebar(HWND hwnd)
{
    auto* phase = selected_phase();
    if (g_selected_action_index < 0 ||
        static_cast<std::size_t>(g_selected_action_index) >= phase->actions.size()) {
        return true;
    }
    auto action = phase->actions[static_cast<std::size_t>(g_selected_action_index)];
    if (!show_phase_action_editor_dialog(hwnd, &action, g_case.vehicle, L"Edit Action")) {
        return true;
    }
    phase->actions[static_cast<std::size_t>(g_selected_action_index)] = std::move(action);
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

// A continuity-held coefficient is determined at runtime from the previous
// phase, so optimizing it is a no-op. When continuity is enabled we therefore
// grey out (and clear) the Opt/Min/Max controls of the governed c0 row.
void update_opt_state_for_continuity_row(const ContinuityBindingRow& crow)
{
    static const std::string suffix = ".continuity";
    if (!crow.check ||
        crow.path.size() <= suffix.size() ||
        crow.path.compare(crow.path.size() - suffix.size(), suffix.size(), suffix) != 0) {
        return;
    }
    const std::string c0_path = crow.path.substr(0, crow.path.size() - suffix.size()) + ".c0";
    const bool continuity_on = Button_GetCheck(crow.check) == BST_CHECKED;
    for (const auto& row : g_phase_numeric_rows) {
        if (row.path != c0_path) {
            continue;
        }
        EnableWindow(row.opt_check, continuity_on ? FALSE : TRUE);
        EnableWindow(row.min_edit, continuity_on ? FALSE : TRUE);
        EnableWindow(row.max_edit, continuity_on ? FALSE : TRUE);
        if (continuity_on) {
            Button_SetCheck(row.opt_check, BST_UNCHECKED);
        }
        break;
    }
}

// Returns true (and re-applies the opt greying) if `control` is a continuity
// checkbox, so the scroll-pane proc can treat the click as fully handled.
bool handle_continuity_opt_toggle(HWND control)
{
    for (const auto& crow : g_phase_continuity_rows) {
        if (crow.check == control) {
            update_opt_state_for_continuity_row(crow);
            return true;
        }
    }
    return false;
}

bool phase_editor_steering_is_upfg()
{
    return lowercase(get_combo_text(g_phase_steering_type)) == "upfg";
}

bool is_phase_throttle_path(const std::string& path)
{
    return path.find(".throttle_model.") != std::string::npos ||
        path.find(".throttle.") != std::string::npos;
}

void remove_phase_throttle_optimization_variables(
    post2::core::OptimizationConfig* optimization,
    const std::string& phase_prefix)
{
    if (!optimization) {
        return;
    }
    optimization->variables.erase(
        std::remove_if(
            optimization->variables.begin(),
            optimization->variables.end(),
            [&](const post2::core::OptimizationVariableConfig& variable) {
                return variable.path.rfind(phase_prefix + ".throttle_model.", 0) == 0 ||
                    variable.path.rfind(phase_prefix + ".throttle.", 0) == 0;
            }),
        optimization->variables.end());
}

void update_phase_upfg_throttle_state()
{
    const bool upfg = phase_editor_steering_is_upfg();
    if (upfg && window_is_live(g_phase_throttle_type)) {
        select_combo_text(g_phase_throttle_type, "poly");
    }
    if (upfg) {
        if (window_is_live(g_phase_throttle_c0)) { SetWindowTextW(g_phase_throttle_c0, L"1"); }
        if (window_is_live(g_phase_throttle_c1)) { SetWindowTextW(g_phase_throttle_c1, L"0"); }
        if (window_is_live(g_phase_throttle_c2)) { SetWindowTextW(g_phase_throttle_c2, L"0"); }
        if (window_is_live(g_phase_throttle_t2w)) { SetWindowTextW(g_phase_throttle_t2w, L"1"); }
    }

    if (window_is_live(g_phase_throttle_type)) {
        EnableWindow(g_phase_throttle_type, upfg ? FALSE : TRUE);
    }
    for (const auto& row : g_phase_numeric_rows) {
        if (!is_phase_throttle_path(row.path)) {
            continue;
        }
        EnableWindow(row.value_edit, upfg ? FALSE : TRUE);
        EnableWindow(row.opt_check, upfg ? FALSE : TRUE);
        EnableWindow(row.min_edit, upfg ? FALSE : TRUE);
        EnableWindow(row.max_edit, upfg ? FALSE : TRUE);
        if (upfg) {
            Button_SetCheck(row.opt_check, BST_UNCHECKED);
        }
    }
    for (const auto& row : g_phase_continuity_rows) {
        if (!is_phase_throttle_path(row.path)) {
            continue;
        }
        EnableWindow(row.check, upfg ? FALSE : TRUE);
        if (upfg) {
            Button_SetCheck(row.check, BST_UNCHECKED);
        } else {
            update_opt_state_for_continuity_row(row);
        }
    }
}

LRESULT CALLBACK phase_scroll_pane_proc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)
{
    // Forward control notifications to the dialog window so the editor's
    // WM_COMMAND switch (Add/Edit/Delete action buttons, Edit termination,
    // etc.) sees them. lparam != 0 distinguishes control notifications from
    // menu/accelerator commands; we only forward the former.
    if (message == WM_COMMAND && lparam != 0) {
        if (HIWORD(wparam) == BN_CLICKED &&
            handle_continuity_opt_toggle(reinterpret_cast<HWND>(lparam))) {
            return 0;
        }
        HWND parent = GetParent(hwnd);
        if (parent) {
            return SendMessageW(parent, message, wparam, lparam);
        }
    }
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

// Places a phase-boundary continuity checkbox to the right of a coefficient row
// (at vertical position row_y, the y of the relevant c0 row) and records its
// bool path so load/apply can round-trip it.
void add_phase_continuity_check(HWND parent, int row_y, const std::string& path, HFONT font)
{
    ContinuityBindingRow row;
    row.path = path;
    row.check = create_dynamic_checkbox(parent, 662, row_y + 1, 44, L"Cont", font);
    g_phase_continuity_rows.push_back(row);
}

void add_phase_poly_rows(
    HWND parent,
    int* y,
    const std::string& label,
    const std::string& path,
    const post2::core::Poly2Config& poly,
    HFONT font,
    bool with_continuity = false)
{
    const int c0_y = *y;
    add_phase_numeric_row(parent, y, label + " c0", path + ".c0", poly.c0, font);
    add_phase_numeric_row(parent, y, label + " c1", path + ".c1", poly.c1, font);
    add_phase_numeric_row(parent, y, label + " c2", path + ".c2", poly.c2, font);
    if (with_continuity) {
        add_phase_continuity_check(parent, c0_y, path + ".continuity", font);
    }
}

double coefficient_or_zero(const std::vector<double>& coefficients, std::size_t index)
{
    return index < coefficients.size() ? coefficients[index] : 0.0;
}

void add_phase_coefficient_rows(
    HWND parent,
    int* y,
    const std::string& label,
    const std::string& path,
    const std::vector<double>& coefficients,
    int order,
    HFONT font)
{
    const int count = std::max(0, std::min(order, 8)) + 1;
    for (int i = 0; i < count; ++i) {
        add_phase_numeric_row(
            parent,
            y,
            label + " c" + std::to_string(i),
            path + ".c" + std::to_string(i),
            coefficient_or_zero(coefficients, static_cast<std::size_t>(i)),
            font);
    }
}

void add_phase_segmented_throttle_rows(
    HWND parent,
    int* y,
    const post2::core::SegmentedPolyConfig& segmented,
    const std::string& path,
    HFONT font)
{
    if (segmented.segments.empty()) {
        return;
    }
    create_phase_section(parent, y, L"Throttle segmented poly", font);
    add_phase_continuity_check(parent, *y, path + ".continuity", font);
    for (std::size_t i = 0; i < segmented.segments.size(); ++i) {
        const auto& segment = segmented.segments[i];
        const std::string item = "Throttle seg " + std::to_string(i + 1);
        const std::string segment_path = path + ".segments[" + std::to_string(i) + "]";
        add_phase_numeric_row(parent, y, item + " start", segment_path + ".start_time_s", segment.start_time_s, font);
        add_phase_coefficient_rows(parent, y, item, segment_path, segment.coefficients, segmented.order, font);
    }
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
    add_phase_continuity_check(parent, *y - 34, path + ".continuity", font);
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

    add_phase_segmented_throttle_rows(
        parent,
        y,
        throttle.segmented_poly,
        path + ".segmented_poly",
        font);
}

void add_phase_upfg_rows(
    HWND parent,
    int* y,
    const post2::core::UpfgConfig& upfg,
    const std::string& path,
    const std::string& label_prefix,
    HFONT font)
{
    add_phase_numeric_row(parent, y, label_prefix + "UPFG periapsis km", path + ".periapsis_km", upfg.periapsis_km, font);
    add_phase_numeric_row(parent, y, label_prefix + "UPFG apoapsis km", path + ".apoapsis_km", upfg.apoapsis_km, font);
    add_phase_numeric_row(parent, y, label_prefix + "UPFG inclination", path + ".inclination_deg", upfg.inclination_deg, font);
}

void add_phase_segmented_steering_rows(
    HWND parent,
    int* y,
    const post2::core::SegmentedSteeringPolyConfig& segmented,
    const std::string& path,
    const std::string& label_prefix,
    HFONT font)
{
    if (segmented.segments.empty()) {
        return;
    }
    create_phase_section(parent, y, L"Steering segmented poly", font);
    add_phase_continuity_check(parent, *y, path + ".continuity", font);
    for (std::size_t i = 0; i < segmented.segments.size(); ++i) {
        const auto& segment = segmented.segments[i];
        const std::string item = label_prefix + "Steer seg " + std::to_string(i + 1);
        const std::string segment_path = path + ".segments[" + std::to_string(i) + "]";
        add_phase_numeric_row(parent, y, item + " start", segment_path + ".start_time_s", segment.start_time_s, font);
        add_phase_coefficient_rows(
            parent,
            y,
            item + " az",
            segment_path + ".azimuth",
            segment.azimuth_coefficients,
            segmented.order,
            font);
        add_phase_coefficient_rows(
            parent,
            y,
            item + " el",
            segment_path + ".elevation",
            segment.elevation_coefficients,
            segmented.order,
            font);
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
    add_phase_poly_rows(parent, y, label_prefix + "Pitch", path + ".pitch", steering.pitch_deg, font, true);
    add_phase_poly_rows(parent, y, label_prefix + "Yaw", path + ".yaw", steering.yaw_deg, font, true);
    const int azimuth_c0_y = *y;
    NumericBindingRow row =
        add_phase_numeric_row(parent, y, label_prefix + "Azimuth c0", path + ".azimuth.c0", steering.azimuth_deg.c0, font);
    if (label_prefix.empty()) {
        g_phase_steering_azimuth = row.value_edit;
    }
    add_phase_continuity_check(parent, azimuth_c0_y, path + ".azimuth.continuity", font);
    add_phase_numeric_row(parent, y, label_prefix + "Azimuth c1", path + ".azimuth.c1", steering.azimuth_deg.c1, font);
    add_phase_numeric_row(parent, y, label_prefix + "Azimuth c2", path + ".azimuth.c2", steering.azimuth_deg.c2, font);
    const int elevation_c0_y = *y;
    row = add_phase_numeric_row(parent, y, label_prefix + "Elevation c0", path + ".elevation.c0", steering.elevation_deg.c0, font);
    if (label_prefix.empty()) {
        g_phase_steering_elevation = row.value_edit;
    }
    add_phase_continuity_check(parent, elevation_c0_y, path + ".elevation.continuity", font);
    add_phase_numeric_row(parent, y, label_prefix + "Elevation c1", path + ".elevation.c1", steering.elevation_deg.c1, font);
    add_phase_numeric_row(parent, y, label_prefix + "Elevation c2", path + ".elevation.c2", steering.elevation_deg.c2, font);

    // Linear/bilinear tangent steering coefficients (tan(elevation) = a*dt + b,
    // with a_dot/b_dot adding the bilinear time-variation). Continuity anchors b.
    add_phase_numeric_row(parent, y, label_prefix + "Tangent a", path + ".tangent.a", steering.tangent.a, font);
    add_phase_numeric_row(parent, y, label_prefix + "Tangent a_dot", path + ".tangent.a_dot", steering.tangent.a_dot, font);
    const int tangent_b_y = *y;
    add_phase_numeric_row(parent, y, label_prefix + "Tangent b", path + ".tangent.b", steering.tangent.b, font);
    add_phase_continuity_check(parent, tangent_b_y, path + ".tangent.continuity", font);
    add_phase_numeric_row(parent, y, label_prefix + "Tangent b_dot", path + ".tangent.b_dot", steering.tangent.b_dot, font);
    add_phase_numeric_row(parent, y, label_prefix + "Tangent t_offset", path + ".tangent.t_offset_s", steering.tangent.t_offset_s, font);

    add_phase_numeric_row(parent, y, label_prefix + "Fixed ECI x", path + ".fixed_direction_eci.x", steering.fixed_direction_eci.x, font);
    add_phase_numeric_row(parent, y, label_prefix + "Fixed ECI y", path + ".fixed_direction_eci.y", steering.fixed_direction_eci.y, font);
    add_phase_numeric_row(parent, y, label_prefix + "Fixed ECI z", path + ".fixed_direction_eci.z", steering.fixed_direction_eci.z, font);

    add_phase_upfg_rows(parent, y, steering.upfg, path + ".upfg", label_prefix, font);
    add_phase_segmented_steering_rows(
        parent,
        y,
        steering.segmented_poly,
        path + ".segmented_poly",
        label_prefix,
        font);

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

void load_phase_continuity_rows_from_case()
{
    for (const auto& row : g_phase_continuity_rows) {
        bool value = false;
        std::string error;
        if (post2::core::read_optimization_flag(g_case, row.path, &value, &error)) {
            Button_SetCheck(row.check, value ? BST_CHECKED : BST_UNCHECKED);
        }
        // Reflect the loaded continuity state in the governed c0 row's Opt/Min/
        // Max enablement (must run after load_phase_numeric_rows_from_case).
        update_opt_state_for_continuity_row(row);
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
    Button_SetCheck(g_phase_aero, phase->force_models.aerodynamic ? BST_CHECKED : BST_UNCHECKED);

    select_combo_text(g_phase_atmosphere_type, phase->force_models.atmosphere_model.type);
    select_combo_text(g_phase_throttle_type, phase->throttle_model.type);
    select_combo_text(g_phase_steering_type, phase->steering_model.type);
    load_phase_numeric_rows_from_case();
    load_phase_continuity_rows_from_case();
    update_phase_upfg_throttle_state();
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
    edited.force_models.aerodynamic = Button_GetCheck(g_phase_aero) == BST_CHECKED;
    edited.force_models.atmosphere_model.type = get_combo_text(g_phase_atmosphere_type);

    edited.throttle_model.type = get_combo_text(g_phase_throttle_type);
    edited.steering_model.type = get_combo_text(g_phase_steering_type);
    const bool steering_upfg = lowercase(edited.steering_model.type) == "upfg";

    if (!update_selected_action_from_controls(hwnd, &edited)) {
        return false;
    }

    candidate.phases[static_cast<std::size_t>(g_selected_phase_index)] = edited;

    for (const auto& row : g_phase_numeric_rows) {
        if (steering_upfg && is_phase_throttle_path(row.path)) {
            continue;
        }
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

    for (const auto& row : g_phase_continuity_rows) {
        if (steering_upfg && is_phase_throttle_path(row.path)) {
            continue;
        }
        const bool flag_value = Button_GetCheck(row.check) == BST_CHECKED;
        std::string flag_error;
        post2::core::write_optimization_flag(&candidate, row.path, flag_value, &flag_error);
    }

    if (steering_upfg) {
        auto& throttle = candidate.phases[static_cast<std::size_t>(g_selected_phase_index)].throttle_model;
        throttle = post2::core::ThrottleModelConfig{};
        throttle.type = "poly";
        throttle.c0 = 1.0;
        throttle.c1 = 0.0;
        throttle.c2 = 0.0;
        throttle.target_t2w = 1.0;
        throttle.continuity = false;
        remove_phase_throttle_optimization_variables(&candidate.optimization, phase_path_prefix());
    }

    const auto& final_phase = candidate.phases[static_cast<std::size_t>(g_selected_phase_index)];
    if (final_phase.termination.type == "time" && final_phase.termination.value <= 0.0) {
        MessageBoxW(hwnd, L"Termination value must be positive for time-based termination.",
            L"Phase", MB_ICONWARNING);
        return false;
    }
    if (final_phase.termination.type == "time") {
        for (const auto& action : final_phase.actions) {
            if (action.time_s < 0.0 || action.time_s > final_phase.termination.value) {
                MessageBoxW(hwnd, L"Action time must be inside the phase duration.", L"Phase", MB_ICONWARNING);
                return false;
            }
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
    const double default_duration_s = g_case.phases.empty() ? g_config.duration_s : 60.0;
    phase.termination = {"time", ">=", default_duration_s};
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
    g_phase_aero = create_checkbox(g_phase_scroll_pane, kPhaseAero, 440, y, 86, L"Aero", font);
    y += 40;

    create_label(g_phase_scroll_pane, 18, y + 4, 86, L"Atmosphere", font);
    g_phase_atmosphere_type = create_combo(g_phase_scroll_pane, kPhaseAtmosphereType, 118, y, 154, font);
    add_combo_item(g_phase_atmosphere_type, L"exponential");
    add_combo_item(g_phase_atmosphere_type, L"us_standard_1976");
    add_combo_item(g_phase_atmosphere_type, L"table");
    add_combo_item(g_phase_atmosphere_type, L"none");
    y += 40;

    // Phase termination — summary + Edit... opens TriggerCondition editor.
    // The editor handles the optimisation Opt/Min/Max bounds inline.
    create_label(g_phase_scroll_pane, 18, y + 4, 86, L"Termination", font);
    HWND termination_label = CreateWindowExW(
        0, L"STATIC", trigger_summary(phase->termination).c_str(),
        WS_CHILD | WS_VISIBLE,
        118, y + 4, 380, 20,
        g_phase_scroll_pane, nullptr, g_instance, nullptr);
    set_child_font(termination_label, font);
    g_phase_termination_label = termination_label;
    create_button(g_phase_scroll_pane, kPhaseTerminationEditButton, 504, y, 76, 26, L"Edit...", font);
    y += 38;

    create_phase_section(g_phase_scroll_pane, &y, L"Throttle", font);
    create_label(g_phase_scroll_pane, 18, y + 4, 86, L"Type", font);
    g_phase_throttle_type = create_combo(g_phase_scroll_pane, kPhaseThrottleType, 118, y, 154, font);
    add_combo_item(g_phase_throttle_type, L"poly");
    add_combo_item(g_phase_throttle_type, L"segmented_poly");
    add_combo_item(g_phase_throttle_type, L"t2w");
    add_combo_item(g_phase_throttle_type, L"interpolated");
    y += 38;
    add_phase_throttle_rows(g_phase_scroll_pane, &y, phase->throttle_model, prefix + ".throttle_model", font);

    create_phase_section(g_phase_scroll_pane, &y, L"Steering", font);
    create_label(g_phase_scroll_pane, 18, y + 4, 86, L"Type", font);
    g_phase_steering_type = create_combo(g_phase_scroll_pane, kPhaseSteeringType, 118, y, 210, font);
    add_combo_item(g_phase_steering_type, L"generic_poly");
    add_combo_item(g_phase_steering_type, L"segmented_poly");
    add_combo_item(g_phase_steering_type, L"rpy_poly");
    add_combo_item(g_phase_steering_type, L"fixed_eci");
    add_combo_item(g_phase_steering_type, L"generic_quat_interp");
    add_combo_item(g_phase_steering_type, L"generic_selectable");
    add_combo_item(g_phase_steering_type, L"linear_tangent");
    add_combo_item(g_phase_steering_type, L"bilinear_tangent");
    add_combo_item(g_phase_steering_type, L"upfg");
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
    create_button(g_phase_scroll_pane, kPhaseActionEdit, 88, y, 62, 26, L"Edit", font);
    g_phase_action_delete_button = create_button(g_phase_scroll_pane, kPhaseActionDelete, 158, y, 62, 26, L"Delete", font);
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
    create_button(hwnd, kCaseEventsList, 18, 440, 290, 26, L"Events...", font);

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
                const LRESULT selected = SendMessageW(g_phase_action_list, LB_GETCURSEL, 0, 0);
                if (selected != LB_ERR) {
                    g_selected_action_index = static_cast<int>(selected);
                    load_selected_action_controls();
                }
                return 0;
            }
            if (HIWORD(wparam) == LBN_DBLCLK) {
                edit_action_from_sidebar(hwnd);
                return 0;
            }
            break;
        case kPhaseActionAdd:
            add_action_from_sidebar(hwnd);
            return 0;
        case kPhaseActionEdit:
            edit_action_from_sidebar(hwnd);
            return 0;
        case kPhaseActionDelete:
            delete_action_from_sidebar(hwnd);
            return 0;
        case kPhaseSteeringType:
            if (HIWORD(wparam) == CBN_SELCHANGE) {
                update_phase_upfg_throttle_state();
                return 0;
            }
            break;
        case kPhaseTerminationEditButton: {
            auto* phase = selected_phase();
            const std::string variable_path =
                "phases[" + std::to_string(g_selected_phase_index) + "].termination.value";
            if (show_trigger_condition_editor_dialog(
                    hwnd, &phase->termination, &g_case.optimization,
                    variable_path, L"Edit Termination")) {
                if (window_is_live(g_phase_termination_label)) {
                    SetWindowTextW(g_phase_termination_label,
                        trigger_summary(phase->termination).c_str());
                }
            }
            return 0;
        }
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

std::string format_double_short(double value, int precision = 1)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
    return buffer;
}

double sum_vehicle_dry_mass_kg(const post2::vehicle::VehicleConfig& vehicle)
{
    if (vehicle.stages.empty()) {
        return vehicle.dry_mass_kg;
    }
    double total = 0.0;
    for (const auto& stage : vehicle.stages) {
        if (stage.attached) {
            total += std::max(0.0, stage.dry_mass_kg);
        }
    }
    return total;
}

double sum_vehicle_propellant_kg(const post2::vehicle::VehicleConfig& vehicle)
{
    if (vehicle.stages.empty()) {
        double total = 0.0;
        for (const auto& tank : vehicle.tanks) {
            total += std::max(0.0, tank.initial_kg);
        }
        return total;
    }
    double total = 0.0;
    for (const auto& stage : vehicle.stages) {
        if (!stage.attached) continue;
        for (const auto& tank : stage.tanks) {
            total += std::max(0.0, tank.initial_kg);
        }
    }
    return total;
}

double sum_vehicle_thrust_vac_n(const post2::vehicle::VehicleConfig& vehicle)
{
    if (vehicle.stages.empty()) {
        return vehicle.engine.thrust_vac_n * std::max(1, vehicle.engine.engine_count);
    }
    double total = 0.0;
    for (const auto& stage : vehicle.stages) {
        if (!stage.attached || !stage.active || !stage.engine.enabled) continue;
        total += stage.engine.thrust_vac_n * std::max(1, stage.engine.engine_count);
    }
    return total;
}

std::vector<std::string> format_pre_takeoff_lines()
{
    std::vector<std::string> lines;
    const auto& v = g_case.vehicle;
    const double dry = sum_vehicle_dry_mass_kg(v);
    const double prop = sum_vehicle_propellant_kg(v);
    const double thrust = sum_vehicle_thrust_vac_n(v);
    lines.push_back(std::string("Name: ") + v.name);
    lines.push_back("Stages: " + std::to_string(v.stages.size()));
    lines.push_back("Dry mass: " + format_double_short(dry, 1) + " kg");
    lines.push_back("Propellant: " + format_double_short(prop, 1) + " kg");
    lines.push_back("Total mass: " + format_double_short(dry + prop, 1) + " kg");
    if (thrust > 0.0) {
        lines.push_back("Vac thrust: " + format_double_short(thrust / 1000.0, 1) + " kN");
        const double t_w = thrust / std::max(1.0, (dry + prop) * 9.80665);
        lines.push_back("T/W (vac): " + format_double_short(t_w, 2));
    } else {
        lines.push_back("Vac thrust: -");
    }
    lines.push_back("Launch site:");
    lines.push_back("  lat " + format_double_short(g_case.launch_site.latitude_deg, 4) + " deg");
    lines.push_back("  lon " + format_double_short(g_case.launch_site.longitude_deg, 4) + " deg");
    lines.push_back("  alt " + format_double_short(g_case.launch_site.altitude_m, 1) + " m");
    return lines;
}

std::vector<std::string> format_final_lines()
{
    std::vector<std::string> lines;
    if (!g_result.ok || g_result.state_log.empty()) {
        lines.push_back("(no result yet)");
        return lines;
    }
    const auto& tail = g_result.state_log.back();
    lines.push_back("Time: " + format_double_short(tail.time_s, 1) + " s");
    lines.push_back("Altitude: " + format_double_short(tail.altitude_m / 1000.0, 2) + " km");
    lines.push_back("Speed: " + format_double_short(tail.speed_mps, 1) + " m/s");
    lines.push_back("Mass: " + format_double_short(tail.total_mass_kg, 1) + " kg");
    lines.push_back("Propellant: " + format_double_short(tail.propellant_mass_kg, 1) + " kg");
    lines.push_back("Engine thrust: " + format_double_short(tail.engine_thrust_n / 1000.0, 1) + " kN");
    lines.push_back(std::string("Hold-down: ") + (tail.hold_down_clamp_active ? "yes" : "no"));
    // Predicted orbit apoapsis (A) / periapsis (P): the max / min geodetic
    // altitude of the integrated orbit drawn in the 3D scene.
    if (!g_predicted_orbit.empty()) {
        double apoapsis_m = g_predicted_orbit.front().altitude_m;
        double periapsis_m = apoapsis_m;
        for (const auto& entry : g_predicted_orbit.entries()) {
            apoapsis_m = std::max(apoapsis_m, entry.altitude_m);
            periapsis_m = std::min(periapsis_m, entry.altitude_m);
        }
        lines.push_back("Apoapsis (A): " + format_double_short(apoapsis_m / 1000.0, 1) + " km");
        lines.push_back("Periapsis (P): " + format_double_short(periapsis_m / 1000.0, 1) + " km");
    } else {
        lines.push_back("Predicted orbit: sub-orbital");
    }
    return lines;
}

void paint_scene(HWND hwnd, HDC hdc)
{
    RECT client;
    GetClientRect(hwnd, &client);

    HBRUSH background = CreateSolidBrush(RGB(248, 250, 252));
    FillRect(hdc, &client, background);
    DeleteObject(background);

    // Left sidebar background + separator.
    RECT sidebar = client;
    sidebar.right = kSidebarWidth;
    HBRUSH sidebar_background = CreateSolidBrush(RGB(241, 245, 249));
    FillRect(hdc, &sidebar, sidebar_background);
    DeleteObject(sidebar_background);

    // Middle "vehicle data" column background + separators on both sides.
    RECT column = client;
    column.left = kVehicleColumnX;
    column.right = kVehicleColumnRight;
    HBRUSH column_background = CreateSolidBrush(RGB(248, 250, 254));
    FillRect(hdc, &column, column_background);
    DeleteObject(column_background);

    HPEN separator = CreatePen(PS_SOLID, 1, RGB(203, 213, 225));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, separator));
    MoveToEx(hdc, kSidebarWidth, 0, nullptr);
    LineTo(hdc, kSidebarWidth, client.bottom);
    MoveToEx(hdc, kVehicleColumnX, 0, nullptr);
    LineTo(hdc, kVehicleColumnX, client.bottom);
    MoveToEx(hdc, kVehicleColumnRight, 0, nullptr);
    LineTo(hdc, kVehicleColumnRight, client.bottom);
    SelectObject(hdc, old_pen);
    DeleteObject(separator);

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(15, 23, 42));

    HFONT title_font = CreateFontW(22, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    HFONT body_font = CreateFontW(14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    HFONT section_font = CreateFontW(14, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");

    // Title and status above the chart row, in the content column.
    HFONT old_font = static_cast<HFONT>(SelectObject(hdc, title_font));
    draw_text_line(hdc, kContentColumnX, 16, "POST2 Lite trajectory");
    SelectObject(hdc, body_font);
    draw_text_line(hdc, kContentColumnX, 56, g_status);
    SelectObject(hdc, old_font);

    // Middle column contents: section headers + line lists.
    const int col_x = kVehicleColumnX + 16;
    int y = 16;
    SelectObject(hdc, section_font);
    SetTextColor(hdc, RGB(30, 41, 59));
    draw_text_line(hdc, col_x, y, "Pre-takeoff vehicle");
    y += 22;
    SelectObject(hdc, body_font);
    SetTextColor(hdc, RGB(51, 65, 85));
    for (const auto& line : format_pre_takeoff_lines()) {
        draw_text_line(hdc, col_x, y, line);
        y += 18;
    }

    y += 12;
    SelectObject(hdc, section_font);
    SetTextColor(hdc, RGB(30, 41, 59));
    draw_text_line(hdc, col_x, y, "Final state");
    y += 22;
    SelectObject(hdc, body_font);
    SetTextColor(hdc, RGB(51, 65, 85));
    for (const auto& line : format_final_lines()) {
        draw_text_line(hdc, col_x, y, line);
        y += 18;
    }

    DeleteObject(title_font);
    DeleteObject(body_font);
    DeleteObject(section_font);
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
    HMENU launch_menu = CreatePopupMenu();
    HMENU vehicle_menu = CreatePopupMenu();

    AppendMenuW(file_menu, MF_STRING, kMenuRefresh, L"Refresh");
    AppendMenuW(file_menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(file_menu, MF_STRING, kMenuCaseLoad, L"Load case...");
    AppendMenuW(file_menu, MF_STRING, kMenuCaseSave, L"Save case...");
    AppendMenuW(file_menu, MF_STRING, kMenuCaseSaveAs, L"Save case as...");
    AppendMenuW(file_menu, MF_SEPARATOR, 0, nullptr);
    AppendMenuW(file_menu, MF_STRING, kMenuExportCsv, L"Export CSV");
    AppendMenuW(file_menu, MF_STRING, kMenuExportKosCsv, L"Export guidance script (post2_player)");
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

    AppendMenuW(launch_menu, MF_STRING, kMenuLaunchSettings, L"Launch site...");

    AppendMenuW(vehicle_menu, MF_STRING, kMenuVehicleEdit, L"Edit vehicle...");
    AppendMenuW(vehicle_menu, MF_STRING, kMenuVehicleImportKsp, L"Import KSP vehicle/site JSON...");

    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(file_menu), L"File");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(mode_menu), L"Mode");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(run_menu), L"Run");
    AppendMenuW(menu, MF_POPUP, reinterpret_cast<UINT_PTR>(optimize_menu), L"Optimize");
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
        ? post2::core::write_svg_file(path, g_result.state_log, &g_predicted_orbit, &error)
        : post2::core::write_csv_file(path, g_result.state_log, &error);

    if (!ok) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Export failed", MB_ICONERROR);
        return;
    }

    MessageBoxW(hwnd, widen("Wrote " + path).c_str(), L"POST2 Lite", MB_ICONINFORMATION);
}

std::filesystem::path default_guidance_script_path()
{
    // The guidance script is fed to the standalone post2_player (C++/kRPC), so it
    // belongs with the project scripts, not the KSP archive. Allow an override.
    const char* env_dir = std::getenv("POST2_GUIDANCE_DIR");
    const std::filesystem::path dir =
        (env_dir && env_dir[0] != '\0') ? std::filesystem::path(env_dir)
                                        : std::filesystem::path("scripts");
    return dir / "guidance_script.csv";
}

void export_guidance_script(HWND hwnd)
{
    if (g_case.phases.empty()) {
        MessageBoxW(hwnd, L"No case is loaded to export.", L"POST2 Lite", MB_ICONWARNING);
        return;
    }

    const std::filesystem::path path = default_guidance_script_path();
    std::error_code fs_error;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), fs_error);
        if (fs_error) {
            MessageBoxW(
                hwnd,
                widen("Failed to create scripts directory: " + fs_error.message()).c_str(),
                L"Export failed",
                MB_ICONERROR);
            return;
        }
    }

    std::string error;
    const std::string path_text = path.string();
    if (!post2::core::write_guidance_script_file(path_text, g_case, &error)) {
        MessageBoxW(hwnd, widen(error).c_str(), L"Export failed", MB_ICONERROR);
        return;
    }

    MessageBoxW(
        hwnd,
        widen(
            "Wrote guidance script CSV:\n" + path_text +
            "\n\nFly it with the standalone player:\n  post2_player \"" + path_text +
            "\"\n(add --dry-run to preview offline).").c_str(),
        L"POST2 Lite",
        MB_ICONINFORMATION);
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
        g_scene_renderer.render(
            g_camera,
            g_result.state_log,
            g_predicted_orbit,
            g_case.earth_rotation_at_epoch_rad,
            g_case.earth_rotation_rad_per_s,
            false);
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

void create_view_buttons(HWND parent)
{
    struct ButtonSpec {
        int id;
        const wchar_t* label;
        DWORD extra_style;
    };
    const ButtonSpec specs[kViewButtonCount] = {
        {kViewButton3D,       L"3D",        WS_GROUP},
        {kViewButtonProfile,  L"2D Profile", 0},
        {kViewButtonQ,        L"Q",         0},
        {kViewButtonThrottle, L"Throttle",  0},
        {kViewButtonSpeed,    L"Speed",     0},
        {kViewButtonMass,     L"Mass",      0},
    };
    constexpr int kButtonWidth = 92;
    int x = kContentColumnX;
    const int y = kViewButtonTop;
    HFONT font = CreateFontW(13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    for (int i = 0; i < kViewButtonCount; ++i) {
        HWND btn = CreateWindowExW(
            0,
            L"BUTTON",
            specs[i].label,
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTORADIOBUTTON | BS_PUSHLIKE | specs[i].extra_style,
            x,
            y,
            kButtonWidth,
            kViewButtonHeight,
            parent,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(specs[i].id)),
            g_instance,
            nullptr);
        g_view_buttons[i] = btn;
        SendMessageW(btn, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        x += kButtonWidth + kViewButtonGap;
    }
    Button_SetCheck(g_view_buttons[0], BST_CHECKED);
}

void create_chart_panels(HWND parent)
{
    const RECT rect = scene_rect(parent);
    g_chart_profile.initialize(parent, rect);
    g_chart_q.initialize(parent, rect);
    g_chart_throttle.initialize(parent, rect);
    g_chart_speed.initialize(parent, rect);
    g_chart_mass.initialize(parent, rect);
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
        create_view_buttons(hwnd);
        create_chart_panels(hwnd);
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

    case kOptimizationFinishedMessage:
        finish_optimization(hwnd);
        return 0;

    case WM_COMMAND:
        if (optimization_is_running()) {
            switch (LOWORD(wparam)) {
            case kViewButton3D:        switch_view(hwnd, ViewKind::Scene3D); return 0;
            case kViewButtonProfile:   switch_view(hwnd, ViewKind::Profile2D); return 0;
            case kViewButtonQ:         switch_view(hwnd, ViewKind::DynamicPressure); return 0;
            case kViewButtonThrottle:  switch_view(hwnd, ViewKind::Throttle); return 0;
            case kViewButtonSpeed:     switch_view(hwnd, ViewKind::Speed); return 0;
            case kViewButtonMass:      switch_view(hwnd, ViewKind::Mass); return 0;
            case kMenuOptimizationExecute:
                execute_optimization(hwnd);
                return 0;
            case kMenuExportCsv:
                export_current(hwnd, false);
                return 0;
            case kMenuExportKosCsv:
                export_guidance_script(hwnd);
                return 0;
            case kMenuExportSvg:
                export_current(hwnd, true);
                return 0;
            case kMenuExit:
                DestroyWindow(hwnd);
                return 0;
            default:
                MessageBoxW(
                    hwnd,
                    L"Optimization is running. Wait for it to finish before editing the case.",
                    L"POST2 Lite",
                    MB_ICONINFORMATION);
                return 0;
            }
        }
        switch (LOWORD(wparam)) {
        case kViewButton3D:        switch_view(hwnd, ViewKind::Scene3D); return 0;
        case kViewButtonProfile:   switch_view(hwnd, ViewKind::Profile2D); return 0;
        case kViewButtonQ:         switch_view(hwnd, ViewKind::DynamicPressure); return 0;
        case kViewButtonThrottle:  switch_view(hwnd, ViewKind::Throttle); return 0;
        case kViewButtonSpeed:     switch_view(hwnd, ViewKind::Speed); return 0;
        case kViewButtonMass:      switch_view(hwnd, ViewKind::Mass); return 0;
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
        case kMenuExportKosCsv:
            export_guidance_script(hwnd);
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
        case kMenuCaseSaveAs:
            save_case_config_as(hwnd);
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
        case kCaseEventsList:
            ensure_case_initialized();
            if (show_case_events_manager_dialog(hwnd, &g_case.events, g_case.vehicle)) {
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
        case kMenuVehicleImportKsp:
            import_ksp_vehicle_site(hwnd);
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
        1400,
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
