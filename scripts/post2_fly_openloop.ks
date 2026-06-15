// POST2-Lite open-loop kOS player.
//
// Modeled on KSPTOT's exec_lvd_control.ks (Launch Vehicle Designer control
// script by Arrowstar): read a precomputed attitude/throttle table, linearly
// interpolate it at the current time, and LOCK STEERING / LOCK THROTTLE to the
// result. The navball yaw/pitch/roll are computed host-side by POST2-Lite (see
// kos_trajectory_to_csv) and written straight into the CSV, so this script does
// NO ECI->navball reconstruction and needs none of the near-vertical / roll
// anti-jump band-aids the old player carried.
//
// POST2-Lite specifics layered on top of the KSPTOT core: auto-launch ignition
// with an engine-startup lead, launch-clamp (hold-down) release, and staging
// driven by the CSV's separation cues.
//
// In POST2 Lite use File > Export kOS trajectory CSV, then in kOS run:
//   runpath("archive:/post2-lite/post2_fly_openloop.ks").

@LAZYGLOBAL OFF.

PARAMETER TRAJ_PATH IS "archive:/post2-lite/gui_trajectory.csv".
PARAMETER AUTO_LAUNCH IS TRUE.
PARAMETER AUTO_RELEASE_LAUNCH_SUPPORT IS TRUE.
PARAMETER ENGINE_STARTUP_LEAD_S IS 3.0.
PARAMETER LAUNCH_SUPPORT_RELEASE_DELAY IS 0.25.
PARAMETER KOS_TARGET_IPU IS 2000.
PARAMETER STAGE_COMMAND_SETTLE_S IS 0.5.
PARAMETER STAGE_SHUTDOWN_SETTLE_S IS 0.15.
PARAMETER STAGE_MIN_PULSES IS 1.
PARAMETER MIN_COMMAND_THROTTLE IS 0.001.
// Stage as soon as the active engines flame out / run dry: available thrust
// collapses to FLAMEOUT_THRUST_FRACTION of the per-stage peak while throttle is
// still commanded. Robust booster separation independent of the CSV cue timing.
PARAMETER AUTO_STAGE_ON_FLAMEOUT IS TRUE.
PARAMETER FLAMEOUT_THRUST_FRACTION IS 0.02.
PARAMETER FLAMEOUT_MIN_ELAPSED_S IS 8.0.
PARAMETER STAGE_REARM_S IS 1.5.
// Roll is left FREE (the steering manager never holds it). Heading offset is a
// safety valve: leave 0 normally; set 180 only if your KSP build flies the
// mirror compass heading (the exported azimuth is verified due-east).
PARAMETER HEADING_OFFSET_DEG IS 0.
PARAMETER PRINT_STATUS IS TRUE.
PARAMETER STATUS_PERIOD_S IS 1.0.

// ----- Commanded state (the steering/throttle locks read these) -------------
GLOBAL POST2_YAW_DEG IS 90.
GLOBAL POST2_PITCH_DEG IS 90.
GLOBAL POST2_ROLL_DEG IS 0.
GLOBAL POST2_ROLL_LOCK IS FALSE.
GLOBAL POST2_THROTTLE IS 0.

// ----- Parsed trajectory table (filled by LOAD_TRAJECTORY) ------------------
GLOBAL POST2_TIMES IS LIST().
GLOBAL POST2_YAW IS LIST().
GLOBAL POST2_PITCH IS LIST().
GLOBAL POST2_ROLL IS LIST().
GLOBAL POST2_ROLLLOCK IS LIST().
GLOBAL POST2_THROT IS LIST().
GLOBAL POST2_PHASE_IDX IS LIST().
GLOBAL POST2_PHASE_NAME IS LIST().
GLOBAL POST2_STAGE_T IS LIST().
GLOBAL POST2_STAGE_PULSES IS LIST().
GLOBAL POST2_STAGE_SHUTDOWN IS LIST().
GLOBAL POST2_SEARCH_IDX IS 0.
// Plan time at which the hold-down clamp releases (flight-elapsed zero), or -1
// if the CSV carries no hold-down clamp.
GLOBAL POST2_RELEASE_PLAN_S IS -1.
// Flameout-staging tracking: peak available thrust seen on the current stage,
// and a "muted until" time set briefly after each stage while the next engine
// spins up (so a separation does not instantly re-trigger).
GLOBAL POST2_MAX_AVAIL_THRUST IS 0.
GLOBAL POST2_STAGE_REARM_AT IS -1.

// ===========================================================================
// Small numeric / string helpers
// ===========================================================================
FUNCTION CLAMP {
    PARAMETER VALUE, LOW_VALUE, HIGH_VALUE.
    RETURN MIN(MAX(VALUE, LOW_VALUE), HIGH_VALUE).
}.

FUNCTION NORMALIZE_HEADING {
    PARAMETER VALUE.
    UNTIL VALUE >= 0 { SET VALUE TO VALUE + 360. }.
    UNTIL VALUE < 360 { SET VALUE TO VALUE - 360. }.
    RETURN VALUE.
}.

// Shortest signed angular step from FROM_DEG toward TO_DEG, in (-180, 180].
FUNCTION ANGLE_DELTA {
    PARAMETER FROM_DEG, TO_DEG.
    LOCAL DELTA IS TO_DEG - FROM_DEG.
    UNTIL DELTA <= 180 { SET DELTA TO DELTA - 360. }.
    UNTIL DELTA >= -180 { SET DELTA TO DELTA + 360. }.
    RETURN DELTA.
}.

FUNCTION FIELD_NUM {
    PARAMETER FIELDS, INDEX_VALUE, DEFAULT_VALUE IS 0.
    IF INDEX_VALUE < 0 OR INDEX_VALUE >= FIELDS:LENGTH { RETURN DEFAULT_VALUE. }.
    LOCAL TEXT_VALUE IS FIELDS[INDEX_VALUE]:TRIM().
    LOCAL LOWER_VALUE IS TEXT_VALUE:TOLOWER().
    IF LOWER_VALUE = "true" { RETURN 1. }.
    IF LOWER_VALUE = "false" { RETURN 0. }.
    RETURN TEXT_VALUE:TONUMBER(DEFAULT_VALUE).
}.

FUNCTION FIELD_TEXT {
    PARAMETER FIELDS, INDEX_VALUE, DEFAULT_VALUE IS "".
    IF INDEX_VALUE < 0 OR INDEX_VALUE >= FIELDS:LENGTH { RETURN DEFAULT_VALUE. }.
    RETURN FIELDS[INDEX_VALUE]:TRIM().
}.

FUNCTION FIND_COL {
    PARAMETER HEADER_FIELDS, NAME_TEXT.
    LOCAL TARGET_TEXT IS NAME_TEXT:TOLOWER().
    LOCAL INDEX_VALUE IS 0.
    UNTIL INDEX_VALUE >= HEADER_FIELDS:LENGTH {
        IF HEADER_FIELDS[INDEX_VALUE]:TRIM():TOLOWER() = TARGET_TEXT { RETURN INDEX_VALUE. }.
        SET INDEX_VALUE TO INDEX_VALUE + 1.
    }.
    RETURN -1.
}.

FUNCTION RAISE_KOS_IPU {
    IF KOS_TARGET_IPU <= 0 { RETURN. }.
    IF CONFIG:IPU < KOS_TARGET_IPU {
        SET CONFIG:IPU TO KOS_TARGET_IPU.
        PRINT "kOS IPU set to " + CONFIG:IPU + ".".
    }.
}.

// ===========================================================================
// Staging / launch-support helpers (KSP side)
// ===========================================================================
FUNCTION CURRENT_KSP_STAGE_NUMBER {
    IF SHIP:HASSUFFIX("STAGENUM") { RETURN SHIP:STAGENUM. }.
    RETURN -999999.
}.

FUNCTION EXECUTE_KSP_STAGE {
    PARAMETER LABEL_TEXT IS "Stage".
    LOCAL BEFORE_STAGE IS CURRENT_KSP_STAGE_NUMBER().
    STAGE.
    WAIT STAGE_COMMAND_SETTLE_S.
    LOCAL AFTER_STAGE IS CURRENT_KSP_STAGE_NUMBER().
    IF BEFORE_STAGE > -999999 OR AFTER_STAGE > -999999 {
        PRINT LABEL_TEXT + " KSP stage " + BEFORE_STAGE + " -> " + AFTER_STAGE + ".".
    }.
}.

FUNCTION PART_HAS_MODULE {
    PARAMETER PART_ITEM, MODULE_NAME.
    IF NOT PART_ITEM:HASSUFFIX("MODULES") { RETURN FALSE. }.
    FOR MOD_NAME IN PART_ITEM:MODULES {
        IF MOD_NAME = MODULE_NAME { RETURN TRUE. }.
    }.
    RETURN FALSE.
}.

FUNCTION STAGE_HAS_LAUNCH_SUPPORT {
    PARAMETER STAGE_NUMBER.
    FOR PART_ITEM IN SHIP:PARTS {
        IF PART_ITEM:STAGE = STAGE_NUMBER {
            IF PART_HAS_MODULE(PART_ITEM, "LaunchClamp") { RETURN TRUE. }.
            IF PART_HAS_MODULE(PART_ITEM, "ModuleLaunchClamp") { RETURN TRUE. }.
        }.
    }.
    RETURN FALSE.
}.

FUNCTION VESSEL_HAS_LAUNCH_SUPPORT {
    FOR PART_ITEM IN SHIP:PARTS {
        IF PART_HAS_MODULE(PART_ITEM, "LaunchClamp") { RETURN TRUE. }.
        IF PART_HAS_MODULE(PART_ITEM, "ModuleLaunchClamp") { RETURN TRUE. }.
    }.
    RETURN FALSE.
}.

FUNCTION RELEASE_LAUNCH_SUPPORT {
    IF NOT AUTO_RELEASE_LAUNCH_SUPPORT { RETURN FALSE. }.

    LOCAL STAGE_MATCH IS FALSE.
    IF SHIP:HASSUFFIX("STAGENUM") {
        LOCAL NEXT_STAGE IS SHIP:STAGENUM.
        SET STAGE_MATCH TO STAGE_HAS_LAUNCH_SUPPORT(NEXT_STAGE).
        IF NOT STAGE_MATCH { SET STAGE_MATCH TO STAGE_HAS_LAUNCH_SUPPORT(NEXT_STAGE - 1). }.
        IF NOT STAGE_MATCH { SET STAGE_MATCH TO STAGE_HAS_LAUNCH_SUPPORT(NEXT_STAGE + 1). }.
    }.

    IF STAGE_MATCH {
        EXECUTE_KSP_STAGE("Launch support release").
        RETURN TRUE.
    }.
    IF VESSEL_HAS_LAUNCH_SUPPORT() {
        // A clamp exists but is not on the staging boundary we detected; fire the
        // next stage anyway so we are not left pinned to the pad.
        EXECUTE_KSP_STAGE("Launch support fallback").
        RETURN TRUE.
    }.
    PRINT "No launch support found on vessel; release skipped.".
    RETURN FALSE.
}.

FUNCTION EXECUTE_STAGE_EVENT {
    PARAMETER LABEL_TEXT, PULSE_COUNT, SHUTDOWN_FIRST.
    LOCAL PULSE_TOTAL IS MAX(STAGE_MIN_PULSES, PULSE_COUNT).
    PRINT LABEL_TEXT + " (pulses " + PULSE_TOTAL + ", shutdown=" + SHUTDOWN_FIRST + ").".
    LOCAL RESTORE_THROTTLE IS POST2_THROTTLE.
    IF SHUTDOWN_FIRST {
        SET POST2_THROTTLE TO 0.
        WAIT STAGE_SHUTDOWN_SETTLE_S.
    }.
    LOCAL PULSE_INDEX IS 0.
    UNTIL PULSE_INDEX >= PULSE_TOTAL {
        EXECUTE_KSP_STAGE(LABEL_TEXT + " pulse " + (PULSE_INDEX + 1)).
        SET PULSE_INDEX TO PULSE_INDEX + 1.
    }.
    SET POST2_THROTTLE TO CLAMP(RESTORE_THROTTLE, 0, 1).
}.

// ===========================================================================
// Steering / interpolation
// ===========================================================================
FUNCTION STEERING_DIRECTION {
    // Roll is left free: command heading + pitch only, never a roll angle, so the
    // steering manager does not fight the vehicle's natural roll.
    RETURN HEADING(POST2_YAW_DEG, POST2_PITCH_DEG).
}.

// Re-arm flameout staging for the newly active stage: forget the dropped stage's
// peak thrust and mute the check briefly while the next engine spins up.
FUNCTION RESET_STAGE_TRACKING {
    SET POST2_MAX_AVAIL_THRUST TO 0.
    SET POST2_STAGE_REARM_AT TO TIME:SECONDS + STAGE_REARM_S.
}.

// TRUE when the active engines have flamed out / run dry: available thrust has
// collapsed to a small fraction of the peak seen on this stage while throttle is
// still commanded. Independent of the CSV cue timing.
FUNCTION FLAMEOUT_STAGE_NEEDED {
    PARAMETER ELAPSED_S.
    IF NOT AUTO_STAGE_ON_FLAMEOUT { RETURN FALSE. }.
    IF POST2_THROTTLE <= MIN_COMMAND_THROTTLE { RETURN FALSE. }.
    IF ELAPSED_S < FLAMEOUT_MIN_ELAPSED_S { RETURN FALSE. }.
    IF POST2_STAGE_REARM_AT >= 0 AND TIME:SECONDS < POST2_STAGE_REARM_AT { RETURN FALSE. }.
    LOCAL AVAIL IS 0.
    IF SHIP:HASSUFFIX("AVAILABLETHRUST") { SET AVAIL TO SHIP:AVAILABLETHRUST. }.
    IF AVAIL > POST2_MAX_AVAIL_THRUST { SET POST2_MAX_AVAIL_THRUST TO AVAIL. }.
    IF POST2_MAX_AVAIL_THRUST <= 0 { RETURN FALSE. }.
    RETURN AVAIL <= FLAMEOUT_THRUST_FRACTION * POST2_MAX_AVAIL_THRUST.
}.

FUNCTION ROW_COMMAND {
    PARAMETER INDEX_VALUE.
    LOCAL RL IS 0.
    IF POST2_ROLLLOCK[INDEX_VALUE] > 0.5 { SET RL TO 1. }.
    RETURN LIST(
        POST2_YAW[INDEX_VALUE],
        POST2_PITCH[INDEX_VALUE],
        POST2_ROLL[INDEX_VALUE],
        RL,
        POST2_THROT[INDEX_VALUE]).
}.

// Linear interpolation of the attitude/throttle table at plan time PLAN_T.
// Heading and roll interpolate along the shortest arc. Returns
// LIST(yaw, pitch, roll, roll_lock, throttle).
FUNCTION COMMAND_AT {
    PARAMETER PLAN_T.
    LOCAL N IS POST2_TIMES:LENGTH.
    IF N = 0 { RETURN LIST(POST2_YAW_DEG, POST2_PITCH_DEG, POST2_ROLL_DEG, 0, 0). }.
    IF PLAN_T <= POST2_TIMES[0] { RETURN ROW_COMMAND(0). }.
    IF PLAN_T >= POST2_TIMES[N - 1] { RETURN ROW_COMMAND(N - 1). }.

    LOCAL I IS POST2_SEARCH_IDX.
    IF I < 0 OR I >= N - 1 OR POST2_TIMES[I] > PLAN_T { SET I TO 0. }.
    UNTIL I + 1 >= N OR POST2_TIMES[I + 1] > PLAN_T { SET I TO I + 1. }.
    SET POST2_SEARCH_IDX TO I.

    LOCAL SPAN_S IS POST2_TIMES[I + 1] - POST2_TIMES[I].
    LOCAL ALPHA IS 0.
    IF SPAN_S > 0 { SET ALPHA TO CLAMP((PLAN_T - POST2_TIMES[I]) / SPAN_S, 0, 1). }.

    LOCAL YAW_DEG IS NORMALIZE_HEADING(POST2_YAW[I] + ANGLE_DELTA(POST2_YAW[I], POST2_YAW[I + 1]) * ALPHA).
    LOCAL PITCH_DEG IS POST2_PITCH[I] + (POST2_PITCH[I + 1] - POST2_PITCH[I]) * ALPHA.
    LOCAL ROLL_DEG IS NORMALIZE_HEADING(POST2_ROLL[I] + ANGLE_DELTA(POST2_ROLL[I], POST2_ROLL[I + 1]) * ALPHA).
    LOCAL THROTTLE_CMD IS CLAMP(POST2_THROT[I] + (POST2_THROT[I + 1] - POST2_THROT[I]) * ALPHA, 0, 1).
    LOCAL RL IS 0.
    IF POST2_ROLLLOCK[I] > 0.5 { SET RL TO 1. }.
    RETURN LIST(YAW_DEG, PITCH_DEG, ROLL_DEG, RL, THROTTLE_CMD).
}.

FUNCTION APPLY_COMMAND {
    PARAMETER COMMAND_ROW.
    SET POST2_YAW_DEG TO NORMALIZE_HEADING(COMMAND_ROW[0] + HEADING_OFFSET_DEG).
    SET POST2_PITCH_DEG TO CLAMP(COMMAND_ROW[1], -90, 90).
    SET POST2_THROTTLE TO CLAMP(COMMAND_ROW[4], 0, 1).
}.

// ===========================================================================
// CSV load
// ===========================================================================
// Returns TRUE on success. Populates the POST2_* table globals.
FUNCTION LOAD_TRAJECTORY {
    IF NOT EXISTS(TRAJ_PATH) {
        PRINT "Missing trajectory CSV: " + TRAJ_PATH.
        PRINT "In POST2 Lite use File > Export kOS trajectory CSV.".
        RETURN FALSE.
    }.

    LOCAL CONTENT IS OPEN(TRAJ_PATH):READALL.
    IF CONTENT:EMPTY {
        PRINT "Trajectory CSV is empty: " + TRAJ_PATH.
        RETURN FALSE.
    }.

    LOCAL HEADER_READY IS FALSE.
    LOCAL COL_TIME IS -1.
    LOCAL COL_THROTTLE IS -1.
    LOCAL COL_HOLD_DOWN IS -1.
    LOCAL COL_PHASE_INDEX IS -1.
    LOCAL COL_PHASE_NAME IS -1.
    LOCAL COL_YAW IS -1.
    LOCAL COL_PITCH IS -1.
    LOCAL COL_ROLL IS -1.
    LOCAL COL_ROLL_LOCK IS -1.
    LOCAL COL_STAGE_CMD IS -1.
    LOCAL COL_STAGE_TIME IS -1.
    LOCAL COL_STAGE_PULSES IS -1.
    LOCAL COL_STAGE_SHUTDOWN IS -1.

    LOCAL LAST_STAGE_TIME IS -1.0E18.
    LOCAL SAW_HOLD_ACTIVE IS FALSE.

    FOR RAW_LINE IN CONTENT {
        LOCAL LINE_TEXT IS RAW_LINE:TRIM().
        IF LINE_TEXT:LENGTH = 0 {
            // skip blank lines
        } ELSE IF NOT HEADER_READY {
            LOCAL H IS LINE_TEXT:SPLIT(",").
            SET COL_TIME TO FIND_COL(H, "time_s").
            SET COL_THROTTLE TO FIND_COL(H, "throttle").
            SET COL_HOLD_DOWN TO FIND_COL(H, "hold_down_clamp_active").
            SET COL_PHASE_INDEX TO FIND_COL(H, "phase_index").
            SET COL_PHASE_NAME TO FIND_COL(H, "phase_name").
            SET COL_YAW TO FIND_COL(H, "kos_yaw_deg").
            SET COL_PITCH TO FIND_COL(H, "kos_pitch_deg").
            SET COL_ROLL TO FIND_COL(H, "kos_roll_deg").
            SET COL_ROLL_LOCK TO FIND_COL(H, "kos_roll_lock").
            SET COL_STAGE_CMD TO FIND_COL(H, "kos_stage_command").
            SET COL_STAGE_TIME TO FIND_COL(H, "kos_stage_plan_time_s").
            SET COL_STAGE_PULSES TO FIND_COL(H, "kos_stage_pulse_count").
            SET COL_STAGE_SHUTDOWN TO FIND_COL(H, "kos_shutdown_before_stage").

            IF COL_TIME < 0 OR COL_THROTTLE < 0 OR COL_YAW < 0 OR COL_PITCH < 0 {
                PRINT "CSV is missing host-computed attitude columns.".
                PRINT "Need time_s, throttle, kos_yaw_deg, kos_pitch_deg.".
                PRINT "Re-export with File > Export kOS trajectory CSV.".
                RETURN FALSE.
            }.
            SET HEADER_READY TO TRUE.
        } ELSE {
            LOCAL F IS LINE_TEXT:SPLIT(",").
            LOCAL PLAN_TIME_S IS FIELD_NUM(F, COL_TIME, 0).

            IF COL_HOLD_DOWN >= 0 {
                LOCAL HOLD_DOWN_ACTIVE IS FIELD_NUM(F, COL_HOLD_DOWN, 0) > 0.5.
                IF HOLD_DOWN_ACTIVE {
                    SET SAW_HOLD_ACTIVE TO TRUE.
                } ELSE IF SAW_HOLD_ACTIVE AND POST2_RELEASE_PLAN_S < 0 {
                    SET POST2_RELEASE_PLAN_S TO PLAN_TIME_S.
                }.
            }.

            POST2_TIMES:ADD(PLAN_TIME_S).
            POST2_YAW:ADD(NORMALIZE_HEADING(FIELD_NUM(F, COL_YAW, 90))).
            POST2_PITCH:ADD(CLAMP(FIELD_NUM(F, COL_PITCH, 90), -90, 90)).
            POST2_ROLL:ADD(FIELD_NUM(F, COL_ROLL, 0)).
            POST2_ROLLLOCK:ADD(FIELD_NUM(F, COL_ROLL_LOCK, 0)).
            POST2_THROT:ADD(CLAMP(FIELD_NUM(F, COL_THROTTLE, 0), 0, 1)).
            POST2_PHASE_IDX:ADD(FIELD_NUM(F, COL_PHASE_INDEX, -999999)).
            POST2_PHASE_NAME:ADD(FIELD_TEXT(F, COL_PHASE_NAME, "")).

            IF FIELD_NUM(F, COL_STAGE_CMD, 0) > 0.5 {
                LOCAL STAGE_TIME_S IS FIELD_NUM(F, COL_STAGE_TIME, PLAN_TIME_S).
                // The exporter can flag the same separation on consecutive rows;
                // keep one event per distinct plan time.
                IF STAGE_TIME_S > LAST_STAGE_TIME + 0.0000001 {
                    POST2_STAGE_T:ADD(STAGE_TIME_S).
                    POST2_STAGE_PULSES:ADD(FIELD_NUM(F, COL_STAGE_PULSES, 1)).
                    POST2_STAGE_SHUTDOWN:ADD(FIELD_NUM(F, COL_STAGE_SHUTDOWN, 1)).
                    SET LAST_STAGE_TIME TO STAGE_TIME_S.
                }.
            }.
        }.
    }.

    IF NOT HEADER_READY {
        PRINT "Trajectory CSV has no header.".
        RETURN FALSE.
    }.
    IF POST2_TIMES:LENGTH = 0 {
        PRINT "Trajectory CSV has no data rows.".
        RETURN FALSE.
    }.
    RETURN TRUE.
}.

FUNCTION FIRST_POWERED_TIME {
    LOCAL I IS 0.
    UNTIL I >= POST2_THROT:LENGTH {
        IF POST2_THROT[I] > MIN_COMMAND_THROTTLE { RETURN POST2_TIMES[I]. }.
        SET I TO I + 1.
    }.
    RETURN -1.
}.

// ===========================================================================
// Main
// ===========================================================================
FUNCTION OPEN_LOOP_MAIN {
    RAISE_KOS_IPU().
    IF NOT LOAD_TRAJECTORY() { RETURN. }.

    SAS OFF.
    RCS OFF.
    SET POST2_YAW_DEG TO 90.
    SET POST2_PITCH_DEG TO 90.
    SET POST2_ROLL_DEG TO 0.
    SET POST2_ROLL_LOCK TO FALSE.
    SET POST2_THROTTLE TO 0.
    SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 0.
    LOCK STEERING TO STEERING_DIRECTION().
    LOCK THROTTLE TO POST2_THROTTLE.

    LOCAL N IS POST2_TIMES:LENGTH.
    LOCAL FIRST_PLAN_S IS POST2_TIMES[0].
    LOCAL LAST_PLAN_S IS POST2_TIMES[N - 1].
    LOCAL FIRST_POWERED_S IS FIRST_POWERED_TIME().

    // Plan time that maps to flight-elapsed zero. If the CSV ramps thrust under a
    // hold-down clamp we replace that ramp with a fixed ignition + lead, so zero
    // is the clamp-release sample; otherwise it is the first powered sample (or,
    // failing that, the first sample).
    LOCAL ZERO_PLAN_S IS FIRST_PLAN_S.
    IF FIRST_POWERED_S >= 0 { SET ZERO_PLAN_S TO FIRST_POWERED_S. }.
    IF POST2_RELEASE_PLAN_S >= 0 { SET ZERO_PLAN_S TO POST2_RELEASE_PLAN_S. }.

    PRINT "POST2 open-loop CSV: " + TRAJ_PATH.
    PRINT "Samples: " + N + ", stage events: " + POST2_STAGE_T:LENGTH + ", t0 plan time: " + ROUND(ZERO_PLAN_S, 2) + "s.".

    // Orient to the launch attitude before ignition.
    LOCAL LAUNCH_CMD IS COMMAND_AT(ZERO_PLAN_S).
    APPLY_COMMAND(LAUNCH_CMD).

    LOCAL LAUNCHED IS FALSE.
    IF AUTO_LAUNCH AND FIRST_POWERED_S >= 0 {
        PRINT "Ignition; release in " + ROUND(ENGINE_STARTUP_LEAD_S + LAUNCH_SUPPORT_RELEASE_DELAY, 2) + "s.".
        SET POST2_THROTTLE TO LAUNCH_CMD[4].
        LOCAL IGNITION_S IS TIME:SECONDS.
        EXECUTE_KSP_STAGE("Ignition").
        SET LAUNCHED TO TRUE.
        LOCAL RELEASE_S IS IGNITION_S + MAX(0, ENGINE_STARTUP_LEAD_S + LAUNCH_SUPPORT_RELEASE_DELAY).
        UNTIL TIME:SECONDS >= RELEASE_S { WAIT 0. }.
        RELEASE_LAUNCH_SUPPORT().
    }.

    // Wall-clock playback (KSPTOT style): plan time advances with real time.
    // START_REAL is chosen so plan time == ZERO_PLAN_S right now.
    LOCAL START_REAL_S IS TIME:SECONDS.
    LOCAL NEXT_EVENT IS 0.
    LOCAL LAST_PHASE_IDX IS -999999.
    LOCAL LAST_STATUS_PLAN_S IS ZERO_PLAN_S - STATUS_PERIOD_S.
    LOCAL TICKS IS 0.

    // Skip stage events that fall before playback start (none expected).
    UNTIL NEXT_EVENT >= POST2_STAGE_T:LENGTH OR POST2_STAGE_T[NEXT_EVENT] >= ZERO_PLAN_S {
        SET NEXT_EVENT TO NEXT_EVENT + 1.
    }.

    UNTIL FALSE {
        LOCAL PLAN_NOW_S IS ZERO_PLAN_S + (TIME:SECONDS - START_REAL_S).
        IF PLAN_NOW_S > LAST_PLAN_S { BREAK. }.

        APPLY_COMMAND(COMMAND_AT(PLAN_NOW_S)).

        LOCAL STAGED IS FALSE.
        LOCAL TARGET_ELAPSED_S IS PLAN_NOW_S - ZERO_PLAN_S.

        // Flameout / depletion: separate as soon as the active stage runs dry,
        // even before the planned CSV time. Early-fire the next pending CSV stage
        // event (using its pulse count / shutdown flag). With no pending event
        // (e.g. the final burn-to-depletion stage) do nothing -- that MECO is a
        // commanded shutdown, not a separation.
        IF LAUNCHED AND (NOT STAGED) AND NEXT_EVENT < POST2_STAGE_T:LENGTH AND FLAMEOUT_STAGE_NEEDED(TARGET_ELAPSED_S) {
            EXECUTE_STAGE_EVENT(
                "Flameout stage t+" + ROUND(TARGET_ELAPSED_S, 2) + "s",
                POST2_STAGE_PULSES[NEXT_EVENT],
                POST2_STAGE_SHUTDOWN[NEXT_EVENT] > 0.5).
            SET NEXT_EVENT TO NEXT_EVENT + 1.
            SET STAGED TO TRUE.
            RESET_STAGE_TRACKING().
        }.

        // Planned CSV stage cues whose plan time has arrived.
        UNTIL NEXT_EVENT >= POST2_STAGE_T:LENGTH OR POST2_STAGE_T[NEXT_EVENT] > PLAN_NOW_S {
            EXECUTE_STAGE_EVENT(
                "Stage event t+" + ROUND(POST2_STAGE_T[NEXT_EVENT] - ZERO_PLAN_S, 2) + "s",
                POST2_STAGE_PULSES[NEXT_EVENT],
                POST2_STAGE_SHUTDOWN[NEXT_EVENT] > 0.5).
            SET NEXT_EVENT TO NEXT_EVENT + 1.
            SET STAGED TO TRUE.
            RESET_STAGE_TRACKING().
        }.

        IF STAGED {
            // A staging maneuver burns real seconds (shutdown + pulses + settle)
            // while plan time should stand still. Re-anchor the clock to "now" so
            // playback resumes where it paused instead of fast-forwarding.
            SET START_REAL_S TO TIME:SECONDS - (PLAN_NOW_S - ZERO_PLAN_S).
        }.

        IF PRINT_STATUS {
            LOCAL PHASE_IDX_NOW IS POST2_PHASE_IDX[POST2_SEARCH_IDX].
            IF PHASE_IDX_NOW <> LAST_PHASE_IDX {
                PRINT "Phase " + PHASE_IDX_NOW + " (" + POST2_PHASE_NAME[POST2_SEARCH_IDX] + ") at t+" + ROUND(PLAN_NOW_S - ZERO_PLAN_S, 2) + "s.".
                SET LAST_PHASE_IDX TO PHASE_IDX_NOW.
            }.
            IF PLAN_NOW_S - LAST_STATUS_PLAN_S >= STATUS_PERIOD_S {
                PRINT "t+" + ROUND(PLAN_NOW_S - ZERO_PLAN_S, 1) + "s yaw " + ROUND(POST2_YAW_DEG, 1) + " pitch " + ROUND(POST2_PITCH_DEG, 1) + " thr " + ROUND(POST2_THROTTLE * 100, 0) + "%".
                SET LAST_STATUS_PLAN_S TO PLAN_NOW_S.
            }.
        }.

        SET TICKS TO TICKS + 1.
        WAIT 0.
    }.

    SET POST2_THROTTLE TO 0.
    WAIT 0.1.
    UNLOCK THROTTLE.
    UNLOCK STEERING.
    PRINT "POST2 open-loop complete, ticks executed: " + TICKS + ".".
}.

OPEN_LOOP_MAIN().
