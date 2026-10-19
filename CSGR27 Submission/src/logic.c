/*
 * CSG Racing embedded assessment - reference implementation.
 *
 * This is a pure DECISION function: it produces a state and two annunciation
 * requests. Real disconnection would require an independent, engineered
 * hardware shutdown path and a separate actuator supervisor. In particular,
 * this mock API has no feedback freshness/validity for PV, grid or relay,
 * and there is no explicit reset/acknowledge input for a persistent trip latch.
 *
 * Policy assumptions (home-battery simulation, NOT CSG vehicle limits):
 *   8-cell bench model: 3000..4200 mV per cell; pack current <= +/-100 A;
 *   battery temperatures -10..60 C; DC PV 0..600 V / 0..30 A;
 *   nominal UK grid envelope 207..253 V, 49..51 Hz;
 *   critical network snapshot age <= 1000 ms.
 * Values require validation against the REAL battery chemistry, sensor,
 * inverter, utility grid-code and relay datasheets before any deployment.
 *
 * Faults are evaluated independently on each call because main.c replays
 * isolated test-world snapshots. A real 24/7 controller needs a persistent
 * trip latch, debounce/restart permission and actuator feedback timeouts.
 */

#include "logic.h"
#include "mock_hw.h"

#include <math.h>
#include <stddef.h>

#define MIN_CELL_MV       3000.0f
#define MAX_CELL_MV       4200.0f
#define MAX_PACK_CURRENT_A 100.0f
#define MIN_BATTERY_TEMP_C (-10.0f)
#define MAX_BATTERY_TEMP_C 60.0f
#define MAX_PV_VOLTAGE_V  600.0f
#define MAX_PV_CURRENT_A  30.0f
#define MIN_GRID_VOLTAGE_V 207.0f
#define MAX_GRID_VOLTAGE_V 253.0f
#define MIN_GRID_FREQ_HZ  49.0f
#define MAX_GRID_FREQ_HZ  51.0f
#define MAX_MESSAGE_AGE_MS 1000u

static bool in_closed_range(float value, float lower, float upper) {
    return isfinite(value) && value >= lower && value <= upper;
}

/* Unsigned subtraction accepts a normal uint32_t millisecond-clock rollover;
   a timestamp implausibly ahead of now is conservatively considered stale. */
static bool is_fresh(uint32_t update_ms, uint32_t now_ms) {
    return update_ms != 0u && (uint32_t)(now_ms - update_ms) <= MAX_MESSAGE_AGE_MS;
}

static bool bms_healthy(const BmsData *bms, uint32_t now_ms) {
    if (!bms->comms_ok || !is_fresh(bms->last_update_ms, now_ms)) {
        return false;
    }
    if (!in_closed_range(bms->pack_current_a,
                         -MAX_PACK_CURRENT_A, MAX_PACK_CURRENT_A) ||
        !in_closed_range(bms->pack_temp_c,
                         MIN_BATTERY_TEMP_C, MAX_BATTERY_TEMP_C)) {
        return false;
    }
    for (size_t i = 0; i < BMS_NUM_CELLS; ++i) {
        if (!in_closed_range(bms->cell_voltages_mv[i],
                             MIN_CELL_MV, MAX_CELL_MV)) {
            return false;
        }
    }
    return true;
}

static bool inverter_healthy(const InverterData *inverter, uint32_t now_ms) {
    return inverter->comms_ok &&
           is_fresh(inverter->last_update_ms, now_ms) &&
           inverter->fault_code == 0u &&
           inverter->running;
}

static bool battery_temp_healthy(const BatteryTempData *temp) {
    return temp->valid && in_closed_range(temp->temp_c,
                                          MIN_BATTERY_TEMP_C,
                                          MAX_BATTERY_TEMP_C);
}

static bool pv_healthy(const PvData *pv) {
    /* No 'present' or 'valid' signal exists; 0 A is permitted at night.
       Impossible, negative and non-finite values are treated as faults. */
    return in_closed_range(pv->voltage_v, 0.0f, MAX_PV_VOLTAGE_V) &&
           in_closed_range(pv->current_a, 0.0f, MAX_PV_CURRENT_A);
}

static bool grid_healthy(const GridData *grid) {
    /* There is no island mode in the task, so grid loss means OFF. */
    return grid->present &&
           in_closed_range(grid->voltage_v,
                           MIN_GRID_VOLTAGE_V, MAX_GRID_VOLTAGE_V) &&
           in_closed_range(grid->frequency_hz,
                           MIN_GRID_FREQ_HZ, MAX_GRID_FREQ_HZ);
}

static bool relay_healthy(const RelayState *relay) {
    /* A mismatch is hazardous; the ON state also requires a closed path.
       Production design would require time-filtered force-guided contacts. */
    return relay->commanded_closed && relay->feedback_closed;
}

SystemOutputs evaluate_system_state(BmsData bms, BatteryTempData batt_temp,
                                     InverterData inverter, PvData pv,
                                     GridData grid, RelayState relay,
                                     DoorSwitchData door) {
    const uint32_t now_ms = read_system_time_ms();
    const bool safe = bms_healthy(&bms, now_ms) &&
                      battery_temp_healthy(&batt_temp) &&
                      inverter_healthy(&inverter, now_ms) &&
                      pv_healthy(&pv) &&
                      grid_healthy(&grid) &&
                      relay_healthy(&relay) &&
                      !door.triggered;

    SystemOutputs out;
    out.state = safe ? SYSTEM_ON : SYSTEM_OFF;
    out.buzzer_on = !safe;
    out.notification_flag = !safe;
    return out;
}
