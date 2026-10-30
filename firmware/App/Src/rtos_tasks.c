#include "rtos_tasks.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

#include "astar.h"
#include "ir_array.h"
#include "kalman.h"
#include "map.h"
#include "motor.h"
#include "mpu6050.h"
#include "navigation.h"
#include "obstacle.h"
#include "obstacle_manager.h"
#include "oled.h"
#include "rfid.h"
#include "rfid_station.h"

/* ==============================================================
 * EXACT WORKING TEST CONFIGURATION FROM THIS UPLOADED FOLDER
 * ============================================================== */

#define RTOS_START_NODE                 MAP_S1
#define RTOS_DEST_NODE                  MAP_S4
#define RTOS_START_HEADING              MAP_DIR_SOUTH

#define RTOS_NAV_KP                     5.0f
#define RTOS_NAV_KI                     0.0f
#define RTOS_NAV_KD                     0.0f
#define RTOS_NAV_DT_S                   0.010f
#define RTOS_NAV_BASE_SPEED             50U

#define RTOS_CONTROL_PERIOD_MS          10U
#define RTOS_OBSTACLE_PERIOD_MS         10U
#define RTOS_RFID_PERIOD_MS             100U
#define RTOS_DISPLAY_PERIOD_MS          50U

#define RTOS_CALIBRATION_TICKS          200U
#define RTOS_GYRO_MIN_SAMPLES           50U
#define RTOS_ROUTE_TIMEOUT_TICKS        9000U
#define RTOS_DEST_WAIT_TICKS            300U
#define RTOS_OBSTACLE_IGNORE_TICKS      50U

/* Priorities: motor/navigation owns the highest application priority. */
#define RTOS_CONTROL_PRIORITY           (tskIDLE_PRIORITY + 4U)
#define RTOS_OBSTACLE_PRIORITY          (tskIDLE_PRIORITY + 3U)
#define RTOS_RFID_PRIORITY              (tskIDLE_PRIORITY + 2U)
#define RTOS_DISPLAY_PRIORITY           (tskIDLE_PRIORITY + 1U)

#define RTOS_CONTROL_STACK_WORDS        768U
#define RTOS_OBSTACLE_STACK_WORDS       256U
#define RTOS_RFID_STACK_WORDS           384U
#define RTOS_DISPLAY_STACK_WORDS        384U

/* Event-group bits. Tasks publish facts; ControlTask owns motion/state. */
#define RTOS_EVENT_START_RFID_OK        (1U << 0)
#define RTOS_EVENT_DEST_RFID_OK         (1U << 1)
#define RTOS_EVENT_OBSTACLE             (1U << 2)

/* ==============================================================
 * NAVIGATION DEBUG EXTERNS
 * ============================================================== */

extern volatile uint8_t debug_nav_ir_pattern;
extern volatile int8_t debug_nav_line_error;
extern volatile uint8_t debug_nav_junction_candidate;
extern volatile uint8_t debug_nav_mode;
extern volatile uint8_t debug_nav_junction_armed;
extern volatile uint8_t debug_nav_junction_accepted;
extern volatile uint8_t debug_nav_action;
extern volatile int8_t debug_nav_edge_bend_direction;
extern volatile uint8_t debug_nav_edge_bend_pending;
extern volatile uint8_t debug_nav_edge_bend_active;
extern volatile uint8_t debug_nav_edge_bend_complete;
extern volatile uint16_t debug_nav_turn_ticks;
extern volatile uint8_t debug_nav_turn_center_lost;
extern volatile uint8_t debug_nav_turn_branch_seen;
extern volatile uint8_t debug_nav_turn_center_count;

/* ==============================================================
 * PRIVATE RTOS OBJECTS / WORKING DATA
 * ============================================================== */

static AStarPath rtos_path;
static MPU6050Data rtos_mpu_data;
static Kalman1D rtos_gyro_filter;

static EventGroupHandle_t rtos_events = NULL;
static SemaphoreHandle_t rtos_i2c_mutex = NULL;

static TaskHandle_t control_task_handle = NULL;
static TaskHandle_t obstacle_task_handle = NULL;
static TaskHandle_t rfid_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;

static volatile RTOSAgvState rtos_state = RTOS_AGV_CALIBRATING;

static uint8_t rtos_route_found = 0U;
static uint8_t rtos_mpu_init_ok = 0U;
static uint8_t rtos_mpu_calibrated = 0U;
static float rtos_gyro_bias_dps = 0.0f;

/* ==============================================================
 * DEBUG VARIABLES
 * ============================================================== */

volatile uint8_t debug_rtos_state = RTOS_AGV_CALIBRATING;
volatile uint8_t debug_rtos_route_found = 0U;
volatile uint8_t debug_rtos_path_length = 0U;
volatile uint8_t debug_rtos_path[MAP_NODE_COUNT];

volatile uint8_t debug_rtos_ir_pattern = 0U;
volatile int8_t debug_rtos_line_error = 0;
volatile uint8_t debug_rtos_path_index = 0U;
volatile uint8_t debug_rtos_current_node = (uint8_t)MAP_NODE_INVALID;
volatile uint8_t debug_rtos_next_node = (uint8_t)MAP_NODE_INVALID;

volatile uint8_t debug_rtos_nav_mode = 0U;
volatile uint8_t debug_rtos_nav_junction_armed = 0U;
volatile uint8_t debug_rtos_nav_junction_accepted = 0U;
volatile uint8_t debug_rtos_nav_action = (uint8_t)NAV_ACTION_STOP;
volatile int8_t debug_rtos_nav_edge_bend_direction = 0;
volatile uint8_t debug_rtos_nav_edge_bend_pending = 0U;
volatile uint8_t debug_rtos_nav_edge_bend_active = 0U;
volatile uint8_t debug_rtos_nav_edge_bend_complete = 0U;
volatile uint16_t debug_rtos_turn_ticks = 0U;
volatile uint8_t debug_rtos_turn_center_lost = 0U;
volatile uint8_t debug_rtos_turn_branch_seen = 0U;
volatile uint8_t debug_rtos_turn_center_count = 0U;

volatile uint8_t debug_rtos_obstacle_raw = 0U;
volatile uint8_t debug_rtos_obstacle_detected = 0U;
volatile uint8_t debug_rtos_replan_count = 0U;

volatile uint8_t debug_rtos_rfid_initialized = 0U;
volatile uint8_t debug_rtos_rfid_detected = 0U;
volatile uint8_t debug_rtos_rfid_station = 0U;
volatile uint8_t debug_rtos_rfid_uid0 = 0U;
volatile uint8_t debug_rtos_rfid_uid1 = 0U;
volatile uint8_t debug_rtos_rfid_uid2 = 0U;
volatile uint8_t debug_rtos_rfid_uid3 = 0U;
volatile uint8_t debug_rtos_start_rfid_ok = 0U;
volatile uint8_t debug_rtos_dest_rfid_ok = 0U;
volatile uint8_t debug_rtos_destination_station = 0U;

volatile uint8_t debug_rtos_mpu_init_ok = 0U;
volatile uint8_t debug_rtos_mpu_who_ok = 0U;
volatile uint8_t debug_rtos_mpu_who_am_i = 0U;
volatile uint8_t debug_rtos_mpu_read_ok = 0U;
volatile uint8_t debug_rtos_mpu_calibrated = 0U;
volatile uint32_t debug_rtos_gyro_samples = 0U;
volatile float debug_rtos_gyro_raw_dps = 0.0f;
volatile float debug_rtos_gyro_bias_dps = 0.0f;
volatile float debug_rtos_gyro_filtered_dps = 0.0f;
volatile float debug_rtos_turn_yaw_deg = 0.0f;

volatile uint8_t debug_rtos_oled_ok = 0U;
volatile uint32_t debug_rtos_control_ticks = 0U;
volatile uint32_t debug_rtos_calibration_ticks = 0U;
volatile uint32_t debug_rtos_dest_wait_ticks = 0U;

volatile uint32_t debug_rtos_control_stack_free = 0U;
volatile uint32_t debug_rtos_obstacle_stack_free = 0U;
volatile uint32_t debug_rtos_rfid_stack_free = 0U;
volatile uint32_t debug_rtos_display_stack_free = 0U;

/* ==============================================================
 * SMALL HELPERS
 * ============================================================== */

static void RTOS_SetState(RTOSAgvState state)
{
    rtos_state = state;
    debug_rtos_state = (uint8_t)state;
}

static void RTOS_CopyPathDebug(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)MAP_NODE_COUNT; i++)
    {
        debug_rtos_path[i] = (uint8_t)MAP_NODE_INVALID;
    }

    debug_rtos_path_length = rtos_path.length;

    for (i = 0U; i < rtos_path.length; i++)
    {
        debug_rtos_path[i] = (uint8_t)rtos_path.nodes[i];
    }
}

static void RTOS_UpdateRouteDebug(void)
{
    uint8_t index = Navigation_GetPathIndex();

    debug_rtos_path_index = index;
    debug_rtos_current_node = (uint8_t)MAP_NODE_INVALID;
    debug_rtos_next_node = (uint8_t)MAP_NODE_INVALID;

    if (index < rtos_path.length)
    {
        debug_rtos_current_node = (uint8_t)rtos_path.nodes[index];
    }

    if ((index + 1U) < rtos_path.length)
    {
        debug_rtos_next_node = (uint8_t)rtos_path.nodes[index + 1U];
    }
}

static void RTOS_CopyNavigationDebug(void)
{
    debug_rtos_ir_pattern = debug_nav_ir_pattern;
    debug_rtos_line_error = debug_nav_line_error;
    debug_rtos_nav_mode = debug_nav_mode;
    debug_rtos_nav_junction_armed = debug_nav_junction_armed;
    debug_rtos_nav_junction_accepted = debug_nav_junction_accepted;
    debug_rtos_nav_action = debug_nav_action;
    debug_rtos_nav_edge_bend_direction = debug_nav_edge_bend_direction;
    debug_rtos_nav_edge_bend_pending = debug_nav_edge_bend_pending;
    debug_rtos_nav_edge_bend_active = debug_nav_edge_bend_active;
    debug_rtos_nav_edge_bend_complete = debug_nav_edge_bend_complete;
    debug_rtos_turn_ticks = debug_nav_turn_ticks;
    debug_rtos_turn_center_lost = debug_nav_turn_center_lost;
    debug_rtos_turn_branch_seen = debug_nav_turn_branch_seen;
    debug_rtos_turn_center_count = debug_nav_turn_center_count;
}

static uint8_t RTOS_ReadMPU(float *filtered_dps)
{
    uint8_t ok = 0U;

    if ((!rtos_mpu_calibrated) || (filtered_dps == NULL))
    {
        return 0U;
    }

    if (xSemaphoreTake(rtos_i2c_mutex, pdMS_TO_TICKS(5U)) == pdTRUE)
    {
        ok = MPU6050_Read(&rtos_mpu_data);
        xSemaphoreGive(rtos_i2c_mutex);
    }

    debug_rtos_mpu_read_ok = ok;

    if (!ok)
    {
        return 0U;
    }

    debug_rtos_gyro_raw_dps = rtos_mpu_data.gyro_z_dps;

    *filtered_dps = Kalman1D_Update(
        &rtos_gyro_filter,
        rtos_mpu_data.gyro_z_dps - rtos_gyro_bias_dps);

    debug_rtos_gyro_filtered_dps = *filtered_dps;

    return 1U;
}

static void RTOS_StopTerminal(RTOSAgvState state)
{
    Navigation_Stop();
    Motor_Stop();
    RTOS_SetState(state);
}

/* ==============================================================
 * CONTROL TASK
 *
 * Sole owner of Navigation_* and motor-motion decisions.
 * ============================================================== */

static void RTOS_ControlTask(void *argument)
{
    TickType_t last_wake;
    const TickType_t period = pdMS_TO_TICKS(RTOS_CONTROL_PERIOD_MS);
    uint32_t calibration_ticks = 0U;
    uint32_t gyro_samples = 0U;
    uint32_t route_ticks = 0U;
    uint16_t destination_wait_ticks = 0U;
    float gyro_sum = 0.0f;

    (void)argument;

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        EventBits_t events;

        vTaskDelayUntil(&last_wake, period);
        debug_rtos_control_ticks++;

        /* ------------------------------------------------------
         * 2-second stationary gyro calibration.
         * ------------------------------------------------------ */
        if (rtos_state == RTOS_AGV_CALIBRATING)
        {
            Motor_Stop();
            calibration_ticks++;
            debug_rtos_calibration_ticks = calibration_ticks;

            if (rtos_mpu_init_ok)
            {
                uint8_t ok = 0U;

                if (xSemaphoreTake(rtos_i2c_mutex, pdMS_TO_TICKS(5U)) == pdTRUE)
                {
                    ok = MPU6050_Read(&rtos_mpu_data);
                    xSemaphoreGive(rtos_i2c_mutex);
                }

                debug_rtos_mpu_read_ok = ok;

                if (ok)
                {
                    gyro_sum += rtos_mpu_data.gyro_z_dps;
                    gyro_samples++;
                    debug_rtos_gyro_samples = gyro_samples;
                }
            }

            if (calibration_ticks >= RTOS_CALIBRATION_TICKS)
            {
                if (gyro_samples >= RTOS_GYRO_MIN_SAMPLES)
                {
                    rtos_gyro_bias_dps = gyro_sum / (float)gyro_samples;
                    rtos_mpu_calibrated = 1U;
                }
                else
                {
                    rtos_gyro_bias_dps = 0.0f;
                    rtos_mpu_calibrated = 0U;
                }

                debug_rtos_gyro_bias_dps = rtos_gyro_bias_dps;
                debug_rtos_mpu_calibrated = rtos_mpu_calibrated;

                Kalman1D_Init(&rtos_gyro_filter, 0.01f, 0.10f, 0.0f, 1.0f);

                if (rtos_route_found)
                {
                    RTOS_SetState(RTOS_AGV_WAIT_START_RFID);
                }
                else
                {
                    RTOS_StopTerminal(RTOS_AGV_ASTAR_FAIL);
                }
            }

            continue;
        }

        /* ------------------------------------------------------
         * Start station RFID gate preserved from working test.
         * ------------------------------------------------------ */
        if (rtos_state == RTOS_AGV_WAIT_START_RFID)
        {
            Motor_Stop();

            /* Keep live line readings available while waiting. */
            debug_rtos_ir_pattern = IRArray_ReadPattern();
            debug_rtos_line_error = IRArray_GetLineError(debug_rtos_ir_pattern);

            events = xEventGroupGetBits(rtos_events);

            if ((events & RTOS_EVENT_START_RFID_OK) != 0U)
            {
                xEventGroupClearBits(rtos_events, RTOS_EVENT_START_RFID_OK);
                Navigation_SetPath(&rtos_path);
                route_ticks = 0U;
                RTOS_UpdateRouteDebug();
                RTOS_CopyNavigationDebug();
                RTOS_SetState(RTOS_AGV_RUNNING);
            }

            continue;
        }

        /* ------------------------------------------------------
         * Route-complete destination RFID confirmation window.
         * ------------------------------------------------------ */
        if (rtos_state == RTOS_AGV_WAIT_DEST_RFID)
        {
            Motor_Stop();
            destination_wait_ticks++;
            debug_rtos_dest_wait_ticks = destination_wait_ticks;

            events = xEventGroupGetBits(rtos_events);

            if ((events & RTOS_EVENT_DEST_RFID_OK) != 0U)
            {
                debug_rtos_dest_rfid_ok = 1U;
                RTOS_StopTerminal(RTOS_AGV_DESTINATION);
                continue;
            }

            /* Preserve the original 3-second post-arrival window. */
            if (destination_wait_ticks >= RTOS_DEST_WAIT_TICKS)
            {
                RTOS_StopTerminal(RTOS_AGV_DESTINATION);
            }

            continue;
        }

        if (rtos_state != RTOS_AGV_RUNNING)
        {
            Motor_Stop();
            continue;
        }

        events = xEventGroupGetBits(rtos_events);

        /* ------------------------------------------------------
         * Destination RFID can confirm while still on station pad,
         * exactly like the original non-blocking polling loop.
         * ------------------------------------------------------ */
        if ((events & RTOS_EVENT_DEST_RFID_OK) != 0U)
        {
            debug_rtos_dest_rfid_ok = 1U;
            RTOS_StopTerminal(RTOS_AGV_DESTINATION);
            continue;
        }

        /* ------------------------------------------------------
         * Dynamic obstacle avoidance/replanning.
         * ObstacleTask only publishes the debounced fact; this task
         * owns Motor_Stop, A*, route replacement and U-turn state.
         * ------------------------------------------------------ */
        if (((events & RTOS_EVENT_OBSTACLE) != 0U) &&
            (route_ticks >= RTOS_OBSTACLE_IGNORE_TICKS) &&
            (debug_nav_mode == NAV_MODE_FOLLOW_LINE))
        {
            uint8_t p_idx = Navigation_GetPathIndex();

            if ((p_idx >= 1U) && (p_idx < rtos_path.length))
            {
                MapNodeId from_node = rtos_path.nodes[p_idx - 1U];
                MapNodeId target_node = rtos_path.nodes[p_idx];
                MapNodeId goal_node = rtos_path.nodes[rtos_path.length - 1U];
                uint8_t bend_pending_return = 0U;
                AStarPath detour_path;
                uint8_t replan_ok;

                Motor_Stop();

                if ((debug_nav_edge_bend_direction != 0) &&
                    (debug_nav_edge_bend_complete == 1U))
                {
                    bend_pending_return = 1U;
                }

                replan_ok = ObstacleManager_ReplanWithHeading(
                    from_node,
                    target_node,
                    goal_node,
                    &detour_path);

                if (replan_ok)
                {
                    debug_rtos_replan_count++;
                    rtos_path = detour_path;
                    RTOS_CopyPathDebug();

                    Navigation_StartMidEdgeUTurn(
                        &rtos_path,
                        bend_pending_return);

                    RTOS_UpdateRouteDebug();
                    RTOS_CopyNavigationDebug();
                    continue;
                }

                RTOS_StopTerminal(RTOS_AGV_BLOCKED_NO_PATH);
                continue;
            }
        }

        /* ------------------------------------------------------
         * Deterministic READ -> THINK -> ACT chain at 100 Hz.
         * ------------------------------------------------------ */
        debug_rtos_ir_pattern = IRArray_ReadPattern();
        debug_rtos_line_error = IRArray_GetLineError(debug_rtos_ir_pattern);

        {
            float gyro_z = 0.0f;
            uint8_t gyro_valid = RTOS_ReadMPU(&gyro_z);

            Navigation_UpdateWithSensors(
                debug_rtos_ir_pattern,
                gyro_z,
                gyro_valid);
        }

        RTOS_CopyNavigationDebug();
        debug_rtos_turn_yaw_deg = Navigation_GetTurnYawDeg();
        RTOS_UpdateRouteDebug();
        route_ticks++;

        if (Navigation_GetPathIndex() >= rtos_path.length)
        {
            RTOS_StopTerminal(RTOS_AGV_INDEX_ERROR);
            continue;
        }

        if (Navigation_IsRouteComplete())
        {
            Motor_Stop();
            destination_wait_ticks = 0U;
            debug_rtos_dest_wait_ticks = 0U;
            RTOS_SetState(RTOS_AGV_WAIT_DEST_RFID);
            continue;
        }

        if (route_ticks >= RTOS_ROUTE_TIMEOUT_TICKS)
        {
            RTOS_StopTerminal(RTOS_AGV_TIMEOUT);
            continue;
        }

        debug_rtos_control_stack_free =
            (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
}

/* ==============================================================
 * OBSTACLE TASK
 *
 * Owns only the obstacle sensor/debounce state. It never writes a
 * motor command and never touches A-star or Navigation state.
 * ============================================================== */

static void RTOS_ObstacleTask(void *argument)
{
    TickType_t last_wake;
    const TickType_t period = pdMS_TO_TICKS(RTOS_OBSTACLE_PERIOD_MS);

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, period);

        Obstacle_Update();
        debug_rtos_obstacle_raw = Obstacle_ReadRaw();
        debug_rtos_obstacle_detected = Obstacle_IsDetected();

        if (debug_rtos_obstacle_detected)
        {
            xEventGroupSetBits(rtos_events, RTOS_EVENT_OBSTACLE);
        }
        else
        {
            xEventGroupClearBits(rtos_events, RTOS_EVENT_OBSTACLE);
        }

        debug_rtos_obstacle_stack_free =
            (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
}

/* ==============================================================
 * RFID TASK
 *
 * Polls at 10 Hz. It publishes start/destination facts only; the
 * ControlTask remains the sole owner of motion and route state.
 * ============================================================== */

static void RTOS_RfidTask(void *argument)
{
    RFID_UID uid;
    const uint8_t expected_start_station = (uint8_t)RTOS_START_NODE + 1U;
    const uint8_t expected_dest_station = (uint8_t)RTOS_DEST_NODE + 1U;

    (void)argument;

    for (;;)
    {
        uint8_t station = 0U;

        if ((rtos_state == RTOS_AGV_WAIT_START_RFID) ||
            (rtos_state == RTOS_AGV_RUNNING) ||
            (rtos_state == RTOS_AGV_WAIT_DEST_RFID))
        {
            if (RFID_ReadUID(&uid))
            {
                debug_rtos_rfid_detected = 1U;
                debug_rtos_rfid_uid0 = uid.uid[0];
                debug_rtos_rfid_uid1 = uid.uid[1];
                debug_rtos_rfid_uid2 = uid.uid[2];
                debug_rtos_rfid_uid3 = uid.uid[3];

                station = RFIDStation_GetStation(&uid);
                debug_rtos_rfid_station = station;

                if ((rtos_state == RTOS_AGV_WAIT_START_RFID) &&
                    (station == expected_start_station))
                {
                    debug_rtos_start_rfid_ok = 1U;
                    xEventGroupSetBits(rtos_events, RTOS_EVENT_START_RFID_OK);
                }

                if (((rtos_state == RTOS_AGV_RUNNING) ||
                     (rtos_state == RTOS_AGV_WAIT_DEST_RFID)) &&
                    (station == expected_dest_station))
                {
                    debug_rtos_destination_station = station;
                    debug_rtos_dest_rfid_ok = 1U;
                    xEventGroupSetBits(rtos_events, RTOS_EVENT_DEST_RFID_OK);
                }
            }
            else
            {
                debug_rtos_rfid_detected = 0U;
                debug_rtos_rfid_station = 0U;
            }
        }

        debug_rtos_rfid_stack_free =
            (uint32_t)uxTaskGetStackHighWaterMark(NULL);

        vTaskDelay(pdMS_TO_TICKS(RTOS_RFID_PERIOD_MS));
    }
}

/* ==============================================================
 * DISPLAY TASK
 *
 * OLED and MPU share I2C1. Every OLED transaction is wrapped by
 * the same mutex used by ControlTask for MPU transactions.
 * ============================================================== */

static void RTOS_DisplayState(RTOSAgvState state)
{
    OLED_Clear();
    OLED_SetCursor(0U, 0U);

    switch (state)
    {
        case RTOS_AGV_CALIBRATING:
            OLED_WriteString("CALIBRATING");
            break;

        case RTOS_AGV_WAIT_START_RFID:
            OLED_WriteString("WAIT START RFID");
            break;

        case RTOS_AGV_RUNNING:
            OLED_WriteString("RUNNING ROUTE");
            break;

        case RTOS_AGV_WAIT_DEST_RFID:
            OLED_WriteString("WAIT DEST RFID");
            break;

        case RTOS_AGV_DESTINATION:
            OLED_WriteString("ARRIVED S4");
            break;

        case RTOS_AGV_ASTAR_FAIL:
            OLED_WriteString("ASTAR FAIL");
            break;

        case RTOS_AGV_INDEX_ERROR:
            OLED_WriteString("INDEX ERROR");
            break;

        case RTOS_AGV_TIMEOUT:
            OLED_WriteString("TIMEOUT");
            break;

        case RTOS_AGV_BLOCKED_NO_PATH:
            OLED_WriteString("NO DETOUR");
            break;

        default:
            OLED_WriteString("RTOS STATE");
            break;
    }
}

static void RTOS_DisplayTask(void *argument)
{
    RTOSAgvState shown_state = (RTOSAgvState)0xFFU;

    (void)argument;

    if (xSemaphoreTake(rtos_i2c_mutex, pdMS_TO_TICKS(100U)) == pdTRUE)
    {
        debug_rtos_oled_ok = OLED_Init();
        xSemaphoreGive(rtos_i2c_mutex);
    }

    for (;;)
    {
        if (debug_rtos_oled_ok && (shown_state != rtos_state))
        {
            if (xSemaphoreTake(rtos_i2c_mutex, pdMS_TO_TICKS(100U)) == pdTRUE)
            {
                RTOS_DisplayState(rtos_state);
                shown_state = rtos_state;
                xSemaphoreGive(rtos_i2c_mutex);
            }
        }

        debug_rtos_display_stack_free =
            (uint32_t)uxTaskGetStackHighWaterMark(NULL);

        vTaskDelay(pdMS_TO_TICKS(RTOS_DISPLAY_PERIOD_MS));
    }
}

/* ==============================================================
 * PUBLIC INITIALIZATION
 * ============================================================== */

void RTOS_Tasks_Init(void)
{
    uint8_t who_am_i = 0U;

    Motor_Stop();
    Map_Init();
    Obstacle_Init();

    rtos_events = xEventGroupCreate();
    rtos_i2c_mutex = xSemaphoreCreateMutex();

    configASSERT(rtos_events != NULL);
    configASSERT(rtos_i2c_mutex != NULL);

    Navigation_Init(
        RTOS_NAV_KP,
        RTOS_NAV_KI,
        RTOS_NAV_KD,
        RTOS_NAV_DT_S,
        RTOS_NAV_BASE_SPEED);

    Kalman1D_Init(&rtos_gyro_filter, 0.01f, 0.10f, 0.0f, 1.0f);

    rtos_mpu_init_ok = MPU6050_Init();
    debug_rtos_mpu_init_ok = rtos_mpu_init_ok;
    debug_rtos_mpu_who_ok = MPU6050_ReadWhoAmI(&who_am_i);
    debug_rtos_mpu_who_am_i = who_am_i;

    RFID_Init();
    debug_rtos_rfid_initialized = 1U;

    rtos_route_found = AStar_FindPathWithHeading(
        RTOS_START_NODE,
        RTOS_DEST_NODE,
        RTOS_START_HEADING,
        &rtos_path);

    debug_rtos_route_found = rtos_route_found;
    RTOS_CopyPathDebug();

    debug_rtos_start_rfid_ok = 0U;
    debug_rtos_dest_rfid_ok = 0U;
    debug_rtos_replan_count = 0U;
    debug_rtos_control_ticks = 0U;
    debug_rtos_calibration_ticks = 0U;
    debug_rtos_dest_wait_ticks = 0U;

    RTOS_SetState(
        rtos_route_found ? RTOS_AGV_CALIBRATING : RTOS_AGV_ASTAR_FAIL);
}

void RTOS_Tasks_Create(void)
{
    BaseType_t result;

    result = xTaskCreate(
        RTOS_ControlTask,
        "CONTROL",
        RTOS_CONTROL_STACK_WORDS,
        NULL,
        RTOS_CONTROL_PRIORITY,
        &control_task_handle);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        RTOS_ObstacleTask,
        "OBSTACLE",
        RTOS_OBSTACLE_STACK_WORDS,
        NULL,
        RTOS_OBSTACLE_PRIORITY,
        &obstacle_task_handle);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        RTOS_RfidTask,
        "RFID",
        RTOS_RFID_STACK_WORDS,
        NULL,
        RTOS_RFID_PRIORITY,
        &rfid_task_handle);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        RTOS_DisplayTask,
        "DISPLAY",
        RTOS_DISPLAY_STACK_WORDS,
        NULL,
        RTOS_DISPLAY_PRIORITY,
        &display_task_handle);
    configASSERT(result == pdPASS);
}

RTOSAgvState RTOS_GetAgvState(void)
{
    return rtos_state;
}
