#include "test_line_map_no_mpu.h"

#include <stdint.h>

#include "map.h"
#include "astar.h"
#include "navigation.h"

#include "ir_array.h"
#include "motor.h"
#include "mpu6050.h"

#include "rfid.h"
#include "rfid_station.h"

#include "obstacle.h"
#include "obstacle_manager.h"

#include "oled.h"
#include "kalman.h"
#include "timer_cfg.h"


/* ==============================================================
 * ROUTE CONFIGURATION
 * ============================================================== */

#define LINEMAP_START_NODE                 MAP_S1
#define LINEMAP_DEST_NODE                  MAP_S4
#define LINEMAP_START_HEADING              MAP_DIR_SOUTH


/* ==============================================================
 * NAVIGATION PID & SPEED SETTINGS
 * ============================================================== */

#define LINEMAP_KP                         5.0f
#define LINEMAP_KI                         0.0f
#define LINEMAP_KD                         0.0f

#define LINEMAP_DT_S                       0.010f
#define LINEMAP_BASE_SPEED                 50U


/* ==============================================================
 * TIMING & THRESHOLDS
 * ============================================================== */

#define LINEMAP_START_DELAY_TICKS          200U
#define LINEMAP_TIMEOUT_TICKS              9000U
#define LINEMAP_GYRO_MIN_CAL_SAMPLES       50U
#define LINEMAP_RFID_POLL_TICKS            10U


/* ==============================================================
 * TEST STATES
 * ============================================================== */

#define LINEMAP_STATE_WAITING_START_RFID   0U
#define LINEMAP_STATE_STARTING             0U
#define LINEMAP_STATE_RUNNING              1U
#define LINEMAP_STATE_WAIT_DEST_RFID       2U
#define LINEMAP_STATE_DESTINATION          3U
#define LINEMAP_STATE_ASTAR_FAIL           4U
#define LINEMAP_STATE_TIMEOUT              5U
#define LINEMAP_STATE_INDEX_ERROR          6U
#define LINEMAP_STATE_BLOCKED_NO_PATH      7U


/* ==============================================================
 * NAVIGATION DEBUG EXTERNS (Defined in navigation.c)
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
 * GENERAL DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_state = LINEMAP_STATE_WAITING_START_RFID;
volatile uint8_t debug_linemap_done = 0U;
volatile uint8_t debug_linemap_astar_ok = 0U;
volatile uint8_t debug_linemap_expected_path_ok = 0U;


/* ==============================================================
 * A* DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_path_length = 0U;
volatile uint8_t debug_linemap_path[MAP_NODE_COUNT];


/* ==============================================================
 * ROUTE DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_path_index = 0U;
volatile uint8_t debug_linemap_current_node = (uint8_t)MAP_NODE_INVALID;
volatile uint8_t debug_linemap_next_node = (uint8_t)MAP_NODE_INVALID;


/* ==============================================================
 * LINE / NAVIGATION DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_ir_pattern = 0U;
volatile int8_t debug_linemap_line_error = 0;
volatile uint8_t debug_linemap_junction_candidate = 0U;
volatile uint8_t debug_linemap_nav_mode = 0U;
volatile uint8_t debug_linemap_nav_junction_armed = 0U;
volatile uint8_t debug_linemap_nav_junction_accepted = 0U;
volatile uint8_t debug_linemap_nav_action = (uint8_t)NAV_ACTION_STOP;
volatile int8_t debug_linemap_nav_edge_bend_direction = 0;
volatile uint8_t debug_linemap_nav_edge_bend_pending = 0U;
volatile uint8_t debug_linemap_nav_edge_bend_active = 0U;
volatile uint16_t debug_linemap_turn_ticks = 0U;
volatile uint8_t debug_linemap_turn_center_lost = 0U;
volatile uint8_t debug_linemap_turn_branch_seen = 0U;
volatile uint8_t debug_linemap_turn_center_count = 0U;


/* ==============================================================
 * RFID DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_rfid_initialized = 0U;
volatile uint8_t debug_linemap_rfid_detected = 0U;
volatile uint8_t debug_linemap_rfid_station = 0U;
volatile uint8_t debug_linemap_rfid_detected_station = 0U;
volatile uint8_t debug_linemap_expected_start_station = (uint8_t)LINEMAP_START_NODE + 1U;
volatile uint8_t debug_linemap_rfid_uid[4] = {0U, 0U, 0U, 0U};
volatile uint8_t debug_linemap_rfid_uid0 = 0U;
volatile uint8_t debug_linemap_rfid_uid1 = 0U;
volatile uint8_t debug_linemap_rfid_uid2 = 0U;
volatile uint8_t debug_linemap_rfid_uid3 = 0U;
volatile uint8_t debug_linemap_start_rfid_ok = 0U;
volatile uint8_t debug_linemap_dest_rfid_ok = 0U;
volatile uint8_t debug_linemap_destination_station = 0U;
volatile uint8_t debug_linemap_destination_confirm = 0U;


/* ==============================================================
 * OBSTACLE DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_obstacle_raw = 0U;
volatile uint8_t debug_linemap_obstacle_detected = 0U;
volatile uint8_t debug_linemap_replan_count = 0U;


/* ==============================================================
 * MPU DEBUG
 * ============================================================== */

volatile uint8_t debug_linemap_mpu_init_ok = 0U;
volatile uint8_t debug_linemap_mpu_who_ok = 0U;
volatile uint8_t debug_linemap_mpu_who_am_i = 0U;
volatile uint8_t debug_linemap_mpu_read_ok = 0U;
volatile uint8_t debug_linemap_mpu_calibrated = 0U;
volatile uint32_t debug_linemap_gyro_cal_samples = 0U;
volatile float debug_linemap_gyro_z_raw_dps = 0.0f;
volatile float debug_linemap_gyro_z_bias_dps = 0.0f;
volatile float debug_linemap_gyro_z_corrected_dps = 0.0f;
volatile float debug_linemap_gyro_z_filtered_dps = 0.0f;
volatile float debug_linemap_turn_yaw_deg = 0.0f;


/* ==============================================================
 * ENCODER DEBUG (RETAINED ZEROED FOR LIVE EXPRESSIONS COMPATIBILITY)
 * ============================================================== */

volatile uint16_t debug_linemap_left_raw = 0U;
volatile uint16_t debug_linemap_right_raw = 0U;
volatile int16_t debug_linemap_left_delta = 0;
volatile int16_t debug_linemap_right_delta = 0;
volatile float debug_linemap_left_filtered = 0.0f;
volatile float debug_linemap_right_filtered = 0.0f;


/* ==============================================================
 * TIME & PERIPHERAL DEBUG
 * ============================================================== */

volatile uint32_t debug_linemap_ticks = 0U;
volatile uint32_t debug_linemap_elapsed_ms = 0U;
volatile uint32_t debug_linemap_start_delay_ms = 0U;
volatile uint8_t debug_linemap_oled_ok = 0U;


/* ==============================================================
 * PRIVATE DATA
 * ============================================================== */

static AStarPath linemap_path;
static Kalman1D linemap_gyro_filter;
static MPU6050Data linemap_mpu_data;


/* ==============================================================
 * READ RFID STATION
 * ============================================================== */

static uint8_t LineMap_ReadRFIDStation(void)
{
    RFID_UID uid;

    if (!RFID_ReadUID(&uid))
    {
        debug_linemap_rfid_detected = 0U;
        debug_linemap_rfid_detected_station = 0U;
        debug_linemap_rfid_station = 0U;
        return 0U;
    }

    debug_linemap_rfid_detected = 1U;

    debug_linemap_rfid_uid[0] = uid.uid[0];
    debug_linemap_rfid_uid[1] = uid.uid[1];
    debug_linemap_rfid_uid[2] = uid.uid[2];
    debug_linemap_rfid_uid[3] = uid.uid[3];

    debug_linemap_rfid_uid0 = uid.uid[0];
    debug_linemap_rfid_uid1 = uid.uid[1];
    debug_linemap_rfid_uid2 = uid.uid[2];
    debug_linemap_rfid_uid3 = uid.uid[3];

    debug_linemap_rfid_detected_station = RFIDStation_GetStation(&uid);
    debug_linemap_rfid_station = debug_linemap_rfid_detected_station;

    return debug_linemap_rfid_detected_station;
}


/* ==============================================================
 * COPY A* PATH TO DEBUG ARRAY
 * ============================================================== */

static void LineMap_CopyPath(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)MAP_NODE_COUNT; i++)
    {
        debug_linemap_path[i] = (uint8_t)MAP_NODE_INVALID;
    }

    debug_linemap_path_length = linemap_path.length;

    for (i = 0U; i < linemap_path.length; i++)
    {
        debug_linemap_path[i] = (uint8_t)linemap_path.nodes[i];
    }
}


/* ==============================================================
 * VERIFY ROUTE INTEGRITY
 * ============================================================== */

static void LineMap_CheckExpectedPath(void)
{
    if ((linemap_path.length >= 2U) &&
        (linemap_path.nodes[0] == LINEMAP_START_NODE) &&
        (linemap_path.nodes[linemap_path.length - 1U] == LINEMAP_DEST_NODE))
    {
        debug_linemap_expected_path_ok = 1U;
    }
    else
    {
        debug_linemap_expected_path_ok = 0U;
    }
}


/* ==============================================================
 * UPDATE MPU SENSOR
 * ============================================================== */

static uint8_t LineMap_UpdateMPU(void)
{
    if (!debug_linemap_mpu_init_ok)
    {
        debug_linemap_mpu_read_ok = 0U;
        return 0U;
    }

    debug_linemap_mpu_read_ok = MPU6050_Read(&linemap_mpu_data);

    if (!debug_linemap_mpu_read_ok)
    {
        return 0U;
    }

    debug_linemap_gyro_z_raw_dps = linemap_mpu_data.gyro_z_dps;
    debug_linemap_gyro_z_corrected_dps =
        debug_linemap_gyro_z_raw_dps - debug_linemap_gyro_z_bias_dps;

    debug_linemap_gyro_z_filtered_dps =
        Kalman1D_Update(&linemap_gyro_filter, debug_linemap_gyro_z_corrected_dps);

    return 1U;
}


/* ==============================================================
 * UPDATE ROUTE CONTEXT
 * ============================================================== */

static void LineMap_UpdateRouteContext(void)
{
    uint8_t index = Navigation_GetPathIndex();

    debug_linemap_path_index = index;
    debug_linemap_current_node = (uint8_t)MAP_NODE_INVALID;
    debug_linemap_next_node = (uint8_t)MAP_NODE_INVALID;

    if (index < linemap_path.length)
    {
        debug_linemap_current_node = (uint8_t)linemap_path.nodes[index];
    }

    if ((index + 1U) < linemap_path.length)
    {
        debug_linemap_next_node = (uint8_t)linemap_path.nodes[index + 1U];
    }
}


/* ==============================================================
 * COPY NAVIGATION DEBUG
 * ============================================================== */

static void LineMap_UpdateNavigationDebug(void)
{
    debug_linemap_ir_pattern = debug_nav_ir_pattern;
    debug_linemap_line_error = debug_nav_line_error;
    debug_linemap_junction_candidate = debug_nav_junction_candidate;
    debug_linemap_nav_mode = debug_nav_mode;
    debug_linemap_nav_junction_armed = debug_nav_junction_armed;
    debug_linemap_nav_junction_accepted = debug_nav_junction_accepted;
    debug_linemap_nav_action = debug_nav_action;
    debug_linemap_nav_edge_bend_direction = debug_nav_edge_bend_direction;
    debug_linemap_nav_edge_bend_pending = debug_nav_edge_bend_pending;
    debug_linemap_nav_edge_bend_active = debug_nav_edge_bend_active;
    debug_linemap_turn_ticks = debug_nav_turn_ticks;
    debug_linemap_turn_center_lost = debug_nav_turn_center_lost;
    debug_linemap_turn_branch_seen = debug_nav_turn_branch_seen;
    debug_linemap_turn_center_count = debug_nav_turn_center_count;
}


/* ==============================================================
 * DISPLAY ARRIVAL ON OLED (IF CONNECTED)
 * ============================================================== */

static void LineMap_DisplayArrival(MapNodeId dest)
{
    if (!debug_linemap_oled_ok)
    {
        return;
    }

    OLED_Clear();
    OLED_SetCursor(2U, 10U);

    switch (dest)
    {
        case MAP_S1: OLED_WriteString("ARRIVED S1"); break;
        case MAP_S2: OLED_WriteString("ARRIVED S2"); break;
        case MAP_S3: OLED_WriteString("ARRIVED S3"); break;
        case MAP_S4: OLED_WriteString("ARRIVED S4"); break;
        case MAP_S5: OLED_WriteString("ARRIVED S5"); break;
        case MAP_S6: OLED_WriteString("ARRIVED S6"); break;
        default:     OLED_WriteString("ARRIVED DEST"); break;
    }
}


/* ==============================================================
 * TERMINAL SAFE STOP
 * ============================================================== */

static void LineMap_Stop(uint8_t state)
{
    Navigation_Stop();
    Motor_Stop();

    debug_linemap_state = state;
    debug_linemap_done = 1U;
}


/* ==============================================================
 * MAIN TEST
 * ============================================================== */

void Test_LineMapNoMPU(void)
{
    uint8_t rfid_poll_counter = 0U;

    Motor_Stop();
    Map_Init();
    Obstacle_Init();

    RFID_Init();
    debug_linemap_rfid_initialized = 1U;

    debug_linemap_oled_ok = OLED_Init();

    Navigation_Init(
        LINEMAP_KP,
        LINEMAP_KI,
        LINEMAP_KD,
        LINEMAP_DT_S,
        LINEMAP_BASE_SPEED);

    Kalman1D_Init(&linemap_gyro_filter, 0.01f, 0.10f, 0.0f, 1.0f);

    debug_linemap_mpu_init_ok = MPU6050_Init();
    {
        uint8_t who_am_i = 0U;
        debug_linemap_mpu_who_ok = MPU6050_ReadWhoAmI(&who_am_i);
        debug_linemap_mpu_who_am_i = who_am_i;
    }

    debug_linemap_astar_ok =
        AStar_FindPathWithHeading(
            LINEMAP_START_NODE,
            LINEMAP_DEST_NODE,
            LINEMAP_START_HEADING,
            &linemap_path);

    if (!debug_linemap_astar_ok)
    {
        LineMap_Stop(LINEMAP_STATE_ASTAR_FAIL);
        while (1)
        {
            Motor_Stop();
        }
    }

    LineMap_CopyPath();
    LineMap_CheckExpectedPath();

    debug_linemap_state = LINEMAP_STATE_WAITING_START_RFID;
    debug_linemap_done = 0U;
    debug_linemap_path_index = 0U;
    debug_linemap_ir_pattern = 0U;
    debug_linemap_ticks = 0U;
    debug_linemap_elapsed_ms = 0U;
    debug_linemap_start_delay_ms = 0U;
    debug_linemap_mpu_calibrated = 0U;
    debug_linemap_gyro_cal_samples = 0U;
    debug_linemap_gyro_z_raw_dps = 0.0f;
    debug_linemap_gyro_z_bias_dps = 0.0f;
    debug_linemap_gyro_z_corrected_dps = 0.0f;
    debug_linemap_gyro_z_filtered_dps = 0.0f;
    debug_linemap_turn_yaw_deg = 0.0f;

    debug_linemap_rfid_detected = 0U;
    debug_linemap_rfid_station = 0U;
    debug_linemap_rfid_detected_station = 0U;
    debug_linemap_start_rfid_ok = 0U;
    debug_linemap_dest_rfid_ok = 0U;
    debug_linemap_destination_station = 0U;
    debug_linemap_destination_confirm = 0U;

    debug_linemap_obstacle_raw = 0U;
    debug_linemap_obstacle_detected = 0U;
    debug_linemap_replan_count = 0U;

    /* Encoders zeroed out */
    debug_linemap_left_raw = 0U;
    debug_linemap_right_raw = 0U;
    debug_linemap_left_delta = 0;
    debug_linemap_right_delta = 0;
    debug_linemap_left_filtered = 0.0f;
    debug_linemap_right_filtered = 0.0f;

    /* 2-Second Stationary Gyro Calibration */
    {
        uint32_t start_ticks = 0U;
        uint32_t gyro_bias_samples = 0U;
        float gyro_bias_sum = 0.0f;

        while (start_ticks < LINEMAP_START_DELAY_TICKS)
        {
            Motor_Stop();

            if (!TIMER_ControlPeriodElapsed())
            {
                continue;
            }

            start_ticks++;
            debug_linemap_start_delay_ms = start_ticks * 10U;

            if (debug_linemap_mpu_init_ok)
            {
                debug_linemap_mpu_read_ok = MPU6050_Read(&linemap_mpu_data);
                if (debug_linemap_mpu_read_ok)
                {
                    gyro_bias_sum += linemap_mpu_data.gyro_z_dps;
                    gyro_bias_samples++;
                }
            }
        }

        debug_linemap_gyro_cal_samples = gyro_bias_samples;

        if (gyro_bias_samples >= LINEMAP_GYRO_MIN_CAL_SAMPLES)
        {
            debug_linemap_gyro_z_bias_dps =
                gyro_bias_sum / (float)gyro_bias_samples;
            debug_linemap_mpu_calibrated = 1U;
        }
        else
        {
            debug_linemap_gyro_z_bias_dps = 0.0f;
            debug_linemap_mpu_calibrated = 0U;
        }
    }

    Kalman1D_Init(&linemap_gyro_filter, 0.01f, 0.10f, 0.0f, 1.0f);

    /* ==============================================================
     * HOME STATION START GATE
     *
     * Hold motors stopped and poll RFID reader until the expected
     * start station RFID tag (LINEMAP_START_NODE) is positively scanned.
     * ============================================================== */
    debug_linemap_state = LINEMAP_STATE_WAITING_START_RFID;

    if (debug_linemap_oled_ok)
    {
        OLED_Clear();
        OLED_SetCursor(0U, 0U);
        OLED_WriteString("WAIT START RFID");
    }

    while (!debug_linemap_start_rfid_ok)
    {
        Motor_Stop();

        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }

        /* Read 5-channel line IR so debug_linemap_ir_pattern updates live in Expressions */
        debug_linemap_ir_pattern = IRArray_ReadPattern();
        debug_linemap_line_error = IRArray_GetLineError(debug_linemap_ir_pattern);

        uint8_t station = LineMap_ReadRFIDStation();
        if (station == ((uint8_t)LINEMAP_START_NODE + 1U))
        {
            debug_linemap_start_rfid_ok = 1U;
            break;
        }
    }

    /* Start Route Navigation */
    Navigation_SetPath(&linemap_path);

    debug_linemap_state = LINEMAP_STATE_RUNNING;
    debug_linemap_done = 0U;

    LineMap_UpdateRouteContext();
    LineMap_UpdateNavigationDebug();

    if (debug_linemap_oled_ok)
    {
        OLED_Clear();
        OLED_SetCursor(0U, 0U);
        OLED_WriteString("RUNNING ROUTE");
    }

    /* ==============================================================
     * MAIN 10 MS NAVIGATION CONTROL LOOP
     * ============================================================== */
    while (1)
    {
        uint8_t gyro_valid = 0U;

        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }

        debug_linemap_ticks++;
        debug_linemap_elapsed_ms = debug_linemap_ticks * 10U;

        /* 1. Dynamic Obstacle Avoidance */
        Obstacle_Update();
        debug_linemap_obstacle_raw = Obstacle_ReadRaw();
        debug_linemap_obstacle_detected = Obstacle_IsDetected();

        if (debug_linemap_obstacle_detected &&
            (debug_linemap_ticks >= 50U) &&
            (debug_linemap_state == LINEMAP_STATE_RUNNING) &&
            (debug_linemap_nav_mode == NAV_MODE_FOLLOW_LINE))
        {
            uint8_t p_idx = Navigation_GetPathIndex();
            if ((p_idx >= 1U) && (p_idx < linemap_path.length))
            {
                MapNodeId from_node = linemap_path.nodes[p_idx - 1U];
                MapNodeId target_node = linemap_path.nodes[p_idx];
                MapNodeId goal_node = linemap_path.nodes[linemap_path.length - 1U];

                Motor_Stop();

                /* Determine if internal bend was already completed */
                uint8_t bend_pending_return = 0U;
                if (debug_nav_edge_bend_direction != 0)
                {
                    if (debug_nav_edge_bend_complete == 1U)
                    {
                        bend_pending_return = 1U;
                    }
                }

                AStarPath detour_path;
                uint8_t replan_ok = ObstacleManager_ReplanWithHeading(
                    from_node,
                    target_node,
                    goal_node,
                    &detour_path);

                if (replan_ok)
                {
                    debug_linemap_replan_count++;
                    linemap_path = detour_path;
                    LineMap_CopyPath();

                    Navigation_StartMidEdgeUTurn(&linemap_path, bend_pending_return);
                    LineMap_UpdateRouteContext();
                    LineMap_UpdateNavigationDebug();
                    continue;
                }
                else
                {
                    LineMap_Stop(LINEMAP_STATE_BLOCKED_NO_PATH);
                    break;
                }
            }
        }

        /* 2. Read 5-Channel Line IR Array */
        debug_linemap_ir_pattern = IRArray_ReadPattern();

        /* 3. MPU6050 Gyro Update */
        if (debug_linemap_mpu_calibrated)
        {
            gyro_valid = LineMap_UpdateMPU();
        }

        /* 4. Navigation Control Update */
        Navigation_UpdateWithSensors(
            debug_linemap_ir_pattern,
            debug_linemap_gyro_z_filtered_dps,
            gyro_valid);

        LineMap_UpdateNavigationDebug();
        debug_linemap_turn_yaw_deg = Navigation_GetTurnYawDeg();
        LineMap_UpdateRouteContext();

        /* 5. Path Index Safety */
        if (Navigation_GetPathIndex() >= linemap_path.length)
        {
            LineMap_Stop(LINEMAP_STATE_INDEX_ERROR);
            break;
        }

        /* 6. Destination Station Non-Blocking Periodic RFID Polling */
        rfid_poll_counter++;
        if (rfid_poll_counter >= LINEMAP_RFID_POLL_TICKS)
        {
            rfid_poll_counter = 0U;

            uint8_t st = LineMap_ReadRFIDStation();
            MapNodeId dest_node = linemap_path.nodes[linemap_path.length - 1U];
            uint8_t expected_dest_st = (uint8_t)dest_node + 1U;

            if (st == expected_dest_st)
            {
                debug_linemap_destination_station = st;
                debug_linemap_dest_rfid_ok = 1U;

                LineMap_Stop(LINEMAP_STATE_DESTINATION);
                LineMap_DisplayArrival(dest_node);
                break;
            }
        }

        /* 7. Destination Reached via Route Complete (Wide Marker Pad) */
        if (Navigation_IsRouteComplete())
        {
            Motor_Stop();
            debug_linemap_state = LINEMAP_STATE_WAIT_DEST_RFID;

            MapNodeId dest_node = linemap_path.nodes[linemap_path.length - 1U];
            uint8_t expected_dest_st = (uint8_t)dest_node + 1U;

            /* Post-arrival confirmation window (up to 300 ticks = 3 seconds) */
            for (uint16_t wait_ticks = 0U; wait_ticks < 300U; wait_ticks++)
            {
                Motor_Stop();

                if (!TIMER_ControlPeriodElapsed())
                {
                    continue;
                }

                uint8_t st = LineMap_ReadRFIDStation();
                if (st == expected_dest_st)
                {
                    debug_linemap_destination_station = st;
                    debug_linemap_dest_rfid_ok = 1U;
                    break;
                }
            }

            LineMap_Stop(LINEMAP_STATE_DESTINATION);
            LineMap_DisplayArrival(dest_node);
            break;
        }

        /* 8. Global Timeout */
        if (debug_linemap_ticks >= LINEMAP_TIMEOUT_TICKS)
        {
            LineMap_Stop(LINEMAP_STATE_TIMEOUT);
            break;
        }
    }

    while (1)
    {
        Motor_Stop();
    }
}
