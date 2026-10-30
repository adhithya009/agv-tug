#include "navigation.h"

#include "map.h"
#include "pid.h"
#include "ir_array.h"
#include "motor.h"


/* ==============================================================
 * SPEED SETTINGS
 *
 * NOTE ON MOTOR DRIVER CLAMPING:
 * In DeviceDrivers/Src/motor.c, MOTOR_MAX_PWM_PERCENT is configured
 * to 100 (previously 80). Any PWM command up to 100% passes directly
 * to the hardware timer CCR registers.
 *
 * PIVOT DRIVE (ONE WHEEL TURNING, OTHER WHEEL ZERO):
 * Left turn:  Left wheel = 0,      Right wheel = +speed
 * Right turn: Left wheel = +speed, Right wheel = 0
 * ============================================================== */

#define NAV_MAX_SPEED                     75

#define NAV_STRONG_CORRECTION_SPEED       75

/*
 * Full available forward torque while leaving
 * the starting station / wide black marker.
 */
#define NAV_START_EXIT_SPEED              75

#define NAV_TURN_SPEED                    95
#define NAV_TURN_SEARCH_SPEED             70

#define NAV_BEND_SPEED                    95

/*
 * Used for actual junction straight-through motion.
 * Slower speed gives maximum sensor recognition time.
 */
#define NAV_JUNCTION_SPEED                55

#define NAV_LOST_FORWARD_SPEED            50
#define NAV_LOST_SEARCH_SPEED             70


/* ==============================================================
 * JUNCTION & TIMING SETTINGS
 * ============================================================== */

/*
 * 3 consecutive 10 ms samples (30 ms) required to accept a junction.
 * Filters out single-tick noise and line-follower jitter.
 */
#define NAV_JUNCTION_CONFIRM_TICKS        3U

/*
 * 20 normal 10 ms samples (200 ms) of clean line following required
 * before another junction can be armed. Prevents premature re-arming.
 */
#define NAV_REARM_TICKS                   20U

/*
 * Minimum 60 ms straight drive through junction to physically clear tape width.
 */
#define NAV_STRAIGHT_MIN_TICKS            6U

/*
 * 3 consecutive 10 ms samples (30 ms) of clean forward line to confirm
 * crossing is complete before incrementing path index.
 */
#define NAV_STRAIGHT_CLEAR_TICKS          3U


/* ==============================================================
 * REAL TURN SETTINGS
 * ============================================================== */

/*
 * 8 ticks = 80 ms minimum commit before checking for the new line.
 */
#define NAV_TURN_COMMIT_TICKS             8U

/*
 * New center line confirmation ticks.
 */
#define NAV_TURN_CENTER_CONFIRM_TICKS     2U

/*
 * Safety/recovery threshold (250 x 10 ms = 2.5 seconds).
 */
#define NAV_TURN_RECOVERY_TICKS           250U


/* ==============================================================
 * INTERNAL BEND SETTINGS
 * ============================================================== */

#define NAV_BEND_CENTER_CONFIRM_TICKS     2U


/* ==============================================================
 * LOST LINE SETTINGS
 * ============================================================== */

#define NAV_LOST_FORWARD_TICKS            2U


/* ==============================================================
 * CURVED FIVE IR SENSOR MASKS
 *
 * bit0 (1)  = LEFT (L)
 * bit1 (2)  = LEFT-CENTER (LC)
 * bit2 (4)  = CENTER (C)
 * bit3 (8)  = RIGHT-CENTER (RC)
 * bit4 (16) = RIGHT (R)
 *
 * Display order: R RC C LC L
 *
 * On the curved front bumper, outer sensors (L, R) sit further back
 * along the chassis arc than center sensors. When approaching a
 * perpendicular branch or cross line, the center-most 3 sensors
 * (LC, C, RC) make contact FIRST:
 *
 *   00110 = 6  (Center + Left-Center on Left T-branch)
 *   01100 = 12 (Center + Right-Center on Right T-branch)
 *   01110 = 14 (Middle 3 sensors on 4-way / cross junction)
 * ============================================================== */

/* 00111 = 7 */
#define NAV_LEFT_BRANCH_MASK \
    (IR_LEFT_BIT | IR_LEFT_CENTER_BIT | IR_CENTER_BIT)

/* 01110 = 14 (Middle 3 sensors on curved array hitting a 4-way/T junction) */
#define NAV_CENTER_WIDE_MASK \
    (IR_LEFT_CENTER_BIT | IR_CENTER_BIT | IR_RIGHT_CENTER_BIT)

/* 11100 = 28 */
#define NAV_RIGHT_BRANCH_MASK \
    (IR_CENTER_BIT | IR_RIGHT_CENTER_BIT | IR_RIGHT_BIT)

/* 00110 = 6 (Center + Left-Center on a single-sided Left T-junction like J4, J3, J6) */
#define NAV_LEFT_T_INNER_MASK \
    (IR_CENTER_BIT | IR_LEFT_CENTER_BIT)

/* 00101 = 5 (Center + Outer Left) */
#define NAV_LEFT_T_SPLIT_MASK \
    (IR_CENTER_BIT | IR_LEFT_BIT)

/* 00011 = 3 (Outer two left sensors on curved array) */
#define NAV_OUTER_LEFT_PAIR_MASK \
    (IR_LEFT_BIT | IR_LEFT_CENTER_BIT)

/* 01100 = 12 (Center + Right-Center on a single-sided Right T-junction) */
#define NAV_RIGHT_T_INNER_MASK \
    (IR_CENTER_BIT | IR_RIGHT_CENTER_BIT)

/* 10100 = 20 (Center + Outer Right) */
#define NAV_RIGHT_T_SPLIT_MASK \
    (IR_CENTER_BIT | IR_RIGHT_BIT)

/* 11000 = 24 (Outer two right sensors on curved array) */
#define NAV_OUTER_RIGHT_PAIR_MASK \
    (IR_RIGHT_CENTER_BIT | IR_RIGHT_BIT)


/* ==============================================================
 * NAVIGATION MODES
 * ============================================================== */

/* NavigationMode enum declared in navigation.h */


/* ==============================================================
 * CORE STATE
 * ============================================================== */

static PIDController line_pid;
static AStarPath active_path;
static uint8_t path_index = 1U;
static uint8_t nav_base_speed = 55U;
static float navigation_dt = 0.010f;
static NavigationMode nav_mode = NAV_MODE_STOPPED;
static uint8_t is_mid_edge_uturn = 0U;


/* ==============================================================
 * LINE STATE
 * ============================================================== */

static int8_t last_line_error = 0;
static int8_t last_nonzero_error = 0;
static uint16_t lost_line_ticks = 0U;


/* ==============================================================
 * JUNCTION STATE
 * ============================================================== */

static uint8_t junction_confirm_count = 0U;
static uint8_t junction_armed = 0U;
static uint8_t normal_line_count = 0U;
static uint16_t straight_ticks = 0U;
static uint8_t straight_clear_count = 0U;


/* ==============================================================
 * REAL TURN STATE
 * ============================================================== */

static uint8_t turn_departed = 0U;
static uint8_t turn_center_lost = 0U;
static uint8_t turn_center_count = 0U;
static uint16_t turn_ticks = 0U;


/* ==============================================================
 * INTERNAL EDGE BEND STATE
 *
 * -1 = LEFT
 *  0 = NONE
 * +1 = RIGHT
 * ============================================================== */

static int8_t edge_bend_direction = 0;
static uint8_t edge_bend_pending = 0U;
static uint8_t edge_bend_active = 0U;
static uint8_t edge_bend_seen_offcentre = 0U;
static uint8_t edge_bend_center_count = 0U;


/* ==============================================================
 * GYRO STATE
 * ============================================================== */

static float current_gyro_z_dps = 0.0f;
static uint8_t current_gyro_valid = 0U;
static float turn_yaw_deg = 0.0f;


/* ==============================================================
 * DEBUG VARIABLES (Exported to test_line_map_no_mpu.c)
 * ============================================================== */

volatile uint8_t debug_nav_ir_pattern = 0U;
volatile int8_t debug_nav_line_error = 0;
volatile uint8_t debug_nav_junction_candidate = 0U;
volatile uint8_t debug_nav_mode = 0U;
volatile uint8_t debug_nav_junction_armed = 0U;
volatile uint8_t debug_nav_junction_accepted = 0U;
volatile uint8_t debug_nav_action = (uint8_t)NAV_ACTION_STOP;
volatile int8_t debug_nav_edge_bend_direction = 0;
volatile uint8_t debug_nav_edge_bend_pending = 0U;
volatile uint8_t debug_nav_edge_bend_active = 0U;
volatile uint8_t debug_nav_edge_bend_complete = 0U;
volatile uint16_t debug_nav_turn_ticks = 0U;
volatile uint8_t debug_nav_turn_center_lost = 0U;
volatile uint8_t debug_nav_turn_branch_seen = 0U;
volatile uint8_t debug_nav_turn_center_count = 0U;


/* ==============================================================
 * ABS FLOAT
 * ============================================================== */

static float Navigation_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}


/* ==============================================================
 * CLAMP NORMAL FORWARD COMMAND
 * ============================================================== */

static int8_t Navigation_ClampForward(float command)
{
    if (command > (float)NAV_MAX_SPEED)
    {
        command = (float)NAV_MAX_SPEED;
    }

    if (command < 0.0f)
    {
        command = 0.0f;
    }

    return (int8_t)command;
}


/* ==============================================================
 * OPPOSITE MAP DIRECTION
 * ============================================================== */

static MapDirection Navigation_OppositeDirection(MapDirection direction)
{
    if (direction == MAP_DIR_INVALID)
    {
        return MAP_DIR_INVALID;
    }

    return (MapDirection)(((uint8_t)direction + 2U) % 4U);
}


/* ==============================================================
 * COUNT ACTIVE IR SENSORS
 * ============================================================== */

static uint8_t Navigation_CountActiveSensors(uint8_t pattern)
{
    uint8_t count = 0U;

    pattern &= IR_PATTERN_MASK;

    while (pattern != 0U)
    {
        count += (uint8_t)(pattern & 1U);
        pattern >>= 1U;
    }

    return count;
}


/* ==============================================================
 * WIDE BLACK (4 OR 5 SENSORS)
 * ============================================================== */

static uint8_t Navigation_IsWideBlack(uint8_t pattern)
{
    if (Navigation_CountActiveSensors(pattern) >= 4U)
    {
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * LEFT BRANCH (00111 = 7)
 * ============================================================== */

static uint8_t Navigation_HasLeftBranch(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if ((pattern & NAV_LEFT_BRANCH_MASK) == NAV_LEFT_BRANCH_MASK)
    {
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * CENTER-WIDE JUNCTION (01110 = 14)
 * ============================================================== */

static uint8_t Navigation_HasCenterWide(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if ((pattern & NAV_CENTER_WIDE_MASK) == NAV_CENTER_WIDE_MASK)
    {
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * RIGHT BRANCH (11100 = 28)
 * ============================================================== */

static uint8_t Navigation_HasRightBranch(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if ((pattern & NAV_RIGHT_BRANCH_MASK) == NAV_RIGHT_BRANCH_MASK)
    {
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * BASE 3+ SENSOR JUNCTION DETECTION
 * ============================================================== */

static uint8_t Navigation_IsJunctionPattern(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if (Navigation_CountActiveSensors(pattern) >= 3U)
    {
        return 1U;
    }

    if (Navigation_IsWideBlack(pattern))
    {
        return 1U;
    }

    if (Navigation_HasCenterWide(pattern))
    {
        return 1U;
    }

    if (Navigation_HasLeftBranch(pattern))
    {
        return 1U;
    }

    if (Navigation_HasRightBranch(pattern))
    {
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * INSPECT MAP TOPOLOGY OF CURRENT TARGET NODE
 *
 * Determines whether active_path.nodes[path_index] has a physical
 * branch to the robot's LEFT, RIGHT, or BOTH relative to its
 * arrival heading.
 *
 * Example on S2 -> J5 -> J4 -> J3 -> J6 -> J9 -> S3:
 * - J5 (arriving WEST): neighbors N (J2) & S (S6) -> has_left=1, has_right=1
 * - J4 (arriving WEST): neighbor S (J7) only      -> has_left=1, has_right=0
 * - J3 (arriving WEST): neighbor S (J6) only      -> has_left=1, has_right=0
 * - J6 (arriving SOUTH): neighbor E (J7) only     -> has_left=1, has_right=0
 * - J9 (arriving SOUTH): neighbors W (J8) & E (J10)-> has_left=1, has_right=1
 * ============================================================== */

static void Navigation_GetCurrentNodeBranchSides(
    uint8_t *has_left,
    uint8_t *has_right)
{
    MapNodeId prev;
    MapNodeId curr;
    MapDirection back_dir;
    MapDirection incoming;
    MapNodeId neighbors[MAP_NODE_COUNT];
    uint8_t count;

    *has_left = 0U;
    *has_right = 0U;

    if ((active_path.length < 2U) ||
        (path_index == 0U) ||
        (path_index >= active_path.length))
    {
        return;
    }

    prev = active_path.nodes[path_index - 1U];
    curr = active_path.nodes[path_index];

    back_dir = Map_GetExitDirection(curr, prev);
    incoming = Navigation_OppositeDirection(back_dir);

    if (incoming == MAP_DIR_INVALID)
    {
        return;
    }

    count = Map_GetNeighbors(curr, neighbors, MAP_NODE_COUNT);

    for (uint8_t i = 0U; i < count; i++)
    {
        MapDirection out_dir;

        if (neighbors[i] == prev)
        {
            continue;
        }

        out_dir = Map_GetExitDirection(curr, neighbors[i]);
        if (out_dir == MAP_DIR_INVALID)
        {
            continue;
        }

        /* Clockwise (+1) = RIGHT branch */
        if ((((uint8_t)incoming + 1U) % 4U) == (uint8_t)out_dir)
        {
            *has_right = 1U;
        }
        /* Counter-clockwise (+3) = LEFT branch */
        else if ((((uint8_t)incoming + 3U) % 4U) == (uint8_t)out_dir)
        {
            *has_left = 1U;
        }
    }
}


/* ==============================================================
 * TOPOLOGY-AWARE JUNCTION PATTERN DETECTION
 *
 * Tailored for the curved front bumper where the center-most 3
 * sensors (LC, C, RC) make contact with side-branch tape first:
 *
 * - 4-way Cross / Wide: 01110 (14) or any 3+ active sensors
 * - Single-sided Left T-branch: 00110 (6 = C + LC), 00101 (5), 00011 (3)
 * - Single-sided Right T-branch: 01100 (12 = C + RC), 10100 (20), 11000 (24)
 *
 * Line drift / correction patterns (LC alone = 00010, RC alone = 01000)
 * are handled by PID steering without triggering false junctions.
 * ============================================================== */

static uint8_t Navigation_IsNodeJunctionPattern(uint8_t pattern)
{
    uint8_t has_left = 0U;
    uint8_t has_right = 0U;

    pattern &= IR_PATTERN_MASK;

    /* Any 3+ sensor pattern is always a junction candidate */
    if (Navigation_IsJunctionPattern(pattern))
    {
        return 1U;
    }

    /*
     * Only match 2-sensor side-branch patterns on the curved bumper (6 or 12)
     * when junction detection is armed and we are NOT waiting for an
     * internal edge bend.
     */
    if (!junction_armed || edge_bend_pending)
    {
        return 0U;
    }

    Navigation_GetCurrentNodeBranchSides(&has_left, &has_right);

    /*
     * Single-sided LEFT T-junction (e.g., J4, J3, J6):
     * On the curved bumper, center sensors hit the perpendicular line first:
     * 00110 (6 = C + LC), 00101 (5 = C + L), or 00011 (3 = LC + L).
     */
    if (has_left && !has_right)
    {
        if ((pattern == NAV_LEFT_T_INNER_MASK) ||
            (pattern == NAV_LEFT_T_SPLIT_MASK) ||
            (pattern == NAV_OUTER_LEFT_PAIR_MASK))
        {
            return 1U;
        }
    }

    /*
     * Single-sided RIGHT T-junction:
     * On the curved bumper, center sensors hit the perpendicular line first:
     * 01100 (12 = C + RC), 10100 (20 = C + R), or 11000 (24 = RC + R).
     */
    if (has_right && !has_left)
    {
        if ((pattern == NAV_RIGHT_T_INNER_MASK) ||
            (pattern == NAV_RIGHT_T_SPLIT_MASK) ||
            (pattern == NAV_OUTER_RIGHT_PAIR_MASK))
        {
            return 1U;
        }
    }

    return 0U;
}


/* ==============================================================
 * CENTERED NORMAL LINE (FOR REAL JUNCTION TURN COMPLETION)
 * ============================================================== */

static uint8_t Navigation_IsCenteredPattern(uint8_t pattern)
{
    int8_t error;

    pattern &= IR_PATTERN_MASK;

    if ((pattern & IR_CENTER_BIT) == 0U)
    {
        return 0U;
    }

    if (Navigation_IsJunctionPattern(pattern))
    {
        return 0U;
    }

    error = IRArray_GetLineError(pattern);

    if ((error >= -1) && (error <= 1))
    {
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * DIFFERENTIAL TURN HELPERS
 *
 * Equal Differential Drive (95 PWM on both wheels):
 * Left turn:  Left wheel backward (-speed), Right wheel forward (+speed)
 * Right turn: Left wheel forward (+speed), Right wheel backward (-speed)
 * ============================================================== */

static void Navigation_DifferentialLeft(int8_t speed)
{
    /* Normal Pivot Left: Left inside wheel stopped (0), Right outside wheel forward (+speed) */
    Motor_Set(0, speed);
}

static void Navigation_DifferentialRight(int8_t speed)
{
    /* Normal Pivot Right: Left outside wheel forward (+speed), Right inside wheel stopped (0) */
    Motor_Set(speed, 0);
}


/* ==============================================================
 * RESET PID
 * ============================================================== */

static void Navigation_ResetPID(void)
{
    PID_Reset(&line_pid);
    PID_SetSetpoint(&line_pid, 0.0f);
}


/* ==============================================================
 * GET INTERNAL BEND OF CURRENT EDGE
 * ============================================================== */

static int8_t Navigation_GetCurrentEdgeBend(void)
{
    MapNodeId from;
    MapNodeId to;
    MapDirection departure;
    MapDirection reverse_departure;
    MapDirection arrival;

    if ((active_path.length < 2U) ||
        (path_index == 0U) ||
        (path_index >= active_path.length))
    {
        return 0;
    }

    from = active_path.nodes[path_index - 1U];
    to = active_path.nodes[path_index];

    departure = Map_GetExitDirection(from, to);
    reverse_departure = Map_GetExitDirection(to, from);
    arrival = Navigation_OppositeDirection(reverse_departure);

    if ((departure == MAP_DIR_INVALID) ||
        (arrival == MAP_DIR_INVALID))
    {
        return 0;
    }

    if (departure == arrival)
    {
        return 0;
    }

    /* Clockwise = RIGHT (+1) */
    if ((((uint8_t)departure + 1U) % 4U) == (uint8_t)arrival)
    {
        return 1;
    }

    /* Counter-clockwise = LEFT (-1) */
    if ((((uint8_t)departure + 3U) % 4U) == (uint8_t)arrival)
    {
        return -1;
    }

    return 0;
}


/* ==============================================================
 * BEGIN NEW EDGE
 * ============================================================== */

static void Navigation_BeginEdge(void)
{
    edge_bend_direction = Navigation_GetCurrentEdgeBend();
    edge_bend_pending = (edge_bend_direction != 0) ? 1U : 0U;
    edge_bend_active = 0U;
    edge_bend_seen_offcentre = 0U;
    edge_bend_center_count = 0U;

    debug_nav_edge_bend_direction = edge_bend_direction;
    debug_nav_edge_bend_pending = edge_bend_pending;
    debug_nav_edge_bend_active = 0U;
    debug_nav_edge_bend_complete = (edge_bend_pending == 0U) ? 1U : 0U;
    debug_nav_junction_accepted = 0U;
}


/* ==============================================================
 * LINE FOUND
 * ============================================================== */

static void Navigation_LineFound(void)
{
    lost_line_ticks = 0U;
}


/* ==============================================================
 * REMEMBER LINE POSITION
 * ============================================================== */

static void Navigation_RememberLine(int8_t error)
{
    last_line_error = error;

    if (error != 0)
    {
        last_nonzero_error = error;
    }

    Navigation_LineFound();
}


/* ==============================================================
 * DISARM JUNCTION
 * ============================================================== */

static void Navigation_DisarmJunction(void)
{
    junction_armed = 0U;
    normal_line_count = 0U;
    junction_confirm_count = 0U;
    debug_nav_junction_armed = 0U;
}


/* ==============================================================
 * REARM JUNCTION
 * ============================================================== */

static void Navigation_UpdateJunctionArming(void)
{
    if (junction_armed)
    {
        debug_nav_junction_armed = 1U;
        return;
    }

    if (normal_line_count < NAV_REARM_TICKS)
    {
        normal_line_count++;
    }

    if (normal_line_count >= NAV_REARM_TICKS)
    {
        junction_armed = 1U;
    }

    debug_nav_junction_armed = junction_armed;
}


/* ==============================================================
 * NORMAL FIVE-SENSOR LINE FOLLOWING
 * ============================================================== */

static void Navigation_DriveNormalLine(uint8_t pattern)
{
    int8_t measurement;
    float correction;
    float left_command;
    float right_command;

    pattern &= IR_PATTERN_MASK;

    if (!IRArray_LineDetected(pattern))
    {
        return;
    }

    measurement = IRArray_GetLineError(pattern);
    debug_nav_line_error = measurement;

    Navigation_RememberLine(measurement);

    /*
     * For extreme off-track errors (outermost sensors only: measurement <= -3 or >= 3),
     * execute differential correction to bring the line back to center.
     * Intermediate errors (-2, -1, 0, 1, 2) are smoothly steered by PID.
     * When drifting to LC alone (00010 = 2) or RC alone (01000 = 8),
     * PID steers smoothly without jerky wheel reversals.
     */
    if (measurement <= -3)
    {
        Navigation_ResetPID();
        Navigation_DifferentialLeft(NAV_STRONG_CORRECTION_SPEED);
        return;
    }

    if (measurement >= 3)
    {
        Navigation_ResetPID();
        Navigation_DifferentialRight(NAV_STRONG_CORRECTION_SPEED);
        return;
    }

    correction = PID_Update(&line_pid, (float)measurement, navigation_dt);

    left_command = (float)nav_base_speed - correction;
    right_command = (float)nav_base_speed + correction;

    Motor_Set(
        Navigation_ClampForward(left_command),
        Navigation_ClampForward(right_command));
}


/* ==============================================================
 * LOST-LINE RECOVERY
 * ============================================================== */

static void Navigation_RecoverLostLine(void)
{
    if (lost_line_ticks < 0xFFFFU)
    {
        lost_line_ticks++;
    }

    Navigation_ResetPID();

    if (lost_line_ticks <= NAV_LOST_FORWARD_TICKS)
    {
        Motor_Set(NAV_LOST_FORWARD_SPEED, NAV_LOST_FORWARD_SPEED);
        return;
    }

    int8_t preferred_direction = last_nonzero_error;
    if (preferred_direction == 0)
    {
        preferred_direction = edge_bend_direction;
    }

    if (preferred_direction < 0)
    {
        Navigation_DifferentialLeft(NAV_LOST_SEARCH_SPEED);
        return;
    }

    if (preferred_direction > 0)
    {
        Navigation_DifferentialRight(NAV_LOST_SEARCH_SPEED);
        return;
    }

    Navigation_DifferentialLeft(NAV_LOST_SEARCH_SPEED);
}


/* ==============================================================
 * EXPECTED INTERNAL BEND PATTERN (CURVED 5-IR ARRAY)
 * ============================================================== */

static uint8_t Navigation_IsExpectedBendPattern(uint8_t pattern)
{
    int8_t error;

    pattern &= IR_PATTERN_MASK;

    if (!IRArray_LineDetected(pattern))
    {
        return 0U;
    }

    if (Navigation_IsJunctionPattern(pattern))
    {
        return 1U;
    }

    error = IRArray_GetLineError(pattern);

    /* Expected RIGHT bend (+1): trigger on 12 (01100), 8 (01000), 24 (11000), etc. */
    if (edge_bend_direction > 0)
    {
        if ((error >= 1) || ((pattern & (IR_RIGHT_CENTER_BIT | IR_RIGHT_BIT)) != 0U))
        {
            return 1U;
        }
    }
    /* Expected LEFT bend (-1): trigger on 6 (00110), 2 (00010), 3 (00011), etc. */
    else if (edge_bend_direction < 0)
    {
        if ((error <= -1) || ((pattern & (IR_LEFT_CENTER_BIT | IR_LEFT_BIT)) != 0U))
        {
            return 1U;
        }
    }

    return 0U;
}


/* ==============================================================
 * START INTERNAL BEND
 * ============================================================== */

static void Navigation_StartInternalBend(void)
{
    edge_bend_active = 1U;
    edge_bend_seen_offcentre = 0U;
    edge_bend_center_count = 0U;
    junction_confirm_count = 0U;

    Navigation_ResetPID();

    debug_nav_edge_bend_active = 1U;
}


/* ==============================================================
 * FINISH INTERNAL BEND
 * ============================================================== */

static void Navigation_FinishInternalBend(void)
{
    edge_bend_pending = 0U;
    edge_bend_active = 0U;
    edge_bend_seen_offcentre = 0U;
    edge_bend_center_count = 0U;

    last_line_error = 0;
    junction_confirm_count = 0U;

    /* Immediately arm junction detection for the upcoming node (e.g., J5) */
    junction_armed = 1U;
    normal_line_count = NAV_REARM_TICKS;
    debug_nav_junction_armed = 1U;

    Navigation_ResetPID();

    debug_nav_edge_bend_pending = 0U;
    debug_nav_edge_bend_active = 0U;
    debug_nav_edge_bend_complete = 1U;
}


/* ==============================================================
 * EXECUTE INTERNAL MAP BEND (CURVED 5-IR ARRAY)
 * ============================================================== */

static void Navigation_ExecuteInternalBend(uint8_t pattern)
{
    int8_t error;

    pattern &= IR_PATTERN_MASK;

    debug_nav_edge_bend_active = 1U;

    /* 1. Line lost or 3+ sensor junction pattern during bend -> differential turn */
    if ((pattern == 0U) || Navigation_IsJunctionPattern(pattern))
    {
        Navigation_LineFound();
        edge_bend_seen_offcentre = 1U;
        edge_bend_center_count = 0U;

        if (edge_bend_direction > 0)
        {
            Navigation_DifferentialRight(NAV_BEND_SPEED);
        }
        else
        {
            Navigation_DifferentialLeft(NAV_BEND_SPEED);
        }

        return;
    }

    error = IRArray_GetLineError(pattern);
    debug_nav_line_error = error;
    Navigation_RememberLine(error);

    /* 2. Actively turn while line is on the bend side */
    if ((edge_bend_direction > 0) && (error >= 1))
    {
        edge_bend_seen_offcentre = 1U;
        edge_bend_center_count = 0U;
        Navigation_ResetPID();
        Navigation_DifferentialRight(NAV_BEND_SPEED);
        return;
    }

    if ((edge_bend_direction < 0) && (error <= -1))
    {
        edge_bend_seen_offcentre = 1U;
        edge_bend_center_count = 0U;
        Navigation_ResetPID();
        Navigation_DifferentialLeft(NAV_BEND_SPEED);
        return;
    }

    /* 3. Count completion when centered */
    Navigation_DriveNormalLine(pattern);

    if (edge_bend_seen_offcentre && Navigation_IsCenteredPattern(pattern))
    {
        if (edge_bend_center_count < NAV_BEND_CENTER_CONFIRM_TICKS)
        {
            edge_bend_center_count++;
        }

        if (edge_bend_center_count >= NAV_BEND_CENTER_CONFIRM_TICKS)
        {
            Navigation_FinishInternalBend();
        }
    }
    else
    {
        edge_bend_center_count = 0U;
    }
}


/* ==============================================================
 * CALCULATE REAL JUNCTION ACTION
 * ============================================================== */

static NavigationAction Navigation_GetJunctionAction(
    MapNodeId previous,
    MapNodeId current,
    MapNodeId next)
{
    MapDirection back_direction = Map_GetExitDirection(current, previous);
    MapDirection incoming = Navigation_OppositeDirection(back_direction);
    MapDirection outgoing = Map_GetExitDirection(current, next);

    if ((incoming == MAP_DIR_INVALID) || (outgoing == MAP_DIR_INVALID))
    {
        return NAV_ACTION_STOP;
    }

    if (incoming == outgoing)
    {
        return NAV_ACTION_STRAIGHT;
    }

    if ((((uint8_t)incoming + 2U) % 4U) == (uint8_t)outgoing)
    {
        return NAV_ACTION_UTURN;
    }

    if ((((uint8_t)incoming + 1U) % 4U) == (uint8_t)outgoing)
    {
        return NAV_ACTION_RIGHT;
    }

    return NAV_ACTION_LEFT;
}


/* ==============================================================
 * UPDATE GYRO YAW
 * ============================================================== */

static void Navigation_UpdateTurnYaw(void)
{
    float rate;

    if (!current_gyro_valid)
    {
        return;
    }

    rate = Navigation_AbsFloat(current_gyro_z_dps);

    if (rate < 2.0f)
    {
        return;
    }

    turn_yaw_deg += rate * navigation_dt;
}


/* ==============================================================
 * RESET REAL TURN STATE
 * ============================================================== */

static void Navigation_ResetTurnState(void)
{
    turn_departed = 0U;
    turn_center_lost = 0U;
    turn_center_count = 0U;
    turn_ticks = 0U;
    turn_yaw_deg = 0.0f;

    debug_nav_turn_ticks = 0U;
    debug_nav_turn_center_lost = 0U;
    debug_nav_turn_branch_seen = 0U;
    debug_nav_turn_center_count = 0U;
}


/* ==============================================================
 * FINISH REAL TURN
 * ============================================================== */

static void Navigation_FinishTurn(void)
{
    path_index++;

    Navigation_ResetPID();
    Navigation_ResetTurnState();
    Navigation_DisarmJunction();

    last_line_error = 0;
    lost_line_ticks = 0U;

    nav_mode = NAV_MODE_FOLLOW_LINE;

    Navigation_BeginEdge();
}




/* ==============================================================
 * FAILED TURN -> LOST-LINE RECOVERY
 * ============================================================== */

static void Navigation_TurnToRecovery(int8_t direction)
{
    if (direction < 0)
    {
        last_nonzero_error = -4;
    }
    else
    {
        last_nonzero_error = 4;
    }

    Navigation_ResetTurnState();

    lost_line_ticks = NAV_LOST_FORWARD_TICKS + 1U;
    nav_mode = NAV_MODE_FOLLOW_LINE;

    Navigation_ResetPID();
}


/* ==============================================================
 * START REAL JUNCTION ACTION
 * ============================================================== */

static void Navigation_StartJunctionAction(void)
{
    MapNodeId previous;
    MapNodeId current;
    MapNodeId next;
    NavigationAction action;

    if (path_index >= active_path.length)
    {
        Motor_Stop();
        nav_mode = NAV_MODE_STOPPED;
        return;
    }

    /* Final destination node */
    if (path_index == (active_path.length - 1U))
    {
        Motor_Stop();
        nav_mode = NAV_MODE_STOPPED;
        debug_nav_action = (uint8_t)NAV_ACTION_STOP;
        return;
    }

    previous = active_path.nodes[path_index - 1U];
    current = active_path.nodes[path_index];
    next = active_path.nodes[path_index + 1U];

    action = Navigation_GetJunctionAction(previous, current, next);
    debug_nav_action = (uint8_t)action;

    Navigation_ResetTurnState();
    Navigation_DisarmJunction();
    Navigation_ResetPID();

    switch (action)
    {
        case NAV_ACTION_LEFT:
            last_nonzero_error = -4;
            nav_mode = NAV_MODE_TURN_LEFT;
            Navigation_DifferentialLeft(NAV_TURN_SPEED);
            break;

        case NAV_ACTION_RIGHT:
            last_nonzero_error = 4;
            nav_mode = NAV_MODE_TURN_RIGHT;
            Navigation_DifferentialRight(NAV_TURN_SPEED);
            break;

        case NAV_ACTION_STRAIGHT:
            nav_mode = NAV_MODE_GO_STRAIGHT;
            straight_ticks = 0U;
            straight_clear_count = 0U;
            Motor_Set(NAV_JUNCTION_SPEED, NAV_JUNCTION_SPEED);
            break;

        case NAV_ACTION_UTURN:
            last_nonzero_error = -4;
            nav_mode = NAV_MODE_UTURN;
            Navigation_DifferentialLeft(NAV_TURN_SPEED);
            break;

        case NAV_ACTION_STOP:
        default:
            Motor_Stop();
            nav_mode = NAV_MODE_STOPPED;
            break;
    }
}


/* ==============================================================
 * INITIALIZATION
 * ============================================================== */

void Navigation_Init(
    float kp,
    float ki,
    float kd,
    float dt,
    uint8_t base_speed)
{
    if (base_speed > NAV_MAX_SPEED)
    {
        base_speed = NAV_MAX_SPEED;
    }

    nav_base_speed = base_speed;
    navigation_dt = (dt > 0.0f) ? dt : 0.010f;

    PID_Init(&line_pid, kp, ki, kd, -20.0f, 20.0f);
    PID_SetSetpoint(&line_pid, 0.0f);

    active_path.length = 0U;
    path_index = 1U;

    last_line_error = 0;
    last_nonzero_error = 0;
    lost_line_ticks = 0U;

    junction_confirm_count = 0U;
    junction_armed = 0U;
    normal_line_count = 0U;
    straight_ticks = 0U;
    straight_clear_count = 0U;

    Navigation_ResetTurnState();

    edge_bend_direction = 0;
    edge_bend_pending = 0U;
    edge_bend_active = 0U;
    edge_bend_seen_offcentre = 0U;
    edge_bend_center_count = 0U;

    current_gyro_z_dps = 0.0f;
    current_gyro_valid = 0U;

    debug_nav_ir_pattern = 0U;
    debug_nav_line_error = 0;
    debug_nav_junction_candidate = 0U;
    debug_nav_mode = (uint8_t)NAV_MODE_STOPPED;
    debug_nav_junction_armed = 0U;
    debug_nav_junction_accepted = 0U;
    debug_nav_action = (uint8_t)NAV_ACTION_STOP;
    debug_nav_edge_bend_direction = 0;
    debug_nav_edge_bend_pending = 0U;
    debug_nav_edge_bend_active = 0U;
    debug_nav_edge_bend_complete = 0U;
    is_mid_edge_uturn = 0U;

    nav_mode = NAV_MODE_STOPPED;
    Motor_Stop();
}


/* ==============================================================
 * SET A* PATH
 * ============================================================== */

void Navigation_SetPath(const AStarPath *path)
{
    if ((path == 0) || (path->length < 2U))
    {
        Navigation_Stop();
        return;
    }

    active_path = *path;
    path_index = 1U;

    last_line_error = 0;
    last_nonzero_error = 0;
    lost_line_ticks = 0U;

    Navigation_ResetTurnState();
    Navigation_DisarmJunction();
    Navigation_ResetPID();
    straight_ticks = 0U;
    straight_clear_count = 0U;

    nav_mode = NAV_MODE_FOLLOW_LINE;
    Navigation_BeginEdge();
}


/* ==============================================================
 * START MID-EDGE U-TURN (DYNAMIC OBSTACLE AVOIDANCE)
 * ============================================================== */

void Navigation_StartMidEdgeUTurn(
    const AStarPath *detour_path,
    uint8_t bend_pending_return)
{
    if ((detour_path == 0) || (detour_path->length < 2U))
    {
        Navigation_Stop();
        return;
    }

    active_path = *detour_path;
    path_index = 1U;

    last_line_error = 0;
    last_nonzero_error = -4;
    lost_line_ticks = 0U;

    Navigation_ResetPID();
    Navigation_ResetTurnState();
    Navigation_DisarmJunction();
    straight_ticks = 0U;
    straight_clear_count = 0U;

    edge_bend_direction = Navigation_GetCurrentEdgeBend();
    edge_bend_pending = (bend_pending_return && (edge_bend_direction != 0)) ? 1U : 0U;
    edge_bend_active = 0U;
    edge_bend_seen_offcentre = 0U;
    edge_bend_center_count = 0U;

    debug_nav_edge_bend_direction = edge_bend_direction;
    debug_nav_edge_bend_pending = edge_bend_pending;
    debug_nav_edge_bend_active = 0U;
    debug_nav_edge_bend_complete = (edge_bend_pending == 0U) ? 1U : 0U;

    is_mid_edge_uturn = 1U;
    nav_mode = NAV_MODE_UTURN;
    debug_nav_mode = (uint8_t)NAV_MODE_UTURN;
    debug_nav_action = (uint8_t)NAV_ACTION_UTURN;

    Navigation_DifferentialLeft(NAV_TURN_SPEED);
}




/* ==============================================================
 * FOLLOW LINE / DETECT JUNCTION
 * ============================================================== */

static void Navigation_FollowLine(uint8_t pattern)
{
    uint8_t junction_candidate;

    pattern &= IR_PATTERN_MASK;

    debug_nav_ir_pattern = pattern;
    debug_nav_line_error = IRArray_GetLineError(pattern);

    /* 1. Lost Line (00000) */
    if (pattern == 0U)
    {
        junction_confirm_count = 0U;
        debug_nav_junction_candidate = 0U;

        if (edge_bend_active)
        {
            Navigation_ExecuteInternalBend(pattern);
            return;
        }

        Navigation_RecoverLostLine();
        return;
    }

    Navigation_LineFound();

    /* 2. Active Internal Bend */
    if (edge_bend_active)
    {
        Navigation_ExecuteInternalBend(pattern);
        return;
    }

    /* 3. Topology-Aware Junction or Bend Candidate Check */
    junction_candidate = Navigation_IsNodeJunctionPattern(pattern);
    debug_nav_junction_candidate = junction_candidate;

    if (junction_candidate)
    {
        /* Leaving starting station marker before junction is armed */
        if (!junction_armed)
        {
            junction_confirm_count = 0U;
            Motor_Set(NAV_START_EXIT_SPEED, NAV_START_EXIT_SPEED);
            return;
        }

        /* Expected internal physical bend along the current edge */
        if (edge_bend_pending)
        {
            Navigation_StartInternalBend();
            Navigation_ExecuteInternalBend(pattern);
            return;
        }

        /* Real A* Junction (4-way cross or single-sided T-junction) */
        if (junction_confirm_count < NAV_JUNCTION_CONFIRM_TICKS)
        {
            junction_confirm_count++;
        }

        if (junction_confirm_count >= NAV_JUNCTION_CONFIRM_TICKS)
        {
            junction_confirm_count = 0U;
            debug_nav_junction_accepted = 1U;
            Navigation_StartJunctionAction();
            return;
        }

        Motor_Set(NAV_JUNCTION_SPEED, NAV_JUNCTION_SPEED);
        return;
    }

    /* 4. Normal Line Tracking */
    junction_confirm_count = 0U;
    debug_nav_junction_accepted = 0U;

    Navigation_UpdateJunctionArming();

    if (edge_bend_pending &&
        junction_armed &&
        Navigation_IsExpectedBendPattern(pattern))
    {
        Navigation_StartInternalBend();
        Navigation_ExecuteInternalBend(pattern);
        return;
    }

    Navigation_DriveNormalLine(pattern);
}


/* ==============================================================
 * EXECUTE LEFT JUNCTION TURN (PURE IR AUTHORITATIVE)
 * ============================================================== */

static void Navigation_ExecuteLeft(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if (turn_ticks < 0xFFFFU)
    {
        turn_ticks++;
    }

    debug_nav_turn_ticks = turn_ticks;

    if (turn_ticks <= NAV_TURN_COMMIT_TICKS)
    {
        Navigation_DifferentialLeft(NAV_TURN_SPEED);
    }
    else
    {
        Navigation_DifferentialLeft(NAV_TURN_SEARCH_SPEED);
    }

    Navigation_UpdateTurnYaw();

    if (!Navigation_IsJunctionPattern(pattern))
    {
        turn_departed = 1U;
    }

    if ((turn_departed && ((pattern & IR_CENTER_BIT) == 0U)) ||
        (((pattern & IR_CENTER_BIT) == 0U) && (turn_ticks >= 2U)))
    {
        turn_center_lost = 1U;
    }

    debug_nav_turn_center_lost = turn_center_lost;

    if ((turn_ticks > NAV_TURN_COMMIT_TICKS) &&
        turn_center_lost &&
        Navigation_IsCenteredPattern(pattern))
    {
        if (turn_center_count < NAV_TURN_CENTER_CONFIRM_TICKS)
        {
            turn_center_count++;
        }
    }
    else
    {
        turn_center_count = 0U;
    }

    debug_nav_turn_center_count = turn_center_count;

    if (turn_center_count >= NAV_TURN_CENTER_CONFIRM_TICKS)
    {
        Navigation_FinishTurn();
        return;
    }

    if (turn_ticks >= NAV_TURN_RECOVERY_TICKS)
    {
        Navigation_TurnToRecovery(-1);
    }
}


/* ==============================================================
 * EXECUTE RIGHT JUNCTION TURN (PURE IR AUTHORITATIVE)
 * ============================================================== */

static void Navigation_ExecuteRight(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if (turn_ticks < 0xFFFFU)
    {
        turn_ticks++;
    }

    debug_nav_turn_ticks = turn_ticks;

    if (turn_ticks <= NAV_TURN_COMMIT_TICKS)
    {
        Navigation_DifferentialRight(NAV_TURN_SPEED);
    }
    else
    {
        Navigation_DifferentialRight(NAV_TURN_SEARCH_SPEED);
    }

    Navigation_UpdateTurnYaw();

    if (!Navigation_IsJunctionPattern(pattern))
    {
        turn_departed = 1U;
    }

    if ((turn_departed && ((pattern & IR_CENTER_BIT) == 0U)) ||
        (((pattern & IR_CENTER_BIT) == 0U) && (turn_ticks >= 2U)))
    {
        turn_center_lost = 1U;
    }

    debug_nav_turn_center_lost = turn_center_lost;

    if ((turn_ticks > NAV_TURN_COMMIT_TICKS) &&
        turn_center_lost &&
        Navigation_IsCenteredPattern(pattern))
    {
        if (turn_center_count < NAV_TURN_CENTER_CONFIRM_TICKS)
        {
            turn_center_count++;
        }
    }
    else
    {
        turn_center_count = 0U;
    }

    debug_nav_turn_center_count = turn_center_count;

    if (turn_center_count >= NAV_TURN_CENTER_CONFIRM_TICKS)
    {
        Navigation_FinishTurn();
        return;
    }

    if (turn_ticks >= NAV_TURN_RECOVERY_TICKS)
    {
        Navigation_TurnToRecovery(1);
    }
}


/* ==============================================================
 * STRAIGHT THROUGH REAL JUNCTION
 *
 * Waits until the robot clears both 3-sensor cross patterns AND
 * single-sided T-junction patterns so it doesn't exit GO_STRAIGHT
 * while still sitting on the branch tape.
 * ============================================================== */

static void Navigation_ExecuteStraight(uint8_t pattern)
{
    uint8_t has_left = 0U;
    uint8_t has_right = 0U;
    uint8_t still_on_t_branch = 0U;

    pattern &= IR_PATTERN_MASK;

    if (straight_ticks < 0xFFFFU)
    {
        straight_ticks++;
    }

    Motor_Set(NAV_JUNCTION_SPEED, NAV_JUNCTION_SPEED);

    Navigation_GetCurrentNodeBranchSides(&has_left, &has_right);

    if (has_left && !has_right)
    {
        if ((pattern == NAV_LEFT_T_INNER_MASK) ||
            (pattern == NAV_LEFT_T_SPLIT_MASK) ||
            (pattern == NAV_OUTER_LEFT_PAIR_MASK) ||
            ((pattern & IR_LEFT_BIT) != 0U))
        {
            still_on_t_branch = 1U;
        }
    }
    else if (has_right && !has_left)
    {
        if ((pattern == NAV_RIGHT_T_INNER_MASK) ||
            (pattern == NAV_RIGHT_T_SPLIT_MASK) ||
            (pattern == NAV_OUTER_RIGHT_PAIR_MASK) ||
            ((pattern & IR_RIGHT_BIT) != 0U))
        {
            still_on_t_branch = 1U;
        }
    }

    /*
     * Must drive straight across the intersection:
     * 1. At least NAV_STRAIGHT_MIN_TICKS (60 ms) to cross the perpendicular tape.
     * 2. Front sensors must no longer see cross or branch patterns.
     * 3. Must see clean line for NAV_STRAIGHT_CLEAR_TICKS (30 ms).
     */
    if ((straight_ticks >= NAV_STRAIGHT_MIN_TICKS) &&
        IRArray_LineDetected(pattern) &&
        !Navigation_IsJunctionPattern(pattern) &&
        !still_on_t_branch)
    {
        if (straight_clear_count < NAV_STRAIGHT_CLEAR_TICKS)
        {
            straight_clear_count++;
        }

        if (straight_clear_count >= NAV_STRAIGHT_CLEAR_TICKS)
        {
            path_index++;

            Navigation_ResetPID();
            Navigation_ResetTurnState();
            Navigation_DisarmJunction();

            straight_ticks = 0U;
            straight_clear_count = 0U;

            last_line_error = 0;
            lost_line_ticks = 0U;

            nav_mode = NAV_MODE_FOLLOW_LINE;
            Navigation_BeginEdge();
        }
    }
    else
    {
        straight_clear_count = 0U;
    }
}


/* ==============================================================
 * U-TURN
 * ============================================================== */

static void Navigation_ExecuteUTurn(uint8_t pattern)
{
    pattern &= IR_PATTERN_MASK;

    if (turn_ticks < 0xFFFFU)
    {
        turn_ticks++;
    }

    debug_nav_turn_ticks = turn_ticks;

    if (turn_ticks <= 25U)
    {
        Navigation_DifferentialLeft(NAV_TURN_SPEED);
    }
    else
    {
        Navigation_DifferentialLeft(NAV_TURN_SEARCH_SPEED);
    }

    Navigation_UpdateTurnYaw();

    if (((pattern & IR_CENTER_BIT) == 0U) && (turn_ticks >= 5U))
    {
        turn_center_lost = 1U;
    }

    debug_nav_turn_center_lost = turn_center_lost;

    if (turn_center_lost &&
        (turn_ticks > 25U) &&
        Navigation_IsCenteredPattern(pattern))
    {
        if (turn_center_count < NAV_TURN_CENTER_CONFIRM_TICKS)
        {
            turn_center_count++;
        }
    }
    else
    {
        turn_center_count = 0U;
    }

    debug_nav_turn_center_count = turn_center_count;

    if (turn_center_count >= NAV_TURN_CENTER_CONFIRM_TICKS)
    {
        if (is_mid_edge_uturn)
        {
            is_mid_edge_uturn = 0U;
            nav_mode = NAV_MODE_FOLLOW_LINE;
            Navigation_ResetPID();
            Navigation_ResetTurnState();
            Navigation_DisarmJunction();
            last_line_error = 0;
            lost_line_ticks = 0U;
            return;
        }

        Navigation_FinishTurn();
        return;
    }

    if (turn_ticks >= 300U)
    {
        if (is_mid_edge_uturn)
        {
            is_mid_edge_uturn = 0U;
        }
        Navigation_TurnToRecovery(-1);
    }
}


/* ==============================================================
 * UPDATE WITH SENSOR VALUES
 * ============================================================== */

void Navigation_UpdateWithSensors(
    uint8_t ir_pattern,
    float gyro_z_dps,
    uint8_t gyro_valid)
{
    ir_pattern &= IR_PATTERN_MASK;

    current_gyro_z_dps = gyro_z_dps;
    current_gyro_valid = gyro_valid;

    debug_nav_ir_pattern = ir_pattern;
    debug_nav_line_error = IRArray_GetLineError(ir_pattern);
    debug_nav_junction_candidate = Navigation_IsNodeJunctionPattern(ir_pattern);
    debug_nav_mode = (uint8_t)nav_mode;
    debug_nav_junction_armed = junction_armed;

    switch (nav_mode)
    {
        case NAV_MODE_FOLLOW_LINE:
            Navigation_FollowLine(ir_pattern);
            break;

        case NAV_MODE_TURN_LEFT:
            Navigation_ExecuteLeft(ir_pattern);
            break;

        case NAV_MODE_TURN_RIGHT:
            Navigation_ExecuteRight(ir_pattern);
            break;

        case NAV_MODE_GO_STRAIGHT:
            Navigation_ExecuteStraight(ir_pattern);
            break;

        case NAV_MODE_UTURN:
            Navigation_ExecuteUTurn(ir_pattern);
            break;

        case NAV_MODE_STOPPED:
        default:
            Motor_Stop();
            break;
    }

    debug_nav_mode = (uint8_t)nav_mode;
    debug_nav_junction_armed = junction_armed;
    debug_nav_edge_bend_direction = edge_bend_direction;
    debug_nav_edge_bend_pending = edge_bend_pending;
    debug_nav_edge_bend_active = edge_bend_active;
}


/* ==============================================================
 * NORMAL UPDATE
 * ============================================================== */

void Navigation_Update(void)
{
    Navigation_UpdateWithSensors(
        IRArray_ReadPattern(),
        0.0f,
        0U);
}


/* ==============================================================
 * STOP
 * ============================================================== */

void Navigation_Stop(void)
{
    Motor_Stop();
    Navigation_ResetPID();
    Navigation_ResetTurnState();

    junction_confirm_count = 0U;
    junction_armed = 0U;
    normal_line_count = 0U;
    straight_ticks = 0U;
    straight_clear_count = 0U;

    edge_bend_direction = 0;
    edge_bend_pending = 0U;
    edge_bend_active = 0U;
    edge_bend_seen_offcentre = 0U;
    edge_bend_center_count = 0U;

    last_line_error = 0;
    last_nonzero_error = 0;
    lost_line_ticks = 0U;

    current_gyro_z_dps = 0.0f;
    current_gyro_valid = 0U;

    nav_mode = NAV_MODE_STOPPED;

    debug_nav_ir_pattern = 0U;
    debug_nav_line_error = 0;
    debug_nav_junction_candidate = 0U;
    debug_nav_mode = (uint8_t)NAV_MODE_STOPPED;
    debug_nav_junction_armed = 0U;
    debug_nav_junction_accepted = 0U;
    debug_nav_action = (uint8_t)NAV_ACTION_STOP;
    debug_nav_edge_bend_direction = 0;
    debug_nav_edge_bend_pending = 0U;
    debug_nav_edge_bend_active = 0U;
    debug_nav_edge_bend_complete = 0U;
    is_mid_edge_uturn = 0U;
}


/* ==============================================================
 * GET PATH INDEX
 * ============================================================== */

uint8_t Navigation_GetPathIndex(void)
{
    return path_index;
}


/* ==============================================================
 * GET TURN YAW
 * ============================================================== */

float Navigation_GetTurnYawDeg(void)
{
    return turn_yaw_deg;
}


/* ==============================================================
 * ROUTE COMPLETE
 * ============================================================== */

uint8_t Navigation_IsRouteComplete(void)
{
    if (active_path.length < 2U)
    {
        return 0U;
    }

    if ((path_index == (active_path.length - 1U)) &&
        (nav_mode == NAV_MODE_STOPPED))
    {
        return 1U;
    }

    return 0U;
}
