#ifndef IR_ARRAY_H
#define IR_ARRAY_H

#include <stdint.h>


/* ==============================================================
 * FIVE-SENSOR LINE ARRAY
 * ==============================================================
 *
 * Physical wiring from LEFT to RIGHT:
 *
 * A0 / PA0 -> LEFT
 * A1 / PA1 -> LEFT-CENTER
 * A2 / PA4 -> CENTER
 * A3 / PB0 -> RIGHT-CENTER
 * A4 / PC1 -> RIGHT
 *
 * A5 / PC0 -> OBSTACLE IR
 *
 *
 * Software pattern:
 *
 * bit 0 -> LEFT
 * bit 1 -> LEFT-CENTER
 * bit 2 -> CENTER
 * bit 3 -> RIGHT-CENTER
 * bit 4 -> RIGHT
 *
 *
 * BLACK = 1
 * WHITE = 0
 * ============================================================== */

#define IR_PATTERN_MASK       0x1FU

#define IR_LEFT_BIT           0x01U
#define IR_LEFT_CENTER_BIT    0x02U
#define IR_CENTER_BIT         0x04U
#define IR_RIGHT_CENTER_BIT   0x08U
#define IR_RIGHT_BIT          0x10U


/*
 * Read all five line sensors.
 *
 * Example:
 *
 * LEFT only:
 *
 * 00001 = 1
 *
 * CENTER only:
 *
 * 00100 = 4
 *
 * RIGHT only:
 *
 * 10000 = 16
 *
 * all black:
 *
 * 11111 = 31
 */
uint8_t IRArray_ReadPattern(void);


/*
 * Convert pattern into weighted line position.
 *
 * Typical values:
 *
 * LEFT         -> -4
 * LEFT-CENTER  -> -2
 * CENTER       ->  0
 * RIGHT-CENTER -> +2
 * RIGHT        -> +4
 *
 * Adjacent combinations automatically give:
 *
 * LEFT + LEFT-CENTER        -> -3
 * LEFT-CENTER + CENTER      -> -1
 * CENTER + RIGHT-CENTER     -> +1
 * RIGHT-CENTER + RIGHT      -> +3
 */
int8_t IRArray_GetLineError(uint8_t pattern);


/*
 * Returns:
 *
 * 1 -> at least one of the five line
 *      sensors detects black
 *
 * 0 -> all five line sensors detect white
 */
uint8_t IRArray_LineDetected(uint8_t pattern);


/*
 * Initial wide-black detector.
 *
 * Returns:
 *
 * 1 -> at least 4 of 5 sensors see black
 * 0 -> otherwise
 *
 * Navigation will later use additional
 * map-aware junction logic.
 */
uint8_t IRArray_IsJunctionCandidate(uint8_t pattern);


/*
 * Read obstacle sensor.
 *
 * A5 -> PC0.
 *
 * This returns the RAW GPIO state.
 */
uint8_t IRArray_ReadObstacleRaw(void);


#endif
