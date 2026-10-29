#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>


/*
 * Initialize encoder software state.
 */
void Encoder_Init(void);


/*
 * Reset hardware/software counters.
 */
void Encoder_Reset(void);


/*
 * Read hardware counters and calculate
 * pulse differences.
 */
void Encoder_Update(void);


/*
 * Total raw encoder counters.
 */
uint16_t Encoder_GetLeftRaw(void);

uint16_t Encoder_GetRightRaw(void);


/*
 * Pulses detected during the latest
 * Encoder_Update() interval.
 *
 * LM393 is single-channel.
 *
 * Pulse quantity is available,
 * physical direction is not.
 */
int16_t Encoder_GetLeftDelta(void);

int16_t Encoder_GetRightDelta(void);


#endif
