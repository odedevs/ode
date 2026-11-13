/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*
 * Library private random number helpers.
 *
 * dRand() draws from one process-wide sequence and advances its seed with a compare-exchange per value. A caller that
 * needs many values at once, such as QuickStep's constraint shuffle, reserves them as a block of the sequence with one
 * compare-exchange (dxRandReserve()) and then computes them itself (dxRandNext()). This matters with island threads:
 * the islands shuffle at the same time, and drawing a value at a time they would all write the seed's cache line,
 * which would then move between cores at every draw while compare-exchanges fail and retry.
 *
 * Single-threaded, a block holds exactly the values that as many dRand() calls would return, and leaves the seed
 * where those calls would, so the results, dRandGetSeed() and dRandSetSeed() are the same as with one call per value.
 * With threads, each caller gets a contiguous part of the sequence; which part still depends on timing, as it does
 * for single values.
 */

#ifndef _ODE__PRIVATE_RANDOM_H_
#define _ODE__PRIVATE_RANDOM_H_

#include <ode/odeconfig.h>
#include "error.h"
#include "typedefs.h"


/* dRand()'s step: x -> dxRAND_MULTIPLIER * x + dxRAND_INCREMENT (mod 2^32). */
static const duint32 dxRAND_MULTIPLIER = 1664525;
static const duint32 dxRAND_INCREMENT = 1013904223;


/* Advances state by one value of dRand()'s sequence and returns that value. */
static inline
duint32 dxRandNext(duint32 &state)
{
    state = dxRAND_MULTIPLIER * state + dxRAND_INCREMENT;
    return state;
}


/*
 * Gives the map that advances dRand()'s sequence by count values at once. The step applied count times is again an
 * affine map, x -> multiplier * x + increment (mod 2^32); it is composed from the step applied 2^k times for the bits
 * k of count, in O(log count) multiplications. A count of 0 gives the identity.
 */
static inline
void dxRandSkip(duint32 &multiplier, duint32 &increment, unsigned int count)
{
    duint32 skipMultiplier = 1, skipIncrement = 0;
    // The step applied 2^k times, for the current bit k of count.
    duint32 powerMultiplier = dxRAND_MULTIPLIER, powerIncrement = dxRAND_INCREMENT;

    for (; count != 0; count >>= 1) {
        if ((count & 1) != 0) {
            skipMultiplier *= powerMultiplier;
            skipIncrement = skipIncrement * powerMultiplier + powerIncrement;
        }

        powerIncrement *= powerMultiplier + 1;
        powerMultiplier *= powerMultiplier;
    }

    multiplier = skipMultiplier;
    increment = skipIncrement;
}


/*
 * Reserves the next count values of dRand()'s sequence with one atomic operation and returns the seed just before
 * them: dxRandNext() applied to the returned value gives them in order. A count of 0 reserves nothing, does no atomic
 * operation and returns the current seed.
 */
duint32 dxRandReserve(unsigned int count);


/* The stride between the islands of a step before dxRandIslandSeed() hashes: 2^32 divided by the golden ratio, odd. */
static const duint32 dxRAND_ISLAND_STRIDE = 0x9E3779B9;


/*
 * Gives an island's starting state for one step of QuickStep's per-island random reordering (quickstep.cpp):
 * MurmurHash3's 32-bit finalizer (fmix32) of stepSeed + islandIndex * dxRAND_ISLAND_STRIDE. Hashed, so that
 * neighbouring islands do not start from neighbouring states: two states d apart are still a^n * d apart n values
 * later (a = dxRAND_MULTIPLIER), a fixed pattern between the islands' shuffles. The multiplication by an odd constant,
 * the addition and fmix32 are all one-to-one on 32-bit values, so the islands of a step (fewer than 2^32) start from
 * distinct states, and an island starts from another state at another step's seed.
 */
static inline
duint32 dxRandIslandSeed(duint32 stepSeed, sizeint islandIndex)
{
    duint32 state = stepSeed + (duint32)islandIndex * dxRAND_ISLAND_STRIDE;

    // fmix32
    state ^= state >> 16;
    state *= (duint32)0x85EBCA6B;
    state ^= state >> 13;
    state *= (duint32)0xC2B2AE35;
    state ^= state >> 16;
    return state;
}


/*
 * Maps a value of dRand()'s sequence to [0, n): dRandInt() is dRand() followed by this mapping.
 *
 * adam's all-int straightforward(?) dRandInt (0..n-1)
 */
static inline
int dxRandIntFromValue(duint32 value, int n)
{
    int result;
    duint32 r = value;

    duint32 un = n;
    dIASSERT(sizeof(n) == sizeof(un));

    // note: probably more aggressive than it needs to be -- might be
    //       able to get away without one or two of the innermost branches.
    // if (un <= 0x00010000UL) {
    //     r ^= (r >> 16);
    //     if (un <= 0x00000100UL) {
    //         r ^= (r >> 8);
    //         if (un <= 0x00000010UL) {
    //             r ^= (r >> 4);
    //             if (un <= 0x00000004UL) {
    //                 r ^= (r >> 2);
    //                 if (un <= 0x00000002UL) {
    //                     r ^= (r >> 1);
    //                 }
    //             }
    //         }
    //     }
    // }
    // Optimized version of above
    if (un <= (duint32)0x00000010) {
        r ^= (r >> 16);
        r ^= (r >> 8);
        r ^= (r >> 4);
        if (un <= (duint32)0x00000002) {
            r ^= (r >> 2);
            r ^= (r >> 1);
            result = (r/* & (duint32)0x01*/) & (un >> 1);
        } else {
            if (un <= (duint32)0x00000004) {
                r ^= (r >> 2);
                result = ((r & (duint32)0x03) * un) >> 2;
            } else {
                result = ((r & (duint32)0x0F) * un) >> 4;
            }
        }
    } else {
        if (un <= (duint32)0x00000100) {
            r ^= (r >> 16);
            r ^= (r >> 8);
            result = ((r & (duint32)0xFF) * un) >> 8;
        } else {
            if (un <= (duint32)0x00010000) {
                r ^= (r >> 16);
                result = ((r & (duint32)0xFFFF) * un) >> 16;
            } else {
                result = (int)(((duint64)r * un) >> 32);
            }
        }
    }

    return result;
}


/*
 * Shuffles items[0 .. count) with the swaps of
 *     for (index = 1; index < count; ++index) swap(items[index], items[dRandInt(index + 1)]);
 * but draws the values from state instead of from dRand(): the first draw is the value that follows state. Leaves
 * state at the last value drawn, count - 1 values further (unchanged for a count of 0 or 1), where the next shuffle
 * continues the sequence.
 */
template<class TItem>
void dxRandShuffleAdvancingState(TItem *items, unsigned int count, duint32 &state)
{
    for (unsigned int index = 1; index < count; ++index) {
        int swapIndex = dxRandIntFromValue(dxRandNext(state), (int)(index + 1));
        TItem tmp = items[index];
        items[index] = items[swapIndex];
        items[swapIndex] = tmp;
    }
}


/* The same shuffle, drawing from a copy of state. */
template<class TItem>
void dxRandShuffleFromState(TItem *items, unsigned int count, duint32 state)
{
    dxRandShuffleAdvancingState(items, count, state);
}


/*
 * The same shuffle drawing from dRand()'s sequence, its count - 1 values reserved with one atomic operation.
 * Single-threaded, it gives the same permutation, and leaves the same seed, as the loop with dRandInt().
 */
template<class TItem>
void dxRandShuffle(TItem *items, unsigned int count)
{
    if (count > 1) {
        dxRandShuffleFromState(items, count, dxRandReserve(count - 1));
    }
}


#endif // _ODE__PRIVATE_RANDOM_H_
