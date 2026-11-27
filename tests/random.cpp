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
 * The library private random number helpers (ode/src/random.h) against dRand() and dRandInt().
 *
 * Only public functions and the header's inline helpers are called: dxRandReserve() is internal to the library and
 * cannot be reached from here when ODE is a DLL. It is the compare-exchange loop of dRand() around dxRandSkip(),
 * which is checked here, as is dxRandShuffleFromState(), the body of dxRandShuffle().
 *
 * dxRandIslandSeed() and dxRandShuffleAdvancingState() are QuickStep's per-island random reordering: each island
 * starts its own state from the step's seed and its index, and its shuffles continue from that state.
 */

#include <UnitTest++.h>
#include <ode/ode.h>
#include "../ode/src/random.h"

#include <algorithm>
#include <cstddef>
#include <vector>


namespace {

const duint32 TEST_SEEDS[] = { 0, 1, 0x12345678, 0x9E3779B9, 0xFFFFFFFF };
const unsigned int TEST_SEED_COUNT = sizeof(TEST_SEEDS) / sizeof(TEST_SEEDS[0]);

// Puts dRand()'s seed back as it was when the test ends.
class SeedRestorer
{
public:
    SeedRestorer(): m_seed(dRandGetSeed()) {}
    ~SeedRestorer() { dRandSetSeed(m_seed); }

private:
    unsigned long m_seed;
};

} // namespace


TEST(test_dxRandNext_matches_dRand)
{
    SeedRestorer seedRestorer;

    for (unsigned int seedIndex = 0; seedIndex != TEST_SEED_COUNT; ++seedIndex) {
        duint32 state = TEST_SEEDS[seedIndex];
        dRandSetSeed(TEST_SEEDS[seedIndex]);

        for (unsigned int draw = 0; draw != 1000; ++draw) {
            unsigned long expected = dRand();
            unsigned long actual = dxRandNext(state);
            CHECK_EQUAL(expected, actual);
        }
    }
}


TEST(test_dxRandSkip_matches_dRand_calls)
{
    SeedRestorer seedRestorer;

    const unsigned int counts[] = { 0, 1, 2, 3, 4, 5, 7, 8, 9, 255, 256, 257, 65535, 65536, 65537, 1000003 };
    const unsigned int countCount = sizeof(counts) / sizeof(counts[0]);

    for (unsigned int seedIndex = 0; seedIndex != TEST_SEED_COUNT; ++seedIndex) {
        const duint32 seed = TEST_SEEDS[seedIndex];

        for (unsigned int countIndex = 0; countIndex != countCount; ++countIndex) {
            const unsigned int count = counts[countIndex];

            dRandSetSeed(seed);
            for (unsigned int draw = 0; draw != count; ++draw) {
                dRand();
            }
            unsigned long expected = dRandGetSeed();

            duint32 multiplier, increment;
            dxRandSkip(multiplier, increment, count);
            unsigned long actual = multiplier * seed + increment;
            CHECK_EQUAL(expected, actual);
        }
    }
}


TEST(test_dxRandIntFromValue_matches_dRandInt)
{
    SeedRestorer seedRestorer;

    // Every branch of the mapping: n <= 2, 4, 16, 256, 65536 and above.
    const int ns[] = { 1, 2, 3, 4, 5, 15, 16, 17, 255, 256, 257, 65535, 65536, 65537, 1000000, 0x7FFFFFFF };
    const unsigned int nCount = sizeof(ns) / sizeof(ns[0]);

    for (unsigned int seedIndex = 0; seedIndex != TEST_SEED_COUNT; ++seedIndex) {
        for (unsigned int nIndex = 0; nIndex != nCount; ++nIndex) {
            const int n = ns[nIndex];
            duint32 state = TEST_SEEDS[seedIndex];
            dRandSetSeed(TEST_SEEDS[seedIndex]);

            for (unsigned int draw = 0; draw != 100; ++draw) {
                int expected = dRandInt(n);
                int actual = dxRandIntFromValue(dxRandNext(state), n);
                CHECK_EQUAL(expected, actual);
            }
        }
    }
}


TEST(test_dxRandShuffleFromState_matches_dRandInt_loop)
{
    SeedRestorer seedRestorer;

    // The last draws of the larger counts reach every branch of the mapping.
    const unsigned int counts[] = { 0, 1, 2, 3, 4, 5, 16, 17, 256, 257, 65536, 65537, 70000 };
    const unsigned int countCount = sizeof(counts) / sizeof(counts[0]);

    for (unsigned int seedIndex = 0; seedIndex != TEST_SEED_COUNT; ++seedIndex) {
        const duint32 seed = TEST_SEEDS[seedIndex];

        for (unsigned int countIndex = 0; countIndex != countCount; ++countIndex) {
            const unsigned int count = counts[countIndex];

            std::vector<unsigned int> expected(count), actual(count);
            for (unsigned int index = 0; index != count; ++index) {
                expected[index] = index;
                actual[index] = index;
            }

            // The reference: the same swaps, each with its own dRandInt() call.
            dRandSetSeed(seed);
            for (unsigned int index = 1; index < count; ++index) {
                int swapIndex = dRandInt((int)(index + 1));
                unsigned int tmp = expected[index];
                expected[index] = expected[swapIndex];
                expected[swapIndex] = tmp;
            }
            unsigned long expectedSeed = dRandGetSeed();

            dxRandShuffleFromState(count != 0 ? &actual[0] : NULL, count, seed);
            CHECK(expected == actual);

            // dxRandShuffle() reserves count - 1 values: the seed must end where the loop left it.
            duint32 multiplier, increment;
            dxRandSkip(multiplier, increment, count > 1 ? count - 1 : 0);
            unsigned long actualSeed = multiplier * seed + increment;
            CHECK_EQUAL(expectedSeed, actualSeed);
        }
    }
}


TEST(test_dxRandIslandSeed_known_values)
{
    // fmix32(stepSeed + islandIndex * 0x9E3779B9), computed outside ODE. The per-island random reordering gives other
    // results if these change.
    CHECK_EQUAL(0x00000000UL, (unsigned long)dxRandIslandSeed(0x00000000, 0));
    CHECK_EQUAL(0x92CA2F0EUL, (unsigned long)dxRandIslandSeed(0x00000000, 1));
    CHECK_EQUAL(0x3CD6E3F3UL, (unsigned long)dxRandIslandSeed(0x00000000, 2));
    CHECK_EQUAL(0x514E28B7UL, (unsigned long)dxRandIslandSeed(0x00000001, 0));
    CHECK_EQUAL(0xE37CD1BCUL, (unsigned long)dxRandIslandSeed(0x12345678, 0));
    CHECK_EQUAL(0x9AABB687UL, (unsigned long)dxRandIslandSeed(0x12345678, 1000));
    CHECK_EQUAL(0x2994C1B5UL, (unsigned long)dxRandIslandSeed(0xFFFFFFFF, 3));
    CHECK_EQUAL(0xA0B40B8DUL, (unsigned long)dxRandIslandSeed(0x9E3779B9, 65535));
}


TEST(test_dxRandIslandSeed_distinct_states)
{
    const unsigned int islandCount = 65536;

    for (unsigned int seedIndex = 0; seedIndex != TEST_SEED_COUNT; ++seedIndex) {
        const duint32 stepSeed = TEST_SEEDS[seedIndex];
        // The next step's seed: the next value of dRand()'s sequence.
        duint32 nextStepSeed = stepSeed;
        dxRandNext(nextStepSeed);

        std::vector<duint32> states(islandCount);
        unsigned int sameStateCount = 0;
        for (unsigned int islandIndex = 0; islandIndex != islandCount; ++islandIndex) {
            states[islandIndex] = dxRandIslandSeed(stepSeed, islandIndex);
            if (dxRandIslandSeed(nextStepSeed, islandIndex) == states[islandIndex]) {
                ++sameStateCount;
            }
        }

        // An island starts from another state at the next step.
        CHECK_EQUAL(0U, sameStateCount);

        // The islands of a step start from distinct states.
        std::sort(states.begin(), states.end());
        CHECK(std::adjacent_find(states.begin(), states.end()) == states.end());
    }
}


TEST(test_dxRandShuffleAdvancingState_continues_the_sequence)
{
    SeedRestorer seedRestorer;

    // Two shuffles from one state, the second continuing where the first left it, against the dRandInt() loop run
    // twice from the same seed; the first shuffle also against dxRandShuffleFromState().
    const unsigned int counts[] = { 0, 1, 2, 3, 5, 17, 257, 65537 };
    const unsigned int countCount = sizeof(counts) / sizeof(counts[0]);

    for (unsigned int seedIndex = 0; seedIndex != TEST_SEED_COUNT; ++seedIndex) {
        const duint32 seed = TEST_SEEDS[seedIndex];

        for (unsigned int countIndex = 0; countIndex != countCount; ++countIndex) {
            const unsigned int count = counts[countIndex];

            std::vector<unsigned int> expected(count), actual(count), firstFromState(count);
            for (unsigned int index = 0; index != count; ++index) {
                expected[index] = index;
                actual[index] = index;
                firstFromState[index] = index;
            }

            // The reference: the swaps of both shuffles, each with its own dRandInt() call.
            dRandSetSeed(seed);
            for (unsigned int shuffle = 0; shuffle != 2; ++shuffle) {
                for (unsigned int index = 1; index < count; ++index) {
                    int swapIndex = dRandInt((int)(index + 1));
                    unsigned int tmp = expected[index];
                    expected[index] = expected[swapIndex];
                    expected[swapIndex] = tmp;
                }
            }
            unsigned long expectedState = dRandGetSeed();

            duint32 state = seed;
            dxRandShuffleAdvancingState(count != 0 ? &actual[0] : NULL, count, state);
            dxRandShuffleFromState(count != 0 ? &firstFromState[0] : NULL, count, seed);
            CHECK(firstFromState == actual);

            dxRandShuffleAdvancingState(count != 0 ? &actual[0] : NULL, count, state);
            CHECK(expected == actual);
            CHECK_EQUAL(expectedState, (unsigned long)state);
        }
    }
}
