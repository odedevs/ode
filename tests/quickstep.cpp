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
 * QuickStep's per-island random reordering (dWorldSetQuickStepRandomReorderingMode(), ode/src/quickstep.cpp).
 *
 * Two worlds are stepped in the per-island mode from the same dRand() seed, without a threading implementation and
 * with the built-in one. In every run, the bodies must end bit-identical, and each step must have used exactly one
 * value of dRand()'s sequence:
 * - piles of boxes resting on a plane and chains of bodies hanging from hinges, with a pool of threads and the default
 *   thread counts (islands and island stepping unlimited, LCP solving 1);
 * - boxes sliding on a plane, each alone in its island, with the same pool and several LCP solving threads. ODE's
 *   threaded LCP solver makes each row wait for one earlier row only, the later of its two bodies' last rows before
 *   it, so in an island of several bodies its results depend on the threads' timing whatever the reordering values.
 *   In an island of one body, each row waits for the one before it: the solver keeps the shuffled order, and only
 *   the values can change the results.
 * The threaded runs are skipped when ODE is built without its built-in threading implementation.
 */

#include <UnitTest++.h>
#include <ode/ode.h>

#include <cstddef>
#include <cstring>
#include <vector>


namespace {

const unsigned long START_SEED = 0x12345678;
const unsigned long OTHER_SEED = START_SEED + 1;
const dReal STEP_SIZE = REAL(0.01);
const unsigned int STEP_COUNT = 50;

const unsigned int POOL_THREAD_COUNT = 4;
const unsigned int LCP_SOLVING_THREAD_COUNT = 4;
// Each threaded setup runs this many times: each run gives the threads' timing another chance to show.
const unsigned int THREADED_RUN_COUNT = 3;

const dReal BOX_SIDE = REAL(0.4);
// The boxes start overlapping the plane and each other by this much, so that they touch from the first step.
const dReal BOX_OVERLAP = REAL(0.001);

const unsigned int PILE_COUNT = 3;
const unsigned int BOXES_PER_PILE = 3;

const unsigned int CHAIN_COUNT = 3;
const unsigned int LINKS_PER_CHAIN = 5;
const dReal LINK_RADIUS = REAL(0.1);
const dReal LINK_SPACING = REAL(0.3);
const dReal CHAIN_HEIGHT = REAL(3.0);

const unsigned int SLIDING_BOX_COUNT = 4;

const unsigned int MAX_CONTACTS = 4;


// Puts dRand()'s seed back as it was when the test ends.
class SeedRestorer
{
public:
    SeedRestorer(): m_seed(dRandGetSeed()) {}
    ~SeedRestorer() { dRandSetSeed(m_seed); }

private:
    unsigned long m_seed;
};


// The worlds the tests step.
enum SceneKind
{
    SK_PILES_AND_CHAINS,    // piles of boxes resting on a plane and chains of bodies hanging from hinges
    SK_SLIDING_BOXES        // boxes sliding and spinning on a plane, each alone in its island
};


// A world of several islands. The chains have no geoms, so only the boxes collide.
class IslandsScene
{
public:
    explicit IslandsScene(SceneKind kind);
    ~IslandsScene();

    dWorldID GetWorld() const { return m_world; }

    // Collides, steps with dWorldQuickStep() and removes the contacts; returns dWorldQuickStep()'s success.
    bool Step();
    // Appends every body's position, quaternion and velocities, in the order the bodies were created.
    void AppendState(std::vector<dReal> &state) const;

private:
    // Each pile and each chain is an island.
    void AddPilesAndChains();
    // Each box is an island of one body.
    void AddSlidingBoxes();
    dBodyID AddBox(dReal x, dReal y, dReal z);

    static void NearCallback(void *data, dGeomID geom1, dGeomID geom2);

    dWorldID m_world;
    dSpaceID m_space;
    dJointGroupID m_contactGroup;
    std::vector<dBodyID> m_bodies;
};


IslandsScene::IslandsScene(SceneKind kind):
    m_world(dWorldCreate()),
    m_space(dSimpleSpaceCreate(0)),
    m_contactGroup(dJointGroupCreate(0))
{
    dWorldSetGravity(m_world, 0, 0, REAL(-9.81));
    dCreatePlane(m_space, 0, 0, 1, 0);

    if (kind == SK_PILES_AND_CHAINS) {
        AddPilesAndChains();
    }
    else {
        AddSlidingBoxes();
    }
}


IslandsScene::~IslandsScene()
{
    dJointGroupDestroy(m_contactGroup);
    dSpaceDestroy(m_space);
    dWorldDestroy(m_world);
}


void IslandsScene::AddPilesAndChains()
{
    for (unsigned int pile = 0; pile != PILE_COUNT; ++pile) {
        for (unsigned int level = 0; level != BOXES_PER_PILE; ++level) {
            // Each box a little off the one below it, so that the contacts differ from box to box.
            AddBox(2 * pile + REAL(0.05) * level, REAL(0.03) * level,
                BOX_SIDE * (level + REAL(0.5)) - BOX_OVERLAP * (level + 1));
        }
    }

    for (unsigned int chain = 0; chain != CHAIN_COUNT; ++chain) {
        const dReal x = 10 + 2 * chain;
        dBodyID previous = 0;
        for (unsigned int link = 0; link != LINKS_PER_CHAIN; ++link) {
            dBodyID body = dBodyCreate(m_world);
            dMass mass;
            dMassSetSphere(&mass, 1, LINK_RADIUS);
            dBodySetMass(body, &mass);
            // The chain starts horizontal and swings down around the x axis.
            dBodySetPosition(body, x, LINK_SPACING * (link + 1), CHAIN_HEIGHT);

            // The first hinge holds the chain to the static environment, the others join neighbouring links.
            dJointID hinge = dJointCreateHinge(m_world, 0);
            dJointAttach(hinge, body, previous);
            dJointSetHingeAnchor(hinge, x, link == 0 ? 0 : LINK_SPACING * (link + REAL(0.5)), CHAIN_HEIGHT);
            dJointSetHingeAxis(hinge, 1, 0, 0);

            previous = body;
            m_bodies.push_back(body);
        }
    }
}


void IslandsScene::AddSlidingBoxes()
{
    for (unsigned int box = 0; box != SLIDING_BOX_COUNT; ++box) {
        // Each box rests on the plane, turned, sliding and spinning a little differently from the others, and far
        // enough from them never to touch them.
        dBodyID body = AddBox(REAL(2.0) * box, 0, BOX_SIDE * REAL(0.5) - BOX_OVERLAP);

        dMatrix3 rotation;
        dRFromAxisAndAngle(rotation, 0, 0, 1, REAL(0.3) + REAL(0.1) * box);
        dBodySetRotation(body, rotation);
        dBodySetLinearVel(body, 1 + REAL(0.25) * box, REAL(0.5), 0);
        dBodySetAngularVel(body, 0, 0, 2 + REAL(0.5) * box);
    }
}


dBodyID IslandsScene::AddBox(dReal x, dReal y, dReal z)
{
    dBodyID body = dBodyCreate(m_world);
    dMass mass;
    dMassSetBox(&mass, 1, BOX_SIDE, BOX_SIDE, BOX_SIDE);
    dBodySetMass(body, &mass);
    dBodySetPosition(body, x, y, z);

    dGeomID geom = dCreateBox(m_space, BOX_SIDE, BOX_SIDE, BOX_SIDE);
    dGeomSetBody(geom, body);
    m_bodies.push_back(body);
    return body;
}


bool IslandsScene::Step()
{
    dSpaceCollide(m_space, this, &NearCallback);
    const int result = dWorldQuickStep(m_world, STEP_SIZE);
    dJointGroupEmpty(m_contactGroup);
    return result != 0;
}


void IslandsScene::AppendState(std::vector<dReal> &state) const
{
    for (std::vector<dBodyID>::const_iterator body = m_bodies.begin(); body != m_bodies.end(); ++body) {
        const dReal *position = dBodyGetPosition(*body);
        const dReal *quaternion = dBodyGetQuaternion(*body);
        const dReal *linearVel = dBodyGetLinearVel(*body);
        const dReal *angularVel = dBodyGetAngularVel(*body);
        state.insert(state.end(), position, position + 3);
        state.insert(state.end(), quaternion, quaternion + 4);
        state.insert(state.end(), linearVel, linearVel + 3);
        state.insert(state.end(), angularVel, angularVel + 3);
    }
}


void IslandsScene::NearCallback(void *data, dGeomID geom1, dGeomID geom2)
{
    IslandsScene *scene = static_cast<IslandsScene *>(data);

    dContact contacts[MAX_CONTACTS];
    std::memset(contacts, 0, sizeof(contacts));
    const int contactCount = dCollide(geom1, geom2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact));

    for (int index = 0; index < contactCount; ++index) {
        dContact &contact = contacts[index];
        contact.surface.mode = dContactApprox1;
        contact.surface.mu = REAL(0.5);

        dJointID joint = dJointCreateContact(scene->m_world, scene->m_contactGroup, &contact);
        dJointAttach(joint, dGeomGetBody(geom1), dGeomGetBody(geom2));
    }
}


enum ThreadingSetup
{
    TS_NONE,                    // no threading implementation
    TS_POOL,                    // the built-in implementation, a pool of threads, the default thread counts
    TS_POOL_AND_LCP_THREADS     // the same with LCP_SOLVING_THREAD_COUNT LCP solving threads
};


// What one run of a scene gives.
struct RunResult
{
    RunResult(): ran(false), stepsSucceeded(false), finalSeed(0) {}

    bool ran;                       // false: the setup needs the built-in threading implementation, which is absent
    bool stepsSucceeded;            // the threading setup was made and every dWorldQuickStep() succeeded
    unsigned long finalSeed;        // dRandGetSeed() after the steps
    std::vector<dReal> finalState;  // IslandsScene::AppendState() after the steps
};


// Steps a new scene of the kind STEP_COUNT times from the seed, in the reordering mode, with the threading setup.
RunResult RunScene(SceneKind kind, int reorderingMode, ThreadingSetup setup, unsigned long seed)
{
    RunResult result;

    dThreadingImplementationID threading = NULL;
    dThreadingThreadPoolID pool = NULL;
    if (setup != TS_NONE) {
        threading = dThreadingAllocateMultiThreadedImplementation();
        if (threading == NULL) {
            // ODE is built without its built-in threading implementation.
            return result;
        }

        pool = dThreadingAllocateThreadPool(POOL_THREAD_COUNT, 0, dAllocateFlagBasicData, NULL);
        if (pool == NULL) {
            dThreadingFreeImplementation(threading);
            result.ran = true;
            return result;
        }

        dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
    }

    result.ran = true;
    {
        IslandsScene scene(kind);
        dWorldID world = scene.GetWorld();
        dWorldSetQuickStepRandomReorderingMode(world, reorderingMode);

        if (setup != TS_NONE) {
            if (setup == TS_POOL_AND_LCP_THREADS) {
                dWorldSteppingThreadingParameters parameters;
                parameters.param_set = dWSTP_LCPSolvingMaxThreads;
                parameters.world_islands_iteration_max_threads = 0; // not in param_set
                parameters.island_stepping_max_threads = 0; // not in param_set
                parameters.lcp_solving_max_threads = LCP_SOLVING_THREAD_COUNT;
                dWorldSetSteppingThreadingParameters(world, &parameters);
            }

            dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);
        }

        dRandSetSeed(seed);
        result.stepsSucceeded = true;
        for (unsigned int step = 0; step != STEP_COUNT; ++step) {
            if (!scene.Step()) {
                result.stepsSucceeded = false;
            }
        }
        result.finalSeed = dRandGetSeed();
        scene.AppendState(result.finalState);

        if (setup != TS_NONE) {
            dThreadingImplementationShutdownProcessing(threading);
            dThreadingFreeThreadPool(pool);
            dWorldSetStepThreadingImplementation(world, NULL, NULL);
            dThreadingFreeImplementation(threading);
        }
    }

    return result;
}


bool SameBits(const std::vector<dReal> &first, const std::vector<dReal> &second)
{
    return first.size() == second.size()
        && (first.empty() || std::memcmp(&first[0], &second[0], first.size() * sizeof(dReal)) == 0);
}


// dRand()'s seed after one value per step from START_SEED.
unsigned long SeedAfterOneValuePerStep()
{
    dRandSetSeed(START_SEED);
    for (unsigned int step = 0; step != STEP_COUNT; ++step) {
        dRand();
    }
    return dRandGetSeed();
}

} // namespace


TEST(test_dWorldQuickStepRandomReorderingMode_set_and_get)
{
    dWorldID world = dWorldCreate();

    CHECK_EQUAL(dWORLDQUICKSTEP_RANDOMREORDERING_SHARED, dWorldGetQuickStepRandomReorderingMode(world));

    dWorldSetQuickStepRandomReorderingMode(world, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND);
    CHECK_EQUAL(dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND, dWorldGetQuickStepRandomReorderingMode(world));

    dWorldSetQuickStepRandomReorderingMode(world, dWORLDQUICKSTEP_RANDOMREORDERING_SHARED);
    CHECK_EQUAL(dWORLDQUICKSTEP_RANDOMREORDERING_SHARED, dWorldGetQuickStepRandomReorderingMode(world));

    dWorldDestroy(world);
}


TEST(test_dWorldQuickStep_per_island_reordering_changes_the_results)
{
    SeedRestorer seedRestorer;

    // Without threads, both modes repeat; they shuffle with different values, so the bodies end elsewhere.
    RunResult shared = RunScene(SK_PILES_AND_CHAINS, dWORLDQUICKSTEP_RANDOMREORDERING_SHARED, TS_NONE, START_SEED);
    RunResult perIsland = RunScene(SK_PILES_AND_CHAINS, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND, TS_NONE,
        START_SEED);

    CHECK(shared.ran && shared.stepsSucceeded);
    CHECK(perIsland.ran && perIsland.stepsSucceeded);
    CHECK_EQUAL(shared.finalState.size(), perIsland.finalState.size());
    CHECK(!SameBits(shared.finalState, perIsland.finalState));
}


TEST(test_dWorldQuickStep_per_island_reordering_does_not_depend_on_island_threads)
{
    SeedRestorer seedRestorer;

    RunResult reference = RunScene(SK_PILES_AND_CHAINS, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND, TS_NONE,
        START_SEED);
    CHECK(reference.ran && reference.stepsSucceeded);
    // One value of dRand()'s sequence per step, whatever the islands.
    CHECK_EQUAL(SeedAfterOneValuePerStep(), reference.finalSeed);

    // The default thread counts: the islands are stepped in parallel, each with one LCP solving thread.
    for (unsigned int run = 0; run != THREADED_RUN_COUNT; ++run) {
        RunResult threaded = RunScene(SK_PILES_AND_CHAINS, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND, TS_POOL,
            START_SEED);
        if (!threaded.ran) {
            // Built without the built-in threading implementation: nothing threaded to compare.
            return;
        }

        CHECK(threaded.stepsSucceeded);
        CHECK(SameBits(reference.finalState, threaded.finalState));
        CHECK_EQUAL(reference.finalSeed, threaded.finalSeed);
    }
}


TEST(test_dWorldQuickStep_per_island_reordering_does_not_depend_on_lcp_solving_threads)
{
    SeedRestorer seedRestorer;

    // Islands of one body: the threaded LCP solver keeps their shuffled order (see the top of the file).
    RunResult reference = RunScene(SK_SLIDING_BOXES, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND, TS_NONE,
        START_SEED);
    CHECK(reference.ran && reference.stepsSucceeded);
    CHECK_EQUAL(SeedAfterOneValuePerStep(), reference.finalSeed);

    // The reordering values show in this world's results; otherwise wrong values would go unnoticed below.
    RunResult otherSeed = RunScene(SK_SLIDING_BOXES, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND, TS_NONE,
        OTHER_SEED);
    CHECK(otherSeed.ran && otherSeed.stepsSucceeded);
    CHECK(!SameBits(reference.finalState, otherSeed.finalState));

    for (unsigned int run = 0; run != THREADED_RUN_COUNT; ++run) {
        RunResult threaded = RunScene(SK_SLIDING_BOXES, dWORLDQUICKSTEP_RANDOMREORDERING_PER_ISLAND,
            TS_POOL_AND_LCP_THREADS, START_SEED);
        if (!threaded.ran) {
            // Built without the built-in threading implementation: nothing threaded to compare.
            return;
        }

        CHECK(threaded.stepsSucceeded);
        CHECK(SameBits(reference.finalState, threaded.finalState));
        CHECK_EQUAL(reference.finalSeed, threaded.finalSeed);
    }
}
