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

#include <UnitTest++.h>
#include <ode/ode.h>
#include <ode/threading_impl.h>
#include <thread>
#include <vector>
#include <atomic>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// A simple collision callback that just counts contacts
struct CollisionCounter {
    dJointGroupID contactGroup;
    dWorldID world;
    int contactCount;
};

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    CollisionCounter *cc = static_cast<CollisionCounter *>(data);
    dContact contacts[4];
    int n = dCollide(o1, o2, 4, &contacts[0].geom, sizeof(dContact));
    for (int i = 0; i < n; ++i) {
        contacts[i].surface.mode = dContactBounce;
        contacts[i].surface.mu = 1.0;
        contacts[i].surface.bounce = 0.5;
        contacts[i].surface.bounce_vel = 0.1;
        dJointID c = dJointCreateContact(cc->world, cc->contactGroup, &contacts[i]);
        dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
        cc->contactCount++;
    }
}

// Fixture that sets up a multi-threaded world with thread pool
struct MultiThreadedWorldFixture
{
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;
    dThreadingImplementationID threading;
    dThreadingThreadPoolID pool;

    MultiThreadedWorldFixture()
    {
        world = dWorldCreate();
        dWorldSetGravity(world, 0, 0, -9.81);
        dWorldSetCFM(world, 1e-5);
        dWorldSetERP(world, 0.8);
        dWorldSetContactMaxCorrectingVel(world, 0.1);
        dWorldSetContactSurfaceLayer(world, 0.001);

        space = dHashSpaceCreate(0);
        contactGroup = dJointGroupCreate(0);

        threading = dThreadingAllocateMultiThreadedImplementation();
        pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
        dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
        dWorldSetStepThreadingImplementation(world,
            dThreadingImplementationGetFunctions(threading), threading);
    }

    ~MultiThreadedWorldFixture()
    {
        dThreadingImplementationShutdownProcessing(threading);
        dThreadingFreeThreadPool(pool);
        dWorldSetStepThreadingImplementation(world, NULL, NULL);
        dThreadingFreeImplementation(threading);

        dJointGroupDestroy(contactGroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
    }
};

// Fixture that sets up a self-threaded world (default)
struct SelfThreadedWorldFixture
{
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;

    SelfThreadedWorldFixture()
    {
        world = dWorldCreate();
        dWorldSetGravity(world, 0, 0, -9.81);
        space = dHashSpaceCreate(0);
        contactGroup = dJointGroupCreate(0);
    }

    ~SelfThreadedWorldFixture()
    {
        dJointGroupDestroy(contactGroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
    }
};


// ---------------------------------------------------------------------------
// SUITE: ThreadingSetup
// Basic threading infrastructure lifecycle tests
// ---------------------------------------------------------------------------
SUITE(ThreadingSetup)
{

TEST(AllocateAndFreeMultiThreadedImplementation)
{
    dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
    CHECK(impl != NULL);
    dThreadingImplementationShutdownProcessing(impl);
    dThreadingFreeImplementation(impl);
}

TEST(AllocateAndFreeSelfThreadedImplementation)
{
    dThreadingImplementationID impl = dThreadingAllocateSelfThreadedImplementation();
    CHECK(impl != NULL);
    dThreadingImplementationShutdownProcessing(impl);
    dThreadingFreeImplementation(impl);
}

TEST(GetFunctionsReturnsNonNull)
{
    dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
    const dThreadingFunctionsInfo *funcs = dThreadingImplementationGetFunctions(impl);
    CHECK(funcs != NULL);
    dThreadingImplementationShutdownProcessing(impl);
    dThreadingFreeImplementation(impl);
}

TEST(AllocateAndFreeThreadPool)
{
    dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
    dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(2, 0, dAllocateFlagBasicData, NULL);
    CHECK(pool != NULL);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);

    dThreadingImplementationShutdownProcessing(impl);
    dThreadingFreeThreadPool(pool);
    dThreadingFreeImplementation(impl);
}

TEST(ThreadPoolVariousSizes)
{
    // Test creating thread pools of different sizes
    for (unsigned count = 1; count <= 8; count *= 2) {
        dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
        dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(count, 0, dAllocateFlagBasicData, NULL);
        CHECK(pool != NULL);
        dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);

        dThreadingImplementationShutdownProcessing(impl);
        dThreadingFreeThreadPool(pool);
        dThreadingFreeImplementation(impl);
    }
}

TEST(AssignAndClearWorldThreading)
{
    dWorldID w = dWorldCreate();

    dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
    dWorldSetStepThreadingImplementation(w,
        dThreadingImplementationGetFunctions(impl), impl);

    // Clear
    dWorldSetStepThreadingImplementation(w, NULL, NULL);

    dThreadingImplementationShutdownProcessing(impl);
    dThreadingFreeImplementation(impl);
    dWorldDestroy(w);
}

TEST(ShutdownAndRestart)
{
    dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
    dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(2, 0, dAllocateFlagBasicData, NULL);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);

    dThreadingImplementationShutdownProcessing(impl);
    dThreadingThreadPoolWaitIdleState(pool);

    dThreadingImplementationCleanupForRestart(impl);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);

    dThreadingImplementationShutdownProcessing(impl);
    dThreadingFreeThreadPool(pool);
    dThreadingFreeImplementation(impl);
}

} // SUITE ThreadingSetup


// ---------------------------------------------------------------------------
// SUITE: ThreadedStepping
// Verify that physics stepping works correctly with threading
// ---------------------------------------------------------------------------
SUITE(ThreadedStepping)
{

TEST_FIXTURE(MultiThreadedWorldFixture, SingleBodyFreeFall)
{
    dBodyID body = dBodyCreate(world);
    dMass mass;
    dMassSetSphere(&mass, 1.0, 0.5);
    dBodySetMass(body, &mass);
    dBodySetPosition(body, 0, 0, 10);

    // Step for some time
    for (int i = 0; i < 100; ++i) {
        dWorldStep(world, 0.01);
    }

    const dReal *pos = dBodyGetPosition(body);
    // After 1 second of freefall from z=10, z ≈ 10 - 0.5*9.81*1^2 = 5.095
    CHECK_CLOSE(5.095, pos[2], 0.1);
    CHECK_CLOSE(0.0, pos[0], 1e-6);
    CHECK_CLOSE(0.0, pos[1], 1e-6);
}

TEST_FIXTURE(MultiThreadedWorldFixture, MultipleBodiesFreeFall)
{
    const int N = 20;
    dBodyID bodies[N];
    for (int i = 0; i < N; ++i) {
        bodies[i] = dBodyCreate(world);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.5);
        dBodySetMass(bodies[i], &mass);
        dBodySetPosition(bodies[i], (dReal)i, 0, 10);
    }

    for (int step = 0; step < 100; ++step) {
        dWorldStep(world, 0.01);
    }

    // All bodies should have fallen the same amount
    const dReal expectedZ = 10.0 - 0.5 * 9.81 * 1.0;
    for (int i = 0; i < N; ++i) {
        const dReal *pos = dBodyGetPosition(bodies[i]);
        CHECK_CLOSE(expectedZ, pos[2], 0.1);
        CHECK_CLOSE((dReal)i, pos[0], 1e-6);
    }
}

TEST_FIXTURE(MultiThreadedWorldFixture, QuickStepMultiThreaded)
{
    dBodyID body = dBodyCreate(world);
    dMass mass;
    dMassSetSphere(&mass, 1.0, 0.5);
    dBodySetMass(body, &mass);
    dBodySetPosition(body, 0, 0, 10);

    for (int i = 0; i < 100; ++i) {
        dWorldQuickStep(world, 0.01);
    }

    const dReal *pos = dBodyGetPosition(body);
    CHECK_CLOSE(5.095, pos[2], 0.2);
}

TEST_FIXTURE(MultiThreadedWorldFixture, ContactJointsStepping)
{
    // Create a ground plane
    dCreatePlane(space, 0, 0, 1, 0);

    // Create a sphere above the ground
    dBodyID body = dBodyCreate(world);
    dMass mass;
    dMassSetSphere(&mass, 1.0, 0.5);
    dBodySetMass(body, &mass);
    dBodySetPosition(body, 0, 0, 2);
    dGeomID geom = dCreateSphere(space, 0.5);
    dGeomSetBody(geom, body);

    CollisionCounter cc;
    cc.contactGroup = contactGroup;
    cc.world = world;
    cc.contactCount = 0;

    bool hadContacts = false;
    for (int i = 0; i < 500; ++i) {
        dSpaceCollide(space, &cc, &nearCallback);
        dWorldStep(world, 0.01);
        dJointGroupEmpty(contactGroup);
        if (cc.contactCount > 0) hadContacts = true;
    }

    // The sphere should have hit the ground
    CHECK(hadContacts);

    // The sphere should be resting near the ground
    const dReal *pos = dBodyGetPosition(body);
    CHECK_CLOSE(0.5, pos[2], 0.2);
}

TEST_FIXTURE(MultiThreadedWorldFixture, MultiBodyCollisions)
{
    dCreatePlane(space, 0, 0, 1, 0);

    // Spread bodies horizontally so they don't pile up and cause LCP instability
    const int N = 6;
    dBodyID bodies[N];
    dGeomID geoms[N];
    for (int i = 0; i < N; ++i) {
        bodies[i] = dBodyCreate(world);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.3);
        dBodySetMass(bodies[i], &mass);
        dBodySetPosition(bodies[i], (dReal)(i * 2), 0, 2.0);
        geoms[i] = dCreateSphere(space, 0.3);
        dGeomSetBody(geoms[i], bodies[i]);
    }

    CollisionCounter cc;
    cc.contactGroup = contactGroup;
    cc.world = world;
    cc.contactCount = 0;

    bool hadContacts = false;
    for (int step = 0; step < 300; ++step) {
        cc.contactCount = 0;
        dSpaceCollide(space, &cc, &nearCallback);
        dWorldStep(world, 0.01);
        dJointGroupEmpty(contactGroup);
        if (cc.contactCount > 0) hadContacts = true;
    }

    CHECK(hadContacts);
    // All bodies should have settled on the ground
    for (int i = 0; i < N; ++i) {
        const dReal *pos = dBodyGetPosition(bodies[i]);
        CHECK_CLOSE(0.3, pos[2], 0.5);
    }
}

TEST_FIXTURE(MultiThreadedWorldFixture, HingeJointMultiThreaded)
{
    dBodyID b1 = dBodyCreate(world);
    dBodyID b2 = dBodyCreate(world);
    dMass mass;
    dMassSetBox(&mass, 1.0, 1, 1, 1);
    dBodySetMass(b1, &mass);
    dBodySetMass(b2, &mass);
    dBodySetPosition(b1, 0, 0, 2);
    dBodySetPosition(b2, 1, 0, 2);

    dJointID hinge = dJointCreateHinge(world, 0);
    dJointAttach(hinge, b1, b2);
    dJointSetHingeAnchor(hinge, 0.5, 0, 2);
    dJointSetHingeAxis(hinge, 0, 1, 0);

    for (int i = 0; i < 200; ++i) {
        dWorldStep(world, 0.01);
    }

    // Both bodies should have moved under gravity
    const dReal *p1 = dBodyGetPosition(b1);
    const dReal *p2 = dBodyGetPosition(b2);
    CHECK(p1[2] < 2.0);
    CHECK(p2[2] < 2.0);
    // They should still be approximately 1 unit apart along x
    dReal dx = p2[0] - p1[0];
    dReal dz = p2[2] - p1[2];
    dReal dist = std::sqrt(dx*dx + dz*dz);
    CHECK_CLOSE(1.0, dist, 0.15);
}

} // SUITE ThreadedStepping


// ---------------------------------------------------------------------------
// SUITE: ThreadedVsSingleThreaded
// Ensure multithreaded results match single-threaded results
// ---------------------------------------------------------------------------
SUITE(ThreadedVsSingleThreaded)
{

// Run the same simulation with and without threading, compare results
TEST(FreeFallConsistency)
{
    dReal posMulti[3], posSingle[3];

    // Multi-threaded run
    {
        dWorldID w = dWorldCreate();
        dWorldSetGravity(w, 0, 0, -9.81);
        dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
        dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
        dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);
        dWorldSetStepThreadingImplementation(w,
            dThreadingImplementationGetFunctions(impl), impl);

        dBodyID body = dBodyCreate(w);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.5);
        dBodySetMass(body, &mass);
        dBodySetPosition(body, 0, 0, 10);

        for (int i = 0; i < 100; ++i)
            dWorldStep(w, 0.01);

        const dReal *p = dBodyGetPosition(body);
        posMulti[0] = p[0]; posMulti[1] = p[1]; posMulti[2] = p[2];

        dThreadingImplementationShutdownProcessing(impl);
        dThreadingFreeThreadPool(pool);
        dWorldSetStepThreadingImplementation(w, NULL, NULL);
        dThreadingFreeImplementation(impl);
        dWorldDestroy(w);
    }

    // Single-threaded run
    {
        dWorldID w = dWorldCreate();
        dWorldSetGravity(w, 0, 0, -9.81);

        dBodyID body = dBodyCreate(w);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.5);
        dBodySetMass(body, &mass);
        dBodySetPosition(body, 0, 0, 10);

        for (int i = 0; i < 100; ++i)
            dWorldStep(w, 0.01);

        const dReal *p = dBodyGetPosition(body);
        posSingle[0] = p[0]; posSingle[1] = p[1]; posSingle[2] = p[2];

        dWorldDestroy(w);
    }

    CHECK_CLOSE(posSingle[0], posMulti[0], 1e-6);
    CHECK_CLOSE(posSingle[1], posMulti[1], 1e-6);
    CHECK_CLOSE(posSingle[2], posMulti[2], 1e-6);
}

TEST(QuickStepConsistency)
{
    dReal posMulti[3], posSingle[3];

    // Multi-threaded
    {
        dWorldID w = dWorldCreate();
        dWorldSetGravity(w, 0, 0, -9.81);
        dThreadingImplementationID impl = dThreadingAllocateMultiThreadedImplementation();
        dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
        dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);
        dWorldSetStepThreadingImplementation(w,
            dThreadingImplementationGetFunctions(impl), impl);

        dBodyID body = dBodyCreate(w);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.5);
        dBodySetMass(body, &mass);
        dBodySetPosition(body, 0, 0, 10);
        for (int i = 0; i < 100; ++i)
            dWorldQuickStep(w, 0.01);

        const dReal *p = dBodyGetPosition(body);
        posMulti[0] = p[0]; posMulti[1] = p[1]; posMulti[2] = p[2];

        dThreadingImplementationShutdownProcessing(impl);
        dThreadingFreeThreadPool(pool);
        dWorldSetStepThreadingImplementation(w, NULL, NULL);
        dThreadingFreeImplementation(impl);
        dWorldDestroy(w);
    }

    // Single-threaded
    {
        dWorldID w = dWorldCreate();
        dWorldSetGravity(w, 0, 0, -9.81);

        dBodyID body = dBodyCreate(w);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.5);
        dBodySetMass(body, &mass);
        dBodySetPosition(body, 0, 0, 10);
        for (int i = 0; i < 100; ++i)
            dWorldQuickStep(w, 0.01);

        const dReal *p = dBodyGetPosition(body);
        posSingle[0] = p[0]; posSingle[1] = p[1]; posSingle[2] = p[2];

        dWorldDestroy(w);
    }

    CHECK_CLOSE(posSingle[0], posMulti[0], 1e-6);
    CHECK_CLOSE(posSingle[1], posMulti[1], 1e-6);
    CHECK_CLOSE(posSingle[2], posMulti[2], 1e-6);
}

TEST(MultiBodyContactConsistency)
{
    dReal zMulti[5], zSingle[5];

    auto runSim = [](bool multiThreaded, dReal *zOut) {
        dWorldID w = dWorldCreate();
        dWorldSetGravity(w, 0, 0, -9.81);
        dWorldSetCFM(w, 1e-5);
        dWorldSetERP(w, 0.8);
        dSpaceID s = dHashSpaceCreate(0);
        dJointGroupID cg = dJointGroupCreate(0);
        dCreatePlane(s, 0, 0, 1, 0);

        dThreadingImplementationID impl = NULL;
        dThreadingThreadPoolID pool = NULL;
        if (multiThreaded) {
            impl = dThreadingAllocateMultiThreadedImplementation();
            pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
            dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl);
            dWorldSetStepThreadingImplementation(w,
                dThreadingImplementationGetFunctions(impl), impl);
        }

        dBodyID bodies[5];
        for (int i = 0; i < 5; ++i) {
            bodies[i] = dBodyCreate(w);
            dMass mass;
            dMassSetSphere(&mass, 1.0, 0.4);
            dBodySetMass(bodies[i], &mass);
            dBodySetPosition(bodies[i], (dReal)i * 2, 0, 2);
            dGeomID geom = dCreateSphere(s, 0.4);
            dGeomSetBody(geom, bodies[i]);
        }

        CollisionCounter cc;
        cc.contactGroup = cg;
        cc.world = w;

        for (int step = 0; step < 300; ++step) {
            cc.contactCount = 0;
            dSpaceCollide(s, &cc, &nearCallback);
            dWorldStep(w, 0.01);
            dJointGroupEmpty(cg);
        }

        for (int i = 0; i < 5; ++i) {
            zOut[i] = dBodyGetPosition(bodies[i])[2];
        }

        if (multiThreaded) {
            dThreadingImplementationShutdownProcessing(impl);
            dThreadingFreeThreadPool(pool);
            dWorldSetStepThreadingImplementation(w, NULL, NULL);
            dThreadingFreeImplementation(impl);
        }
        dJointGroupDestroy(cg);
        dSpaceDestroy(s);
        dWorldDestroy(w);
    };

    runSim(true, zMulti);
    runSim(false, zSingle);

    for (int i = 0; i < 5; ++i) {
        CHECK_CLOSE(zSingle[i], zMulti[i], 0.01);
    }
}

} // SUITE ThreadedVsSingleThreaded


// ---------------------------------------------------------------------------
// SUITE: TLSDataAllocation
// Test per-thread data allocation via dAllocateODEDataForThread
// ---------------------------------------------------------------------------
SUITE(TLSDataAllocation)
{

TEST(AllocateDataForMainThread)
{
    // Main thread already has data allocated (dInitODE does this).
    // Re-allocating should succeed.
    int result = dAllocateODEDataForThread(dAllocateMaskAll);
    CHECK(result != 0);
}

TEST(AllocateDataForWorkerThreads)
{
    // Spawn threads that each allocate ODE data
    const int N = 4;
    std::atomic<int> successCount{0};

    std::vector<std::thread> threads;
    for (int i = 0; i < N; ++i) {
        threads.emplace_back([&successCount]() {
            int result = dAllocateODEDataForThread(dAllocateMaskAll);
            if (result != 0) successCount++;
        });
    }
    for (auto &t : threads) t.join();

    CHECK_EQUAL(N, successCount.load());
}

TEST(CollisionDataPerThread)
{
    // Verify that collision data can be allocated on multiple threads
    const int N = 4;
    std::atomic<int> successCount{0};

    std::vector<std::thread> threads;
    for (int i = 0; i < N; ++i) {
        threads.emplace_back([&successCount]() {
            int result = dAllocateODEDataForThread(dAllocateFlagCollisionData);
            if (result != 0) successCount++;
        });
    }
    for (auto &t : threads) t.join();

    CHECK_EQUAL(N, successCount.load());
}

} // SUITE TLSDataAllocation


// ---------------------------------------------------------------------------
// SUITE: ConcurrentCollision
// Test collision detection from multiple threads on separate spaces
// ---------------------------------------------------------------------------
SUITE(ConcurrentCollision)
{

TEST(ParallelCollisionSeparateSpaces)
{
    // Each thread runs collision detection in its own space —
    // this exercises the per-thread TLS collision caches.
    const int N = 4;
    std::atomic<int> contactsTotal{0};
    std::atomic<int> failCount{0};

    std::vector<std::thread> threads;
    for (int i = 0; i < N; ++i) {
        threads.emplace_back([&contactsTotal, &failCount]() {
            if (!dAllocateODEDataForThread(dAllocateFlagCollisionData)) {
                failCount++;
                return;
            }

            dWorldID w = dWorldCreate();
            dSpaceID s = dHashSpaceCreate(0);
            dJointGroupID cg = dJointGroupCreate(0);

            dCreatePlane(s, 0, 0, 1, 0);

            dBodyID body = dBodyCreate(w);
            dMass mass;
            dMassSetSphere(&mass, 1.0, 0.5);
            dBodySetMass(body, &mass);
            dBodySetPosition(body, 0, 0, 0.5);
            dGeomID geom = dCreateSphere(s, 0.5);
            dGeomSetBody(geom, body);

            CollisionCounter cc;
            cc.contactGroup = cg;
            cc.world = w;
            cc.contactCount = 0;

            dSpaceCollide(s, &cc, &nearCallback);
            contactsTotal += cc.contactCount;

            dJointGroupDestroy(cg);
            dSpaceDestroy(s);
            dWorldDestroy(w);
        });
    }
    for (auto &t : threads) t.join();

    CHECK_EQUAL(0, failCount.load());
    // Each thread should detect at least one contact
    CHECK(contactsTotal.load() >= N);
}

TEST(ParallelTrimeshCollisionSeparateSpaces)
{
    // Exercise the TLS trimesh collider cache from multiple threads
    const int N = 4;
    std::atomic<int> contactsTotal{0};
    std::atomic<int> failCount{0};

    std::vector<std::thread> threads;
    for (int i = 0; i < N; ++i) {
        threads.emplace_back([&contactsTotal, &failCount]() {
            if (!dAllocateODEDataForThread(dAllocateFlagCollisionData)) {
                failCount++;
                return;
            }

            dWorldID w = dWorldCreate();
            dSpaceID s = dSimpleSpaceCreate(0);
            dJointGroupID cg = dJointGroupCreate(0);

            // Create a simple triangle mesh (a flat quad)
            float vertices[] = {
                -5, -5, 0,
                 5, -5, 0,
                 5,  5, 0,
                -5,  5, 0
            };
            dTriIndex indices[] = { 0, 1, 2, 0, 2, 3 };

            dTriMeshDataID meshData = dGeomTriMeshDataCreate();
            dGeomTriMeshDataBuildSingle(meshData,
                vertices, 3 * sizeof(float), 4,
                indices, 6, 3 * sizeof(dTriIndex));
            dGeomID trimesh = dCreateTriMesh(s, meshData, NULL, NULL, NULL);

            // Sphere sitting on the trimesh
            dBodyID body = dBodyCreate(w);
            dMass mass;
            dMassSetSphere(&mass, 1.0, 0.5);
            dBodySetMass(body, &mass);
            dBodySetPosition(body, 0, 0, 0.5);
            dGeomID sphere = dCreateSphere(s, 0.5);
            dGeomSetBody(sphere, body);

            CollisionCounter cc;
            cc.contactGroup = cg;
            cc.world = w;
            cc.contactCount = 0;

            dSpaceCollide(s, &cc, &nearCallback);
            contactsTotal += cc.contactCount;

            dJointGroupDestroy(cg);
            dSpaceDestroy(s);
            dGeomTriMeshDataDestroy(meshData);
            dWorldDestroy(w);
        });
    }
    for (auto &t : threads) t.join();

    CHECK_EQUAL(0, failCount.load());
    CHECK(contactsTotal.load() >= N);
}

TEST(RepeatedCollisionAcrossThreads)
{
    // Run collision detection repeatedly across many short-lived threads
    // to stress TLS allocation/cleanup via thread_local destructors
    const int N = 16;
    std::atomic<int> successCount{0};

    std::vector<std::thread> threads;
    for (int i = 0; i < N; ++i) {
        threads.emplace_back([&successCount]() {
            dAllocateODEDataForThread(dAllocateFlagCollisionData);

            dWorldID w = dWorldCreate();
            dSpaceID s = dHashSpaceCreate(0);
            dCreatePlane(s, 0, 0, 1, 0);

            dBodyID body = dBodyCreate(w);
            dMass mass;
            dMassSetSphere(&mass, 1.0, 0.5);
            dBodySetMass(body, &mass);
            dBodySetPosition(body, 0, 0, 0.5);
            dGeomID geom = dCreateSphere(s, 0.5);
            dGeomSetBody(geom, body);

            // Each world needs its own threading impl for thread-safe stepping
            dThreadingImplementationID impl = dThreadingAllocateSelfThreadedImplementation();
            dWorldSetStepThreadingImplementation(w,
                dThreadingImplementationGetFunctions(impl), impl);

            dJointGroupID cg = dJointGroupCreate(0);
            CollisionCounter cc;
            cc.contactGroup = cg;
            cc.world = w;
            cc.contactCount = 0;

            for (int step = 0; step < 10; ++step) {
                cc.contactCount = 0;
                dSpaceCollide(s, &cc, &nearCallback);
                dWorldStep(w, 0.01);
                dJointGroupEmpty(cg);
            }

            successCount++;

            dWorldSetStepThreadingImplementation(w, NULL, NULL);
            dThreadingImplementationShutdownProcessing(impl);
            dThreadingFreeImplementation(impl);
            dJointGroupDestroy(cg);
            dSpaceDestroy(s);
            dWorldDestroy(w);
        });
    }
    for (auto &t : threads) t.join();

    CHECK_EQUAL(N, successCount.load());
}

} // SUITE ConcurrentCollision


// ---------------------------------------------------------------------------
// SUITE: ThreadedWorldStep
// Full simulation stepping with thread pool
// ---------------------------------------------------------------------------
SUITE(ThreadedWorldStep)
{

TEST_FIXTURE(MultiThreadedWorldFixture, ManyIslands)
{
    // Create multiple independent groups of bodies (islands)
    // Threading should parallelize island processing
    const int ISLANDS = 8;
    const int BODIES_PER_ISLAND = 3;
    dBodyID bodies[ISLANDS * BODIES_PER_ISLAND];

    for (int island = 0; island < ISLANDS; ++island) {
        for (int b = 0; b < BODIES_PER_ISLAND; ++b) {
            int idx = island * BODIES_PER_ISLAND + b;
            bodies[idx] = dBodyCreate(world);
            dMass mass;
            dMassSetBox(&mass, 1.0, 1, 1, 1);
            dBodySetMass(bodies[idx], &mass);
            dBodySetPosition(bodies[idx],
                (dReal)(island * 100),
                (dReal)(b * 2),
                5.0);

            // Connect bodies within same island with ball joints
            if (b > 0) {
                int prevIdx = island * BODIES_PER_ISLAND + b - 1;
                dJointID ball = dJointCreateBall(world, 0);
                dJointAttach(ball, bodies[prevIdx], bodies[idx]);
                dJointSetBallAnchor(ball,
                    (dReal)(island * 100),
                    (dReal)(b * 2 - 1),
                    5.0);
            }
        }
    }

    // Step the simulation
    for (int step = 0; step < 100; ++step) {
        dWorldStep(world, 0.01);
    }

    // Verify all bodies moved under gravity
    for (int i = 0; i < ISLANDS * BODIES_PER_ISLAND; ++i) {
        const dReal *pos = dBodyGetPosition(bodies[i]);
        CHECK(pos[2] < 5.0);
    }
}

TEST_FIXTURE(MultiThreadedWorldFixture, QuickStepManyIslands)
{
    const int ISLANDS = 8;
    dBodyID bodies[ISLANDS];

    for (int i = 0; i < ISLANDS; ++i) {
        bodies[i] = dBodyCreate(world);
        dMass mass;
        dMassSetSphere(&mass, 1.0, 0.5);
        dBodySetMass(bodies[i], &mass);
        dBodySetPosition(bodies[i], (dReal)(i * 10), 0, 5);
    }

    for (int step = 0; step < 100; ++step) {
        dWorldQuickStep(world, 0.01);
    }

    for (int i = 0; i < ISLANDS; ++i) {
        const dReal *pos = dBodyGetPosition(bodies[i]);
        CHECK(pos[2] < 5.0);
    }
}

TEST_FIXTURE(MultiThreadedWorldFixture, SteppingStability)
{
    // Run many steps to check for race conditions / crashes
    dCreatePlane(space, 0, 0, 1, 0);

    // Place bodies well separated to avoid solver instability
    const int N = 8;
    for (int i = 0; i < N; ++i) {
        dBodyID body = dBodyCreate(world);
        dMass mass;
        dMassSetBox(&mass, 1.0, 0.5, 0.5, 0.5);
        dBodySetMass(body, &mass);
        dBodySetPosition(body, (dReal)(i % 4) * 3.0, (dReal)(i / 4) * 3.0, 2.0);
        dGeomID geom = dCreateBox(space, 0.5, 0.5, 0.5);
        dGeomSetBody(geom, body);
    }

    CollisionCounter cc;
    cc.contactGroup = contactGroup;
    cc.world = world;

    for (int step = 0; step < 500; ++step) {
        cc.contactCount = 0;
        dSpaceCollide(space, &cc, &nearCallback);
        dWorldStep(world, 0.005);
        dJointGroupEmpty(contactGroup);
    }

    // If we get here without crashing, the threading is stable
    CHECK(true);
}

} // SUITE ThreadedWorldStep


// ---------------------------------------------------------------------------
// SUITE: ManualThreadCleanup
// Test the manual thread cleanup mode
// ---------------------------------------------------------------------------
SUITE(ManualThreadCleanup)
{

TEST(ManualCleanupMode)
{
    // dInitODE is already called without manual cleanup in main().
    // We can still test the space manual cleanup setting.
    dSpaceID s = dHashSpaceCreate(0);

    dSpaceSetManualCleanup(s, 1);
    CHECK_EQUAL(1, dSpaceGetManualCleanup(s));

    dSpaceSetManualCleanup(s, 0);
    CHECK_EQUAL(0, dSpaceGetManualCleanup(s));

    dSpaceDestroy(s);
}

} // SUITE ManualThreadCleanup
