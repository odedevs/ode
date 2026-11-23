#include <UnitTest++.h>
#include <ode/ode.h>

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

// dxHashSpace reuses its collide() scratch buffers across calls (and, when
// built with ALLOW_HASH_SPACE_SHRINKING, periodically shrinks them). These
// tests check that the reported pairs stay correct while the buffers are
// grown, reused and shrunk, by comparing against a simple space holding an
// identical scene.

namespace {

typedef std::set< std::pair<int, int> > PairSet;

// Deterministic pseudo-random numbers, independent of the platform's rand().
struct Lcg {
    unsigned long state;
    explicit Lcg(unsigned long seed): state(seed) {}
    dReal uniform(dReal lo, dReal hi)
    {
        state = (state * 1103515245UL + 12345UL) & 0x7fffffffUL;
        return lo + (hi - lo) * dReal(state) / dReal(0x7fffffff);
    }
};

int geomIndex(dGeomID g)
{
    return (int)(dsizeint)dGeomGetData(g);
}

void collectPair(void *data, dGeomID g1, dGeomID g2)
{
    int a = geomIndex(g1), b = geomIndex(g2);
    static_cast<PairSet *>(data)->insert(std::make_pair(std::min(a, b), std::max(a, b)));
}

PairSet collidePairs(dSpaceID space)
{
    PairSet pairs;
    dSpaceCollide(space, &pairs, &collectPair);
    return pairs;
}

// The same scene mirrored into a hash space and a simple space.
struct MirroredScene {
    dSpaceID hash;
    dSpaceID simple;
    std::vector< std::pair<dGeomID, dGeomID> > boxes;
    int next_index;
    Lcg rng;

    MirroredScene(): hash(dHashSpaceCreate(0)), simple(dSimpleSpaceCreate(0)),
        next_index(0), rng(12345)
    {
        // an infinite plane always lands in the hash space's "big boxes" list
        dGeomID hp = dCreatePlane(hash, 0, 0, 1, 0);
        dGeomID sp = dCreatePlane(simple, 0, 0, 1, 0);
        dGeomSetData(hp, (void *)(dsizeint)next_index);
        dGeomSetData(sp, (void *)(dsizeint)next_index);
        ++next_index;
    }

    ~MirroredScene()
    {
        dSpaceDestroy(hash);    // cleanup mode destroys the contained geoms
        dSpaceDestroy(simple);
    }

    void resize(dsizeint target, dReal extent)
    {
        while (boxes.size() > target) {
            dGeomDestroy(boxes.back().first);
            dGeomDestroy(boxes.back().second);
            boxes.pop_back();
        }
        while (boxes.size() < target) {
            dReal lx = rng.uniform(0.1, 2), ly = rng.uniform(0.1, 2), lz = rng.uniform(0.1, 2);
            dGeomID hb = dCreateBox(hash, lx, ly, lz);
            dGeomID sb = dCreateBox(simple, lx, ly, lz);
            dGeomSetData(hb, (void *)(dsizeint)next_index);
            dGeomSetData(sb, (void *)(dsizeint)next_index);
            ++next_index;
            boxes.push_back(std::make_pair(hb, sb));
        }
        for (dsizeint i = 0; i < boxes.size(); ++i) {
            dReal x = rng.uniform(-extent, extent);
            dReal y = rng.uniform(-extent, extent);
            dReal z = rng.uniform(-1, extent);
            dGeomSetPosition(boxes[i].first, x, y, z);
            dGeomSetPosition(boxes[i].second, x, y, z);
        }
    }

    void jitter()
    {
        for (dsizeint i = 0; i < boxes.size(); ++i) {
            const dReal *p = dGeomGetPosition(boxes[i].first);
            dReal x = p[0] + rng.uniform(-0.1, 0.1);
            dReal y = p[1] + rng.uniform(-0.1, 0.1);
            dReal z = p[2] + rng.uniform(-0.1, 0.1);
            dGeomSetPosition(boxes[i].first, x, y, z);
            dGeomSetPosition(boxes[i].second, x, y, z);
        }
    }
};

} // namespace


TEST(test_hash_space_matches_simple_space_while_scratch_grows_and_shrinks)
{
    MirroredScene scene;

    // Grow, collapse, empty and regrow the scene. More than 256 calls in total
    // with long stretches at low counts, so that a shrinking build actually
    // releases memory and then has to grow again.
    struct Phase { dsizeint boxes; dReal extent; int frames; };
    const Phase phases[] = {
        { 400, 12, 40 },
        {   3,  2, 300 },
        {   0,  2, 20 },
        { 250,  8, 300 },
        {  20,  4, 10 },
        { 600, 14, 20 },
    };

    int frames_with_pairs = 0;
    for (dsizeint p = 0; p < sizeof(phases) / sizeof(phases[0]); ++p) {
        scene.resize(phases[p].boxes, phases[p].extent);
        for (int f = 0; f < phases[p].frames; ++f) {
            const PairSet expected = collidePairs(scene.simple);
            const PairSet actual = collidePairs(scene.hash);
            CHECK(expected == actual);
            if (!expected.empty()) ++frames_with_pairs;
            scene.jitter();
        }
    }
    // sanity: the scenes were dense enough to exercise the pair logic
    CHECK(frames_with_pairs > 100);
}


TEST(test_hash_space_disabled_geoms_are_skipped_with_reused_scratch)
{
    MirroredScene scene;
    scene.resize(100, 6);

    for (int f = 0; f < 20; ++f) {
        // toggle a changing subset of boxes between frames
        for (dsizeint i = 0; i < scene.boxes.size(); ++i) {
            if ((i + f) % 3 == 0) {
                dGeomDisable(scene.boxes[i].first);
                dGeomDisable(scene.boxes[i].second);
            } else {
                dGeomEnable(scene.boxes[i].first);
                dGeomEnable(scene.boxes[i].second);
            }
        }
        CHECK(collidePairs(scene.simple) == collidePairs(scene.hash));
        scene.jitter();
    }
}


namespace {

struct ReentrantState {
    dSpaceID space;
    PairSet outer;
    PairSet inner;
    bool reentered;
};

void reentrantCallback(void *data, dGeomID g1, dGeomID g2)
{
    ReentrantState *state = static_cast<ReentrantState *>(data);
    if (!state->reentered) {
        // collide the space we are currently being called from
        state->reentered = true;
        dSpaceCollide(state->space, &state->inner, &collectPair);
    }
    collectPair(&state->outer, g1, g2);
}

} // namespace


TEST(test_hash_space_reentrant_collide_does_not_clobber_outer_call)
{
    MirroredScene scene;
    scene.resize(200, 8);

    const PairSet expected = collidePairs(scene.simple);
    CHECK(expected.size() > 1);

    ReentrantState state;
    state.space = scene.hash;
    state.reentered = false;
    dSpaceCollide(scene.hash, &state, &reentrantCallback);

    CHECK(state.reentered);
    CHECK(expected == state.inner);
    CHECK(expected == state.outer);

    // and the space still works normally afterwards
    CHECK(expected == collidePairs(scene.hash));
}
