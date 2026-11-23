/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

spaces

*/

#include <vector>

#include <ode/common.h>
#include <ode/collision_space.h>
#include <ode/collision.h>
#include "config.h"
#include "matrix.h"
#include "collision_kernel.h"
#include "collision_space_internal.h"
#include "util.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

//****************************************************************************
// make the geom dirty by setting the GEOM_DIRTY and GEOM_BAD_AABB flags
// and moving it to the front of the space's list. all the parents of a
// dirty geom also become dirty.

void dGeomMoved (dxGeom *geom)
{
    dAASSERT (geom);

    // if geom is offset, mark it as needing a calculate
    if (geom->offset_posr) {
        geom->gflags |= GEOM_POSR_BAD;
    }

    // from the bottom of the space heirarchy up, process all clean geoms
    // turning them into dirty geoms.
    dxSpace *parent = geom->parent_space;

    while (parent && (geom->gflags & GEOM_DIRTY)==0) {
        geom->markAABBBad();
        parent->dirty (geom);
        geom = parent;
        parent = parent->parent_space;
    }

    // all the remaining dirty geoms must have their AABB_BAD flags set, to
    // ensure that their AABBs get recomputed
    while (geom) {
        geom->markAABBBad();
        geom = geom->parent_space;
    }
}

#define GEOM_ENABLED(g) (((g)->gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE)

//****************************************************************************
// dxSpace

dxSpace::dxSpace (dSpaceID _space) : dxGeom (_space,0)
{
    count = 0;
    first = 0;
    cleanup = 1;
    sublevel = 0;
    tls_kind = dSPACE_TLS_KIND_INIT_VALUE;
    current_index = 0;
    current_geom = 0;
    lock_count = 0;
}


dxSpace::~dxSpace()
{
    CHECK_NOT_LOCKED (this);
    if (cleanup) {
        // note that destroying each geom will call remove()
        dxGeom *g,*n;
        for (g = first; g; g=n) {
            n = g->next;
            dGeomDestroy (g);
        }
    }
    else {
        dxGeom *g,*n;
        for (g = first; g; g=n) {
            n = g->next;
            remove (g);
        }
    }
}


void dxSpace::computeAABB()
{
    if (first) {
        int i;
        dReal a[6];
        a[0] = dInfinity;
        a[1] = -dInfinity;
        a[2] = dInfinity;
        a[3] = -dInfinity;
        a[4] = dInfinity;
        a[5] = -dInfinity;
        for (dxGeom *g=first; g; g=g->next) {
            g->recomputeAABB();
            for (i=0; i<6; i += 2) if (g->aabb[i] < a[i]) a[i] = g->aabb[i];
            for (i=1; i<6; i += 2) if (g->aabb[i] > a[i]) a[i] = g->aabb[i];
        }
        memcpy(aabb,a,6*sizeof(dReal));
    }
    else {
        dSetZero (aabb,6);
    }
}


// the dirty geoms are numbered 0..k, the clean geoms are numbered k+1..count-1

dxGeom *dxSpace::getGeom (int i)
{
    dUASSERT (i >= 0 && i < count,"index out of range");
    if (current_geom && current_index == i-1) {
        current_geom = current_geom->next;
        current_index = i;
        return current_geom;
    }
    else {
        dxGeom *g=first;
        for (int j=0; j<i; j++) {
            if (g) g = g->next; else return 0;
        }
        current_geom = g;
        current_index = i;
        return g;
    }
}


void dxSpace::add (dxGeom *geom)
{
    CHECK_NOT_LOCKED (this);
    dAASSERT (geom);
    dUASSERT (geom->parent_space == 0 && geom->next == 0,
        "geom is already in a space");

    // add
    geom->parent_space = this;
    geom->spaceAdd (&first);
    count++;

    // enumerator has been invalidated
    current_geom = 0;

    dGeomMoved (this);
}


void dxSpace::remove (dxGeom *geom)
{
    CHECK_NOT_LOCKED (this);
    dAASSERT (geom);
    dUASSERT (geom->parent_space == this,"object is not in this space");

    // remove
    geom->spaceRemove();
    count--;

    // safeguard
    geom->next = 0;
    geom->tome = 0;
    geom->parent_space = 0;

    // enumerator has been invalidated
    current_geom = 0;

    // the bounding box of this space (and that of all the parents) may have
    // changed as a consequence of the removal.
    dGeomMoved (this);
}


void dxSpace::dirty (dxGeom *geom)
{
    geom->spaceRemove();
    geom->spaceAdd (&first);
}

//****************************************************************************
// simple space - reports all n^2 object intersections

struct dxSimpleSpace : public dxSpace {
    dxSimpleSpace (dSpaceID _space);
    void cleanGeoms();
    void collide (void *data, dNearCallback *callback);
    void collide2 (void *data, dxGeom *geom, dNearCallback *callback);
};


dxSimpleSpace::dxSimpleSpace (dSpaceID _space) : dxSpace (_space)
{
    type = dSimpleSpaceClass;
}


void dxSimpleSpace::cleanGeoms()
{
    // compute the AABBs of all dirty geoms, and clear the dirty flags
    lock_count++;
    for (dxGeom *g=first; g && (g->gflags & GEOM_DIRTY); g=g->next) {
        if (IS_SPACE(g)) {
            ((dxSpace*)g)->cleanGeoms();
        }

        g->recomputeAABB();
        dIASSERT((g->gflags & GEOM_AABB_BAD) == 0);

        g->gflags &= ~GEOM_DIRTY;
    }
    lock_count--;
}


void dxSimpleSpace::collide (void *data, dNearCallback *callback)
{
    dAASSERT (callback);

    lock_count++;
    cleanGeoms();

    // intersect all bounding boxes
    for (dxGeom *g1=first; g1; g1=g1->next) {
        if (GEOM_ENABLED(g1)){
            for (dxGeom *g2=g1->next; g2; g2=g2->next) {
                if (GEOM_ENABLED(g2)){
                    collideAABBs (g1,g2,data,callback);
                }
            }
        }
    }

    lock_count--;
}


void dxSimpleSpace::collide2 (void *data, dxGeom *geom,
                              dNearCallback *callback)
{
    dAASSERT (geom && callback);

    lock_count++;
    cleanGeoms();
    geom->recomputeAABB();

    // intersect bounding boxes
    for (dxGeom *g=first; g; g=g->next) {
        if (GEOM_ENABLED(g)){
            collideAABBs (g,geom,data,callback);
        }
    }

    lock_count--;
}

//****************************************************************************
// utility stuff for hash table space

// kind of silly, but oh well...
#ifndef MAXINT
#define MAXINT ((int)((((unsigned int)(-1)) << 1) >> 1))
#endif


// prime[i] is the largest prime smaller than 2^i
#define NUM_PRIMES 31
static const unsigned long int prime[NUM_PRIMES] = {1L,2L,3L,7L,13L,31L,61L,127L,251L,509L,
1021L,2039L,4093L,8191L,16381L,32749L,65521L,131071L,262139L,
524287L,1048573L,2097143L,4194301L,8388593L,16777213L,33554393L,
67108859L,134217689L,268435399L,536870909L,1073741789L};


// an axis aligned bounding box in the hash table
struct dxAABB {
    int level;		// the level this is stored in (cell size = 2^level)
    int dbounds[6];	// AABB bounds, discretized to cell size
    dxGeom *geom;		// corresponding geometry object (AABB stored here)
    sizeint index;		// index of this AABB, starting from 0
};


// a hash table node that represents an AABB that intersects a particular cell
// at a particular level
struct Node {
    Node *next;		// next node in hash table collision list, 0 if none
    int x,y,z;		// cell position in space, discretized to cell size
    dxAABB *aabb;		// axis aligned bounding box that intersects this cell
};


// Opt-in periodic shrinking of the hash space scratch buffers.
//
// By default a hash space keeps its collide() scratch buffers at their
// high-water mark until the space is destroyed. This avoids any heap traffic
// in steady state, but a single spike in geom count pins that memory forever
// (the `tested' bit matrix alone is n*n/8 bytes). Define
// ALLOW_HASH_SPACE_SHRINKING to have the space periodically release capacity
// it has not needed recently:
//
//   every HASH_SPACE_SHRINK_INTERVAL collide() calls, any buffer whose
//   capacity exceeds HASH_SPACE_SHRINK_FACTOR times its peak use during that
//   window (and is at least HASH_SPACE_SHRINK_MIN_BYTES large) is reallocated
//   down to that peak.
//
// All three tunables may be overridden on the compiler command line.
#ifdef ALLOW_HASH_SPACE_SHRINKING
#ifndef HASH_SPACE_SHRINK_INTERVAL
#define HASH_SPACE_SHRINK_INTERVAL 256
#endif
#ifndef HASH_SPACE_SHRINK_FACTOR
#define HASH_SPACE_SHRINK_FACTOR 4
#endif
#ifndef HASH_SPACE_SHRINK_MIN_BYTES
#define HASH_SPACE_SHRINK_MIN_BYTES 4096
#endif
#endif // ALLOW_HASH_SPACE_SHRINKING


// Scratch memory used by dxHashSpace::collide(). A hash space owns one of
// these so that, once the buffers have grown to fit the scene, collide() does
// not touch the heap at all. Contents are only meaningful during a call.
struct dxHashSpaceScratch {
    std::vector<dxAABB> hash_boxes;	// AABBs in the hash table
    std::vector<dxAABB> big_boxes;	// AABBs too big for the hash table
    std::vector<uint8> tested;		// n*n bit matrix of tested pairs
    std::vector<Node*> table;		// hash table buckets
    std::vector<Node> nodes;		// (AABB, cell) nodes; must not reallocate
                                        // while the table points into it

#ifdef ALLOW_HASH_SPACE_SHRINKING
    // peak element counts seen during the current shrink window
    sizeint peak_hash_boxes, peak_big_boxes, peak_tested, peak_table, peak_nodes;
    unsigned calls_in_window;

    dxHashSpaceScratch():
        peak_hash_boxes(0), peak_big_boxes(0), peak_tested(0), peak_table(0),
        peak_nodes(0), calls_in_window(0) {}

    // Record the usage of the call that just finished and, at the end of each
    // window, release capacity that went unused during the whole window.
    void endCall()
    {
        recordPeak(peak_hash_boxes, hash_boxes.size());
        recordPeak(peak_big_boxes, big_boxes.size());
        recordPeak(peak_tested, tested.size());
        recordPeak(peak_table, table.size());
        recordPeak(peak_nodes, nodes.size());

        if (++calls_in_window < HASH_SPACE_SHRINK_INTERVAL) return;

        shrinkToPeak(hash_boxes, peak_hash_boxes);
        shrinkToPeak(big_boxes, peak_big_boxes);
        shrinkToPeak(tested, peak_tested);
        shrinkToPeak(table, peak_table);
        shrinkToPeak(nodes, peak_nodes);

        peak_hash_boxes = peak_big_boxes = peak_tested = peak_table = peak_nodes = 0;
        calls_in_window = 0;
    }

private:
    static void recordPeak(sizeint &peak, sizeint used)
    {
        if (used > peak) peak = used;
    }

    // Only called between collide() calls, so contents can be discarded.
    template <class T>
    static void shrinkToPeak(std::vector<T> &v, sizeint peak)
    {
        if (v.capacity() * sizeof(T) < HASH_SPACE_SHRINK_MIN_BYTES) return;
        if (v.capacity() <= peak * HASH_SPACE_SHRINK_FACTOR) return;
        // shrink_to_fit() is non-binding; swapping is guaranteed to release.
        std::vector<T> resized;
        resized.reserve(peak);
        v.swap(resized);
    }
#endif // ALLOW_HASH_SPACE_SHRINKING
};


// return the `level' of an AABB. the AABB will be put into cells at this
// level - the cell size will be 2^level. the level is chosen to be the
// smallest value such that the AABB occupies no more than 8 cells, regardless
// of its placement. this means that:
//	size/2 < q <= size
// where q is the maximum AABB dimension.

static int findLevel (dReal bounds[6])
{
    if (bounds[0] <= -dInfinity || bounds[1] >= dInfinity ||
        bounds[2] <= -dInfinity || bounds[3] >= dInfinity ||
        bounds[4] <= -dInfinity || bounds[5] >= dInfinity) {
            return MAXINT;
    }

    // compute q
    dReal q,q2;
    q = bounds[1] - bounds[0];	// x bounds
    q2 = bounds[3] - bounds[2];	// y bounds
    if (q2 > q) q = q2;
    q2 = bounds[5] - bounds[4];	// z bounds
    if (q2 > q) q = q2;

    // find level such that 0.5 * 2^level < q <= 2^level
    int level;
    frexp (q,&level);	// q = (0.5 .. 1.0) * 2^level (definition of frexp)
    return level;
}


// find a virtual memory address for a cell at the given level and x,y,z
// position.
// @@@ currently this is not very sophisticated, e.g. the scaling
// factors could be better designed to avoid collisions, and they should
// probably depend on the hash table physical size.

static unsigned long getVirtualAddressBase (unsigned int level, unsigned int x, unsigned int y)
{
    return level * 1000UL + x * 100UL + y * 10UL;
}

//****************************************************************************
// hash space

struct dxHashSpace : public dxSpace {
    int global_minlevel;	// smallest hash table level to put AABBs in
    int global_maxlevel;	// objects that need a level larger than this will be
    // put in a "big objects" list instead of a hash table

    // Reused across collide() calls. Not safe for concurrent collide() calls
    // on the same space from multiple threads (each space must be collided by
    // one thread at a time).
    dxHashSpaceScratch scratch;
    // true while a collide() call is using `scratch'. A near callback that
    // re-enters collide() on this same space gets temporary buffers instead.
    bool scratch_in_use;

    dxHashSpace (dSpaceID _space);
    void setLevels (int minlevel, int maxlevel);
    void getLevels (int *minlevel, int *maxlevel);
    void cleanGeoms();
    void collide (void *data, dNearCallback *callback);
    void collide2 (void *data, dxGeom *geom, dNearCallback *callback);

private:
    void collideWithScratch (dxHashSpaceScratch &s, void *data, dNearCallback *callback);
};


dxHashSpace::dxHashSpace (dSpaceID _space) : dxSpace (_space)
{
    type = dHashSpaceClass;
    global_minlevel = -3;
    global_maxlevel = 10;
    scratch_in_use = false;
}


void dxHashSpace::setLevels (int minlevel, int maxlevel)
{
    dAASSERT (minlevel <= maxlevel);
    global_minlevel = minlevel;
    global_maxlevel = maxlevel;
}


void dxHashSpace::getLevels (int *minlevel, int *maxlevel)
{
    if (minlevel) *minlevel = global_minlevel;
    if (maxlevel) *maxlevel = global_maxlevel;
}


void dxHashSpace::cleanGeoms()
{
    // compute the AABBs of all dirty geoms, and clear the dirty flags
    lock_count++;
    for (dxGeom *g=first; g && (g->gflags & GEOM_DIRTY); g=g->next) {
        if (IS_SPACE(g)) {
            ((dxSpace*)g)->cleanGeoms();
        }

        g->recomputeAABB();
        dIASSERT((g->gflags & GEOM_AABB_BAD) == 0);
        
        g->gflags &= ~GEOM_DIRTY;
    }
    lock_count--;
}


void dxHashSpace::collide (void *data, dNearCallback *callback)
{
    dAASSERT(this && callback);

    if (scratch_in_use) {
        // re-entered from a near callback of an outer collide() on this same
        // space: the outer call's nodes still point into `scratch', so use
        // throwaway buffers rather than clobbering them
        dxHashSpaceScratch temporary;
        collideWithScratch (temporary, data, callback);
        return;
    }

    scratch_in_use = true;
    collideWithScratch (scratch, data, callback);
    scratch_in_use = false;

#ifdef ALLOW_HASH_SPACE_SHRINKING
    scratch.endCall();
#endif
}


void dxHashSpace::collideWithScratch (dxHashSpaceScratch &s, void *data,
                                      dNearCallback *callback)
{
    dxGeom *geom;
    int i,maxlevel;

    // empty the scratch buffers (keeping their capacity) up front, so that
    // the early return below also reports zero usage for this call
    s.hash_boxes.clear();
    s.big_boxes.clear();
    s.tested.clear();
    s.table.clear();
    s.nodes.clear();

    // 0 or 1 geoms can't collide with anything
    if (count < 2) return;

    lock_count++;
    cleanGeoms();

    // create a list of auxiliary information for all geom axis aligned bounding
    // boxes. set the level for all AABBs. put AABBs larger than the space's
    // global_maxlevel in the big_boxes list, check everything else against
    // that list at the end. for AABBs that are not too big, record the maximum
    // level that we need.

    typedef std::vector<dxAABB> AABBlist;
    AABBlist &hash_boxes = s.hash_boxes; // list of AABBs in hash table
    AABBlist &big_boxes = s.big_boxes; // list of AABBs too big for hash table
    // Node::aabb points into hash_boxes, so it must not reallocate below
    hash_boxes.reserve(count);
    // total number of (AABB, cell) hash table nodes, see the node pool below
    sizeint numNodes = 0;
    maxlevel = global_minlevel - 1;
    for (geom = first; geom; geom=geom->next) {
        if (!GEOM_ENABLED(geom)){
            continue;
        }
        dxAABB aabb;
        aabb.geom = geom;
        // compute level, but prevent cells from getting too small
        int level = findLevel (geom->aabb);
        if (level < global_minlevel) level = global_minlevel;
        if (level <= global_maxlevel) {
            aabb.level = level;
            if (level > maxlevel) maxlevel = level;
            // cellsize = 2^level
            dReal cellSizeRecip = dRecip(ldexp(REAL(1.0), level)); // No computational errors here!
            // discretize AABB position to cell size
            for (i=0; i < 6; i++) {
                dReal aabbBound = geom->aabb[i] * cellSizeRecip; // No computational errors so far!
                dICHECK(aabbBound >= dMinIntExact && aabbBound </*=*/ dMaxIntExact); // Otherwise the scene is too large for integer types used 

                aabb.dbounds[i] = (int) dFloor(aabbBound);
            }
            // count the cells this AABB will be inserted into
            numNodes += sizeint(aabb.dbounds[1] - aabb.dbounds[0] + 1) *
                        sizeint(aabb.dbounds[3] - aabb.dbounds[2] + 1) *
                        sizeint(aabb.dbounds[5] - aabb.dbounds[4] + 1);
            // set AABB index
            aabb.index = hash_boxes.size();
            // aabb goes in main list
            hash_boxes.push_back(aabb);
        }
        else {
            // aabb is too big, put it in the big_boxes list. we don't care about
            // setting level, dbounds, index, or the maxlevel
            big_boxes.push_back(aabb);
        }
    }

    sizeint n = hash_boxes.size(); // number of AABBs in main list

    // for `n' objects, an n*n array of bits is used to record if those objects
    // have been intersection-tested against each other yet. this array can
    // grow large with high n, but oh well...
    int tested_rowsize = (n+7) >> 3;	// number of bytes needed for n bits
    std::vector<uint8> &tested = s.tested;
    tested.assign(n * tested_rowsize, 0);

    // create a hash table to store all AABBs. each AABB may take up to 8 cells.
    // we use chaining to resolve collisions, but we use a relatively large table
    // to reduce the chance of collisions.

    // compute hash table size sz to be a prime > 8*n
    for (i=0; i<NUM_PRIMES; i++) {
        if ((sizeint)prime[i] >= (8*n)) break;
    }
    if (i >= NUM_PRIMES) {
        i = NUM_PRIMES-1;	// probably pointless
    }

    const unsigned long sz = prime[i];

    // allocate and initialize hash table node pointers
    typedef std::vector<Node*> HashTable;
    HashTable &table = s.table;
    table.assign(sz, (Node*)0);

    // All nodes come from one block owned by the space instead of one heap
    // allocation per (AABB, cell) on every collide() call. numNodes was
    // counted while discretizing the AABBs above so that the block never
    // reallocates (nodes point at each other); that count must match the
    // cells visited by the insertion loop below. Each AABB covers at most 2
    // cells per axis (findLevel() picks a cell at least as big as the box),
    // so numNodes <= 8 * hash_boxes.size().
    std::vector<Node> &nodes = s.nodes;
    nodes.reserve(numNodes);

    // add each AABB to the hash table (may need to add it to up to 8 cells)
    const AABBlist::iterator hashend = hash_boxes.end();
    for (AABBlist::iterator aabb = hash_boxes.begin(); aabb != hashend; ++aabb) {
        const int *dbounds = aabb->dbounds;
        const int xend = dbounds[1];
        for (int xi = dbounds[0]; xi <= xend; xi++) {
            const int yend = dbounds[3];
            for (int yi = dbounds[2]; yi <= yend; yi++) {
                int zbegin = dbounds[4];
                unsigned long hi = (getVirtualAddressBase(aabb->level,xi,yi) + zbegin) % sz;
                const int zend = dbounds[5];
                for (int zi = zbegin; zi <= zend; (hi = hi + 1U != sz ? hi + 1U : 0UL), zi++) {
                    // get the hash index
                    // add a new node to the hash table
                    nodes.push_back(Node());
                    Node *node = &nodes.back();
                    node->x = xi;
                    node->y = yi;
                    node->z = zi;
                    node->aabb = &*aabb;
                    node->next = table[hi];
                    table[hi] = node;
                }
            }
        }
    }
    // a mismatch with the counting loop would have reallocated `nodes' and
    // left every node->next dangling
    dIASSERT(nodes.size() == numNodes);

    // now that all AABBs are loaded into the hash table, we do the actual
    // collision detection. for all AABBs, check for other AABBs in the
    // same cells for collisions, and then check for other AABBs in all
    // intersecting higher level cells.

    int db[6];			// discrete bounds at current level
    for (AABBlist::iterator aabb = hash_boxes.begin(); aabb != hashend; ++aabb) {
        // we are searching for collisions with aabb
        for (i=0; i<6; i++) db[i] = aabb->dbounds[i];
        for (int level = aabb->level; ; ) {
            dIASSERT(level <= maxlevel);
            const int xend = db[1];
            for (int xi = db[0]; xi <= xend; xi++) {
                const int yend = db[3];
                for (int yi = db[2]; yi <= yend; yi++) {
                    int zbegin = db[4];
                    // get the hash index
                    unsigned long hi = (getVirtualAddressBase(level, xi, yi) + zbegin) % sz;
                    const int zend = db[5];
                    for (int zi = zbegin; zi <= zend; (hi = hi + 1U != sz ? hi + 1U : 0UL), zi++) {
                        // search all nodes at this index
                        for (Node* node = table[hi]; node; node=node->next) {
                            // node points to an AABB that may intersect aabb
                            if (node->aabb == &*aabb)
                                continue;
                            if (node->aabb->level == level &&
                                node->x == xi && node->y == yi && node->z == zi) {
                                    // see if aabb and node->aabb have already been tested
                                    // against each other
                                    unsigned char mask;
                                    if (aabb->index <= node->aabb->index) {
                                        i = (aabb->index * tested_rowsize)+(node->aabb->index >> 3);
                                        mask = 1 << (node->aabb->index & 7);
                                    }
                                    else {
                                        i = (node->aabb->index * tested_rowsize)+(aabb->index >> 3);
                                        mask = 1 << (aabb->index & 7);
                                    }
                                    dIASSERT (i >= 0 && (sizeint)i < (tested_rowsize*n));
                                    if ((tested[i] & mask)==0) {
                                        tested[i] |= mask;
                                        collideAABBs (aabb->geom,node->aabb->geom,data,callback);
                                    }
                            }
                        }
                    }
                }
            }

            if (level == maxlevel) {
                break;
            }
            ++level;
            // get the discrete bounds for the next level up
            for (i=0; i<6; i++) db[i] >>= 1;
        }
    }

    // every AABB in the normal list must now be intersected against every
    // AABB in the big_boxes list. so let's hope there are not too many objects
    // in the big_boxes list.
    const AABBlist::iterator bigend = big_boxes.end();
    for (AABBlist::iterator aabb = hash_boxes.begin(); aabb != hashend; ++aabb) {
        for (AABBlist::iterator aabb2 = big_boxes.begin(); aabb2 != bigend; ++aabb2) {
            collideAABBs (aabb->geom, aabb2->geom, data, callback);
        }
    }

    // intersected all AABBs in the big_boxes list together
    for (AABBlist::iterator aabb = big_boxes.begin(); aabb != bigend; ++aabb) {
        AABBlist::iterator aabb2 = aabb;
        while (++aabb2 != bigend) {
            collideAABBs (aabb->geom, aabb2->geom, data, callback);
        }
    }

    lock_count--;
}


void dxHashSpace::collide2 (void *data, dxGeom *geom,
                            dNearCallback *callback)
{
    dAASSERT (geom && callback);

    // this could take advantage of the hash structure to avoid
    // O(n2) complexity, but it does not yet.

    lock_count++;
    cleanGeoms();
    geom->recomputeAABB();

    // intersect bounding boxes
    for (dxGeom *g=first; g; g=g->next) {
        if (GEOM_ENABLED(g)) collideAABBs (g,geom,data,callback);
    }

    lock_count--;
}

//****************************************************************************
// space functions

dxSpace *dSimpleSpaceCreate (dxSpace *space)
{
    return new dxSimpleSpace (space);
}


dxSpace *dHashSpaceCreate (dxSpace *space)
{
    return new dxHashSpace (space);
}


void dHashSpaceSetLevels (dxSpace *space, int minlevel, int maxlevel)
{
    dAASSERT (space);
    dUASSERT (minlevel <= maxlevel,"must have minlevel <= maxlevel");
    dUASSERT (space->type == dHashSpaceClass,"argument must be a hash space");
    dxHashSpace *hspace = (dxHashSpace*) space;
    hspace->setLevels (minlevel,maxlevel);
}


void dHashSpaceGetLevels (dxSpace *space, int *minlevel, int *maxlevel)
{
    dAASSERT (space);
    dUASSERT (space->type == dHashSpaceClass,"argument must be a hash space");
    dxHashSpace *hspace = (dxHashSpace*) space;
    hspace->getLevels (minlevel,maxlevel);
}


void dSpaceDestroy (dxSpace *space)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    dGeomDestroy (space);
}


void dSpaceSetCleanup (dxSpace *space, int mode)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    space->setCleanup (mode);
}


int dSpaceGetCleanup (dxSpace *space)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->getCleanup();
}


void dSpaceSetSublevel (dSpaceID space, int sublevel)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    space->setSublevel (sublevel);
}


int dSpaceGetSublevel (dSpaceID space)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->getSublevel();
}

void dSpaceSetManualCleanup (dSpaceID space, int mode)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    space->setManulCleanup(mode);
}

int dSpaceGetManualCleanup (dSpaceID space)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->getManualCleanup();
}

void dSpaceAdd (dxSpace *space, dxGeom *g)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    CHECK_NOT_LOCKED (space);
    space->add (g);
}


void dSpaceRemove (dxSpace *space, dxGeom *g)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    CHECK_NOT_LOCKED (space);
    space->remove (g);
}


int dSpaceQuery (dxSpace *space, dxGeom *g)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->query (g);
}

void dSpaceClean (dxSpace *space){
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");

    space->cleanGeoms();
}

int dSpaceGetNumGeoms (dxSpace *space)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->getNumGeoms();
}


dGeomID dSpaceGetGeom (dxSpace *space, int i)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->getGeom (i);
}

int dSpaceGetClass (dxSpace *space)
{
    dAASSERT (space);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    return space->type;
}


void dSpaceCollide (dxSpace *space, void *data, dNearCallback *callback)
{
    dAASSERT (space && callback);
    dUASSERT (dGeomIsSpace(space),"argument not a space");
    space->collide (data,callback);
}


struct DataCallback {
    void *data;
    dNearCallback *callback;
};
// Invokes the callback with arguments swapped
static void swap_callback(void *data, dxGeom *g1, dxGeom *g2)
{
    DataCallback *dc = (DataCallback*)data;
    dc->callback(dc->data, g2, g1);
}


void dSpaceCollide2 (dxGeom *g1, dxGeom *g2, void *data,
                     dNearCallback *callback)
{
    dAASSERT (g1 && g2 && callback);
    dxSpace *s1,*s2;

    // see if either geom is a space
    if (IS_SPACE(g1)) s1 = (dxSpace*) g1; else s1 = 0;
    if (IS_SPACE(g2)) s2 = (dxSpace*) g2; else s2 = 0;

    if (s1 && s2) {
        int l1 = s1->getSublevel();
        int l2 = s2->getSublevel();
        if (l1 != l2) {
            if (l1 > l2) {
                s2 = 0;
            } else {
                s1 = 0;
            }
        }
    }

    // handle the four space/geom cases
    if (s1) {
        if (s2) {
            // g1 and g2 are spaces.
            if (s1==s2) {
                // collide a space with itself --> interior collision
                s1->collide (data,callback);
            }
            else {
                // iterate through the space that has the fewest geoms, calling
                // collide2 in the other space for each one.
                if (s1->count < s2->count) {
                    DataCallback dc = {data, callback};
                    for (dxGeom *g = s1->first; g; g=g->next) {
                        s2->collide2 (&dc,g,swap_callback);
                    }
                }
                else {
                    for (dxGeom *g = s2->first; g; g=g->next) {
                        s1->collide2 (data,g,callback);
                    }
                }
            }
        }
        else {
            // g1 is a space, g2 is a geom
            s1->collide2 (data,g2,callback);
        }
    }
    else {
        if (s2) {
            // g1 is a geom, g2 is a space
            DataCallback dc = {data, callback};
            s2->collide2 (&dc,g1,swap_callback);
        }
        else {
            // g1 and g2 are geoms
            // make sure they have valid AABBs
            g1->recomputeAABB();
            g2->recomputeAABB();
            collideAABBs(g1,g2, data, callback);
        }
    }
}
