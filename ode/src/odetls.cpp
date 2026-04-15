/*************************************************************************
 *                                                                       *
 * Thread local storage for Open Dynamics Engine,                        *
 * Copyright (C) 2008-2025 Oleh Derevenko. All rights reserved.          *
 * Email: odar@eleks.com (change all "a" to "e")                         *
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
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

ODE Thread Local Storage — implemented with C++ thread_local.

*/

#include <ode/common.h>
#include "config.h"
#include "odetls.h"
#include "collision_trimesh_internal.h"

#ifdef _WIN32
// Undefine ARRAYSIZE from OPCODE's IceUtils.h before windows.h redefines it
#undef ARRAYSIZE
#include <windows.h>
#endif


//////////////////////////////////////////////////////////////////////////
// Thread-local storage

struct OdeTlsData
{
    OdeTlsSlot slots[OTK__MAX] = {};

    void cleanup()
    {
        for (int i = 0; i < OTK__MAX; ++i)
        {
            if (slots[i].trimeshCache)
            {
                COdeTls::FreeTrimeshCollidersCache(slots[i].trimeshCache);
                slots[i].trimeshCache = nullptr;
            }
        }
    }
};


#ifdef _WIN32

// On Windows, use Fiber-Local Storage (FLS) for reliable per-thread cleanup.
// MinGW's thread_local destructors have known issues with memory corruption
// when threads are created and destroyed rapidly.

static DWORD g_flsIndex = FLS_OUT_OF_INDEXES;

static void NTAPI odeTlsFlsCallback(void *data)
{
    OdeTlsData *tls = static_cast<OdeTlsData *>(data);
    if (tls)
    {
        tls->cleanup();
        delete tls;
    }
}

static OdeTlsData &getOdeTlsDataRef()
{
    OdeTlsData *tls = static_cast<OdeTlsData *>(FlsGetValue(g_flsIndex));
    if (!tls)
    {
        tls = new OdeTlsData();
        FlsSetValue(g_flsIndex, tls);
    }
    return *tls;
}

#else

// Non-Windows: thread_local destructors are reliable on POSIX platforms
struct OdeTlsDataWrapper
{
    OdeTlsData data;
    ~OdeTlsDataWrapper() { data.cleanup(); }
};

static thread_local OdeTlsDataWrapper g_odeTlsWrapper;

static OdeTlsData &getOdeTlsDataRef()
{
    return g_odeTlsWrapper.data;
}

#endif


//////////////////////////////////////////////////////////////////////////
// Private helpers

OdeTlsSlot &COdeTls::getSlot(EODETLSKIND tkTLSKind)
{
    return getOdeTlsDataRef().slots[tkTLSKind];
}


//////////////////////////////////////////////////////////////////////////
// Initialization and finalization

bool COdeTls::Initialize(EODETLSKIND /*tkTLSKind*/)
{
#ifdef _WIN32
    if (g_flsIndex == FLS_OUT_OF_INDEXES)
    {
        g_flsIndex = FlsAlloc(odeTlsFlsCallback);
        if (g_flsIndex == FLS_OUT_OF_INDEXES)
            return false;
    }
#endif
    return true;
}

void COdeTls::Finalize(EODETLSKIND tkTLSKind)
{
    OdeTlsSlot &slot = getSlot(tkTLSKind);

    if (slot.trimeshCache)
    {
        FreeTrimeshCollidersCache(slot.trimeshCache);
        slot.trimeshCache = nullptr;
    }
    slot.allocationFlags = 0;
}

void COdeTls::CleanupForThread()
{
    OdeTlsSlot &slot = getSlot(OTK_MANUALCLEANUP);

    if (slot.trimeshCache)
    {
        FreeTrimeshCollidersCache(slot.trimeshCache);
        slot.trimeshCache = nullptr;
    }
    slot.allocationFlags = 0;
}


//////////////////////////////////////////////////////////////////////////
// Accessors

unsigned COdeTls::GetDataAllocationFlags(EODETLSKIND tkTLSKind)
{
    return getSlot(tkTLSKind).allocationFlags;
}

void COdeTls::SignalDataAllocationFlags(EODETLSKIND tkTLSKind, unsigned uFlagsMask)
{
    getSlot(tkTLSKind).allocationFlags |= uFlagsMask;
}

void COdeTls::DropDataAllocationFlags(EODETLSKIND tkTLSKind, unsigned uFlagsMask)
{
    getSlot(tkTLSKind).allocationFlags &= ~uFlagsMask;
}

TrimeshCollidersCache *COdeTls::GetTrimeshCollidersCache(EODETLSKIND tkTLSKind)
{
    return getSlot(tkTLSKind).trimeshCache;
}


//////////////////////////////////////////////////////////////////////////
// Value modifiers

bool COdeTls::AssignDataAllocationFlags(EODETLSKIND tkTLSKind, unsigned uInitializationFlags)
{
    getSlot(tkTLSKind).allocationFlags = uInitializationFlags;
    return true;
}

bool COdeTls::AssignTrimeshCollidersCache(EODETLSKIND tkTLSKind, TrimeshCollidersCache *pccInstance)
{
    getSlot(tkTLSKind).trimeshCache = pccInstance;
    return true;
}

void COdeTls::DestroyTrimeshCollidersCache(EODETLSKIND tkTLSKind)
{
    OdeTlsSlot &slot = getSlot(tkTLSKind);

    if (slot.trimeshCache)
    {
        FreeTrimeshCollidersCache(slot.trimeshCache);
        slot.trimeshCache = nullptr;
    }
}


//////////////////////////////////////////////////////////////////////////
// Value type destructors

void COdeTls::FreeTrimeshCollidersCache(TrimeshCollidersCache *pccCacheInstance)
{
#if dTRIMESH_ENABLED 
    delete pccCacheInstance;
#else
    dIASSERT(pccCacheInstance == NULL);
#endif
}

