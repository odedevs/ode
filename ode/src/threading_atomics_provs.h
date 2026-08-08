/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * Threading atomics providers file.                                     *
 * Copyright (C) 2011-2025 Oleh Derevenko. All rights reserved.          *
 * e-mail: odar@eleks.com (change all "a" to "e")                        *
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
 *  C++ standard atomics provider for built-in threading support provider.
 *
 *  The classes have been moved into a separate header as they are to be used 
 *  in both WIN and POSIX implementations.
 */


#ifndef _ODE_THREADING_ATOMICS_PROVS_H_
#define _ODE_THREADING_ATOMICS_PROVS_H_


#include <atomic>
#include <cstdint>
#include <cstddef>
#include <ode/odeconfig.h>
#include <ode/error.h>
#include "typedefs.h"


/************************************************************************/
/* std::atomic based atomics provider class implementation              */
/************************************************************************/

class dxStdAtomicsProvider
{
public:
    typedef std::atomic<uint32_t> atomicord_t;
    typedef std::atomic<void *> atomicptr_t;

public:
    static void IncrementTargetNoRet(atomicord_t *value_accumulator_ptr)
    {
        value_accumulator_ptr->fetch_add(1);
    }

    static void DecrementTargetNoRet(atomicord_t *value_accumulator_ptr)
    {
        value_accumulator_ptr->fetch_sub(1);
    }

    static uint32_t UnorderedQueryTargetValue(const atomicord_t *value_storage_ptr)
    {
        return value_storage_ptr->load(std::memory_order_relaxed);
    }

    static uint32_t QueryTargetValue(const atomicord_t *value_storage_ptr)
    {
        return value_storage_ptr->load();
    }

    template<unsigned type_size>
    static sizeint AddValueToTarget(void *value_accumulator_ptr, diffint value_addend);

    static bool CompareExchangeTargetValue(atomicord_t *value_storage_ptr,
        uint32_t comparand_value, uint32_t new_value)
    {
        return value_storage_ptr->compare_exchange_strong(comparand_value, new_value);
    }

    static void *UnorderedQueryTargetPtr(const atomicptr_t *pointer_storage_ptr)
    {
        return pointer_storage_ptr->load(std::memory_order_relaxed);
    }

    static bool CompareExchangeTargetPtr(atomicptr_t *pointer_storage_ptr,
        void *comparand_value, void *new_value)
    {
        return pointer_storage_ptr->compare_exchange_strong(comparand_value, new_value);
    }
};

template<>
inline sizeint dxStdAtomicsProvider::AddValueToTarget<sizeof(uint32_t)>(void *value_accumulator_ptr, diffint value_addend)
{
    std::atomic<uint32_t> *ptr = static_cast<std::atomic<uint32_t> *>(value_accumulator_ptr);
    return ptr->fetch_add(static_cast<uint32_t>(value_addend));
}

template<>
inline sizeint dxStdAtomicsProvider::AddValueToTarget<2 * sizeof(uint32_t)>(void *value_accumulator_ptr, diffint value_addend)
{
    std::atomic<sizeint> *ptr = static_cast<std::atomic<sizeint> *>(value_accumulator_ptr);
    return ptr->fetch_add(static_cast<sizeint>(value_addend));
}

// Legacy aliases — both self-threaded and multi-threaded paths now use std::atomic
typedef dxStdAtomicsProvider dxFakeAtomicsProvider;

#if dBUILTIN_THREADING_IMPL_ENABLED
typedef dxStdAtomicsProvider dxOUAtomicsProvider;
#endif // #if dBUILTIN_THREADING_IMPL_ENABLED


#endif // #ifndef _ODE_THREADING_ATOMICS_PROVS_H_
