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

#ifndef _ODE_THREADINGUTILS_H_
#define _ODE_THREADINGUTILS_H_


#include <atomic>
#include <cstdint>
#include "typedefs.h"


static inline
bool ThrsafeCompareExchange(std::atomic<uint32_t> *paoDestination, uint32_t aoComparand, uint32_t aoExchange)
{
    return paoDestination->compare_exchange_strong(aoComparand, aoExchange);
}

static inline
uint32_t ThrsafeExchange(std::atomic<uint32_t> *paoDestination, uint32_t aoExchange)
{
    return paoDestination->exchange(aoExchange);
}

static inline
void ThrsafeIncrementNoResult(std::atomic<uint32_t> *paoDestination)
{
    paoDestination->fetch_add(1);
}

static inline
void ThrsafeDecrementNoResult(std::atomic<uint32_t> *paoDestination)
{
    paoDestination->fetch_sub(1);
}

static inline
uint32_t ThrsafeIncrement(std::atomic<uint32_t> *paoDestination)
{
    return paoDestination->fetch_add(1) + 1;
}

static inline
uint32_t ThrsafeDecrement(std::atomic<uint32_t> *paoDestination)
{
    return paoDestination->fetch_sub(1) - 1;
}

static inline
void ThrsafeAdd(std::atomic<uint32_t> *paoDestination, uint32_t aoAddend)
{
    paoDestination->fetch_add(aoAddend);
}

static inline
uint32_t ThrsafeExchangeAdd(std::atomic<uint32_t> *paoDestination, uint32_t aoAddend)
{
    return paoDestination->fetch_add(aoAddend);
}

static inline
bool ThrsafeCompareExchangePointer(std::atomic<void *> *papDestination, void *apComparand, void *apExchange)
{
    return papDestination->compare_exchange_strong(apComparand, apExchange);
}

static inline
void *ThrsafeExchangePointer(std::atomic<void *> *papDestination, void *apExchange)
{
    return papDestination->exchange(apExchange);
}


static inline
unsigned int ThrsafeIncrementIntUpToLimit(std::atomic<uint32_t> *storagePointer, unsigned int limitValue)
{
    unsigned int resultValue;
    while (true) {
        resultValue = storagePointer->load();
        // The ">=" comparison is used here to allow continuing incrementing the destination 
        // without waiting for all the threads to pass the barrier of checking its value
        if (resultValue >= limitValue) {
            resultValue = limitValue;
            break;
        }
        if (storagePointer->compare_exchange_strong(resultValue, resultValue + 1)) {
            break;
        }
    }
    return resultValue;
}

static inline
sizeint ThrsafeIncrementSizeUpToLimit(std::atomic<sizeint> *storagePointer, sizeint limitValue)
{
    sizeint resultValue;
    while (true) {
        resultValue = storagePointer->load();
        // The ">=" comparison is not required here at present ("==" could be used). 
        // It is just used this way to match the other function above.
        if (resultValue >= limitValue) {
            resultValue = limitValue;
            break;
        }
        if (storagePointer->compare_exchange_strong(resultValue, resultValue + 1)) {
            break;
        }
    }
    return resultValue;
}



#endif // _ODE_THREADINGUTILS_H_
