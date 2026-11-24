/*************************************************************************
*                                                                       *
* OU library interface file for Open Dynamics Engine,                   *
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

ODE utility types: enum-indexed arrays and simple bitflags.

*/


#ifndef _ODE_ODEOU_H_
#define _ODE_ODEOU_H_

#include <cstdint>


//****************************************************************************
// Enum-indexed lookup array (unsorted, linear-search decode)

template<typename EnumType, EnumType EnumMax, typename ElementType>
struct CEnumUnsortedElementArray
{
    ElementType data[static_cast<unsigned>(EnumMax)];

    const ElementType &Encode(EnumType index) const
    {
        return data[static_cast<unsigned>(index)];
    }

    EnumType Decode(const ElementType &value) const
    {
        for (unsigned i = 0; i < static_cast<unsigned>(EnumMax); ++i)
        {
            if (data[i] == value) return static_cast<EnumType>(i);
        }
        return static_cast<EnumType>(EnumMax);
    }

    bool IsValidDecode(EnumType val) const
    {
        return static_cast<unsigned>(val) != static_cast<unsigned>(EnumMax);
    }

    const ElementType *GetStoragePointer() const
    {
        return data;
    }
};


//****************************************************************************
// Enum-indexed lookup array (sorted, binary-search decode)

template<typename EnumType, EnumType EnumMax, typename ElementType>
struct CEnumSortedElementArray
{
    ElementType data[static_cast<unsigned>(EnumMax)];

    const ElementType &Encode(EnumType index) const
    {
        return data[static_cast<unsigned>(index)];
    }

    EnumType Decode(const ElementType &value) const
    {
        unsigned lo = 0, hi = static_cast<unsigned>(EnumMax);
        while (lo < hi)
        {
            unsigned mid = lo + (hi - lo) / 2;
            if (data[mid] < value) lo = mid + 1;
            else hi = mid;
        }
        if (lo < static_cast<unsigned>(EnumMax) && !(value < data[lo]))
            return static_cast<EnumType>(lo);
        return static_cast<EnumType>(EnumMax);
    }

    bool IsValidDecode(EnumType val) const
    {
        return static_cast<unsigned>(val) != static_cast<unsigned>(EnumMax);
    }

    const ElementType *GetStoragePointer() const
    {
        return data;
    }
};


//****************************************************************************
// Simple bitfield flags wrapper

class CSimpleFlags
{
public:
    typedef std::uint32_t value_type;

    CSimpleFlags(): m_value(0) {}
    explicit CSimpleFlags(value_type v): m_value(v) {}

    void AssignFlagsAllValues(value_type v) { m_value = v; }
    value_type QueryFlagsAllValues() const { return m_value; }

    void SignalFlagsMaskValue(value_type mask) { m_value |= mask; }
    void DropFlagsMaskValue(value_type mask) { m_value &= ~mask; }
    void SetFlagsMaskValue(value_type mask, bool val)
    {
        m_value = val ? (m_value | mask) : (m_value & ~mask);
    }
    bool GetFlagsMaskValue(value_type mask) const { return (m_value & mask) != 0; }

private:
    value_type m_value;
};


#endif // _ODE_ODEOU_H_
