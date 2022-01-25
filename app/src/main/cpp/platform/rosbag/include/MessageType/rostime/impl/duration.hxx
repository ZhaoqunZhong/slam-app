/*****************************************************************************
 *  Orbbec OB_VISLAM
 *  Copyright (C) 2020 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec OB_VISLAM.
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 * parties who have NOT received written authorization from the owner.
 *
 *  Description
 ****************************************************************************/

#ifndef __OB_VISLAM_DURATION_HXX__
#define __OB_VISLAM_DURATION_HXX__

#include "MessageType/rostime/duration.h"

namespace ob_slam
{

template<class T>
DurationBase<T>::DurationBase(int32_t _sec, int32_t _nsec)
    : sec(_sec), nsec(_nsec)
{
    normalizeSecNSecSigned(sec, nsec);
}

template<class T>
T& DurationBase<T>::fromSec(double d)
{
    sec = (int32_t)floor(d);
    nsec = (int32_t)((d - (double)sec)*1000000000);
    return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::fromNSec(int64_t t)
{
    sec  = (int32_t)(t / 1000000000);
    nsec = (int32_t)(t % 1000000000);

    normalizeSecNSecSigned(sec, nsec);

    return *static_cast<T*>(this);
}

template<class T>
T DurationBase<T>::operator+(const T &rhs) const
{
    return T(sec + rhs.sec, nsec + rhs.nsec);
}

template<class T>
T DurationBase<T>::operator*(double scale) const
{
    return T(toSec() * scale);
}

template<class T>
T DurationBase<T>::operator-(const T &rhs) const
{
    return T(sec - rhs.sec, nsec - rhs.nsec);
}

template<class T>
T DurationBase<T>::operator-() const
{
    return T(-sec , -nsec);
}

template<class T>
T& DurationBase<T>::operator+=(const T &rhs)
{
    *this = *this + rhs;
    return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator-=(const T &rhs)
{
    *this += (-rhs);
    return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator*=(double scale)
{
    fromSec(toSec() * scale);
    return *static_cast<T*>(this);
}

template<class T>
bool DurationBase<T>::operator<(const T &rhs) const
{
    if (sec < rhs.sec)
        return true;
    else if (sec == rhs.sec && nsec < rhs.nsec)
        return true;
    return false;
}

template<class T>
bool DurationBase<T>::operator>(const T &rhs) const
{
    if (sec > rhs.sec)
        return true;
    else if (sec == rhs.sec && nsec > rhs.nsec)
        return true;
    return false;
}

template<class T>
bool DurationBase<T>::operator<=(const T &rhs) const
{
    if (sec < rhs.sec)
        return true;
    else if (sec == rhs.sec && nsec <= rhs.nsec)
        return true;
    return false;
}

template<class T>
bool DurationBase<T>::operator>=(const T &rhs) const
{
    if (sec > rhs.sec)
        return true;
    else if (sec == rhs.sec && nsec >= rhs.nsec)
        return true;
    return false;
}

template<class T>
bool DurationBase<T>::operator==(const T &rhs) const
{
    return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T>
bool DurationBase<T>::isZero() const
{
    return sec == 0 && nsec == 0;
}


}

#endif //__OB_VISLAM_DURATION_HXX__
