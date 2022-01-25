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
#include "MessageType/rostime/rate.h"

namespace ob_slam {
Rate::Rate(double frequency)
    : start_(Time::now())
    , expected_cycle_time_(1.0 / frequency)
    , actual_cycle_time_(0.0)
{
}

Rate::Rate(const Duration& d)
    : start_(Time::now())
    , expected_cycle_time_(d.sec, d.nsec)
    , actual_cycle_time_(0.0)
{
}

bool Rate::sleep()
{
    Time expected_end = start_ + expected_cycle_time_;

    Time actual_end = Time::now();

    // detect backward jumps in time
    if (actual_end < start_) {
        expected_end = actual_end + expected_cycle_time_;
    }

    //calculate the time we'll sleep for
    Duration sleep_time = expected_end - actual_end;

    //set the actual amount of time the loop took in case the user wants to know
    actual_cycle_time_ = actual_end - start_;

    //make sure to reset our start time
    start_ = expected_end;

    //if we've taken too much time we won't sleep
    if (sleep_time <= Duration(0.0)) {
        // if we've jumped forward in time, or the loop has taken more than a full extra
        // cycle, reset our cycle
        if (actual_end > expected_end + expected_cycle_time_) {
            start_ = actual_end;
        }
        // return false to show that the desired rate was not met
        return false;
    }

    return sleep_time.sleep();
}

void Rate::reset()
{
    start_ = Time::now();
}

Duration Rate::cycleTime() const
{
    return actual_cycle_time_;
}

WallRate::WallRate(double frequency)
    : start_(WallTime::now())
    , expected_cycle_time_(1.0 / frequency)
    , actual_cycle_time_(0.0)
{
}

WallRate::WallRate(const Duration& d)
    : start_(WallTime::now())
    , expected_cycle_time_(d.sec, d.nsec)
    , actual_cycle_time_(0.0)
{
}

bool WallRate::sleep()
{
    WallTime expected_end = start_ + expected_cycle_time_;

    WallTime actual_end = WallTime::now();

    // detect backward jumps in time
    if (actual_end < start_) {
        expected_end = actual_end + expected_cycle_time_;
    }

    //calculate the time we'll sleep for
    WallDuration sleep_time = expected_end - actual_end;

    //set the actual amount of time the loop took in case the user wants to know
    actual_cycle_time_ = actual_end - start_;

    //make sure to reset our start time
    start_ = expected_end;

    //if we've taken too much time we won't sleep
    if (sleep_time <= WallDuration(0.0)) {
        // if we've jumped forward in time, or the loop has taken more than a full extra
        // cycle, reset our cycle
        if (actual_end > expected_end + expected_cycle_time_) {
            start_ = actual_end;
        }
        return false;
    }

    return sleep_time.sleep();
}

void WallRate::reset()
{
    start_ = WallTime::now();
}

WallDuration WallRate::cycleTime() const
{
    return actual_cycle_time_;
}
}
