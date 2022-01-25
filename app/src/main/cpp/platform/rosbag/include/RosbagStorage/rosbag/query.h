/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef OB_SLAM_ROSBAG_QUERY_H
#define OB_SLAM_ROSBAG_QUERY_H

#include "MessageType/rostime/time.h"
#include "structures.h"

#include <functional>
#include <map>
#include <set>
#include <vector>

namespace ob_slam {
namespace rosbag {

    class Bag;

    class Query {
    public:
        //! The base query takes an optional time-range
        /*!
     * param start_time the beginning of the time_range for the query
     * param end_time   the end of the time_range for the query
     */
        Query(std::function<bool(ConnectionInfo const*)>& query,
            ob_slam::Time const& start_time = ob_slam::TIME_MIN,
            ob_slam::Time const& end_time = ob_slam::TIME_MAX);

        std::function<bool(ConnectionInfo const*)> const& getQuery() const; //!< Get the query functor

        ob_slam::Time const& getStartTime() const; //!< Get the start-time
        ob_slam::Time const& getEndTime() const; //!< Get the end-time

    private:
        std::function<bool(ConnectionInfo const*)> query_;
        ob_slam::Time start_time_;
        ob_slam::Time end_time_;
    };

    class TopicQuery {
    public:
        TopicQuery(std::string const& topic);
        TopicQuery(std::vector<std::string> const& topics);

        bool operator()(ConnectionInfo const*) const;

    private:
        std::vector<std::string> topics_;
    };

    class TypeQuery {
    public:
        TypeQuery(std::string const& type);
        TypeQuery(std::vector<std::string> const& types);

        bool operator()(ConnectionInfo const*) const;

    private:
        std::vector<std::string> types_;
    };

    //! Pairs of queries and the bags they come from (used internally by View)
    struct BagQuery {
        BagQuery(Bag const* _bag, Query const& _query, uint32_t _bag_revision);

        Bag const* bag;
        Query query;
        uint32_t bag_revision;
    };

    struct MessageRange {
        MessageRange(std::multiset<IndexEntry>::const_iterator const& _begin,
            std::multiset<IndexEntry>::const_iterator const& _end,
            ConnectionInfo const* _connection_info,
            BagQuery const* _bag_query);

        std::multiset<IndexEntry>::const_iterator begin;
        std::multiset<IndexEntry>::const_iterator end;
        ConnectionInfo const* connection_info;
        BagQuery const* bag_query; //!< pointer to vector of queries in View
    };

    //! The actual iterator data structure
    struct ViewIterHelper {
        ViewIterHelper(std::multiset<IndexEntry>::const_iterator _iter, MessageRange const* _range);

        std::multiset<IndexEntry>::const_iterator iter;
        MessageRange const* range; //!< pointer to vector of ranges in View
    };

    struct ViewIterHelperCompare {
        bool operator()(ViewIterHelper const& a, ViewIterHelper const& b);
    };

} // namespace rosbag
}

#endif
