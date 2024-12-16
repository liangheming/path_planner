// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#pragma once
#include "commons.h"
#include "collision_checker.h"
#include <costmap_2d/costmap_2d.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

struct SmoothInfo
{
    float minimum_turning_radius;
    MotionType motion_type;
    bool is_holonomic{false};
    unsigned int max_iter{1000};
    double tolerance = 1e-4;
    double w_smooth = 0.3;
    double w_data = 0.2;
};

struct Segment
{
    unsigned int start;
    unsigned int end;
};

using Segments = std::vector<Segment>;

struct BoundaryExpansion
{
    unsigned int path_end_idx{0};
    double expansion_path_length{0.0};
    double original_path_length{0.0};
    Pose2ds pts;
    bool is_collision{false};
};

using BoundaryExpansions = std::vector<BoundaryExpansion>;
using ForwardPathIterator = Coordinates::iterator;
using BackwardPathIterator = Coordinates::reverse_iterator;

class Smoother
{
public:
    Smoother(SmoothInfo &search_info, CollisionChecker &collision_checker);

    bool smoothPath(Coordinates &path);

    Segments findDirectionSegments(Coordinates &path);

    bool smoothSegment(Coordinates &path, bool &is_reverse);

    void updateApproximatePathOrientations(Coordinates &path, bool &is_reverse);
    void enforceStartBoundaryConditions(const Coordinate &start_coord, Coordinates &path, const bool &is_reverse);
    void enforceEndBoundaryConditions(const Coordinate &start_coord, Coordinates &path, const bool &is_reverse);

    template <typename IteratorT>
    BoundaryExpansions generateBoundaryExpansionPoints(IteratorT start, IteratorT end);

    void findBoundaryExpansion(const Coordinate &start, const Coordinate &end, BoundaryExpansion &expansion);

    unsigned int findShortestBoundaryExpansionIdx(const BoundaryExpansions &boundary_expansions);

private:
    SmoothInfo _smooth_info;
    CollisionChecker _collision_checker;
    costmap_2d::Costmap2D *_costmap;
    ompl::base::StateSpacePtr _state_space;
};