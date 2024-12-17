#pragma once
#include "commons.h"
#include "node_hybrid.h"
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

struct MotionHybrid
{
    Motion getProjection(NodeHybrid *node, unsigned int motion_index);

    Motions getProjections(NodeHybrid *node);

    void init(const SearchInfo &search_info);

    void initDubins();

    void initReedsShepp();

    void normalizeHeading(float &heading);

    unsigned int getIndex(const unsigned int &x, const unsigned int &y, const unsigned int &theta);
    
    Coordinate getCoordinate(const unsigned int &index);

    Motions projections;
    unsigned int size_x;
    unsigned int size_y;
    unsigned int size_theta;

    float size_theta_float;
    float bin_size;
    float minimum_turning_radius;

    ompl::base::StateSpacePtr state_space;
};