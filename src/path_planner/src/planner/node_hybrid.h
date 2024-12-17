#pragma once

#include "commons.h"
#include <limits>

class NodeHybrid
{
public:
    NodeHybrid *parent;
    
    NodeHybrid(unsigned int index);
    bool &visited() { return _visited; }
    float &cellCost() { return _cell_cost; }
    float &accumulatedCost() { return _accumulated_cost; }
    unsigned int &index() { return _index; }
    unsigned int &motionIndex() { return _motion_index; }
    Coordinate &coordinate() { return _coord; }
    bool operator==(const NodeHybrid &rhs) { return this->_index == rhs._index; }
    bool operator!=(const NodeHybrid &rhs) { return !(*this == rhs); }

    static float NeutralCost;

private:
    bool _visited;
    float _cell_cost;
    float _accumulated_cost;
    unsigned int _index;
    unsigned int _motion_index;
    Coordinate _coord;
};