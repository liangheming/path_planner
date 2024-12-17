#include "node_hybrid.h"
NodeHybrid::NodeHybrid(unsigned int index) : parent(nullptr),
                                             _visited(false),
                                             _index(index),
                                             _motion_index(std::numeric_limits<unsigned int>::max()),
                                             _cell_cost(std::numeric_limits<float>::quiet_NaN()),
                                             _accumulated_cost(std::numeric_limits<float>::max()),
                                             _coord()
{
}
float NodeHybrid::NeutralCost = sqrtf(2.0f);
