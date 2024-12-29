#pragma once
#include "../commons.h"
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>

template <int D, typename M, typename N>
class TebUnaryEdge : public g2o::BaseUnaryEdge<D, M, N>
{
public:
    virtual bool read(std::istream &is) override
    {
        return true;
    }
    virtual bool write(std::ostream &os) const override
    {
        return os.good();
    }
};

template <int D, typename M, typename N1, typename N2>
class TebBinaryEdge : public g2o::BaseBinaryEdge<D, M, N1, N2>
{
public:
    virtual bool read(std::istream &is) override
    {
        return true;
    }
    virtual bool write(std::ostream &os) const override
    {
        return os.good();
    }
};

template <int D, typename M>
class TebMultiEdge : public g2o::BaseMultiEdge<D, M>
{
public:
    virtual bool read(std::istream &is) override
    {
        return true;
    }
    virtual bool write(std::ostream &os) const override
    {
        return os.good();
    }
};