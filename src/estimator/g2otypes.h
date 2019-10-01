//
// Created by wangweihan on 2019/8/22.
//

#ifndef ORB_SLAM2_G2OTYPES_H
#define ORB_SLAM2_G2OTYPES_H
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "src/estimator/VertexState.h"
#include "utility.h"
#include "src/estimator/integration.h"
namespace g2o
{

using namespace ORB_SLAM2;

//Vertex of P & R construction in graph optimization
class VertexPR : public BaseVertex<6,  VertexState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPR() : BaseVertex<6, VertexState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl()
    {
        _estimate =  VertexState();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<const Vector6d> update(update_);
        _estimate.IncSmallPR(update);
    }
};
//Vertex of Velocity(V) and bias construction in graph optimization
class VertexVBias: public BaseVertex<9, VertexState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexVBias() : BaseVertex<9, VertexState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl() {
        _estimate = VertexState();
    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector9d> update(update_);
        _estimate.IncSmallVBias(update);
    }

};
//Vertex of Velocity(V) construction in graph optimization
class VertexV: public BaseVertex<3, VertexState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexV() : BaseVertex<3, VertexState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl() {
        _estimate = VertexState();
    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector9d> update(update_);
        _estimate.IncSmallVBias(update);
    }

};

//Vertex of Bias(bias) construction in graph optimization
class VertexBias : public BaseVertex<6, VertexState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexBias() : BaseVertex<6, VertexState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl() {
        _estimate = VertexState();

    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector6d> update(update_);

        _estimate.IncSmallBias(update);
    }
};

//Edges of P,V,R, Bias in graph optimization
class EdgeStatePRVB : public BaseMultiEdge<15, Integration>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeStatePRVB():BaseMultiEdge<15, Integration>()
    {
      resize(4);  // there are 4 vertex
    }

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError();

    virtual void linearizeOplus();

    void setGravity(const Vector3d& gw) {
        GravityVec = gw;
    }

// Gravity vector in world frame(c0 frame)
    Vector3d GravityVec;
};

}
#endif //ORB_SLAM2_G2OTYPES_H
