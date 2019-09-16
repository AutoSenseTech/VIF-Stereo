//
// Created by wangweihan on 19-8-26.
//

#ifndef ORB_SLAM2_SO3_H
#define ORB_SLAM2_SO3_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Geometry>


namespace Sophus
{
    using namespace Eigen;

    const double SMALL_EPS = 1e-10;

    class SO3
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Jr, right jacobian of SO(3)
        static Matrix3d JacobianR(const Vector3d& w);
        // Jr^(-1)
        static Matrix3d JacobianRInv(const Vector3d& w);
        // Jl, left jacobian of SO(3), Jl(x) = Jr(-x)
        static Matrix3d JacobianL(const Vector3d& w);
        // Jl^(-1)
        static Matrix3d JacobianLInv(const Vector3d& w);

        // ----------------------------------------

        SO3                        ();

        SO3                        (const SO3 & other);

        explicit
        SO3                        (const Matrix3d & _R);

        explicit
        SO3                        (const Quaterniond & unit_quaternion);

        SO3                        (double rot_x,
                                    double rot_y,
                                    double rot_z);
        void
        operator=                  (const SO3 & so3);

        SO3
        operator*                  (const SO3 & so3) const;

        void
        operator*=                 (const SO3 & so3);

        Vector3d
        operator*                  (const Vector3d & xyz) const;

        SO3
        inverse                    () const;

        Matrix3d
        matrix                     () const;

        Matrix3d
        Adj                        () const;

        Matrix3d
        generator                  (int i);

        Vector3d
        log                        () const;

        static SO3
        exp                        (const Vector3d & omega);

        static SO3
        expAndTheta                (const Vector3d & omega,
                                    double * theta);
        static Vector3d
        log                        (const SO3 & so3);

        static Vector3d
        logAndTheta                (const SO3 & so3,
                                    double * theta);

        static Matrix3d
        hat                        (const Vector3d & omega);

        static Vector3d
        vee                        (const Matrix3d & Omega);

        static Vector3d
        lieBracket                 (const Vector3d & omega1,
                                    const Vector3d & omega2);

        static Matrix3d
        d_lieBracketab_by_d_a      (const Vector3d & b);

        void
        setQuaternion              (const Quaterniond& quaternion);

        const Quaterniond & unit_quaternion() const
        {
            return unit_quaternion_;
        }

        static const int DoF = 3;

    protected:
        Quaterniond unit_quaternion_;
    };

    inline std::ostream& operator <<(std::ostream & out_str,
                                     const SO3 & so3)
    {

        out_str << so3.log().transpose() << std::endl;
        return out_str;
    }

} // end namespace

#endif //ORB_SLAM2_SO3_H
